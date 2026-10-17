#include "core/Merge.h"

#include <algorithm>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include "core/Kinematics.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "sim/MujocoBridge.h"

namespace mjf {

namespace {

// A floor: an upward-facing plane of the world (walls and ceilings are not).
bool isFloor(const Geom& g) { return g.body == kWorldId && g.type == GeomType::Plane && rotate(g.quat, {0, 0, 1}).z > 0.999; }

bool sameTexture(const Texture& a, const Texture& b) {
  return a.type == b.type && a.builtin == b.builtin && a.rgb1 == b.rgb1 && a.rgb2 == b.rgb2 && a.mark == b.mark &&
         a.markrgb == b.markrgb && a.width == b.width && a.height == b.height && a.file == b.file &&
         a.random == b.random && a.gridsize[0] == b.gridsize[0] && a.gridsize[1] == b.gridsize[1] &&
         a.gridlayout == b.gridlayout && a.hflip == b.hflip && a.vflip == b.vflip;
}

bool sameMaterial(const Material& a, const Material& b, const Document& da, const Document& db) {
  const Texture* ta = da.texture(a.texture);
  const Texture* tb = db.texture(b.texture);
  bool texSame = (!ta && !tb) || (ta && tb && sameTexture(*ta, *tb));
  return a.rgba == b.rgba && a.texrepeat == b.texrepeat && a.texuniform == b.texuniform && a.emission == b.emission &&
         a.specular == b.specular && a.shininess == b.shininess && a.reflectance == b.reflectance && texSame;
}

template <class T>
bool sameData(const std::shared_ptr<const std::vector<T>>& a, const std::shared_ptr<const std::vector<T>>& b) {
  return a == b || (a && b && *a == *b);
}

// Settings that decide the mass of bodies without an explicit inertial.
bool sameMassSettings(const CompilerSettings& a, const CompilerSettings& b) {
  return a.inertiafromgeom == b.inertiafromgeom && a.boundmass == b.boundmass && a.boundinertia == b.boundinertia &&
         a.settotalmass == b.settotalmass && a.balanceinertia == b.balanceinertia;
}

class Merger {
 public:
  Merger(Document& dst, const Document& src, const MergeOptions& opt) : dst_(dst), src_(src), opt_(opt) {}

  MergeResult run() {
    if (opt_.environment && opt_.replaceEnvironment) removeEnvironment();
    selectElements();
    copyBodies();
    copyAttached();
    copyDependents();
    copyKeyframes();
    keepMasses();
    if (opt_.environment) copySkybox();
    if (opt_.settings) {
      // The compiler's mass settings stay: they would re-weigh the models already here (the imported
      // bodies keep their own masses, see keepMasses).
      dst_.option = src_.option;
      dst_.compiler.autolimits = src_.compiler.autolimits;
      dst_.visual = src_.visual;
    }
    for (Id root : res_.modelRoots)
      if (Body* b = dst_.body(root)) b->pos += opt_.modelOffset;
    offsetWorldTies();
    dst_.touch();
    return res_;
  }

 private:
  // ---------------------------------------------------------------- selection
  void removeEnvironment() {
    std::vector<Id> ids;
    for (const Geom& g : dst_.geoms) if (g.body == kWorldId) ids.push_back(g.id);
    for (const Site& s : dst_.sites) if (s.body == kWorldId) ids.push_back(s.id);
    for (const Camera& c : dst_.cameras) if (c.body == kWorldId) ids.push_back(c.id);
    for (const Light& l : dst_.lights) if (l.body == kWorldId) ids.push_back(l.id);
    for (const Body& b : dst_.bodies)
      if (b.id != kWorldId && b.parent == kWorldId && b.role == Role::Environment) ids.push_back(b.id);
    if (!ids.empty()) deleteElements(dst_, ids);
    // The source's skybox replaces ours.
    dst_.textures.erase(std::remove_if(dst_.textures.begin(), dst_.textures.end(),
                                       [](const Texture& t) { return t.type == TextureType::Skybox; }),
                        dst_.textures.end());
    dst_.touch();
  }

  void selectElements() {
    for (const Body& b : src_.bodies) {
      if (b.id == kWorldId || b.parent != kWorldId) continue;
      bool model = b.role == Role::Model;
      if ((model && opt_.models) || (!model && opt_.environment)) topBodies_.push_back(b.id);
    }
    for (Id top : topBodies_)
      for (Id b : src_.subtree(top)) bodies_.insert(b);
    Id dstFloor = kNoId;
    for (const Geom& g : dst_.geoms)
      if (isFloor(g) && !dstFloor) dstFloor = g.id;
    auto attachedSelected = [&](Id body) { return bodies_.count(body) > 0 || (body == kWorldId && opt_.environment); };
    for (const Geom& g : src_.geoms) {
      if (isFloor(g) && dstFloor) {
        // The scene has a floor already: what referred to this one (contact pairs, sensors) uses it.
        remap_[g.id] = dstFloor;
        if (attachedSelected(g.body)) ++res_.skippedFloors;
        continue;
      }
      if (attachedSelected(g.body)) geoms_.push_back(&g);
    }
    for (const Site& s : src_.sites) if (attachedSelected(s.body)) sites_.push_back(&s);
    for (const Camera& c : src_.cameras) if (attachedSelected(c.body)) cameras_.push_back(&c);
    for (const Light& l : src_.lights) if (attachedSelected(l.body)) lights_.push_back(&l);
    for (const Joint& j : src_.joints) if (bodies_.count(j.body)) joints_.push_back(&j);
  }

  // ---------------------------------------------------------------- helpers
  std::string name(Kind kind, const std::string& n) {
    if (n.empty()) return n;
    std::string u = dst_.uniqueName(kind, n);
    if (u != n) ++res_.renamed;
    return u;
  }
  Id map(Id id) const {
    auto it = remap_.find(id);
    return it == remap_.end() ? kNoId : it->second;
  }
  Id mapBody(Id id) const { return id == kWorldId ? kWorldId : map(id); }

  Id material(Id srcId) {
    if (!srcId) return kNoId;
    if (Id m = map(srcId)) return m;
    const Material* sm = src_.material(srcId);
    if (!sm) return kNoId;
    for (const Material& dm : dst_.materials)
      if (dm.name == sm->name && sameMaterial(dm, *sm, dst_, src_)) return remap_[srcId] = dm.id;
    Material m = *sm;
    m.id = dst_.newId();
    m.name = name(Kind::Material, m.name);
    m.texture = texture(sm->texture);
    remap_[srcId] = m.id;
    dst_.materials.push_back(m);
    dst_.touch();
    ++res_.elements;
    return m.id;
  }

  Id texture(Id srcId) {
    if (!srcId) return kNoId;
    if (Id t = map(srcId)) return t;
    const Texture* st = src_.texture(srcId);
    if (!st) return kNoId;
    for (const Texture& dt : dst_.textures)
      if (dt.name == st->name && sameTexture(dt, *st)) return remap_[srcId] = dt.id;
    Texture t = *st;
    t.id = dst_.newId();
    t.name = name(Kind::Texture, t.name);
    remap_[srcId] = t.id;
    dst_.textures.push_back(t);
    dst_.touch();
    ++res_.elements;
    return t.id;
  }

  Id hfield(Id srcId) {
    if (!srcId) return kNoId;
    if (Id h = map(srcId)) return h;
    const Hfield* sh = src_.hfield(srcId);
    if (!sh) return kNoId;
    for (const Hfield& dh : dst_.hfields)
      if (dh.nrow == sh->nrow && dh.ncol == sh->ncol && dh.size == sh->size && sameData(dh.data, sh->data))
        return remap_[srcId] = dh.id;
    Hfield h = *sh;  // data is shared copy-on-write
    h.id = dst_.newId();
    h.name = name(Kind::Hfield, h.name);
    remap_[srcId] = h.id;
    dst_.hfields.push_back(h);
    dst_.touch();
    ++res_.elements;
    return h.id;
  }

  Id mesh(Id srcId) {
    if (!srcId) return kNoId;
    if (Id m = map(srcId)) return m;
    const MeshAsset* sm = src_.mesh(srcId);
    if (!sm) return kNoId;
    // The same mesh imported twice (e.g. two copies of one robot) is stored once.
    for (const MeshAsset& dm : dst_.meshes)
      if (dm.inertia == sm->inertia && dm.maxhullvert == sm->maxhullvert && sameData(dm.vert, sm->vert) &&
          sameData(dm.face, sm->face))
        return remap_[srcId] = dm.id;
    MeshAsset m = *sm;  // vertex data is shared, never modified in place
    m.id = dst_.newId();
    m.name = name(Kind::Mesh, m.name);
    remap_[srcId] = m.id;
    dst_.meshes.push_back(m);
    dst_.touch();
    ++res_.elements;
    return m.id;
  }

  // ---------------------------------------------------------------- copying
  void copyBodies() {
    for (Id top : topBodies_) {
      for (Id sb : src_.subtree(top)) {
        Body b = *src_.body(sb);
        b.id = dst_.newId();
        b.name = name(Kind::Body, b.name);
        b.parent = sb == top ? kWorldId : map(b.parent);
        remap_[sb] = b.id;
        dst_.bodies.push_back(b);
        dst_.touch();
        ++res_.elements;
      }
      Id nb = map(top);
      res_.topLevel.push_back(nb);
      if (src_.body(top)->role == Role::Model) res_.modelRoots.push_back(nb);
    }
  }

  void copyAttached() {
    for (const Joint* sj : joints_) {
      Joint j = *sj;
      j.id = dst_.newId();
      j.name = name(Kind::Joint, j.name);
      j.body = mapBody(sj->body);
      remap_[sj->id] = j.id;
      dst_.joints.push_back(j);
      ++res_.elements;
    }
    for (const Geom* sg : geoms_) {
      Geom g = *sg;
      g.id = dst_.newId();
      g.name = name(Kind::Geom, g.name);
      g.body = mapBody(sg->body);
      g.material = material(sg->material);
      g.hfield = hfield(sg->hfield);
      g.mesh = mesh(sg->mesh);
      remap_[sg->id] = g.id;
      dst_.geoms.push_back(g);
      if (g.body == kWorldId) res_.topLevel.push_back(g.id);
      ++res_.elements;
    }
    for (const Site* ss : sites_) {
      Site s = *ss;
      s.id = dst_.newId();
      s.name = name(Kind::Site, s.name);
      s.body = mapBody(ss->body);
      s.material = material(ss->material);
      remap_[ss->id] = s.id;
      dst_.sites.push_back(s);
      ++res_.elements;
    }
    for (const Camera* sc : cameras_) {
      Camera c = *sc;
      c.id = dst_.newId();
      c.name = name(Kind::Camera, c.name);
      c.body = mapBody(sc->body);
      c.target = sc->target ? mapBody(sc->target) : kNoId;  // kept only if the target came along
      if (!c.target && (c.mode == CameraMode::TargetBody || c.mode == CameraMode::TargetBodyCom)) c.mode = CameraMode::Fixed;
      remap_[sc->id] = c.id;
      dst_.cameras.push_back(c);
      ++res_.elements;
    }
    for (const Light* sl : lights_) {
      Light l = *sl;
      l.id = dst_.newId();
      l.name = name(Kind::Light, l.name);
      l.body = mapBody(sl->body);
      l.target = sl->target ? mapBody(sl->target) : kNoId;
      if (!l.target && (l.mode == CameraMode::TargetBody || l.mode == CameraMode::TargetBodyCom)) l.mode = CameraMode::Fixed;
      remap_[sl->id] = l.id;
      dst_.lights.push_back(l);
      ++res_.elements;
    }
    dst_.touch();
  }

  void copyDependents() {
    auto mapOpt = [&](Id id, bool* ok) -> Id {
      if (!id) return kNoId;
      Id m = id == kWorldId ? kWorldId : map(id);
      if (!m) *ok = false;
      return m;
    };
    // Tendons first: actuators, sensors and equality constraints can refer to them.
    for (const Tendon& st : src_.tendons) {
      bool ok = true;
      Tendon t = st;
      for (TendonJoint& j : t.joints) j.joint = mapOpt(j.joint, &ok);
      for (TendonPoint& p : t.path) {
        p.obj = mapOpt(p.obj, &ok);
        p.sidesite = mapOpt(p.sidesite, &ok);
      }
      if (!ok || (t.joints.empty() && t.path.empty())) continue;
      t.id = dst_.newId();
      t.name = name(Kind::Tendon, t.name);
      t.material = material(st.material);
      remap_[st.id] = t.id;
      dst_.tendons.push_back(t);
      ++res_.elements;
    }
    for (const Actuator& sa : src_.actuators) {
      bool ok = true;
      Actuator a = sa;
      a.target = mapOpt(sa.target, &ok);
      a.refsite = mapOpt(sa.refsite, &ok);
      a.slidersite = mapOpt(sa.slidersite, &ok);
      if (!ok || !a.target) continue;
      a.id = dst_.newId();
      a.name = name(Kind::Actuator, a.name);
      remap_[sa.id] = a.id;
      dst_.actuators.push_back(a);
      ++res_.elements;
    }
    for (const Sensor& ss : src_.sensors) {
      SensorObj kind = sensorInfo(ss.type).obj;
      // A contact sensor without objects watches every contact, like a global sensor.
      bool global = kind == SensorObj::None || (kind == SensorObj::Contact && !ss.obj && !ss.ref);
      bool ok = true;
      Sensor s = ss;
      s.obj = mapOpt(ss.obj, &ok);
      s.ref = mapOpt(ss.ref, &ok);
      bool anchored = s.obj || (kind == SensorObj::Contact && s.ref);
      if (!ok || (global && !opt_.models) || (!global && !anchored)) continue;
      s.id = dst_.newId();
      s.name = name(Kind::Sensor, s.name);
      dst_.sensors.push_back(s);
      ++res_.elements;
    }
    for (const Exclude& se : src_.excludes) {
      Id b1 = mapBody(se.body1), b2 = mapBody(se.body2);
      if (!b1 || !b2) continue;
      Exclude e = se;
      e.id = dst_.newId();
      e.name = name(Kind::Exclude, e.name);
      e.body1 = b1;
      e.body2 = b2;
      dst_.excludes.push_back(e);
      ++res_.elements;
    }
    for (const ContactPair& sc : src_.pairs) {
      bool ok = true;
      ContactPair c = sc;
      c.geom1 = mapOpt(sc.geom1, &ok);
      c.geom2 = mapOpt(sc.geom2, &ok);
      if (!ok || !c.geom1 || !c.geom2) continue;
      c.id = dst_.newId();
      c.name = name(Kind::Pair, c.name);
      dst_.pairs.push_back(c);
      ++res_.elements;
    }
    for (const Equality& se : src_.equalities) {
      bool ok = true;
      Equality e = se;
      e.obj1 = mapOpt(se.obj1, &ok);
      e.obj2 = mapOpt(se.obj2, &ok);
      if (!ok || !e.obj1) continue;
      e.id = dst_.newId();
      e.name = name(Kind::Equality, e.name);
      importedEqualities_.insert(e.id);
      dst_.equalities.push_back(e);
      ++res_.elements;
    }
    dst_.touch();
  }

  // Keyframe 0 is the start state: the imported start pose joins the destination's keyframe 0
  // whatever their names. Other keyframes join a destination keyframe of the same (non-empty)
  // name or are added; two imported keyframes never share one.
  void copyKeyframes() {
    std::vector<bool> taken(dst_.keyframes.size(), false);
    auto isRoot = [&](Id body) { return std::find(res_.modelRoots.begin(), res_.modelRoots.end(), body) != res_.modelRoots.end(); };
    for (size_t si = 0; si < src_.keyframes.size(); ++si) {
      const Keyframe& sk = src_.keyframes[si];
      Keyframe add;
      auto mapped = [&](const auto& from, auto& to) {
        for (const auto& [id, val] : from)
          if (Id m = map(id)) to.push_back({m, val});
      };
      mapped(sk.qpos, add.qpos);
      mapped(sk.qvel, add.qvel);
      mapped(sk.ctrl, add.ctrl);
      mapped(sk.act, add.act);
      mapped(sk.mocap, add.mocap);
      // Free joints and mocap bodies of moved model roots: their start poses move with the root.
      for (auto& [id, q] : add.qpos) {
        const Joint* j = dst_.joint(id);
        if (j && j->type == JointType::Free && q.size() == 7 && isRoot(j->body))
          for (int i = 0; i < 3; ++i) q[i] += opt_.modelOffset[i];
      }
      for (auto& [id, p] : add.mocap)
        if (isRoot(id)) p.pos += opt_.modelOffset;
      if (add.qpos.empty() && add.qvel.empty() && add.ctrl.empty() && add.act.empty() && add.mocap.empty()) continue;
      int target = -1;
      if (si == 0 && !taken.empty() && !taken[0]) target = 0;
      for (size_t d = 0; target < 0 && !sk.name.empty() && d < dst_.keyframes.size(); ++d)
        if (!taken[d] && dst_.keyframes[d].name == sk.name) target = int(d);
      if (target < 0) {
        Keyframe k;
        k.name = sk.name;
        k.time = sk.time;
        for (int n = 2; !k.name.empty() && std::any_of(dst_.keyframes.begin(), dst_.keyframes.end(),
                                                         [&](const Keyframe& x) { return x.name == k.name; });
             ++n)
          k.name = sk.name + "_" + std::to_string(n);
        dst_.keyframes.push_back(std::move(k));
        taken.push_back(false);
        target = int(dst_.keyframes.size()) - 1;
      }
      taken[size_t(target)] = true;
      Keyframe& t = dst_.keyframes[size_t(target)];
      t.qpos.insert(t.qpos.end(), add.qpos.begin(), add.qpos.end());
      t.qvel.insert(t.qvel.end(), add.qvel.begin(), add.qvel.end());
      t.ctrl.insert(t.ctrl.end(), add.ctrl.begin(), add.ctrl.end());
      t.act.insert(t.act.end(), add.act.begin(), add.act.end());
      t.mocap.insert(t.mocap.end(), add.mocap.begin(), add.mocap.end());
    }
  }

  // Imported bodies keep the masses they had in their own file: where the destination's compiler
  // mass settings differ, MuJoCo's compiled inertia of the source becomes their explicit inertial.
  void keepMasses() {
    if (sameMassSettings(src_.compiler, dst_.compiler)) return;
    ExportMap exported;
    CompileResult cr = compileMjcf(writeMjcf(src_, {}, &exported));
    cr.map = exported;
    InertialTable table = compiledInertials(cr);
    if (table.empty()) return;  // the source does not compile: nothing better to keep
    for (const auto& [srcId, inertial] : table) {
      Body* b = dst_.body(map(srcId));
      if (!b) continue;
      b->hasInertial = true;
      b->mass = inertial.mass;
      b->ipos = inertial.ipos;
      b->iquat = inertial.iquat;
      b->diaginertia = inertial.diaginertia;
    }
  }

  // Constraints that hold an imported model to the world move with the model offset, so the scene
  // is the source shifted as a whole: world anchors and world sites the model is tied to.
  void offsetWorldTies() {
    const Vec3& d = opt_.modelOffset;
    if (length(d) == 0) return;
    std::unordered_set<Id> moved;
    for (Id root : res_.modelRoots)
      for (Id b : dst_.subtree(root)) moved.insert(b);
    for (Equality& e : dst_.equalities) {
      if (!importedEqualities_.count(e.id) || (e.type != EqualityType::Weld && e.type != EqualityType::Connect)) continue;
      if (dst_.kindOf(e.obj1) == Kind::Site) {
        // Two sites: a world site tied to a moved model goes along.
        Site* s1 = dst_.site(e.obj1);
        Site* s2 = dst_.site(e.obj2);
        if (s1 && s2 && moved.count(s1->body) && s2->body == kWorldId) s2->pos += d;
        if (s1 && s2 && moved.count(s2->body) && s1->body == kWorldId) s1->pos += d;
        continue;
      }
      bool worldSecond = e.obj2 == kNoId || e.obj2 == kWorldId;
      if (e.type == EqualityType::Weld && moved.count(e.obj1) && worldSecond) {
        e.anchor += d;  // a weld's anchor is in body2's frame, here the world's
      } else if (e.obj1 == kWorldId && moved.count(e.obj2)) {
        if (e.type == EqualityType::Connect) e.anchor += d;  // in body1's frame: the world's
        else if (e.relpose) e.relpos += d;                   // body2's place in the world frame
      }
      // Other anchors are in a moved body's frame, or derived from the start pose.
    }
  }

  void copySkybox() {
    for (const Texture& t : dst_.textures)
      if (t.type == TextureType::Skybox) return;
    for (const Texture& t : src_.textures)
      if (t.type == TextureType::Skybox) texture(t.id);
  }

  Document& dst_;
  const Document& src_;
  const MergeOptions& opt_;
  MergeResult res_;
  std::vector<Id> topBodies_;
  std::unordered_set<Id> bodies_;
  std::vector<const Geom*> geoms_;
  std::vector<const Site*> sites_;
  std::vector<const Camera*> cameras_;
  std::vector<const Light*> lights_;
  std::vector<const Joint*> joints_;
  std::unordered_map<Id, Id> remap_;
  std::unordered_set<Id> importedEqualities_;
};

}  // namespace

MergeResult mergeDocument(Document& dst, const Document& src, const MergeOptions& opt) {
  if (&dst == &src) {  // the merger reads the source while it grows the destination
    Document copy = src;
    return Merger(dst, copy, opt).run();
  }
  return Merger(dst, src, opt).run();
}

DocSummary summarize(const Document& doc) {
  DocSummary s;
  for (const Body& b : doc.bodies) {
    if (b.id == kWorldId || b.parent != kWorldId) continue;
    if (b.role == Role::Model) s.models.push_back(b.name);
    else ++s.environmentBodies;
  }
  for (const Geom& g : doc.geoms)
    if (g.body == kWorldId) {
      ++s.worldGeoms;
      s.hasFloor |= isFloor(g);
    }
  for (const Light& l : doc.lights) s.lights += l.body == kWorldId;
  return s;
}

Aabb modelsBounds(const Document& doc) {
  Aabb box;
  auto poses = computeBodyPoses(doc);
  for (const Body& b : doc.bodies)
    if (b.id != kWorldId && b.parent == kWorldId && b.role == Role::Model) box.add(subtreeAabb(doc, poses, b.id));
  return box;
}

}  // namespace mjf
