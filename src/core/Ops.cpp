#include "core/Ops.h"

#include <algorithm>
#include <cctype>
#include <functional>
#include <unordered_map>
#include <unordered_set>

#include "core/Geometry.h"
#include "core/Kinematics.h"

#include <cmath>

namespace mjf {

// ---------------------------------------------------------------------------------- creation

Id addBody(Document& doc, Id parent, const Pose& local, const std::string& name, Role role) {
  Body b;
  b.id = doc.newId();
  b.name = doc.uniqueName(Kind::Body, name);
  b.parent = doc.body(parent) ? parent : kWorldId;
  b.pos = local.pos;
  b.quat = canonical(local.quat);
  b.role = role;
  doc.bodies.push_back(b);
  doc.touch();
  return b.id;
}

Id addGeom(Document& doc, Id body, GeomType type, const Vec3& size, const Pose& local, const std::string& name) {
  Geom g;
  g.id = doc.newId();
  g.body = doc.body(body) ? body : kWorldId;
  g.type = type;
  g.size = size;
  g.pos = local.pos;
  g.quat = canonical(local.quat);
  std::string base = name;
  if (base.empty()) {
    const Body* b = doc.body(g.body);
    base = (g.body == kWorldId) ? std::string(enumXml(kGeomTypes, type)) : b->name + "_geom";
  }
  g.name = doc.uniqueName(Kind::Geom, base);
  doc.geoms.push_back(g);
  doc.touch();
  return g.id;
}

Id addJoint(Document& doc, Id body, JointType type, const Vec3& pos, const Vec3& axis, const std::string& name) {
  Joint j;
  j.id = doc.newId();
  j.body = body;
  j.type = type;
  j.pos = pos;
  j.axis = normalize(axis);
  std::string base = name;
  if (base.empty()) {
    const Body* b = doc.body(body);
    base = (b ? b->name : std::string("joint")) + (type == JointType::Free ? "_free" : "_joint");
  }
  j.name = doc.uniqueName(Kind::Joint, base);
  doc.joints.push_back(j);
  doc.touch();
  return j.id;
}

Id addSite(Document& doc, Id body, const Pose& local, const std::string& name) {
  Site s;
  s.id = doc.newId();
  s.body = doc.body(body) ? body : kWorldId;
  s.pos = local.pos;
  s.quat = canonical(local.quat);
  s.size = {0.01, 0.01, 0.01};
  s.rgba = {0.95f, 0.75f, 0.1f, 1.0f};
  const Body* b = doc.body(s.body);
  s.name = doc.uniqueName(Kind::Site, name.empty() ? (b->id == kWorldId ? "site" : b->name + "_site") : name);
  doc.sites.push_back(s);
  doc.touch();
  return s.id;
}

Id addCamera(Document& doc, Id body, const Pose& local, const std::string& name) {
  Camera c;
  c.id = doc.newId();
  c.body = doc.body(body) ? body : kWorldId;
  c.pos = local.pos;
  c.quat = canonical(local.quat);
  c.name = doc.uniqueName(Kind::Camera, name.empty() ? "camera" : name);
  doc.cameras.push_back(c);
  doc.touch();
  return c.id;
}

Id addLight(Document& doc, Id body, const Vec3& pos, const Vec3& dir, const std::string& name) {
  Light l;
  l.id = doc.newId();
  l.body = doc.body(body) ? body : kWorldId;
  l.pos = pos;
  l.dir = normalize(dir, {0, 0, -1});
  l.name = doc.uniqueName(Kind::Light, name.empty() ? "light" : name);
  doc.lights.push_back(l);
  doc.touch();
  return l.id;
}

void setActuatorType(Document& doc, Actuator& a, ActuatorType type) {
  // Adhesion acts on a body's contacts; muscles need a joint or tendon length.
  if (type == ActuatorType::Adhesion && a.trn != TrnType::Body) {
    Id body = doc.ownerBody(a.target);
    a.trn = TrnType::Body;
    a.target = body != kWorldId ? body : kNoId;
    a.refsite = a.slidersite = kNoId;
    a.gear = Vec6{{1, 0, 0, 0, 0, 0}};
  } else if (a.trn == TrnType::Body && type != ActuatorType::Adhesion && type != ActuatorType::General) {
    Id joint = kNoId;
    for (const Joint& j : doc.joints)
      if (j.body == a.target && joint == kNoId) joint = j.id;
    a.trn = TrnType::Joint;
    a.target = joint;
  } else if (type == ActuatorType::Muscle && a.trn == TrnType::Site) {
    a.trn = TrnType::Joint;
    a.target = kNoId;
    a.refsite = kNoId;
  }
  a.type = type;
  a.kp = actuatorDefaultKp(type);
  a.kv = actuatorDefaultKv(type);
  a.dampratio = 0;
  a.timeconst = 0;
  a.inheritrange = 0;
  const Joint* j = a.trn == TrnType::Joint ? doc.joint(a.target) : nullptr;
  switch (type) {
    case ActuatorType::Position:
      if (j && j->limited) {
        a.ctrllimited = true;
        a.ctrlrange = j->range;
      }
      a.dampratio = 1.0;  // critically damped servo is a sensible starting point
      break;
    case ActuatorType::IntVelocity:
      a.ctrllimited = true;
      a.ctrlrange = {-1, 1};
      if (j && j->limited) {
        a.actlimited = true;
        a.actrange = j->range;
      }
      break;
    case ActuatorType::Damper:
      a.ctrllimited = true;
      a.ctrlrange = {0, 1};
      break;
    case ActuatorType::Adhesion:
      a.ctrllimited = true;
      a.ctrlrange = {0, 1};
      break;
    case ActuatorType::Muscle:
      a.ctrllimited = true;
      a.ctrlrange = {0, 1};
      break;
    default:
      a.ctrllimited = true;
      a.ctrlrange = {-1, 1};
      break;
  }
}

Id addActuator(Document& doc, ActuatorType type, Id joint, const std::string& name) {
  Actuator a;
  a.id = doc.newId();
  a.trn = TrnType::Joint;
  a.target = joint;
  const Joint* j = doc.joint(joint);
  std::string base = name.empty() ? (j ? j->name : std::string("actuator")) : name;
  if (name.empty()) {
    switch (type) {
      case ActuatorType::Motor: base += "_motor"; break;
      case ActuatorType::Position: base += "_pos"; break;
      case ActuatorType::Velocity: base += "_vel"; break;
      default: base += std::string("_") + toXml(type); break;
    }
  }
  a.name = doc.uniqueName(Kind::Actuator, base);
  setActuatorType(doc, a, type);
  doc.actuators.push_back(a);
  doc.touch();
  return a.id;
}

Id addSensor(Document& doc, SensorType type, Id obj, const std::string& name) {
  Sensor s;
  s.id = doc.newId();
  s.type = type;
  s.obj = obj;
  SensorObj kind = sensorInfo(type).obj;
  if (kind == SensorObj::Frame || kind == SensorObj::GeomPair || kind == SensorObj::Contact || kind == SensorObj::Inside) {
    switch (doc.kindOf(obj)) {
      case Kind::Body: s.objtype = FrameObj::Body; break;
      case Kind::Geom: s.objtype = FrameObj::Geom; break;
      case Kind::Camera: s.objtype = FrameObj::Camera; break;
      default: s.objtype = kind == SensorObj::GeomPair ? FrameObj::Geom : FrameObj::Site; break;
    }
    if (kind == SensorObj::GeomPair || kind == SensorObj::Contact) s.reftype = FrameObj::Geom;
  }
  std::string base = name;
  if (base.empty()) {
    std::string on = doc.nameOf(obj);
    base = (on.empty() ? std::string() : on + "_") + sensorInfo(type).xml;
  }
  s.name = doc.uniqueName(Kind::Sensor, base);
  doc.sensors.push_back(s);
  doc.touch();
  return s.id;
}

Id addExclude(Document& doc, Id body1, Id body2) {
  for (const Exclude& e : doc.excludes)
    if ((e.body1 == body1 && e.body2 == body2) || (e.body1 == body2 && e.body2 == body1)) return e.id;
  Exclude e;
  e.id = doc.newId();
  e.body1 = body1;
  e.body2 = body2;
  doc.excludes.push_back(e);
  doc.touch();
  return e.id;
}

void pairParamsFromGeoms(const Document& doc, ContactPair& c) {
  const Geom* a = doc.geom(c.geom1);
  const Geom* b = doc.geom(c.geom2);
  if (!a || !b) return;
  // MuJoCo's rules for geoms in contact: the higher priority wins, otherwise maxima and a
  // solmix-weighted average of the solver parameters.
  if (a->priority != b->priority) {
    const Geom& g = a->priority > b->priority ? *a : *b;
    c.condim = g.condim;
    c.friction = Vec5{{g.friction.x, g.friction.x, g.friction.y, g.friction.z, g.friction.z}};
    c.solref = g.solref;
    c.solimp = g.solimp;
    c.adhesion = g.adhesion;
  } else {
    c.condim = std::max(a->condim, b->condim);
    double s = std::max(a->friction.x, b->friction.x), t = std::max(a->friction.y, b->friction.y),
           r = std::max(a->friction.z, b->friction.z);
    c.friction = Vec5{{s, s, t, r, r}};
    double wa = a->solmix, wb = b->solmix;
    double mix = wa + wb > 1e-15 ? wa / (wa + wb) : 0.5;
    bool standard = a->solref[0] > 0 && b->solref[0] > 0;  // direct (negative) solref: elementwise minimum
    for (int i = 0; i < 2; ++i) c.solref[i] = standard ? mix * a->solref[i] + (1 - mix) * b->solref[i] : std::min(a->solref[i], b->solref[i]);
    for (int i = 0; i < 5; ++i) c.solimp[i] = mix * a->solimp[i] + (1 - mix) * b->solimp[i];
    c.adhesion = a->adhesion + b->adhesion;
  }
  c.margin = std::max(a->margin, b->margin);
  c.gap = std::max(a->gap, b->gap);
}

Id addContactPair(Document& doc, Id geom1, Id geom2) {
  ContactPair c;
  c.id = doc.newId();
  c.geom1 = geom1;
  c.geom2 = geom2;
  pairParamsFromGeoms(doc, c);
  doc.pairs.push_back(c);
  doc.touch();
  return c.id;
}

Id addEquality(Document& doc, EqualityType type, Id obj1, Id obj2) {
  Equality e;
  e.id = doc.newId();
  e.type = type;
  e.obj1 = obj1;
  e.obj2 = obj2;
  // Connect at body2's origin (or body1's when connecting to the world), as seen from body1.
  if (type == EqualityType::Connect && doc.body(obj1)) {
    Pose p1 = bodyWorldPose(doc, obj1);
    Vec3 at = doc.body(obj2) ? bodyWorldPose(doc, obj2).pos : p1.pos;
    e.anchor = inverse(p1).apply(at);
  }
  doc.equalities.push_back(e);
  doc.touch();
  return e.id;
}

Id addTendon(Document& doc, TendonType type, const std::string& name) {
  Tendon t;
  t.id = doc.newId();
  t.type = type;
  t.name = doc.uniqueName(Kind::Tendon, name.empty() ? "tendon" : name);
  doc.tendons.push_back(t);
  doc.touch();
  return t.id;
}

Id addTexture(Document& doc, const Texture& t_) {
  Texture t = t_;
  t.id = doc.newId();
  t.name = doc.uniqueName(Kind::Texture, t.name.empty() ? "texture" : t.name);
  doc.textures.push_back(t);
  doc.touch();
  return t.id;
}

Id addMaterial(Document& doc, const Material& m_) {
  Material m = m_;
  m.id = doc.newId();
  m.name = doc.uniqueName(Kind::Material, m.name.empty() ? "material" : m.name);
  doc.materials.push_back(m);
  doc.touch();
  return m.id;
}

void geomEndpoints(const Geom& g, Vec3* a, Vec3* b) {
  double h = (g.type == GeomType::Box || g.type == GeomType::Ellipsoid) ? g.size.z : g.size.y;
  Vec3 axis = rotate(g.quat, {0, 0, 1});
  if (a) *a = g.pos - axis * h;
  if (b) *b = g.pos + axis * h;
}

void setGeomFromTo(Geom& g, const Vec3& a, const Vec3& b) {
  Vec3 d = b - a;
  double len = length(d);
  g.pos = (a + b) * 0.5;
  if (len > 1e-9) g.quat = canonical(frameFromZ(d / len));
  double h = len * 0.5;
  if (g.type == GeomType::Box || g.type == GeomType::Ellipsoid) g.size.z = h;
  else g.size.y = h;
}

// ---------------------------------------------------------------------------------- deletion

void deleteElements(Document& doc, const std::vector<Id>& ids) {
  std::unordered_set<Id> del;
  for (Id id : ids) {
    if (id == kWorldId || id == kNoId) continue;
    if (doc.kindOf(id) == Kind::Body) {
      for (Id b : doc.subtree(id)) del.insert(b);
    } else {
      del.insert(id);
    }
  }
  if (del.empty()) return;
  auto bodyDeleted = [&](Id b) { return del.count(b) > 0; };
  // Elements attached to deleted bodies.
  for (const Geom& g : doc.geoms) if (bodyDeleted(g.body)) del.insert(g.id);
  for (const Joint& j : doc.joints) if (bodyDeleted(j.body)) del.insert(j.id);
  for (const Site& s : doc.sites) if (bodyDeleted(s.body)) del.insert(s.id);
  for (const Camera& c : doc.cameras) if (bodyDeleted(c.body)) del.insert(c.id);
  for (const Light& l : doc.lights) if (bodyDeleted(l.body)) del.insert(l.id);
  // Tendons lose deleted joints / path points; one with nothing usable left goes too.
  for (Tendon& t : doc.tendons) {
    if (del.count(t.id)) continue;
    t.joints.erase(std::remove_if(t.joints.begin(), t.joints.end(), [&](const TendonJoint& j) { return del.count(j.joint) > 0; }),
                   t.joints.end());
    t.path.erase(std::remove_if(t.path.begin(), t.path.end(), [&](const TendonPoint& p) { return p.obj && del.count(p.obj) > 0; }),
                 t.path.end());
    for (TendonPoint& p : t.path) if (del.count(p.sidesite)) p.sidesite = kNoId;
    int sites = 0;
    for (const TendonPoint& p : t.path) sites += p.type == TendonPoint::Type::Site;
    if ((t.type == TendonType::Fixed && t.joints.empty()) || (t.type == TendonType::Spatial && sites < 2) ||
        (t.type == TendonType::Spatial && !t.path.empty() && t.path.front().type != TendonPoint::Type::Site))
      del.insert(t.id);
  }
  // Dependents.
  for (const Actuator& a : doc.actuators)
    if (del.count(a.target) || del.count(a.refsite) || del.count(a.slidersite)) del.insert(a.id);
  for (const Sensor& s : doc.sensors)
    if (del.count(s.obj) || del.count(s.ref)) del.insert(s.id);
  for (const Exclude& e : doc.excludes)
    if (del.count(e.body1) || del.count(e.body2)) del.insert(e.id);
  for (const ContactPair& c : doc.pairs)
    if (del.count(c.geom1) || del.count(c.geom2)) del.insert(c.id);
  for (const Equality& e : doc.equalities)
    if (del.count(e.obj1) || del.count(e.obj2)) del.insert(e.id);

  auto erase = [&](auto& vec) {
    vec.erase(std::remove_if(vec.begin(), vec.end(), [&](const auto& e) { return del.count(e.id) > 0; }), vec.end());
  };
  erase(doc.bodies);
  erase(doc.geoms);
  erase(doc.joints);
  erase(doc.sites);
  erase(doc.cameras);
  erase(doc.lights);
  erase(doc.actuators);
  erase(doc.sensors);
  erase(doc.excludes);
  erase(doc.pairs);
  erase(doc.equalities);
  erase(doc.tendons);
  erase(doc.materials);
  erase(doc.textures);
  erase(doc.hfields);
  erase(doc.meshes);
  // Clear dangling soft references.
  for (Geom& g : doc.geoms) {
    if (del.count(g.material)) g.material = kNoId;
    if (del.count(g.hfield)) g.hfield = kNoId;
    if (del.count(g.mesh)) g.mesh = kNoId;
  }
  for (Site& s : doc.sites) if (del.count(s.material)) s.material = kNoId;
  for (Tendon& t : doc.tendons) if (del.count(t.material)) t.material = kNoId;
  for (Material& m : doc.materials) if (del.count(m.texture)) m.texture = kNoId;
  for (Camera& c : doc.cameras) if (del.count(c.target)) c.target = kNoId;
  for (Light& l : doc.lights) if (del.count(l.target)) l.target = kNoId;
  for (Keyframe& k : doc.keyframes) {
    auto drop = [&](auto& v) {
      v.erase(std::remove_if(v.begin(), v.end(), [&](const auto& e) { return del.count(e.first) > 0; }), v.end());
    };
    drop(k.qpos);
    drop(k.qvel);
    drop(k.ctrl);
  }
  doc.touch();
}

// ---------------------------------------------------------------------------------- hierarchy

bool canReparent(const Document& doc, Id body, Id newParent) {
  if (body == kWorldId || !doc.body(body) || !doc.body(newParent)) return false;
  if (doc.body(body)->parent == newParent) return false;
  return !doc.isAncestor(body, newParent);  // can't move into its own subtree
}

bool reparentBody(Document& doc, Id body, Id newParent, bool keepWorldPose) {
  if (!canReparent(doc, body, newParent)) return false;
  Pose world = bodyWorldPose(doc, body);
  Pose parentWorld = bodyWorldPose(doc, newParent);
  Body* b = doc.body(body);
  b->parent = newParent;
  if (keepWorldPose) {
    Pose local = inverse(parentWorld) * world;
    b->pos = local.pos;
    b->quat = canonical(local.quat);
  }
  // Move the body to the end of the list so it becomes the last child of its new parent.
  int bi = doc.bodyIndex(body);
  Body moved = doc.bodies[bi];
  doc.bodies.erase(doc.bodies.begin() + bi);
  doc.bodies.push_back(moved);
  doc.touch();
  return true;
}

bool moveToBody(Document& doc, Id element, Id newBody, bool keepWorldPose) {
  Kind k = doc.kindOf(element);
  if (k == Kind::Body) return reparentBody(doc, element, newBody, keepWorldPose);
  if (!doc.body(newBody)) return false;
  Id oldBody = doc.ownerBody(element);
  if (oldBody == newBody) return false;
  if (k == Kind::Joint && newBody == kWorldId) return false;  // joints can't live in the world
  Pose world = bodyWorldPose(doc, oldBody) * elementLocalPose(doc, element);
  Pose local = inverse(bodyWorldPose(doc, newBody)) * world;
  switch (k) {
    case Kind::Geom: doc.geom(element)->body = newBody; break;
    case Kind::Joint: doc.joint(element)->body = newBody; break;
    case Kind::Site: doc.site(element)->body = newBody; break;
    case Kind::Camera: doc.camera(element)->body = newBody; break;
    case Kind::Light: doc.light(element)->body = newBody; break;
    default: return false;
  }
  if (keepWorldPose) setElementLocalPose(doc, element, local);
  doc.touch();
  return true;
}

template <class T, class SameGroup>
static bool moveInVector(std::vector<T>& vec, Id id, int dir, SameGroup same) {
  int i = -1;
  for (int k = 0; k < int(vec.size()); ++k) if (vec[k].id == id) { i = k; break; }
  if (i < 0) return false;
  int j = i + (dir < 0 ? -1 : 1);
  while (j >= 0 && j < int(vec.size()) && !same(vec[i], vec[j])) j += (dir < 0 ? -1 : 1);
  if (j < 0 || j >= int(vec.size())) return false;
  std::swap(vec[i], vec[j]);
  return true;
}

bool moveSibling(Document& doc, Id element, int direction) {
  bool ok = false;
  switch (doc.kindOf(element)) {
    case Kind::Body:
      if (element == kWorldId) return false;
      ok = moveInVector(doc.bodies, element, direction, [](const Body& a, const Body& b) { return a.parent == b.parent; });
      break;
    case Kind::Geom: ok = moveInVector(doc.geoms, element, direction, [](const Geom& a, const Geom& b) { return a.body == b.body; }); break;
    case Kind::Joint: ok = moveInVector(doc.joints, element, direction, [](const Joint& a, const Joint& b) { return a.body == b.body; }); break;
    case Kind::Site: ok = moveInVector(doc.sites, element, direction, [](const Site& a, const Site& b) { return a.body == b.body; }); break;
    case Kind::Actuator: ok = moveInVector(doc.actuators, element, direction, [](const Actuator&, const Actuator&) { return true; }); break;
    case Kind::Sensor: ok = moveInVector(doc.sensors, element, direction, [](const Sensor&, const Sensor&) { return true; }); break;
    default: break;
  }
  if (ok) doc.touch();
  return ok;
}

// ---------------------------------------------------------------------------------- copy

namespace {

using RenameFn = std::function<std::string(const std::string&)>;

// Copies bodies (with subtrees) and loose attached elements; returns old->new ID map and the
// new top-level IDs (in input order). Dependent actuators/sensors/excludes are copied as well.
struct CopyResult {
  std::unordered_map<Id, Id> remap;
  std::vector<Id> tops;
};

CopyResult copyElements(Document& doc, const std::vector<Id>& ids, const RenameFn& rename) {
  CopyResult r;
  // Normalize: drop elements contained in a selected body's subtree.
  std::unordered_set<Id> selectedBodies;
  for (Id id : ids) if (doc.kindOf(id) == Kind::Body && id != kWorldId) selectedBodies.insert(id);
  auto coveredByBody = [&](Id body) {
    for (Id sb : selectedBodies) if (doc.isAncestor(sb, body)) return true;
    return false;
  };
  std::vector<Id> topBodies, loose;
  for (Id id : ids) {
    Kind k = doc.kindOf(id);
    if (k == Kind::Body) {
      if (id == kWorldId) continue;
      if (doc.body(id)->parent != kWorldId && coveredByBody(doc.body(id)->parent)) continue;
      topBodies.push_back(id);
    } else if (k == Kind::Geom || k == Kind::Joint || k == Kind::Site || k == Kind::Camera || k == Kind::Light) {
      if (!coveredByBody(doc.ownerBody(id))) loose.push_back(id);
    }
  }
  auto newName = [&](Kind k, const std::string& old) { return doc.uniqueName(k, rename(old)); };

  // Bodies.
  std::vector<Id> copiedBodies;
  for (Id top : topBodies) {
    for (Id b : doc.subtree(top)) {
      Body nb = *doc.body(b);
      nb.id = doc.newId();
      nb.name = newName(Kind::Body, nb.name);
      if (b != top) nb.parent = r.remap.at(nb.parent);
      r.remap[b] = nb.id;
      doc.bodies.push_back(nb);
      doc.touch();
      copiedBodies.push_back(b);
    }
    r.tops.push_back(r.remap[top]);
  }
  std::unordered_set<Id> copiedSet(copiedBodies.begin(), copiedBodies.end());
  auto copyAttached = [&](auto& vec, Kind kind, auto&& include) {
    size_t n = vec.size();
    for (size_t i = 0; i < n; ++i) {
      if (!include(vec[i])) continue;
      auto e = vec[i];
      Id old = e.id;
      e.id = doc.newId();
      e.name = newName(kind, e.name);
      auto it = r.remap.find(e.body);
      if (it != r.remap.end()) e.body = it->second;
      r.remap[old] = e.id;
      vec.push_back(e);
      doc.touch();
    }
  };
  std::unordered_set<Id> looseSet(loose.begin(), loose.end());
  auto inCopy = [&](const auto& e) { return copiedSet.count(e.body) > 0 || looseSet.count(e.id) > 0; };
  copyAttached(doc.geoms, Kind::Geom, inCopy);
  copyAttached(doc.joints, Kind::Joint, inCopy);
  copyAttached(doc.sites, Kind::Site, inCopy);
  copyAttached(doc.cameras, Kind::Camera, inCopy);
  copyAttached(doc.lights, Kind::Light, inCopy);
  for (Id id : loose) r.tops.push_back(r.remap[id]);

  auto mapId = [&](Id id) {
    auto it = r.remap.find(id);
    return it == r.remap.end() ? id : it->second;
  };
  // Tendons entirely inside the copy (before actuators and sensors, which may use them).
  size_t nt = doc.tendons.size();
  for (size_t i = 0; i < nt; ++i) {
    const Tendon& t = doc.tendons[i];
    bool inside = !t.joints.empty() || !t.path.empty();
    for (const TendonJoint& j : t.joints) inside &= r.remap.count(j.joint) > 0;
    for (const TendonPoint& p : t.path) inside &= (p.obj == kNoId || r.remap.count(p.obj) > 0) && (!p.sidesite || r.remap.count(p.sidesite) > 0);
    if (!inside) continue;
    Tendon x = t;
    x.id = doc.newId();
    x.name = newName(Kind::Tendon, x.name);
    for (TendonJoint& j : x.joints) j.joint = mapId(j.joint);
    for (TendonPoint& p : x.path) {
      p.obj = mapId(p.obj);
      p.sidesite = mapId(p.sidesite);
    }
    r.remap[t.id] = x.id;
    doc.tendons.push_back(x);
    doc.touch();
  }
  // Dependents.
  size_t na = doc.actuators.size();
  for (size_t i = 0; i < na; ++i) {
    if (!r.remap.count(doc.actuators[i].target)) continue;
    Actuator a = doc.actuators[i];
    Id old = a.id;
    a.id = doc.newId();
    a.name = newName(Kind::Actuator, a.name);
    a.target = mapId(a.target);
    a.refsite = mapId(a.refsite);
    a.slidersite = mapId(a.slidersite);
    r.remap[old] = a.id;
    doc.actuators.push_back(a);
    doc.touch();
  }
  size_t ns = doc.sensors.size();
  for (size_t i = 0; i < ns; ++i) {
    if (!r.remap.count(doc.sensors[i].obj)) continue;
    Sensor s = doc.sensors[i];
    Id old = s.id;
    s.id = doc.newId();
    s.name = newName(Kind::Sensor, s.name);
    s.obj = mapId(s.obj);
    s.ref = mapId(s.ref);
    r.remap[old] = s.id;
    doc.sensors.push_back(s);
    doc.touch();
  }
  size_t ne = doc.excludes.size();
  for (size_t i = 0; i < ne; ++i) {
    const Exclude& e = doc.excludes[i];
    if (!r.remap.count(e.body1) && !r.remap.count(e.body2)) continue;
    Exclude x = e;
    x.id = doc.newId();
    x.body1 = mapId(e.body1);
    x.body2 = mapId(e.body2);
    doc.excludes.push_back(x);
    doc.touch();
  }
  size_t npair = doc.pairs.size();
  for (size_t i = 0; i < npair; ++i) {
    const ContactPair& c = doc.pairs[i];
    if (!r.remap.count(c.geom1) && !r.remap.count(c.geom2)) continue;
    ContactPair x = c;
    x.id = doc.newId();
    x.name = newName(Kind::Pair, x.name);
    x.geom1 = mapId(c.geom1);
    x.geom2 = mapId(c.geom2);
    r.remap[c.id] = x.id;
    doc.pairs.push_back(x);
    doc.touch();
  }
  size_t neq = doc.equalities.size();
  for (size_t i = 0; i < neq; ++i) {
    const Equality& e = doc.equalities[i];
    if (!r.remap.count(e.obj1) && !r.remap.count(e.obj2)) continue;
    Equality x = e;
    x.id = doc.newId();
    x.name = newName(Kind::Equality, x.name);
    x.obj1 = mapId(e.obj1);
    x.obj2 = mapId(e.obj2);
    r.remap[e.id] = x.id;
    doc.equalities.push_back(x);
    doc.touch();
  }
  for (Camera& c : doc.cameras) if (r.remap.count(c.id) && r.remap.count(c.target)) c.target = mapId(c.target);
  for (Light& l : doc.lights) if (r.remap.count(l.id) && r.remap.count(l.target)) l.target = mapId(l.target);
  // Copies start in the same keyframe poses as the originals.
  for (Keyframe& k : doc.keyframes) {
    auto copyEntries = [&](auto& v) {
      size_t n = v.size();
      for (size_t i = 0; i < n; ++i) {
        auto it = r.remap.find(v[i].first);
        if (it != r.remap.end()) v.push_back({it->second, v[i].second});
      }
    };
    copyEntries(k.qpos);
    copyEntries(k.qvel);
    copyEntries(k.ctrl);
  }
  doc.touch();
  return r;
}

}  // namespace

std::vector<Id> duplicate(Document& doc, const std::vector<Id>& ids, const Vec3& worldOffset) {
  CopyResult r = copyElements(doc, ids, [](const std::string& n) { return n; });
  for (Id top : r.tops) {
    Kind k = doc.kindOf(top);
    if (k == Kind::Body) {
      Body* b = doc.body(top);
      Pose parentWorld = bodyWorldPose(doc, b->parent);
      b->pos += rotate(conj(parentWorld.quat), worldOffset);
      // Free bodies: keyframe positions are world positions, shift them too.
      for (const Joint& j : doc.joints)
        if (j.body == top && j.type == JointType::Free)
          for (Keyframe& key : doc.keyframes)
            for (auto& [jid, q] : key.qpos)
              if (jid == j.id && q.size() == 7)
                for (int i = 0; i < 3; ++i) q[i] += worldOffset[i];
    } else {
      Id owner = doc.ownerBody(top);
      Pose ownerWorld = bodyWorldPose(doc, owner);
      Pose local = elementLocalPose(doc, top);
      local.pos += rotate(conj(ownerWorld.quat), worldOffset);
      setElementLocalPose(doc, top, local);
    }
  }
  doc.touch();
  return r.tops;
}

std::vector<Id> arrayDuplicate(Document& doc, Id element, int copies, const Pose& step) {
  std::vector<Id> out;
  Kind k = doc.kindOf(element);
  if (element == kWorldId || copies <= 0 ||
      !(k == Kind::Body || k == Kind::Geom || k == Kind::Site || k == Kind::Camera || k == Kind::Light))
    return out;
  Pose local = elementLocalPose(doc, element);
  Pose current = local;
  for (int i = 0; i < copies; ++i) {
    CopyResult r = copyElements(doc, {element}, [](const std::string& n) { return n; });
    if (r.tops.empty()) break;
    current = step * current;
    setElementLocalPose(doc, r.tops[0], current);
    out.push_back(r.tops[0]);
  }
  doc.touch();
  return out;
}

Id addLimb(Document& doc, Id parent, const Vec3& worldPoint, const LimbSpec& spec) {
  if (!doc.body(parent) || spec.segments < 1) return kNoId;
  Pose parentWorld = bodyWorldPose(doc, parent);
  // Body frame: local -Z along the limb, local +Y along the hinge axis.
  Vec3 dir = normalize(spec.direction, {0, 0, -1});
  Vec3 ax = spec.axis - dir * dot(spec.axis, dir);
  ax = normalize(ax, perpendicular(dir));
  Vec3 zc = -dir, yc = ax, xc = cross(yc, zc);
  Mat3 m;
  for (int i = 0; i < 3; ++i) {
    m(i, 0) = xc[i];
    m(i, 1) = yc[i];
    m(i, 2) = zc[i];
  }
  Pose world{worldPoint, fromMat3(m)};
  Pose local = inverse(parentWorld) * world;
  Id first = kNoId, body = parent;
  double r = spec.radius;
  auto segmentName = [&](int s) -> std::string {
    static const char* two[] = {"upper", "lower"};
    static const char* three[] = {"upper", "middle", "lower"};
    if (spec.segments == 1) return spec.name;
    if (spec.segments == 2) return spec.name + "_" + two[s];
    if (spec.segments == 3) return spec.name + "_" + three[s];
    return spec.name + "_seg" + std::to_string(s + 1);
  };
  for (int s = 0; s < spec.segments; ++s) {
    std::string bname = segmentName(s);
    Pose bpose = s == 0 ? local : Pose{{0, 0, -spec.length}, {}};
    body = addBody(doc, body, bpose, bname, doc.roleOf(parent));
    if (!first) first = body;
    static const char* jointNames[] = {"hip", "knee", "ankle", "toe"};
    std::string jname = spec.name + "_" + (s < 4 ? jointNames[s] : "joint" + std::to_string(s + 1));
    Id j = addJoint(doc, body, JointType::Hinge, {}, {0, 1, 0}, jname);
    Joint* jj = doc.joint(j);
    jj->limited = true;
    jj->range = {-spec.range, spec.range};
    jj->damping = 0.5;
    jj->armature = 0.01;
    Id g = addGeom(doc, body, GeomType::Capsule, {r, 0.1, 0}, {}, bname + "_geom");
    setGeomFromTo(*doc.geom(g), {0, 0, 0}, {0, 0, -spec.length});
    doc.geom(g)->rgba = {0.30f, 0.33f, 0.38f, 1};
    if (spec.actuators) {
      Id a = addActuator(doc, spec.actuatorType, j);  // named after the joint: <joint>_pos / _motor
      Actuator* act = doc.actuator(a);
      double torque = spec.maxTorque * std::pow(0.7, s);
      act->forcelimited = true;
      act->forcerange = {-torque, torque};
      if (spec.actuatorType == ActuatorType::Position) {
        act->kp = std::max(1.0, std::round(3.0 * torque));  // full torque at ~19 degrees of error
        act->dampratio = 1.0;
        act->ctrlrange = jj->range;
      } else if (spec.actuatorType == ActuatorType::Motor) {
        act->gear[0] = torque;
        act->forcelimited = false;
      }
    }
    r *= spec.taper;
  }
  if (spec.foot) {
    Id f = addGeom(doc, body, GeomType::Sphere, {r * 1.25, 0, 0}, {{0, 0, -spec.length}, {}}, spec.name + "_foot");
    doc.geom(f)->rgba = {0.12f, 0.12f, 0.13f, 1};
  }
  doc.touch();
  return first;
}

void translateWorld(Document& doc, Id id, const Vec3& delta) {
  Kind k = doc.kindOf(id);
  if (id == kWorldId || k == Kind::None) return;
  if (k == Kind::Body) {
    Body* b = doc.body(id);
    b->pos += rotate(conj(bodyWorldPose(doc, b->parent).quat), delta);
    // Free bodies: keyframe positions are world positions, shift them too.
    for (const Joint& j : doc.joints)
      if (j.body == id && j.type == JointType::Free)
        for (Keyframe& key : doc.keyframes)
          for (auto& [jid, q] : key.qpos)
            if (jid == j.id && q.size() == 7)
              for (int i = 0; i < 3; ++i) q[i] += delta[i];
  } else if (k == Kind::Geom || k == Kind::Site || k == Kind::Camera || k == Kind::Light) {
    Pose local = elementLocalPose(doc, id);
    local.pos += rotate(conj(bodyWorldPose(doc, doc.ownerBody(id)).quat), delta);
    setElementLocalPose(doc, id, local);
  }
  doc.touch();
}

Aabb elementWorldBounds(const Document& doc, Id id) {
  Aabb box;
  std::vector<Pose> poses = computeBodyPoses(doc);
  switch (doc.kindOf(id)) {
    case Kind::Body: box = subtreeAabb(doc, poses, id); break;
    case Kind::Geom: {
      const Geom* g = doc.geom(id);
      if (g->type != GeomType::Plane) box.add(geomWorldAabb(doc, *g, poses[doc.bodyIndex(g->body)] * Pose{g->pos, g->quat}));
      break;
    }
    default: break;
  }
  if (!box.valid() && doc.exists(id)) box.add(elementWorldPose(doc, poses, id).pos);
  return box;
}

void alignElements(Document& doc, const std::vector<Id>& ids, int axis, int side) {
  if (ids.size() < 2 || axis < 0 || axis > 2) return;
  std::vector<Aabb> boxes;
  for (Id id : ids) boxes.push_back(elementWorldBounds(doc, id));
  double target = side < 0 ? 1e300 : side > 0 ? -1e300 : 0;
  for (const Aabb& b : boxes) {
    if (side < 0) target = std::min(target, b.lo[axis]);
    else if (side > 0) target = std::max(target, b.hi[axis]);
    else target += b.center()[axis] / double(boxes.size());
  }
  for (size_t i = 0; i < ids.size(); ++i) {
    double cur = side < 0 ? boxes[i].lo[axis] : side > 0 ? boxes[i].hi[axis] : boxes[i].center()[axis];
    Vec3 d;
    d[axis] = target - cur;
    if (std::fabs(d[axis]) > 1e-12) translateWorld(doc, ids[i], d);
  }
}

void distributeElements(Document& doc, const std::vector<Id>& ids, int axis) {
  if (ids.size() < 3 || axis < 0 || axis > 2) return;
  std::vector<std::pair<double, Id>> order;
  for (Id id : ids) order.push_back({elementWorldBounds(doc, id).center()[axis], id});
  std::sort(order.begin(), order.end());
  double a = order.front().first, b = order.back().first;
  for (size_t i = 1; i + 1 < order.size(); ++i) {
    Vec3 d;
    d[axis] = a + (b - a) * double(i) / double(order.size() - 1) - order[i].first;
    if (std::fabs(d[axis]) > 1e-12) translateWorld(doc, order[i].second, d);
  }
}

Keyframe& startKeyframe(Document& doc) {
  if (doc.keyframes.empty()) {
    Keyframe k;
    k.name = "home";
    doc.keyframes.push_back(k);
    doc.touch();
  }
  return doc.keyframes[0];
}

double keyJointValue(const Keyframe* key, const Joint& joint) {
  const std::vector<double>* q = key ? key->jointQpos(joint.id) : nullptr;
  return q && q->size() == 1 ? (*q)[0] : joint.ref;
}

void setKeyJointQpos(Keyframe& key, Id joint, std::vector<double> qpos) {
  for (auto& e : key.qpos)
    if (e.first == joint) {
      e.second = std::move(qpos);
      return;
    }
  key.qpos.emplace_back(joint, std::move(qpos));
}

void syncKeyframeControls(const Document& doc, Keyframe& key) {
  for (const Actuator& a : doc.actuators) {
    if (a.type != ActuatorType::Position || a.trn != TrnType::Joint) continue;
    const Joint* j = doc.joint(a.target);
    if (!j || (j->type != JointType::Hinge && j->type != JointType::Slide)) continue;
    double c = a.gear[0] * keyJointValue(&key, *j);
    if (a.ctrllimited && a.ctrlrange.y > a.ctrlrange.x) c = std::clamp(c, a.ctrlrange.x, a.ctrlrange.y);
    bool found = false;
    for (auto& e : key.ctrl)
      if (e.first == a.id) {
        e.second = c;
        found = true;
      }
    if (!found) key.ctrl.emplace_back(a.id, c);
  }
}

void resetKeyPose(const Document& doc, Keyframe& key, Id root) {
  std::vector<Id> bodies = doc.subtree(root);
  std::unordered_set<Id> in(bodies.begin(), bodies.end());
  key.qpos.erase(std::remove_if(key.qpos.begin(), key.qpos.end(),
                                [&](const auto& e) {
                                  const Joint* j = doc.joint(e.first);
                                  return j && j->type != JointType::Free && in.count(j->body);
                                }),
                 key.qpos.end());
  syncKeyframeControls(doc, key);
}

int mirrorKeyPose(const Document& doc, Keyframe& key, Id root, bool leftToRight) {
  std::vector<Id> bodies = doc.subtree(root);
  std::unordered_set<Id> in(bodies.begin(), bodies.end());
  std::unordered_map<std::string, const Joint*> byName;
  for (const Joint& j : doc.joints)
    if (in.count(j.body) && !j.name.empty()) byName[j.name] = &j;
  // The side comes from where the joint sits in the model's frame (y > 0 is left in MuJoCo's
  // convention), so any naming scheme works as long as mirrorName pairs the names.
  Pose rootInv = inverse(bodyWorldPose(doc, root));
  auto side = [&](const Joint& j) { return rootInv.apply(bodyWorldPose(doc, j.body).apply(j.pos)).y; };
  int n = 0;
  for (const auto& [name, j] : byName) {
    double y = side(*j);
    if (j->type == JointType::Free || (leftToRight ? y < 1e-6 : y > -1e-6)) continue;
    bool changed = false;
    std::string other = mirrorName(name, &changed);
    auto it = byName.find(other);
    if (!changed || it == byName.end() || it->second->type != j->type) continue;
    if (leftToRight ? side(*it->second) > -1e-6 : side(*it->second) < 1e-6) continue;  // not across
    const Joint& t = *it->second;
    const std::vector<double>* q = key.jointQpos(j->id);
    if (j->type == JointType::Hinge || j->type == JointType::Slide) {
      // Mirror-symmetric motion: hinge axes reflect as pseudo-vectors, slide axes as vectors. If
      // the other joint's axis points the opposite way, the value changes sign.
      auto axisIn = [&](const Joint& x) { return rotate(rootInv.quat, rotate(bodyWorldPose(doc, x.body).quat, normalize(x.axis))); };
      Vec3 a = axisIn(*j), b = axisIn(t);
      Vec3 mirrored = j->type == JointType::Hinge ? Vec3{-a.x, a.y, -a.z} : Vec3{a.x, -a.y, a.z};
      double sign = dot(mirrored, b) < -0.5 ? -1.0 : 1.0;
      double v = t.ref + sign * (keyJointValue(&key, *j) - j->ref);
      if (t.limited && t.range.y > t.range.x) v = std::clamp(v, t.range.x, t.range.y);
      setKeyJointQpos(key, t.id, {v});
    } else if (q) {
      setKeyJointQpos(key, t.id, *q);
    } else {
      key.qpos.erase(std::remove_if(key.qpos.begin(), key.qpos.end(), [&](const auto& e) { return e.first == t.id; }), key.qpos.end());
    }
    ++n;
  }
  syncKeyframeControls(doc, key);
  return n;
}

void liftStartPose(Document& doc, Id root, double dz) {
  if (!doc.keyframes.empty())
    for (const Joint& j : doc.joints)
      if (j.body == root && j.type == JointType::Free)
        for (auto& [jid, q] : doc.keyframes[0].qpos)
          if (jid == j.id && q.size() == 7) {
            q[2] += dz;
            doc.touch();
            return;
          }
  if (Body* b = doc.body(root)) b->pos.z += dz;
  doc.touch();
}

void placeOnFloor(Document& doc, Id root, double floorZ, double clearance) {
  if (root == kWorldId || !doc.body(root)) return;
  auto poses = computeBodyPoses(doc, doc.keyframes.empty() ? nullptr : &doc.keyframes[0]);
  // Visual-only geoms never touch the ground; use them only when nothing collides.
  bool colliding = subtreeAabb(doc, poses, root, true).valid();
  double minZ = subtreeMinZ(doc, poses, root, colliding);
  if (std::isfinite(minZ)) liftStartPose(doc, root, floorZ + clearance - minZ);
}

std::vector<Id> lowestGeoms(const Document& doc, Id root, double tolerance) {
  std::vector<Id> out;
  auto poses = computeBodyPoses(doc);
  // Visual-only geoms never touch the ground; ignore them unless nothing collides.
  bool colliding = subtreeAabb(doc, poses, root, true).valid();
  Aabb box = subtreeAabb(doc, poses, root, colliding);
  if (!box.valid()) return out;
  if (tolerance < 0) tolerance = std::max(0.004, 0.02 * (box.hi.z - box.lo.z));
  for (Id b : doc.subtree(root)) {
    int bi = doc.bodyIndex(b);
    for (uint32_t gi : doc.index().bodyGeoms[bi]) {
      const Geom& g = doc.geoms[gi];
      if (g.type == GeomType::Plane || (colliding && !geomCollides(g))) continue;
      Aabb gb = geomWorldAabb(doc, g, poses[bi] * Pose{g.pos, g.quat});
      if (gb.lo.z <= box.lo.z + tolerance) out.push_back(g.id);
    }
  }
  return out;
}

int applyCollisionPreset(Document& doc, Id root, CollisionPreset preset) {
  std::vector<Id> feet = preset == CollisionPreset::FeetOnly ? lowestGeoms(doc, root) : std::vector<Id>{};
  int changed = 0;
  for (Id b : doc.subtree(root)) {
    int bi = doc.bodyIndex(b);
    for (uint32_t gi : doc.index().bodyGeoms[bi]) {
      Geom& g = doc.geoms[gi];
      int ct = 1, ca = 1;
      switch (preset) {
        case CollisionPreset::AllCollide: ct = 1; ca = 1; break;
        case CollisionPreset::NoSelfCollision: ct = 2; ca = 1; break;
        case CollisionPreset::FeetOnly: {
          bool foot = std::find(feet.begin(), feet.end(), g.id) != feet.end();
          ct = foot ? 2 : 0;
          ca = foot ? 1 : 0;
          break;
        }
      }
      if (g.contype != ct || g.conaffinity != ca) {
        g.contype = ct;
        g.conaffinity = ca;
        ++changed;
      }
    }
  }
  return changed;
}

std::string mirrorName(const std::string& name, bool* changed) {
  std::string out = name;
  bool did = false;
  // Whole words left/right in any case style.
  struct Pair { const char* a; const char* b; };
  const Pair words[] = {{"left", "right"}, {"Left", "Right"}, {"LEFT", "RIGHT"}};
  {
    std::string res;
    size_t i = 0;
    while (i < out.size()) {
      bool replaced = false;
      for (const auto& p : words) {
        for (int dirn = 0; dirn < 2 && !replaced; ++dirn) {
          const char* from = dirn ? p.b : p.a;
          const char* to = dirn ? p.a : p.b;
          size_t len = std::char_traits<char>::length(from);
          // Only at word boundaries: "left_leg", "leftLeg", "LeftLeg", "leg_LEFT" but not "upright".
          auto alpha = [](char c) { return std::isalpha(static_cast<unsigned char>(c)) != 0; };
          auto lower = [](char c) { return std::islower(static_cast<unsigned char>(c)) != 0; };
          bool upperStart = !lower(from[0]);
          bool startOk = i == 0 || !alpha(out[i - 1]) || (upperStart && lower(out[i - 1]));
          size_t e = i + len;
          bool endOk = e >= out.size() || !alpha(out[e]) || (lower(from[len - 1]) && !lower(out[e]));
          if (startOk && endOk && out.compare(i, len, from) == 0) {
            res += to;
            i += len;
            replaced = did = true;
          }
        }
        if (replaced) break;
      }
      if (!replaced) res += out[i++];
    }
    out = res;
  }
  if (!did) {
    // Token rules: l <-> r, FL <-> FR, RL <-> RR, LF <-> RF ...
    auto isSep = [](char c) { return c == '_' || c == '-' || c == '.' || c == ' '; };
    auto swapLR = [](char c) {
      switch (c) {
        case 'l': return 'r';
        case 'r': return 'l';
        case 'L': return 'R';
        case 'R': return 'L';
        default: return c;
      }
    };
    auto isLR = [](char c) { return c == 'l' || c == 'r' || c == 'L' || c == 'R'; };
    auto isPos = [](char c) {
      char u = char(std::toupper(static_cast<unsigned char>(c)));
      return u == 'F' || u == 'R' || u == 'H' || u == 'B' || u == 'M';
    };
    size_t i = 0;
    while (i < out.size()) {
      size_t j = i;
      while (j < out.size() && !isSep(out[j])) ++j;
      size_t len = j - i;
      if (len == 1 && isLR(out[i])) {
        out[i] = swapLR(out[i]);
        did = true;
      } else if (len == 2) {
        if (isPos(out[i]) && isLR(out[i + 1])) {
          out[i + 1] = swapLR(out[i + 1]);
          did = true;
        } else if (isLR(out[i]) && isPos(out[i + 1])) {
          out[i] = swapLR(out[i]);
          did = true;
        }
      }
      i = j + 1;
    }
  }
  if (changed) *changed = did;
  return out;
}

Id mirrorSubtree(Document& doc, Id body, int axis) {
  if (body == kWorldId || !doc.body(body) || axis < 0 || axis > 2) return kNoId;
  CopyResult r = copyElements(doc, {body}, [](const std::string& n) { return mirrorName(n); });
  if (r.tops.empty()) return kNoId;
  auto M = [axis](Vec3 v) { v[axis] = -v[axis]; return v; };
  // Conjugation of a rotation by the reflection: axis transforms as a pseudo-vector.
  auto Mq = [axis](const Quat& q) {
    Vec3 v{q.x, q.y, q.z};
    v[axis] = -v[axis];
    v = -v;
    return canonical(Quat{q.w, v.x, v.y, v.z});
  };
  std::unordered_set<Id> newIds;
  for (auto& kv : r.remap) newIds.insert(kv.second);
  for (Body& b : doc.bodies) {
    if (!newIds.count(b.id)) continue;
    b.pos = M(b.pos);
    b.quat = Mq(b.quat);
    b.ipos = M(b.ipos);
    b.iquat = Mq(b.iquat);
  }
  for (Geom& g : doc.geoms) if (newIds.count(g.id)) { g.pos = M(g.pos); g.quat = Mq(g.quat); }
  for (Site& s : doc.sites) if (newIds.count(s.id)) { s.pos = M(s.pos); s.quat = Mq(s.quat); }
  for (Camera& c : doc.cameras) if (newIds.count(c.id)) { c.pos = M(c.pos); c.quat = Mq(c.quat); }
  for (Light& l : doc.lights) if (newIds.count(l.id)) { l.pos = M(l.pos); l.dir = M(l.dir); }
  for (Joint& j : doc.joints) {
    if (!newIds.count(j.id)) continue;
    j.pos = M(j.pos);
    if (j.type == JointType::Hinge) j.axis = -M(j.axis);  // pseudo-vector: symmetric motion
    else if (j.type == JointType::Slide) j.axis = M(j.axis);
  }
  // Anchors and relative poses are expressed in (mirrored) body frames.
  for (Equality& e : doc.equalities) {
    if (!newIds.count(e.id)) continue;
    e.anchor = M(e.anchor);
    e.relpos = M(e.relpos);
    e.relquat = Mq(e.relquat);
  }
  doc.touch();
  return r.tops.front();
}

}  // namespace mjf
