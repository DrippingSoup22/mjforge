#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

#include <imgui.h>
#include <imgui_internal.h>
#include <ImGuizmo.h>

#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Ops.h"
#include "core/Templates.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace mjf {

namespace {

Shape shapeOf(GeomType t) {
  switch (t) {
    case GeomType::Sphere: return Shape::Sphere;
    case GeomType::Capsule: return Shape::Capsule;
    case GeomType::Cylinder: return Shape::Cylinder;
    case GeomType::Box: return Shape::Box;
    case GeomType::Ellipsoid: return Shape::Ellipsoid;
    case GeomType::Plane: return Shape::Plane;
    default: return Shape::Box;
  }
}

Color mul(const Color& a, const Vec3& b) { return {float(a.r * b.x), float(a.g * b.y), float(a.b * b.z), a.a}; }

// Applies MuJoCo's material rules to a draw item.
void applyAppearance(const Document& doc, DrawItem& it, const Color& rgba, Id materialId, const Vec3& size, bool isPlane) {
  const Color def{0.5f, 0.5f, 0.5f, 1.0f};
  it.color = rgba;
  const Material* m = doc.material(materialId);
  if (!m) return;
  it.specular = float(m->specular);
  it.shininess = float(m->shininess);
  it.emission = float(m->emission);
  bool geomOverrides = rgba != def && rgba != Color{1, 1, 1, 1};
  if (!geomOverrides) it.color = m->rgba;
  const Texture* t = doc.texture(m->texture);
  if (t && t->type == TextureType::TwoD && t->builtin == TextureBuiltin::Checker && isPlane) {
    it.checker = true;
    it.color = mul(it.color, t->rgb1);
    it.checker2 = mul(m->rgba, t->rgb2);
    double rep = m->texrepeat.x;
    if (m->texuniform) it.checkerDensity = float(2.0 * rep);
    else it.checkerDensity = float(2.0 * rep / std::max(1e-6, 2.0 * (size.x > 0 ? size.x : 10.0)));
    it.checkerEdges = t->mark == TextureMark::Edge;
    it.edgeColor = {float(t->markrgb.x), float(t->markrgb.y), float(t->markrgb.z), 1};
  } else if (t && t->builtin != TextureBuiltin::None && t->type == TextureType::TwoD) {
    it.color = mul(it.color, (t->rgb1 + t->rgb2) * 0.5);
  }
}

// Joint overlay size derived from the size of the body it belongs to.
double bodyScale(const Document& doc, const std::vector<Pose>& poses, Id body) {
  int bi = doc.bodyIndex(body);
  if (bi < 0) return 0.1;
  Aabb box;
  for (uint32_t gi : doc.index().bodyGeoms[bi]) {
    const Geom& g = doc.geoms[gi];
    if (g.type == GeomType::Plane) continue;
    box.add(primitiveWorldAabb(g.type, g.size, poses[bi] * Pose{g.pos, g.quat}));
  }
  if (!box.valid()) return 0.1;
  return clampv(length(box.extent()) * 0.5, 0.03, 1.0);
}

}  // namespace

// ------------------------------------------------------------------------------------ scene

void Editor::buildScene(std::vector<DrawItem>& items) {
  poses_ = computeBodyPoses(doc_);
  if (sim_.active()) applySimPoses();
  const DocIndex& ix = doc_.index();
  std::unordered_set<Id> selected(sel_.ids.begin(), sel_.ids.end());
  Id primary = sel_.primary();
  // Hidden state is inherited down the tree.
  std::vector<uint8_t> hidden(doc_.bodies.size(), 0);
  for (uint32_t bi : ix.dfsOrder) {
    const Body& b = doc_.bodies[bi];
    int pi = doc_.bodyIndex(b.parent);
    hidden[bi] = b.hidden || (pi >= 0 && hidden[pi]);
  }
  auto highlightFor = [&](Id id, Id body) -> uint8_t {
    if (selected.count(id)) return id == primary ? 3 : 2;
    if (body != kWorldId && selected.count(body)) return body == primary ? 3 : 2;
    if (vp_.hoverId == id || (pickMode_ == PickMode::Body && body != kWorldId && vp_.hoverId == body)) return 1;
    return 0;
  };
  for (uint32_t bi = 0; bi < doc_.bodies.size(); ++bi) {
    if (hidden[bi]) continue;
    const Body& b = doc_.bodies[bi];
    for (uint32_t gi : ix.bodyGeoms[bi]) {
      const Geom& g = doc_.geoms[gi];
      if (g.hidden || !vp_.groupShown(g.group)) continue;
      DrawItem it;
      it.pose = poses_[bi] * Pose{g.pos, g.quat};
      it.size = g.size;
      if (g.type == GeomType::Hfield) {
        const Hfield* hf = doc_.hfield(g.hfield);
        if (!hf || !hf->data) continue;
        static std::unordered_map<Id, std::pair<const void*, Mesh>> cache;
        auto& entry = cache[hf->id];
        if (entry.first != hf->data.get()) {
          buildHfieldMesh(entry.second, hf->nrow, hf->ncol, *hf->data, hf->size.w / std::max(1e-9, hf->size.z));
          entry.first = hf->data.get();
        }
        it.shape = Shape::Custom;
        it.mesh = &entry.second;
        it.size = {hf->size.x, hf->size.y, hf->size.z};
      } else {
        it.shape = shapeOf(g.type);
      }
      applyAppearance(doc_, it, g.rgba, g.material, g.size, g.type == GeomType::Plane);
      it.highlight = highlightFor(g.id, b.id);
      items.push_back(it);
    }
    if (vp_.showSites) {
      for (uint32_t si : ix.bodySites[bi]) {
        const Site& s = doc_.sites[si];
        if (s.hidden || !vp_.groupShown(s.group)) continue;
        DrawItem it;
        it.shape = shapeOf(s.type);
        it.pose = poses_[bi] * Pose{s.pos, s.quat};
        it.size = s.size;
        it.color = s.rgba;
        it.color.a = std::min(it.color.a, 0.55f);
        it.castShadow = false;
        it.highlight = selected.count(s.id) ? 3 : (vp_.hoverId == s.id ? 1 : 0);
        items.push_back(it);
      }
    }
  }
  // Light / COM markers as small geometry.
  if (vp_.showLights) {
    for (const Light& l : doc_.lights) {
      if (l.hidden) continue;
      DrawItem it;
      it.shape = Shape::Sphere;
      it.pose = elementWorldPose(doc_, poses_, l.id);
      it.size = {0.045, 0, 0};
      it.color = {1.0f, 0.93f, 0.55f, 1};
      it.emission = 0.9f;
      it.castShadow = false;
      it.highlight = selected.count(l.id) ? 3 : (vp_.hoverId == l.id ? 1 : 0);
      items.push_back(it);
    }
  }
}

void Editor::buildOverlays() {
  dbg_.clear();
  const DocIndex& ix = doc_.index();
  std::unordered_set<Id> selected(sel_.ids.begin(), sel_.ids.end());
  std::unordered_set<Id> actuated;
  for (const Actuator& a : doc_.actuators)
    if (a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) actuated.insert(a.target);

  // Joints.
  for (const Joint& j : doc_.joints) {
    const Body* b = doc_.body(j.body);
    if (!b || j.hidden || b->hidden) continue;
    bool sel = selected.count(j.id) > 0 || selected.count(j.body) > 0;
    bool hov = vp_.hoverId == j.id;
    if (!vp_.showJoints && !sel) continue;
    Vec3 anchor, axis;
    jointWorldFrame(doc_, poses_, j, &anchor, &axis);
    double L = bodyScale(doc_, poses_, j.body) * 0.9;
    Color c = actuated.count(j.id) ? Color{1.0f, 0.55f, 0.18f, 1} : Color{0.55f, 0.72f, 0.95f, 1};
    if (sel) c = {1.0f, 0.85f, 0.35f, 1};
    if (hov) c = {0.8f, 0.92f, 1.0f, 1};
    DebugDraw::Layer layer = sel || hov ? DebugDraw::Overlay : DebugDraw::World;
    float w = sel ? 3.0f : 2.0f;
    switch (j.type) {
      case JointType::Hinge: {
        dbg_.arrow(anchor - axis * (L * 0.5), anchor + axis * L, c, w, layer);
        // Reference direction: towards the body's geometry (or any perpendicular).
        int bi = doc_.bodyIndex(j.body);
        Vec3 ref = perpendicular(axis);
        Aabb box;
        for (uint32_t gi : ix.bodyGeoms[bi]) {
          const Geom& g = doc_.geoms[gi];
          box.add(primitiveWorldAabb(g.type, g.size, poses_[bi] * Pose{g.pos, g.quat}));
        }
        if (box.valid()) {
          Vec3 d = box.center() - anchor;
          d -= axis * dot(d, axis);
          if (length(d) > 1e-6) ref = normalize(d);
        }
        double r = L * 0.6;
        // While simulating, the range stays attached to the parent and the needle shows the angle.
        Vec3 zero = ref;
        double q = simJointOffset(j.id);
        if (q != 0) zero = rotate(axisAngle(axis, -q), ref);
        if (j.limited && j.range.y > j.range.x) {
          Color fill = c;
          fill.a = sel ? 0.35f : 0.18f;
          dbg_.sector(anchor, axis, zero, r, j.range.x, j.range.y, fill, layer);
          dbg_.arc(anchor, axis, zero, r, j.range.x, j.range.y, c, w * 0.8f, layer);
        } else {
          Color cc = c;
          cc.a = 0.5f;
          dbg_.circle(anchor, axis, r, cc, w * 0.7f, layer);
        }
        dbg_.line(anchor, anchor + ref * r, c, w * 0.8f, layer);  // current direction (q = 0 when editing)
        break;
      }
      case JointType::Slide: {
        dbg_.arrow(anchor - axis * (L * 0.6), anchor + axis * L, c, w, layer);
        dbg_.arrow(anchor + axis * (L * 0.4), anchor - axis * (L * 0.8), c, w, layer);
        if (j.limited) {
          Vec3 u = perpendicular(axis) * (L * 0.15);
          Vec3 zero = anchor - axis * simJointOffset(j.id);
          for (double v : {j.range.x, j.range.y}) dbg_.line(zero + axis * v - u, zero + axis * v + u, c, w + 1, layer);
        }
        break;
      }
      case JointType::Ball: {
        double r = L * 0.35;
        dbg_.circle(anchor, {1, 0, 0}, r, c, w, layer);
        dbg_.circle(anchor, {0, 1, 0}, r, c, w, layer);
        dbg_.circle(anchor, {0, 0, 1}, r, c, w, layer);
        break;
      }
      case JointType::Free:
        if (sel) dbg_.axes(elementWorldPose(doc_, poses_, j.body), L * 0.6, 2.5f, DebugDraw::Overlay);
        break;
    }
  }

  // Body frames.
  for (uint32_t bi = 1; bi < doc_.bodies.size(); ++bi) {
    const Body& b = doc_.bodies[bi];
    if (b.hidden) continue;
    bool sel = selected.count(b.id) > 0;
    if (vp_.showFrames || sel) dbg_.axes(poses_[bi], bodyScale(doc_, poses_, b.id) * 0.7, sel ? 2.5f : 1.5f,
                                         sel ? DebugDraw::Overlay : DebugDraw::World, sel ? 1.0f : 0.7f);
  }

  // Centers of mass and inertia boxes.
  if (vp_.showCom || vp_.showInertia) {
    MassAccumulator total;
    for (uint32_t bi = 1; bi < doc_.bodies.size(); ++bi) {
      const Body& b = doc_.bodies[bi];
      MassProps mp = bodyMassProps(doc_, poses_, b.id);
      if (mp.mass <= 0) continue;
      if (doc_.roleOf(b.id) == Role::Model) total.add(mp.mass, mp.com, mp.inertia);
      if (vp_.showCom) {
        double s = clampv(std::cbrt(mp.mass) * 0.025, 0.01, 0.12);
        dbg_.cross3(mp.com, s, {1, 1, 1, 0.95f}, 2.0f, DebugDraw::Overlay);
      }
      if (vp_.showInertia) {
        Vec3 evals;
        Mat3 evecs;
        eigSym3(mp.inertia, &evals, &evecs);
        // Equivalent box: I1 = m/3 (b^2 + c^2) ...
        double m3 = 3.0 / (2.0 * mp.mass);
        Vec3 h{std::sqrt(std::max(0.0, m3 * (evals.y + evals.z - evals.x))),
               std::sqrt(std::max(0.0, m3 * (evals.x + evals.z - evals.y))),
               std::sqrt(std::max(0.0, m3 * (evals.x + evals.y - evals.z)))};
        Pose p{mp.com, fromMat3(evecs)};
        dbg_.box(p, h, {0.5f, 0.9f, 0.9f, 0.85f}, 1.5f, DebugDraw::World);
      }
    }
    if (vp_.showCom && total.mass() > 0) {
      Vec3 c = total.com();
      dbg_.cross3(c, 0.06, {1.0f, 0.55f, 0.2f, 1}, 3.0f, DebugDraw::Overlay);
      dbg_.circle(c, vp_.cam.forward(), 0.05, {1.0f, 0.55f, 0.2f, 1}, 2.5f, DebugDraw::Overlay);
      // Projection on the floor: shows whether the COM is above the support polygon.
      dbg_.line(c, {c.x, c.y, 0}, {1.0f, 0.55f, 0.2f, 0.6f}, 1.5f, DebugDraw::World);
      dbg_.circle({c.x, c.y, 0.001}, {0, 0, 1}, 0.04, {1.0f, 0.55f, 0.2f, 0.9f}, 2.0f, DebugDraw::World);
    }
  }

  // Lights and cameras.
  if (vp_.showLights) {
    for (const Light& l : doc_.lights) {
      if (l.hidden) continue;
      Pose p = elementWorldPose(doc_, poses_, l.id);
      Vec3 d = normalize(rotate(p.quat, {0, 0, -1}));
      bool sel = selected.count(l.id) > 0;
      dbg_.arrow(p.pos, p.pos + d * 0.35, {1.0f, 0.92f, 0.5f, sel ? 1.0f : 0.7f}, sel ? 3.0f : 2.0f,
                 sel ? DebugDraw::Overlay : DebugDraw::World);
    }
  }
  if (vp_.showCameras) {
    for (const Camera& c : doc_.cameras) {
      if (c.hidden) continue;
      Pose p = elementWorldPose(doc_, poses_, c.id);
      bool sel = selected.count(c.id) > 0;
      Color col = sel ? Color{1.0f, 0.85f, 0.35f, 1} : Color{0.7f, 0.6f, 0.95f, 0.9f};
      double d = 0.25, t = std::tan(deg2rad(c.fovy) * 0.5) * d;
      double a = t * 4.0 / 3.0;
      Vec3 cs[4] = {p.apply({-a, -t, -d}), p.apply({a, -t, -d}), p.apply({a, t, -d}), p.apply({-a, t, -d})};
      DebugDraw::Layer layer = sel ? DebugDraw::Overlay : DebugDraw::World;
      for (int i = 0; i < 4; ++i) {
        dbg_.line(p.pos, cs[i], col, 1.5f, layer);
        dbg_.line(cs[i], cs[(i + 1) % 4], col, 1.5f, layer);
      }
      dbg_.triangle(p.apply({-a * 0.4, t * 1.1, -d}), p.apply({a * 0.4, t * 1.1, -d}), p.apply({0, t * 1.5, -d}), col, layer);
    }
  }

  if (sim_.active()) {
    buildSimOverlays();
    return;
  }
  // Penetrating contacts at the reference pose (from the last MuJoCo compile).
  if (vp_.showContacts && compileValid()) {
    for (const InitialContact& c : compiled_.initialContacts) {
      if (c.dist > -1e-4) continue;
      dbg_.cross3(c.pos, 0.03, {1.0f, 0.25f, 0.25f, 1}, 3.0f, DebugDraw::Overlay);
    }
  }
}

// ------------------------------------------------------------------------------------ picking

Id Editor::pick(const Ray& ray, double* tHit) {
  double best = 1e300;
  Id bestId = kNoId;
  const DocIndex& ix = doc_.index();
  ImVec2 mouse = ImGui::GetIO().MousePos;
  // Small handles first (screen-space tolerance): joints and sites.
  Mat4 vpm = vp_.cam.proj(vp_.w / std::max(1.0f, vp_.h)) * vp_.cam.view();
  auto screenDist = [&](const Vec3& p, double* depth) {
    Vec3 ndc = vpm.transformPoint(p);
    Vec3 eyeDir = p - vp_.cam.eye();
    if (dot(eyeDir, vp_.cam.forward()) <= 0) return 1e9;
    double sx = vp_.x + (ndc.x * 0.5 + 0.5) * vp_.w, sy = vp_.y + (1.0 - (ndc.y * 0.5 + 0.5)) * vp_.h;
    if (depth) *depth = length(eyeDir);
    return std::hypot(sx - mouse.x, sy - mouse.y);
  };
  const double tol = 9.0 * scale_;
  double bestScreen = tol;
  if (vp_.showJoints && !sim_.active()) {
    for (const Joint& j : doc_.joints) {
      const Body* b = doc_.body(j.body);
      if (!b || j.hidden || b->hidden) continue;
      Vec3 a;
      jointWorldFrame(doc_, poses_, j, &a, nullptr);
      double depth = 0;
      double d = screenDist(a, &depth);
      if (d < bestScreen) {
        bestScreen = d;
        bestId = j.id;
        best = depth;
      }
    }
  }
  if (bestId) {
    if (tHit) *tHit = best;
    return bestId;
  }
  for (uint32_t bi = 0; bi < doc_.bodies.size(); ++bi) {
    const Body& b = doc_.bodies[bi];
    if (b.hidden) continue;
    for (uint32_t gi : ix.bodyGeoms[bi]) {
      const Geom& g = doc_.geoms[gi];
      if (g.hidden || g.locked || !vp_.groupShown(g.group)) continue;
      Pose p = poses_[bi] * Pose{g.pos, g.quat};
      double t;
      if (g.type == GeomType::Hfield) {
        const Hfield* hf = doc_.hfield(g.hfield);
        if (!hf) continue;
        t = rayPrimitive(ray, GeomType::Box, {hf->size.x, hf->size.y, hf->size.z * 0.5},
                         p * Pose{{0, 0, hf->size.z * 0.5}, {}});
      } else {
        t = rayPrimitive(ray, g.type, g.size, p, std::max(60.0, vp_.cam.distance * 30));
      }
      if (t >= 0 && t < best) {
        best = t;
        bool alt = ImGui::GetIO().KeyAlt;
        bool bodyMode = (pickMode_ == PickMode::Body) != alt;
        bestId = (bodyMode && b.id != kWorldId && !b.locked) ? b.id : g.id;
      }
    }
    if (vp_.showSites) {
      for (uint32_t si : ix.bodySites[bi]) {
        const Site& s = doc_.sites[si];
        if (s.hidden || !vp_.groupShown(s.group)) continue;
        double t = rayPrimitive(ray, s.type, s.size, poses_[bi] * Pose{s.pos, s.quat});
        if (t >= 0 && t <= best + 1e-6) {
          best = t;
          bestId = s.id;
        }
      }
    }
  }
  if (vp_.showLights) {
    for (const Light& l : doc_.lights) {
      double t = rayPrimitive(ray, GeomType::Sphere, {0.06, 0, 0}, elementWorldPose(doc_, poses_, l.id));
      if (t >= 0 && t < best) { best = t; bestId = l.id; }
    }
  }
  if (vp_.showCameras) {
    for (const Camera& c : doc_.cameras) {
      double t = rayPrimitive(ray, GeomType::Sphere, {0.08, 0, 0}, elementWorldPose(doc_, poses_, c.id));
      if (t >= 0 && t < best) { best = t; bestId = c.id; }
    }
  }
  if (tHit) *tHit = best;
  return bestId;
}

// ------------------------------------------------------------------------------------ input

void Editor::viewportInput() {
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 m = io.MousePos;
  bool inside = m.x >= vp_.x && m.y >= vp_.y && m.x < vp_.x + vp_.w && m.y < vp_.y + vp_.h;
  bool overGizmo = ImGuizmo::IsOver() || ImGuizmo::IsUsing();
  Ray ray = vp_.cam.ray(m.x - vp_.x, m.y - vp_.y, vp_.w, vp_.h);

  // Hover.
  if (vp_.hovered && vp_.dragButton < 0 && !overGizmo) vp_.hoverId = pick(ray);
  else if (!vp_.hovered) vp_.hoverId = kNoId;

  // Zoom towards the point under the cursor.
  if (vp_.hovered && io.MouseWheel != 0) {
    double f = std::pow(0.88, io.MouseWheel);
    double t = 0;
    Id hit = pick(ray, &t);
    Vec3 anchor;
    if (hit && t < 1e299) anchor = ray.at(t);
    else if (std::fabs(ray.dir.z) > 1e-6 && -ray.origin.z / ray.dir.z > 0) anchor = ray.at(-ray.origin.z / ray.dir.z);
    else anchor = vp_.cam.target;
    if (length(anchor - vp_.cam.eye()) > vp_.cam.distance * 40) anchor = vp_.cam.target;
    vp_.cam.target = anchor + (vp_.cam.target - anchor) * f;
    vp_.cam.zoom(f);
  }

  // Start drags.
  if (vp_.hovered && vp_.dragButton < 0) {
    for (int b = 0; b < 3; ++b) {
      if (ImGui::IsMouseClicked(b)) {
        if (b == 0 && overGizmo) break;
        vp_.dragButton = b;
        vp_.dragMoved = false;
        vp_.pressX = m.x;
        vp_.pressY = m.y;
        if (b == 0 && sim_.active()) simMouseDown(ray);  // grab a moving body to push it
        break;
      }
    }
  }
  if (vp_.dragButton >= 0) {
    int b = vp_.dragButton;
    if (std::hypot(m.x - vp_.pressX, m.y - vp_.pressY) > 3.0f * scale_) vp_.dragMoved = true;
    if (ImGui::IsMouseDown(b)) {
      if (b == 0 && simDrag_.active) simMouseDrag(ray);
      ImVec2 d = io.MouseDelta;
      bool pan = b == 2 || (b == 1 && io.KeyShift);
      if (b == 1 && !pan) vp_.cam.orbit(-d.x * 0.35, d.y * 0.35);
      if (pan) vp_.cam.pan(d.x, d.y, vp_.h);
    } else {
      // Released.
      if (simDrag_.active) {
        sim_.clearPerturb();
        simDrag_.active = false;
      }
      if (b == 0 && !vp_.dragMoved && inside) {
        Id hit = pick(ray);
        if (hit) select(hit, io.KeyShift, io.KeyCtrl);
        else if (!io.KeyShift && !io.KeyCtrl) sel_.clear();
        if (hit && ImGui::IsMouseDoubleClicked(0)) focusSelection();
      }
      vp_.dragButton = -1;
    }
  }
}

// ------------------------------------------------------------------------------------ gizmo

static Vec3 gizmoScaleOf(const Geom& g) {
  switch (g.type) {
    case GeomType::Sphere: return {g.size.x, g.size.x, g.size.x};
    case GeomType::Capsule:
    case GeomType::Cylinder: return {g.size.x, g.size.x, g.size.y};
    case GeomType::Plane: return {g.size.x > 0 ? g.size.x : 1, g.size.y > 0 ? g.size.y : 1, 1};
    default: return g.size;
  }
}

void Editor::drawGizmo() {
  vp_.gizmoUsing = false;
  Id id = sel_.primary();
  Kind k = doc_.kindOf(id);
  if (gizmoOp_ == GizmoOp::Select || id == kNoId || id == kWorldId) return;
  if (k != Kind::Body && k != Kind::Geom && k != Kind::Joint && k != Kind::Site && k != Kind::Camera && k != Kind::Light) return;
  if (gizmoOp_ == GizmoOp::Scale && k != Kind::Geom && k != Kind::Site) return;
  // While simulating only static things (environment) can be moved; the rest follows the physics.
  if (sim_.active() && movesInSimulation(id)) return;

  Pose world = elementWorldPose(doc_, poses_, id);
  Vec3 scale{1, 1, 1};
  if (gizmoOp_ == GizmoOp::Scale) {
    if (k == Kind::Geom) scale = gizmoScaleOf(*doc_.geom(id));
    else scale = doc_.site(id)->size;
  }
  Mat4 model = Mat4::fromPose(world, scale);
  Mat4 view = vp_.cam.view();
  Mat4 proj = vp_.cam.proj(vp_.w / std::max(1.0f, vp_.h));

  ImGuizmo::SetOrthographic(vp_.cam.ortho);
  ImGuizmo::SetDrawlist(ImGui::GetWindowDrawList());
  ImGuizmo::SetRect(vp_.x, vp_.y, vp_.w, vp_.h);
  ImGuizmo::OPERATION op = gizmoOp_ == GizmoOp::Translate ? ImGuizmo::TRANSLATE
                         : gizmoOp_ == GizmoOp::Rotate ? ImGuizmo::ROTATE
                                                       : ImGuizmo::SCALE;
  ImGuizmo::MODE mode = (gizmoLocal_ || gizmoOp_ == GizmoOp::Scale) ? ImGuizmo::LOCAL : ImGuizmo::WORLD;
  bool snapping = snap_ != ImGui::GetIO().KeyCtrl;
  float snapVals[3];
  if (gizmoOp_ == GizmoOp::Translate) snapVals[0] = snapVals[1] = snapVals[2] = snapMove_;
  else if (gizmoOp_ == GizmoOp::Rotate) snapVals[0] = snapVals[1] = snapVals[2] = snapAngle_;
  else snapVals[0] = snapVals[1] = snapVals[2] = snapScale_ * 10;

  bool changed = ImGuizmo::Manipulate(view.m, proj.m, op, mode, model.m, nullptr, snapping ? snapVals : nullptr);
  bool using_ = ImGuizmo::IsUsing();
  vp_.gizmoUsing = using_;

  if (using_ && !gizmoDrag_.active) {
    // Drag start: record undo and the starting poses of everything that moves together.
    checkpoint(gizmoOp_ == GizmoOp::Translate ? "Move" : gizmoOp_ == GizmoOp::Rotate ? "Rotate" : "Resize");
    gizmoDrag_.active = true;
    gizmoDrag_.primaryStart = world;
    gizmoDrag_.starts.clear();
    for (Id s : sel_.ids) {
      if (s == id || s == kWorldId) continue;
      Kind sk = doc_.kindOf(s);
      if (sk != Kind::Body && sk != Kind::Geom && sk != Kind::Site && sk != Kind::Camera && sk != Kind::Light && sk != Kind::Joint) continue;
      // Skip elements that move with a selected ancestor body.
      Id owner = sk == Kind::Body ? doc_.body(s)->parent : doc_.ownerBody(s);
      bool covered = false;
      for (Id o : sel_.ids) if (o != s && doc_.kindOf(o) == Kind::Body && doc_.isAncestor(o, owner)) covered = true;
      if (!covered) gizmoDrag_.starts.push_back({s, elementWorldPose(doc_, poses_, s)});
    }
  }
  if (!using_ && gizmoDrag_.active) gizmoDrag_.active = false;
  if (!changed || !using_) return;

  // Decompose the manipulated matrix.
  Vec3 cols[3];
  for (int c = 0; c < 3; ++c) cols[c] = {model(0, c), model(1, c), model(2, c)};
  Vec3 newScale{length(cols[0]), length(cols[1]), length(cols[2])};
  Mat3 R;
  for (int c = 0; c < 3; ++c) {
    Vec3 n = normalize(cols[c]);
    R(0, c) = n.x;
    R(1, c) = n.y;
    R(2, c) = n.z;
  }
  Pose newWorld{{model(0, 3), model(1, 3), model(2, 3)}, fromMat3(R)};

  if (gizmoOp_ == GizmoOp::Scale) {
    auto ratio = [](double n, double o) { return o > 1e-12 ? n / o : 1.0; };
    if (k == Kind::Geom) {
      Geom* g = doc_.geom(id);
      Vec3 r{ratio(newScale.x, scale.x), ratio(newScale.y, scale.y), ratio(newScale.z, scale.z)};
      auto pickRatio = [](double a, double b) { return std::fabs(a - 1) > std::fabs(b - 1) ? a : b; };
      switch (g->type) {
        case GeomType::Sphere: g->size.x *= pickRatio(pickRatio(r.x, r.y), r.z); break;
        case GeomType::Capsule:
        case GeomType::Cylinder:
          g->size.x *= pickRatio(r.x, r.y);
          g->size.y *= r.z;
          break;
        case GeomType::Plane:
          if (g->size.x > 0) g->size.x *= r.x;
          if (g->size.y > 0) g->size.y *= r.y;
          break;
        default: g->size = cwiseMul(g->size, r); break;
      }
      for (int i = 0; i < 3; ++i) g->size[i] = std::max(1e-4, g->size[i]);
    } else if (k == Kind::Site) {
      Site* s = doc_.site(id);
      s->size = vmax(newScale, {1e-4, 1e-4, 1e-4});
    }
    markDirty();
    return;
  }

  // Translate / rotate: convert to the element's local frame.
  auto applyWorld = [&](Id e, const Pose& w, bool keepOrientation) {
    Kind ek = doc_.kindOf(e);
    Id owner = ek == Kind::Body ? doc_.body(e)->parent : doc_.ownerBody(e);
    Pose parentWorld = owner == kWorldId || owner == kNoId ? Pose{} : poses_[doc_.bodyIndex(owner)];
    Pose local = inverse(parentWorld) * w;
    for (int i = 0; i < 3; ++i)
      if (std::fabs(local.pos[i]) < 1e-12) local.pos[i] = 0;
    if (keepOrientation) local.quat = elementLocalPose(doc_, e).quat;
    setElementLocalPose(doc_, e, local);
  };
  if (gizmoOp_ == GizmoOp::Translate) {
    // ImGuizmo works in float; apply the total drag as a clean double-precision offset, with the
    // components orthogonal to the dragged axis zeroed (in the gizmo's own frame).
    Vec3 d = newWorld.pos - gizmoDrag_.primaryStart.pos;
    Quat frame = gizmoLocal_ ? gizmoDrag_.primaryStart.quat : Quat{};
    Vec3 dl = rotate(conj(frame), d);
    double mag = length(dl);
    for (int i = 0; i < 3; ++i)
      if (std::fabs(dl[i]) < 2e-6 * std::max(1.0, mag)) dl[i] = 0;
    d = rotate(frame, dl);
    Pose target = gizmoDrag_.primaryStart;
    target.pos += d;
    applyWorld(id, target, true);
    for (const auto& [e, start] : gizmoDrag_.starts) {
      Pose w = start;
      w.pos += d;
      applyWorld(e, w, true);
    }
  } else {
    applyWorld(id, newWorld, false);
    // Apply the same world-space rotation delta to the rest of the selection.
    Pose delta = newWorld * inverse(gizmoDrag_.primaryStart);
    for (const auto& [e, start] : gizmoDrag_.starts) applyWorld(e, delta * start, false);
  }
  markDirty();
}

// ------------------------------------------------------------------------------------ overlay UI

void Editor::drawViewportOverlayUI() {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  // Labels.
  if (vp_.showLabels || !sel_.empty()) {
    Mat4 vpm = vp_.cam.proj(vp_.w / std::max(1.0f, vp_.h)) * vp_.cam.view();
    auto label = [&](const Vec3& p, const std::string& text, ImU32 col) {
      if (text.empty()) return;
      Vec3 eyeDir = p - vp_.cam.eye();
      if (dot(eyeDir, vp_.cam.forward()) <= 0) return;
      Vec3 ndc = vpm.transformPoint(p);
      ImVec2 s(vp_.x + float(ndc.x * 0.5 + 0.5) * vp_.w, vp_.y + float(1.0 - (ndc.y * 0.5 + 0.5)) * vp_.h);
      ImVec2 ts = ImGui::CalcTextSize(text.c_str());
      ImVec2 a(s.x - ts.x * 0.5f - 4, s.y - ts.y - 12);
      dl->AddRectFilled(a, ImVec2(a.x + ts.x + 8, a.y + ts.y + 2), IM_COL32(20, 22, 27, 170), 3);
      dl->AddText(ImVec2(a.x + 4, a.y + 1), col, text.c_str());
    };
    if (vp_.showLabels) {
      for (uint32_t bi = 1; bi < doc_.bodies.size(); ++bi) label(poses_[bi].pos, doc_.bodies[bi].name, IM_COL32(200, 215, 240, 255));
    }
    for (Id id : sel_.ids) {
      if (!doc_.exists(id) || id == kWorldId) continue;
      label(elementWorldPose(doc_, poses_, id).pos, doc_.nameOf(id), IM_COL32(255, 200, 120, 255));
    }
  }

  // Navigation cube (top right).
  float cube = 96 * scale_;
  Mat4 view = vp_.cam.view();
  Mat4 before = view;
  ImGuizmo::SetDrawlist(dl);
  ImGuizmo::ViewManipulate(view.m, float(vp_.cam.distance), ImVec2(vp_.x + vp_.w - cube - 8, vp_.y + 8), ImVec2(cube, cube),
                           IM_COL32(30, 33, 40, 150));
  bool viewChanged = false;
  for (int i = 0; i < 16; ++i) viewChanged |= std::fabs(view.m[i] - before.m[i]) > 1e-6f;
  if (viewChanged) {
    Mat4 inv = view.inverted();
    Vec3 eye{inv(0, 3), inv(1, 3), inv(2, 3)};
    Vec3 fwd{-inv(0, 2), -inv(1, 2), -inv(2, 2)};
    vp_.cam.target = eye + fwd * vp_.cam.distance;
    Vec3 d = normalize(eye - vp_.cam.target);
    vp_.cam.elevation = clampv(rad2deg(std::asin(clampv(d.z, -1.0, 1.0))), -89.0, 89.0);
    vp_.cam.azimuth = rad2deg(std::atan2(d.y, d.x));
  }

  // Quick toggles (top left).
  ImGui::SetCursorScreenPos(ImVec2(vp_.x + 8, vp_.y + 8));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(2, 2));
  float bs = ImGui::GetFrameHeight();
  ImVec2 p0 = ImGui::GetCursorScreenPos();
  dl->AddRectFilled(ImVec2(p0.x - 3, p0.y - 3), ImVec2(p0.x + bs * 6 + 13, p0.y + bs + 3), IM_COL32(24, 26, 32, 190), 5);
  if (iconButton("##vjoints", Icon::Hinge, vp_.showJoints, "Show joints (J)")) vp_.showJoints = !vp_.showJoints;
  ImGui::SameLine();
  if (iconButton("##vcom", Icon::Body, vp_.showCom, "Show centers of mass (C)")) vp_.showCom = !vp_.showCom;
  ImGui::SameLine();
  if (iconButton("##vxray", Icon::EyeOpen, vp_.xray, "X-ray: see joints through bodies (X)")) vp_.xray = !vp_.xray;
  ImGui::SameLine();
  if (iconButton("##vsites", Icon::Site, vp_.showSites, "Show sites")) vp_.showSites = !vp_.showSites;
  ImGui::SameLine();
  if (iconButton("##vlabels", Icon::Info, vp_.showLabels, "Show names (L)")) vp_.showLabels = !vp_.showLabels;
  ImGui::SameLine();
  if (iconButton("##vframes", Icon::Free, vp_.showFrames, "Show body frames")) vp_.showFrames = !vp_.showFrames;
  ImGui::PopStyleVar();

  if (sim_.active()) drawSimBadge();

  // Hover name and hints (bottom left).
  std::string hint;
  if (vp_.hoverId) {
    hint = std::string(kindName(doc_.kindOf(vp_.hoverId))) + "  " + doc_.nameOf(vp_.hoverId);
    if (sim_.active() && movesInSimulation(vp_.hoverId)) hint += "   (drag to push)";
  } else if (sim_.active()) {
    hint = "Left-drag a body: push it   Space: pause / resume   Shift+Space: stop   Ctrl+R: reset";
  } else {
    hint = "Right-drag: orbit   Middle/Shift+right-drag: pan   Wheel: zoom   Click: select   F: frame";
  }
  ImVec2 ts = ImGui::CalcTextSize(hint.c_str());
  ImVec2 hp(vp_.x + 10, vp_.y + vp_.h - ts.y - 10);
  dl->AddRectFilled(ImVec2(hp.x - 5, hp.y - 3), ImVec2(hp.x + ts.x + 5, hp.y + ts.y + 3), IM_COL32(20, 22, 27, 160), 4);
  dl->AddText(hp, vp_.hoverId ? IM_COL32(230, 235, 245, 255) : IM_COL32(150, 156, 168, 255), hint.c_str());

  // Welcome card for an empty scene.
  bool emptyScene = doc_.bodies.size() <= 1 && doc_.geoms.size() <= 1 && !modified_;
  if (emptyScene) {
    ImVec2 size(440 * scale_, 0);
    ImGui::SetNextWindowPos(ImVec2(vp_.x + vp_.w * 0.5f, vp_.y + vp_.h * 0.42f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(size);
    ImGui::SetNextWindowBgAlpha(0.92f);
    if (ImGui::Begin("##welcome", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
                                               ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize |
                                               ImGuiWindowFlags_NoDocking)) {
      ImGui::TextUnformatted("Start building");
      ImGui::TextDisabled("Build a creature, an environment, or both. Everything exports to MJCF.");
      ImGui::Separator();
      for (const auto& t : templateList()) {
        if (std::string(t.id) == "empty") continue;
        if (ImGui::Button(t.name, ImVec2(130 * scale_, 0))) newDocument(t.id);
        ImGui::SameLine();
        ImGui::TextWrapped("%s", t.description);
      }
      ImGui::Separator();
      if (ImGui::Button("Add a model root", ImVec2(130 * scale_, 0))) addModelRoot();
      ImGui::SameLine();
      ImGui::TextDisabled("an empty free-floating body to build from");
    }
    ImGui::End();
  }
}

// ------------------------------------------------------------------------------------ panel

void Editor::drawViewport() {
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
  bool open = ImGui::Begin("Viewport", nullptr, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImGui::PopStyleVar();
  if (!open) {
    ImGui::End();
    return;
  }
  ImVec2 pos = ImGui::GetCursorScreenPos();
  ImVec2 avail = ImGui::GetContentRegionAvail();
  vp_.x = pos.x;
  vp_.y = pos.y;
  vp_.w = std::max(1.0f, avail.x);
  vp_.h = std::max(1.0f, avail.y);
  ImVec2 fbScale = ImGui::GetIO().DisplayFramebufferScale;
  int fw = int(vp_.w * fbScale.x), fh = int(vp_.h * fbScale.y);

  // Render.
  std::vector<DrawItem> items;
  items.reserve(doc_.geoms.size() + 16);
  buildScene(items);
  buildOverlays();
  vp_.target.resize(fw, fh, 4);
  vp_.target.bind();
  FrameParams fp;
  fp.width = fw;
  fp.height = fh;
  fp.shadows = vp_.showShadows;
  fp.grid = vp_.showGrid;
  bool hasFloor = false;
  for (const Geom& g : doc_.geoms) hasFloor |= g.body == kWorldId && g.type == GeomType::Plane;
  fp.gridAlpha = hasFloor ? 0.45f : 1.0f;
  fp.xray = vp_.xray ? 0.3f : 0.0f;
  renderer_.render(items, vp_.cam, fp, &dbg_);
  vp_.target.resolve();

  ImGui::Image((ImTextureID)(intptr_t)vp_.target.texture(), ImVec2(vp_.w, vp_.h), ImVec2(0, 1), ImVec2(1, 0));
  vp_.hovered = ImGui::IsItemHovered();
  ImGuizmo::BeginFrame();
  drawGizmo();
  drawViewportOverlayUI();
  bool overUi = ImGui::IsAnyItemHovered() && !vp_.hovered;
  if (!overUi) viewportInput();

  // Context menu on right-click without drag.
  if (vp_.hovered && ImGui::IsMouseReleased(1) && !vp_.dragMoved) {
    Id hit = pick(vp_.cam.ray(ImGui::GetIO().MousePos.x - vp_.x, ImGui::GetIO().MousePos.y - vp_.y, vp_.w, vp_.h));
    if (hit && !sel_.contains(hit)) sel_.set(hit);
    ImGui::OpenPopup("##vpctx");
  }
  if (ImGui::BeginPopup("##vpctx")) {
    Id p = sel_.primary();
    if (p) drawElementContextMenu(p);
    else drawAddMenuItems(kWorldId);
    ImGui::EndPopup();
  }
  ImGui::End();
}

}  // namespace mjf
