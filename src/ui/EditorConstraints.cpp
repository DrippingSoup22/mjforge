// Contact pairs, equality constraints and tendons: creating them from the selection and drawing
// them in the viewport.
#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <imgui.h>

#include "core/Kinematics.h"
#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Theme.h"

namespace mjf {

void Editor::drawConstraintMenu() {
  std::vector<Id> bodies, geoms, joints, sites, tendons;
  for (Id id : sel_.ids) {
    switch (doc_.kindOf(id)) {
      case Kind::Body: if (id != kWorldId) bodies.push_back(id); break;
      case Kind::Geom: geoms.push_back(id); break;
      case Kind::Joint: {
        JointType t = doc_.joint(id)->type;
        if (t == JointType::Hinge || t == JointType::Slide) joints.push_back(id);
        break;
      }
      case Kind::Site: sites.push_back(id); break;
      case Kind::Tendon: tendons.push_back(id); break;
      default: break;
    }
  }
  auto tip = [](const char* t) {
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled)) ImGui::SetTooltip("%s", t);
  };
  if (ImGui::MenuItem("Contact pair between the 2 selected geoms", nullptr, false, geoms.size() == 2)) {
    checkpoint("Add contact pair");
    sel_.set(addContactPair(doc_, geoms[0], geoms[1]));
  }
  tip("These two geoms always collide with the pair's own friction and solver settings, whatever their contype/"
      "conaffinity. Select two geoms (Alt+click picks geoms).");
  if (ImGui::MenuItem("Connect the selected bodies (ball joint)", nullptr, false, bodies.size() == 1 || bodies.size() == 2)) {
    checkpoint("Add connect constraint");
    sel_.set(addEquality(doc_, EqualityType::Connect, bodies[0], bodies.size() == 2 ? bodies[1] : kNoId));
  }
  tip("Closes a kinematic loop: the bodies share a point but can rotate (four-bar linkages, parallel legs). With one "
      "body selected, it is pinned to the world at its origin.");
  if (ImGui::MenuItem("Weld the selected bodies", nullptr, false, bodies.size() == 1 || bodies.size() == 2)) {
    checkpoint("Add weld constraint");
    sel_.set(addEquality(doc_, EqualityType::Weld, bodies[0], bodies.size() == 2 ? bodies[1] : kNoId));
  }
  tip("Holds two bodies at their current relative pose. With one body selected, it is fixed in the world (hang a "
      "creature to test its legs).");
  if (ImGui::MenuItem("Couple the 2 selected joints", nullptr, false, joints.size() == 2)) {
    checkpoint("Add joint coupling");
    sel_.set(addEquality(doc_, EqualityType::Joint, joints[0], joints[1]));
  }
  tip("The first joint follows the second (equal angles by default; edit the polynomial in the inspector).");
  ImGui::Separator();
  if (ImGui::MenuItem("Fixed tendon from the selected joints", nullptr, false, !joints.empty())) {
    checkpoint("Add fixed tendon");
    Id t = addTendon(doc_, TendonType::Fixed, "tendon");
    for (Id j : joints) doc_.tendon(t)->joints.push_back({j, 1.0});
    sel_.set(t);
  }
  tip("A length made of joint positions (coefficient 1 each). One actuator on it drives all of them.");
  if (ImGui::MenuItem("Spatial tendon through the selected sites", nullptr, false, sites.size() >= 2)) {
    checkpoint("Add spatial tendon");
    Id t = addTendon(doc_, TendonType::Spatial, "cable");
    for (Id s : sites) doc_.tendon(t)->path.push_back({TendonPoint::Type::Site, s});
    doc_.tendon(t)->width = 0.005;
    sel_.set(t);
  }
  tip("A cable or muscle path through the sites, in the order you selected them.");
  if (ImGui::MenuItem("Motor on the selected tendon", nullptr, false, tendons.size() == 1)) {
    checkpoint("Add tendon motor");
    const Tendon* t = doc_.tendon(tendons[0]);
    Id a = addActuator(doc_, ActuatorType::Motor, kNoId, t->name.empty() ? "tendon_motor" : t->name + "_motor");
    doc_.actuator(a)->trn = TrnType::Tendon;
    doc_.actuator(a)->target = tendons[0];
    sel_.set(a);
  }
}

void Editor::buildConstraintOverlays() {
  std::unordered_set<Id> selected(sel_.ids.begin(), sel_.ids.end());
  auto bodyPose = [&](Id b) -> Pose {
    int bi = doc_.bodyIndex(b);
    return bi >= 0 && bi < int(poses_.size()) ? poses_[bi] : Pose{};
  };
  auto sitePos = [&](Id s, Vec3* out) {
    const Site* site = doc_.site(s);
    if (!site) return false;
    *out = bodyPose(site->body).apply(site->pos);
    return true;
  };

  // Spatial tendons: MuJoCo's own path (with wrapping) while simulating, straight segments
  // between the sites otherwise.
  std::unordered_set<Id> drawnBySim;
  if (sim_.active()) {
    const mjModel* m = sim_.model();
    const mjData* d = sim_.data();
    const ExportMap& map = sim_.map();
    for (int t = 0; t < m->ntendon && t < int(map.tendons.size()); ++t) {
      const Tendon* tendon = doc_.tendon(map.tendons[t]);
      if (!tendon || tendon->type != TendonType::Spatial) continue;
      drawnBySim.insert(tendon->id);
      bool sel = selected.count(tendon->id) > 0;
      Color c = tendon->rgba;
      if (sel) c = pal::color(pal::kAccent);
      float w = std::clamp(float(tendon->width * 600), 1.5f, 6.0f);
      int adr = d->ten_wrapadr[t], num = d->ten_wrapnum[t];
      for (int j = adr; j < adr + num - 1; ++j) {
        if (d->wrap_obj[j] == -2 || d->wrap_obj[j + 1] == -2) continue;  // pulley: a new branch starts
        const mjtNum* a = d->wrap_xpos + 3 * j;
        const mjtNum* b = d->wrap_xpos + 3 * j + 3;
        dbg_.line({a[0], a[1], a[2]}, {b[0], b[1], b[2]}, c, w, sel ? DebugDraw::Overlay : DebugDraw::World);
      }
    }
  }
  for (const Tendon& t : doc_.tendons) {
    bool sel = selected.count(t.id) > 0;
    if (t.type == TendonType::Fixed) {
      // Fixed tendons have no path; show which joints they combine when selected.
      if (!sel) continue;
      for (const TendonJoint& tj : t.joints) {
        const Joint* j = doc_.joint(tj.joint);
        if (!j) continue;
        Pose bp = bodyPose(j->body);
        dbg_.circle(bp.apply(j->pos), rotate(bp.quat, j->axis), 0.04, pal::color(pal::kAccent), 3.0f, DebugDraw::Overlay);
      }
      continue;
    }
    if (drawnBySim.count(t.id)) continue;
    Color c = sel ? pal::color(pal::kAccent) : t.rgba;
    float w = std::clamp(float(t.width * 600), 1.5f, 6.0f);
    Vec3 prev;
    bool havePrev = false;
    for (const TendonPoint& p : t.path) {
      if (p.type == TendonPoint::Type::Pulley) {
        havePrev = false;
        continue;
      }
      if (p.type != TendonPoint::Type::Site) continue;  // wrapping is only computed by MuJoCo
      Vec3 x;
      if (!sitePos(p.obj, &x)) continue;
      if (havePrev) dbg_.line(prev, x, c, w, sel ? DebugDraw::Overlay : DebugDraw::World);
      prev = x;
      havePrev = true;
    }
  }

  // Equality constraints: the shared point of a connect, the link of a weld.
  if (vp_.showJoints || !selected.empty()) {
    for (const Equality& e : doc_.equalities) {
      bool sel = selected.count(e.id) > 0;
      if (!vp_.showJoints && !sel) continue;
      if (e.type != EqualityType::Connect && e.type != EqualityType::Weld) continue;
      Color c = sel ? pal::color(pal::kAccent) : Color{0.95f, 0.75f, 0.3f, e.active ? 0.9f : 0.4f};
      DebugDraw::Layer layer = sel ? DebugDraw::Overlay : DebugDraw::World;
      Vec3 p1, p2;
      if (doc_.kindOf(e.obj1) == Kind::Site) {
        if (!sitePos(e.obj1, &p1)) continue;
        if (!sitePos(e.obj2, &p2)) p2 = p1;
      } else {
        if (!doc_.body(e.obj1)) continue;
        Pose b1 = bodyPose(e.obj1);
        Pose b2 = doc_.body(e.obj2) ? bodyPose(e.obj2) : Pose{};
        if (e.type == EqualityType::Connect) {
          p1 = b1.apply(e.anchor);
          p2 = b2.pos;
          dbg_.line(b1.pos, p1, c, 1.5f, layer);
        } else {
          p1 = b1.pos;
          p2 = b2.apply(e.anchor);
        }
      }
      dbg_.cross3(p1, 0.025, c, 2.5f, layer);
      dbg_.line(p1, p2, c, e.type == EqualityType::Weld ? 3.0f : 1.5f, layer);
    }
  }

  // Contact pairs: link the two geoms when selected.
  for (const ContactPair& cp : doc_.pairs) {
    if (!selected.count(cp.id)) continue;
    const Geom* a = doc_.geom(cp.geom1);
    const Geom* b = doc_.geom(cp.geom2);
    if (!a || !b) continue;
    Vec3 pa = bodyPose(a->body).apply(a->pos), pb = bodyPose(b->body).apply(b->pos);
    if (b->type == GeomType::Plane) pb = {pa.x, pa.y, pb.z};  // straight down onto a floor
    if (a->type == GeomType::Plane) pa = {pb.x, pb.y, pa.z};
    Color c = pal::color(pal::kAccent);
    dbg_.line(pa, pb, c, 2.5f, DebugDraw::Overlay);
    dbg_.cross3(pa, 0.03, c, 2.5f, DebugDraw::Overlay);
    dbg_.cross3(pb, 0.03, c, 2.5f, DebugDraw::Overlay);
  }
}

}  // namespace mjf
