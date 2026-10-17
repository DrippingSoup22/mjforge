// Modeling tools: the add-limb tool (click a body to grow a jointed limb) and linear / radial
// arrays.
#include <algorithm>
#include <cmath>

#include <imgui.h>

#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace mjf {

namespace {

double niceRound(double v) {
  if (v <= 0) return v;
  double e = std::pow(10.0, std::floor(std::log10(v)));
  for (double m : {1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0})
    if (m * e >= v) return m * e;
  return 10 * e;
}

}  // namespace

// ------------------------------------------------------------------------------------ limb tool

bool Editor::pickSurface(const Ray& ray, Id* geomOut, Vec3* point, Vec3* normal) {
  double best = 1e300;
  Id bestGeom = kNoId;
  Vec3 bestN;
  const DocIndex& ix = doc_.index();
  for (uint32_t bi = 0; bi < doc_.bodies.size(); ++bi) {
    const Body& b = doc_.bodies[bi];
    if (b.hidden) continue;
    for (uint32_t gi : ix.bodyGeoms[bi]) {
      const Geom& g = doc_.geoms[gi];
      if (g.hidden || !vp_.groupShown(g.group) || g.type == GeomType::Hfield) continue;
      Vec3 n;
      double t = rayGeom(doc_, ray, g, poses_[bi] * Pose{g.pos, g.quat}, 60.0, &n);
      if (t >= 0 && t < best) {
        best = t;
        bestGeom = g.id;
        bestN = n;
      }
    }
  }
  if (!bestGeom) return false;
  if (geomOut) *geomOut = bestGeom;
  if (point) *point = ray.at(best);
  if (normal) *normal = normalize(bestN, {0, 0, 1});
  return true;
}

LimbSpec Editor::limbSpecAt(Id body, const Vec3& normal) const {
  LimbSpec spec = limbSpec_;
  Id root = doc_.rootOf(body);
  Pose rootPose = bodyWorldPose(doc_, root);
  Vec3 down{0, 0, -1};
  if (limbDirMode_ == 1 || (limbDirMode_ == 0 && normal.z > 0.5)) spec.direction = normal;
  else spec.direction = down;
  spec.axis = rotate(rootPose.quat, {0, 1, 0});  // swing forward/backward relative to the creature
  // Strong enough to carry the creature's weight at full extension (the analysis refines this).
  std::vector<Pose> poses = computeBodyPoses(doc_);
  double mass = std::max(0.2, subtreeMassProps(doc_, poses, root).mass);
  spec.maxTorque = niceRound(std::max(1.0, mass * 9.81 * spec.length * spec.segments * 0.75));
  return spec;
}

void Editor::limbToolInput(const Ray& ray, bool clicked) {
  limbHover_ = {};
  Id g = kNoId;
  Vec3 p, n;
  if (!pickSurface(ray, &g, &p, &n)) return;
  Id body = doc_.geom(g)->body;
  if (body == kWorldId) return;  // limbs grow from bodies, not from the world
  limbHover_ = {true, body, p, n};
  if (!clicked) return;
  checkpoint("Add limb");
  LimbSpec spec = limbSpecAt(body, n);
  // Names like leg1_left / leg1_right (mirrorName swaps the side), numbered per pair.
  Id root = doc_.rootOf(body);
  double side = inverse(bodyWorldPose(doc_, root)).apply(p).y;
  std::string sideName = side > 0.01 ? "_left" : side < -0.01 ? "_right" : "";
  auto taken = [&](const std::string& base) {
    for (const char* suffix : {"", "_upper", "_middle", "_lower", "_seg1"})
      if (doc_.nameTaken(Kind::Body, base + suffix) || doc_.nameTaken(Kind::Body, mirrorName(base + suffix))) return true;
    return false;
  };
  int k = 1;
  while (taken(limbSpec_.name + std::to_string(k) + sideName)) ++k;
  spec.name = limbSpec_.name + std::to_string(k) + sideName;
  Id limb = addLimb(doc_, body, p, spec);
  std::vector<Id> made{limb};
  // Mirror onto the other side of the creature when attached off-center to the root body.
  if (limbMirror_ && body == root && !sideName.empty()) {
    Id m = mirrorSubtree(doc_, limb, 1);
    if (m) made.push_back(m);
  }
  sel_.ids = made;
  notify(made.size() > 1 ? "Added a limb and its mirror image" : "Added a limb", pal::kOk, 2);
}

void Editor::buildLimbPreview() {
  if (!limbTool_ || !limbHover_.valid) return;
  LimbSpec spec = limbSpecAt(limbHover_.body, limbHover_.normal);
  Vec3 d = normalize(spec.direction, {0, 0, -1});
  Color c{0.4f, 0.95f, 0.6f, 0.95f};
  Vec3 a = limbHover_.point;
  double r = spec.radius;
  for (int s = 0; s < spec.segments; ++s) {
    Vec3 b = a + d * spec.length;
    dbg_.line(a, b, c, 3.0f, DebugDraw::Overlay);
    dbg_.circle(a, d, r * 1.6, c, 2.0f, DebugDraw::Overlay, 20);
    a = b;
    r *= spec.taper;
  }
  if (spec.foot) dbg_.circle(a, vp_.cam.forward(), r * 1.25, c, 2.0f, DebugDraw::Overlay, 20);
  dbg_.arrow(limbHover_.point, limbHover_.point + limbHover_.normal * 0.06, {1, 1, 1, 0.8f}, 1.5f, DebugDraw::Overlay);
  // Mirror preview.
  Id body = limbHover_.body;
  if (limbMirror_ && body == doc_.rootOf(body)) {
    Pose bw = bodyWorldPose(doc_, body);
    Vec3 local = inverse(bw).apply(limbHover_.point);
    if (std::fabs(local.y) > 0.01) {
      auto M = [&](const Vec3& w) {
        Vec3 l = inverse(bw).apply(w);
        l.y = -l.y;
        return bw.apply(l);
      };
      Vec3 ma = M(limbHover_.point), md = M(limbHover_.point + d * spec.length * spec.segments);
      Color cm = c;
      cm.a = 0.45f;
      dbg_.line(ma, md, cm, 2.0f, DebugDraw::Overlay);
    }
  }
}

void Editor::drawLimbToolPanel() {
  if (!limbTool_) return;
  const float s = scale_;
  ImGui::SetCursorScreenPos(ImVec2(vp_.x + 8, vp_.y + 8 + ImGui::GetFrameHeight() + 10 * s));
  ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(24, 26, 32, 225));
  ImGui::BeginChild("##limbtool", ImVec2(250 * s, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders);
  ImGui::TextColored(pal::v4(pal::kOk), "Add limb");
  ImGui::SameLine();
  ImGui::TextDisabled("click a body  (Esc: done)");
  ImGui::PushItemWidth(120 * s);
  ImGui::SliderInt("Segments", &limbSpec_.segments, 1, 4);
  float len = float(limbSpec_.length), rad = float(limbSpec_.radius), range = float(rad2deg(limbSpec_.range));
  if (ImGui::DragFloat("Segment length", &len, 0.002f, 0.01f, 5.0f, "%.3f m")) limbSpec_.length = len;
  if (ImGui::DragFloat("Radius", &rad, 0.0005f, 0.002f, 1.0f, "%.3f m")) limbSpec_.radius = rad;
  if (ImGui::DragFloat("Joint range", &range, 0.5f, 5.0f, 180.0f, "+-%.0f deg")) limbSpec_.range = deg2rad(range);
  ImGui::Combo("Direction", &limbDirMode_, "Auto\0Surface normal\0Down\0");
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Auto: legs hang down from sides and bottom; on top surfaces limbs point along the normal.");
  ImGui::PopItemWidth();
  ImGui::Checkbox("Mirror to the other side", &limbMirror_);
  ImGui::Checkbox("Actuators", &limbSpec_.actuators);
  if (limbSpec_.actuators) {
    ImGui::SameLine();
    int t = limbSpec_.actuatorType == ActuatorType::Motor ? 1 : 0;
    ImGui::SetNextItemWidth(100 * s);
    if (ImGui::Combo("##acttype", &t, "Position\0Motor\0")) limbSpec_.actuatorType = t ? ActuatorType::Motor : ActuatorType::Position;
  }
  ImGui::Checkbox("Foot sphere", &limbSpec_.foot);
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

// ------------------------------------------------------------------------------------ arrays

void Editor::drawArrayModal() {
  if (showArray_) {
    ImGui::OpenPopup("Array");
    showArray_ = false;
  }
  ImGui::SetNextWindowSize(ImVec2(420 * scale_, 0), ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal("Array", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  Id id = sel_.primary();
  Kind k = doc_.kindOf(id);
  bool ok = id != kWorldId && (k == Kind::Body || k == Kind::Geom || k == Kind::Site);
  if (!ok) {
    ImGui::TextDisabled("Select a body, geom or site first.");
    if (ImGui::Button("Close")) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
    return;
  }
  ImGui::Text("Copies of %s", doc_.displayName(id).c_str());
  ImGui::RadioButton("Linear", &arrayRadial_, 0);
  ImGui::SameLine();
  ImGui::RadioButton("Radial (around the parent's origin)", &arrayRadial_, 1);
  ImGui::SliderInt("Copies", &arrayCopies_, 1, 64);
  if (arrayRadial_) {
    ImGui::Combo("Axis (parent frame)", &arrayAxis_, "X\0Y\0Z\0");
    ImGui::DragFloat("Total angle", &arrayAngle_, 1.0f, 1.0f, 360.0f, "%.0f deg");
    ImGui::TextDisabled("360 deg spreads the original and its copies evenly around the circle,");
    ImGui::TextDisabled("e.g. 5 copies of one hexapod leg give six legs 60 deg apart.");
  } else {
    ImGui::DragFloat3("Offset (parent frame)", arrayOffset_, 0.005f, -100, 100, "%.3f m");
  }
  if (ImGui::Button("Create", ImVec2(110 * scale_, 0))) {
    Pose step;
    if (arrayRadial_) {
      double total = deg2rad(arrayAngle_);
      bool fullCircle = std::fabs(arrayAngle_ - 360.0f) < 0.5f;
      double a = fullCircle ? total / (arrayCopies_ + 1) : total / arrayCopies_;
      Vec3 axis{arrayAxis_ == 0 ? 1.0 : 0.0, arrayAxis_ == 1 ? 1.0 : 0.0, arrayAxis_ == 2 ? 1.0 : 0.0};
      step = {{}, axisAngle(axis, a)};
    } else {
      step = {{arrayOffset_[0], arrayOffset_[1], arrayOffset_[2]}, {}};
    }
    checkpoint("Array");
    std::vector<Id> made = arrayDuplicate(doc_, id, arrayCopies_, step);
    sel_.ids = made;
    sel_.add(id);
    notify("Created " + std::to_string(made.size()) + " copies", pal::kOk, 2);
    ImGui::CloseCurrentPopup();
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(90 * scale_, 0))) ImGui::CloseCurrentPopup();
  ImGui::EndPopup();
}

}  // namespace mjf
