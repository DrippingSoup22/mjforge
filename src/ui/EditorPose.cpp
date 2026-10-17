// Pose mode: edit the start pose (keyframe 0) by moving joints instead of bodies. The viewport
// shows the model in that pose; rings rotate hinge joints, the gizmo moves the free root, and a
// panel lists every joint of the model with a slider.
#include <algorithm>
#include <cmath>

#include <imgui.h>
#include <ImGuizmo.h>

#include "core/Kinematics.h"
#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace mjf {

void Editor::setPoseMode(bool on) {
  if (on == poseMode_) return;
  if (on && sim_.active()) stopSimulation();
  poseMode_ = on;
  if (on) {
    limbTool_ = false;
    notify(doc_.keyframes.empty() ? "Pose mode: move joints to set the start pose (a 'home' keyframe is created on the first change)"
                                  : "Pose mode: editing the start pose '" + doc_.keyframes[0].name + "'",
           pal::kTextDim, 4);
  }
}

const Keyframe* Editor::poseKey() const {
  return poseMode_ && !doc_.keyframes.empty() ? &doc_.keyframes[0] : nullptr;
}

double Editor::jointDisplayOffset(Id joint) const {
  if (sim_.active()) return simJointOffset(joint);
  const Keyframe* key = poseKey();
  const Joint* j = doc_.joint(joint);
  if (!key || !j || (j->type != JointType::Hinge && j->type != JointType::Slide)) return 0;
  return keyJointValue(key, *j) - j->ref;
}

void Editor::poseEdited() {
  syncKeyframeControls(doc_, doc_.keyframes[0]);
  doc_.touch();
}

namespace {

double clampToRange(const Joint& j, double v) {
  if (j.limited && j.range.y > j.range.x) return std::clamp(v, j.range.x, j.range.y);
  return v;
}

}  // namespace

void Editor::drawPoseGizmo() {
  Id id = sel_.primary();
  Kind k = doc_.kindOf(id);
  Id body = k == Kind::Body ? id : (k == Kind::Joint ? doc_.joint(id)->body : doc_.ownerBody(id));
  int bi = doc_.bodyIndex(body);
  if (body == kNoId || body == kWorldId || bi < 0) return;
  // The selected joint, or every joint of the selected body.
  std::vector<const Joint*> joints;
  for (uint32_t ji : doc_.index().bodyJoints[bi])
    if (k != Kind::Joint || doc_.joints[ji].id == id) joints.push_back(&doc_.joints[ji]);
  if (joints.empty()) return;

  Mat4 view = vp_.cam.view();
  Mat4 proj = vp_.cam.proj(vp_.w / std::max(1.0f, vp_.h));
  ImGuizmo::SetOrthographic(vp_.cam.ortho);
  ImGuizmo::SetDrawlist(ImGui::GetWindowDrawList());
  ImGuizmo::SetRect(vp_.x, vp_.y, vp_.w, vp_.h);
  bool snapping = snap_ != ImGui::GetIO().KeyCtrl;
  const Pose bw = poses_[bi];  // the posed body frame

  // One drag is one undo step; the keyframe is created inside it.
  auto beginEdit = [&]() -> Keyframe& {
    if (!poseDrag_) {
      checkpoint("Pose");
      poseDrag_ = true;
    }
    return startKeyframe(doc_);
  };

  // Rings of several joints on one body share a center: give each its own size, and the color of
  // the body axis it is closest to (roll about X red, pitch about Y green, yaw about Z blue).
  ImGuizmo::Style& style = ImGuizmo::GetStyle();
  const ImVec4 zColor = style.Colors[ImGuizmo::DIRECTION_Z];
  int ring = 0;
  for (const Joint* jp : joints) {
    const Joint j = *jp;  // the document may change below
    ImGuizmo::PushID(int(j.id));
    if (j.type == JointType::Hinge || j.type == JointType::Slide) {
      Vec3 a{std::fabs(j.axis.x), std::fabs(j.axis.y), std::fabs(j.axis.z)};
      int dominant = a.x >= a.y && a.x >= a.z ? 0 : (a.y >= a.z ? 1 : 2);
      style.Colors[ImGuizmo::DIRECTION_Z] = dominant == 2 ? zColor : style.Colors[dominant];
      ImGuizmo::SetGizmoSizeClipSpace(0.1f * (1.0f - 0.18f * float(ring++ % 3)));
      Vec3 anchor = bw.apply(j.pos);
      Vec3 axis = normalize(rotate(bw.quat, j.axis), {0, 0, 1});
      Quat frame = frameFromZ(axis);
      Mat4 m = Mat4::fromPose({anchor, frame});
      bool hinge = j.type == JointType::Hinge;
      float snapVal[3] = {hinge ? snapAngle_ : snapMove_, hinge ? snapAngle_ : snapMove_, hinge ? snapAngle_ : snapMove_};
      bool changed = ImGuizmo::Manipulate(view.m, proj.m, hinge ? ImGuizmo::ROTATE_Z : ImGuizmo::TRANSLATE_Z, ImGuizmo::LOCAL,
                                          m.m, nullptr, snapping ? snapVal : nullptr);
      if (changed && ImGuizmo::IsUsing()) {
        double delta;
        if (hinge) {
          // This frame's rotation about the axis: how far the gizmo's X axis turned.
          Vec3 x0 = rotate(frame, {1, 0, 0});
          Vec3 x1 = normalize(Vec3{m(0, 0), m(1, 0), m(2, 0)}, x0);
          delta = std::atan2(dot(cross(x0, x1), axis), dot(x0, x1));
        } else {
          delta = dot(Vec3{m(0, 3), m(1, 3), m(2, 3)} - anchor, axis);
        }
        Keyframe& key = beginEdit();
        setKeyJointQpos(key, j.id, {clampToRange(j, keyJointValue(&key, j) + delta)});
        poseEdited();
      }
    } else if (j.type == JointType::Free && (gizmoOp_ == GizmoOp::Translate || gizmoOp_ == GizmoOp::Rotate)) {
      Mat4 m = Mat4::fromPose(bw);
      float snapVal[3];
      snapVal[0] = snapVal[1] = snapVal[2] = gizmoOp_ == GizmoOp::Translate ? snapMove_ : snapAngle_;
      bool changed = ImGuizmo::Manipulate(view.m, proj.m, gizmoOp_ == GizmoOp::Translate ? ImGuizmo::TRANSLATE : ImGuizmo::ROTATE,
                                          gizmoLocal_ ? ImGuizmo::LOCAL : ImGuizmo::WORLD, m.m, nullptr,
                                          snapping ? snapVal : nullptr);
      if (changed && ImGuizmo::IsUsing()) {
        Mat3 R;
        for (int c = 0; c < 3; ++c) {
          Vec3 col = normalize(Vec3{m(0, c), m(1, c), m(2, c)});
          R(0, c) = col.x;
          R(1, c) = col.y;
          R(2, c) = col.z;
        }
        Quat q = normalize(fromMat3(R));
        Keyframe& key = beginEdit();
        setKeyJointQpos(key, j.id, {m(0, 3), m(1, 3), m(2, 3), q.w, q.x, q.y, q.z});
        poseEdited();
      }
    } else if (j.type == JointType::Ball && joints.size() == 1) {
      // Ball joint as the body's only joint: body frame = parent * local * ball rotation.
      int pi = doc_.bodyIndex(doc_.body(body)->parent);
      Pose before = (pi >= 0 ? poses_[pi] : Pose{}) * Pose{doc_.body(body)->pos, doc_.body(body)->quat};
      Mat4 m = Mat4::fromPose({bw.apply(j.pos), bw.quat});
      float snapVal[3] = {snapAngle_, snapAngle_, snapAngle_};
      bool changed = ImGuizmo::Manipulate(view.m, proj.m, ImGuizmo::ROTATE, gizmoLocal_ ? ImGuizmo::LOCAL : ImGuizmo::WORLD,
                                          m.m, nullptr, snapping ? snapVal : nullptr);
      if (changed && ImGuizmo::IsUsing()) {
        Mat3 R;
        for (int c = 0; c < 3; ++c) {
          Vec3 col = normalize(Vec3{m(0, c), m(1, c), m(2, c)});
          R(0, c) = col.x;
          R(1, c) = col.y;
          R(2, c) = col.z;
        }
        Quat q = normalize(conj(before.quat) * fromMat3(R));
        Keyframe& key = beginEdit();
        setKeyJointQpos(key, j.id, {q.w, q.x, q.y, q.z});
        poseEdited();
      }
    }
    ImGuizmo::PopID();
    style.Colors[ImGuizmo::DIRECTION_Z] = zColor;
    ImGuizmo::SetGizmoSizeClipSpace(0.1f);
  }
  if (!ImGuizmo::IsUsingAny()) poseDrag_ = false;
  vp_.gizmoUsing = ImGuizmo::IsUsingAny();
}

void Editor::drawPosePanel() {
  if (!poseMode_) return;
  const float s = scale_;
  // The model being posed: the one containing the selection, else the first model.
  Id root = kNoId;
  if (Id p = sel_.primary()) {
    Id b = doc_.kindOf(p) == Kind::Body ? p : (doc_.kindOf(p) == Kind::Joint ? doc_.joint(p)->body : doc_.ownerBody(p));
    if (b != kNoId && b != kWorldId) root = doc_.rootOf(b);
  }
  if (!root)
    for (const Body& b : doc_.bodies)
      if (b.id != kWorldId && b.parent == kWorldId && b.role == Role::Model) {
        root = b.id;
        break;
      }

  ImGui::SetCursorScreenPos(ImVec2(vp_.x + 8, vp_.y + 8 + ImGui::GetFrameHeight() + 10 * s));
  ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(24, 26, 32, 228));
  ImGui::BeginChild("##posepanel", ImVec2(290 * s, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders);
  iconInline(Icon::Pose, pal::kAccent);
  ImGui::SameLine();
  ImGui::TextColored(pal::v4(pal::kAccent), "Start pose");
  ImGui::SameLine();
  ImGui::TextDisabled(doc_.keyframes.empty() ? "(reference pose)" : "(keyframe '%s')", doc_.keyframes.empty() ? "" : doc_.keyframes[0].name.c_str());
  ImGui::SameLine(ImGui::GetContentRegionMax().x - 48 * s);
  if (ImGui::SmallButton("Done")) setPoseMode(false);
  if (!root) {
    ImGui::TextDisabled("Add a model to pose it.");
    ImGui::EndChild();
    ImGui::PopStyleColor();
    return;
  }
  ImGui::TextDisabled("%s: drag the rings or the sliders", doc_.nameOf(root).c_str());

  const Keyframe* key = doc_.keyframes.empty() ? nullptr : &doc_.keyframes[0];
  std::vector<const Joint*> joints;
  for (Id b : doc_.subtree(root)) {
    int bi = doc_.bodyIndex(b);
    for (uint32_t ji : doc_.index().bodyJoints[bi])
      if (doc_.joints[ji].type == JointType::Hinge || doc_.joints[ji].type == JointType::Slide) joints.push_back(&doc_.joints[ji]);
  }
  float rowH = ImGui::GetFrameHeightWithSpacing();
  float listH = std::min(float(joints.size()) * rowH + 4, vp_.h * 0.5f);
  if (!joints.empty() && ImGui::BeginChild("##posejoints", ImVec2(0, listH), ImGuiChildFlags_None)) {
    for (const Joint* jp : joints) {
      const Joint j = *jp;
      ImGui::PushID(int(j.id));
      bool hinge = j.type == JointType::Hinge;
      double v = keyJointValue(key, j);
      bool limited = j.limited && j.range.y > j.range.x;
      float lo = float(limited ? j.range.x : (hinge ? -kPi : -1.0)), hi = float(limited ? j.range.y : (hinge ? kPi : 1.0));
      float f = float(hinge ? rad2deg(v) : v);
      float flo = hinge ? float(rad2deg(lo)) : lo, fhi = hinge ? float(rad2deg(hi)) : hi;
      bool sel = sel_.contains(j.id);
      if (sel) ImGui::PushStyleColor(ImGuiCol_Text, pal::v4(pal::kAccent));
      ImGui::SetNextItemWidth(150 * s);
      bool changed = ImGui::SliderFloat("##v", &f, flo, fhi, hinge ? "%.1f deg" : "%.3f m");
      bool activated = ImGui::IsItemActivated();
      if (sel) ImGui::PopStyleColor();
      if (activated) checkpoint("Pose");
      if (changed) {
        setKeyJointQpos(startKeyframe(doc_), j.id, {clampToRange(j, hinge ? deg2rad(f) : f)});
        key = &doc_.keyframes[0];
        poseEdited();
      }
      ImGui::SameLine();
      if (ImGui::Selectable(j.name.empty() ? "(joint)" : j.name.c_str(), sel)) sel_.set(j.id);
      ImGui::PopID();
    }
  }
  if (!joints.empty()) ImGui::EndChild();
  if (ImGui::Button("Reset")) {
    checkpoint("Reset pose");
    resetKeyPose(doc_, startKeyframe(doc_), root);
    doc_.touch();
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("All joints of %s back to their reference positions", doc_.nameOf(root).c_str());
  ImGui::SameLine();
  if (ImGui::Button("Left > right")) {
    checkpoint("Mirror pose");
    int n = mirrorKeyPose(doc_, startKeyframe(doc_), root, true);
    doc_.touch();
    notify(n ? "Copied " + std::to_string(n) + " joint values to the right side" : "No left/right joint pairs found (names like leg_left / leg_right)",
           n ? pal::kOk : pal::kWarning, 3);
  }
  ImGui::SameLine();
  if (ImGui::Button("Right > left")) {
    checkpoint("Mirror pose");
    int n = mirrorKeyPose(doc_, startKeyframe(doc_), root, false);
    doc_.touch();
    notify(n ? "Copied " + std::to_string(n) + " joint values to the left side" : "No left/right joint pairs found (names like leg_left / leg_right)",
           n ? pal::kOk : pal::kWarning, 3);
  }
  bool floating = false;
  for (const Joint& j : doc_.joints) floating |= j.body == root && j.type == JointType::Free;
  if (floating) {
    if (ImGui::Button("Place on floor")) {
      checkpoint("Place on floor");
      double floorZ = 0;
      placeOnFloor(doc_, root, floorZ);
    }
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Lowers or raises the start pose until the lowest colliding geom touches z = 0");
    ImGui::SameLine();
    ImGui::TextDisabled("W / E: move or turn it");
  }
  ImGui::TextWrapped("Simulations and exports start from this pose; position servos are set to hold it.");
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

}  // namespace mjf
