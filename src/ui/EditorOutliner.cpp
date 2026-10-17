#include <algorithm>
#include <cstring>

#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_stdlib.h>

#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace mjf {

namespace {

const char* kPayload = "MJF_ELEMENTS";

Icon geomIcon(GeomType t) {
  switch (t) {
    case GeomType::Plane: return Icon::Plane;
    case GeomType::Hfield: return Icon::Hfield;
    case GeomType::Sphere: return Icon::Sphere;
    case GeomType::Capsule: return Icon::Capsule;
    case GeomType::Ellipsoid: return Icon::Ellipsoid;
    case GeomType::Cylinder: return Icon::Cylinder;
    default: return Icon::Box;
  }
}

Icon jointIcon(JointType t) {
  switch (t) {
    case JointType::Free: return Icon::Free;
    case JointType::Ball: return Icon::Ball;
    case JointType::Slide: return Icon::Slide;
    default: return Icon::Hinge;
  }
}

}  // namespace

static void iconAt(ImVec2 itemMin, float offsetX, Icon icon, ImU32 col) {
  float h = ImGui::GetTextLineHeight();
  ImVec2 c(itemMin.x + offsetX + h * 0.5f, itemMin.y + ImGui::GetStyle().FramePadding.y + h * 0.5f);
  drawIcon(ImGui::GetWindowDrawList(), c, h * 0.85f, icon, col);
}

void Editor::drawElementContextMenu(Id id) {
  Kind k = doc_.kindOf(id);
  if (!sel_.contains(id)) sel_.set(id);
  ImGui::TextDisabled("%s %s", kindName(k), doc_.nameOf(id).c_str());
  ImGui::Separator();
  if (k == Kind::Body) {
    if (ImGui::BeginMenu("Add")) {
      drawAddMenuItems(id);
      ImGui::EndMenu();
    }
    std::vector<Id> joints;
    int bi = doc_.bodyIndex(id);
    for (uint32_t ji : doc_.index().bodyJoints[bi]) joints.push_back(doc_.joints[ji].id);
    if (ImGui::BeginMenu("Add actuators to this body's joints", !joints.empty())) {
      for (const auto& e : kActuatorTypes) if (ImGui::MenuItem(e.label)) addActuatorsFor(joints, e.value);
      ImGui::EndMenu();
    }
    if (id != kWorldId) {
      if (ImGui::BeginMenu("Mirror subtree")) {
        if (ImGui::MenuItem("Left / right (across XZ)")) mirrorSelection(1);
        if (ImGui::MenuItem("Front / back (across YZ)")) mirrorSelection(0);
        if (ImGui::MenuItem("Up / down (across XY)")) mirrorSelection(2);
        ImGui::EndMenu();
      }
      Body* b = doc_.body(id);
      if (b->parent == kWorldId && ImGui::BeginMenu("Role")) {
        if (ImGui::MenuItem("Model (creature / robot)", nullptr, b->role == Role::Model)) {
          checkpoint("Set role");
          b->role = Role::Model;
        }
        if (ImGui::MenuItem("Environment (prop)", nullptr, b->role == Role::Environment)) {
          checkpoint("Set role");
          b->role = Role::Environment;
        }
        ImGui::EndMenu();
      }
    }
  }
  if (k == Kind::Joint) {
    if (ImGui::BeginMenu("Add actuator")) {
      for (const auto& e : kActuatorTypes) if (ImGui::MenuItem(e.label)) addActuatorsFor({id}, e.value);
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Add sensor")) {
      for (SensorType t : {SensorType::JointPos, SensorType::JointVel, SensorType::JointActuatorFrc, SensorType::JointLimitFrc}) {
        if (ImGui::MenuItem(sensorInfo(t).label)) {
          checkpoint("Add sensor");
          sel_.set(addSensor(doc_, t, id));
        }
      }
      ImGui::EndMenu();
    }
  }
  if (k == Kind::Site && ImGui::BeginMenu("Add sensor")) {
    for (const auto& s : kSensorTypes) {
      if (s.obj != SensorObj::Site && s.obj != SensorObj::Frame) continue;
      if (ImGui::MenuItem(s.label)) {
        checkpoint("Add sensor");
        sel_.set(addSensor(doc_, s.value, id));
      }
    }
    ImGui::EndMenu();
  }
  if (k == Kind::Body || k == Kind::Geom || k == Kind::Site || k == Kind::Joint || k == Kind::Camera || k == Kind::Light) {
    if (id != kWorldId) {
      if (ImGui::MenuItem("Duplicate", "Ctrl+D")) duplicateSelection();
      if (ImGui::MenuItem("Frame in viewport", "F")) focusSelection();
    }
  }
  if (id != kWorldId) {
    if (ImGui::MenuItem("Rename", "F2")) {
      renameId_ = id;
      renameBuf_ = doc_.nameOf(id);
      focusRename_ = true;
    }
    if (k == Kind::Body || k == Kind::Geom) {
      bool hidden = k == Kind::Body ? doc_.body(id)->hidden : doc_.geom(id)->hidden;
      if (ImGui::MenuItem(hidden ? "Show" : "Hide")) {
        if (k == Kind::Body) doc_.body(id)->hidden = !hidden;
        else doc_.geom(id)->hidden = !hidden;
      }
      bool locked = k == Kind::Body ? doc_.body(id)->locked : doc_.geom(id)->locked;
      if (ImGui::MenuItem("Lock (not pickable in the viewport)", nullptr, locked)) {
        if (k == Kind::Body) doc_.body(id)->locked = !locked;
        else doc_.geom(id)->locked = !locked;
      }
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Delete", "Del")) deleteSelection();
  }
}

bool Editor::outlinerDragDrop(Id target) {
  bool dropped = false;
  if (ImGui::BeginDragDropTarget()) {
    if (const ImGuiPayload* p = ImGui::AcceptDragDropPayload(kPayload)) {
      int n = p->DataSize / int(sizeof(Id));
      const Id* ids = static_cast<const Id*>(p->Data);
      std::vector<Id> moving(ids, ids + n);
      bool any = false;
      for (Id id : moving) {
        Kind k = doc_.kindOf(id);
        if (k == Kind::Body ? canReparent(doc_, id, target) : (doc_.ownerBody(id) != target && doc_.exists(id))) {
          if (k == Kind::Joint && target == kWorldId) continue;
          if (!any) checkpoint("Reparent");
          any = true;
          if (k == Kind::Body) reparentBody(doc_, id, target, true);
          else moveToBody(doc_, id, target, true);
        }
      }
      if (any) markDirty();
      dropped = any;
    }
    ImGui::EndDragDropTarget();
  }
  return dropped;
}

void Editor::outlinerLeaf(Id id, const char* name, int iconIdx, unsigned int color) {
  ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_Leaf | ImGuiTreeNodeFlags_NoTreePushOnOpen | ImGuiTreeNodeFlags_SpanAvailWidth |
                          ImGuiTreeNodeFlags_FramePadding;
  if (sel_.contains(id)) fl |= ImGuiTreeNodeFlags_Selected;
  float h = ImGui::GetTextLineHeight();
  ImGui::PushID(int(id));
  if (renameId_ == id) {
    ImGui::SetNextItemWidth(-1);
    if (focusRename_) {
      ImGui::SetKeyboardFocusHere();
      focusRename_ = false;
    }
    if (ImGui::InputText("##rename", &renameBuf_, ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll)) {
      Kind k = doc_.kindOf(id);
      if (!renameBuf_.empty() && doc_.nameTaken(k, renameBuf_, id)) notify("Name already used: " + renameBuf_, pal::kWarning);
      else {
        checkpoint("Rename");
        doc_.setName(id, renameBuf_);
      }
      renameId_ = kNoId;
    }
    if (ImGui::IsItemDeactivated()) renameId_ = kNoId;
    ImGui::PopID();
    return;
  }
  std::string label = std::string("      ") + (name && *name ? name : "(unnamed)");
  ImGui::TreeNodeEx("##leaf", fl, "%s", label.c_str());
  ImVec2 mn = ImGui::GetItemRectMin();
  iconAt(mn, ImGui::GetTreeNodeToLabelSpacing() - h * 0.2f, Icon(iconIdx), color);
  if (ImGui::IsItemClicked(0) && !ImGui::IsItemToggledOpen()) select(id, ImGui::GetIO().KeyShift, ImGui::GetIO().KeyCtrl);
  if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) focusSelection();
  if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_F2)) {
    renameId_ = id;
    renameBuf_ = doc_.nameOf(id);
    focusRename_ = true;
  }
  if (ImGui::BeginDragDropSource()) {
    std::vector<Id> payload = sel_.contains(id) ? sel_.ids : std::vector<Id>{id};
    ImGui::SetDragDropPayload(kPayload, payload.data(), payload.size() * sizeof(Id));
    ImGui::Text("Move %s%s", name, payload.size() > 1 ? " (+ others)" : "");
    ImGui::EndDragDropSource();
  }
  if (ImGui::BeginPopupContextItem("##ctx")) {
    drawElementContextMenu(id);
    ImGui::EndPopup();
  }
  ImGui::PopID();
}

void Editor::outlinerBody(uint32_t bi, const std::string& filter) {
  const DocIndex& ix = doc_.index();
  const Body& b = doc_.bodies[bi];
  bool isWorld = b.id == kWorldId;
  bool hasChildren = !ix.childBodies[bi].empty() || !ix.bodyGeoms[bi].empty() || !ix.bodyJoints[bi].empty() ||
                     !ix.bodySites[bi].empty() || !ix.bodyCameras[bi].empty() || !ix.bodyLights[bi].empty();
  ImGuiTreeNodeFlags fl = ImGuiTreeNodeFlags_OpenOnArrow | ImGuiTreeNodeFlags_OpenOnDoubleClick |
                          ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_FramePadding;
  if (!hasChildren) fl |= ImGuiTreeNodeFlags_Leaf;
  if (sel_.contains(b.id)) fl |= ImGuiTreeNodeFlags_Selected;
  if (b.expanded || isWorld) fl |= ImGuiTreeNodeFlags_DefaultOpen;
  ImGui::PushID(int(b.id));
  float h = ImGui::GetTextLineHeight();
  bool open;
  if (renameId_ == b.id) {
    open = false;
    outlinerLeaf(b.id, b.name.c_str(), int(Icon::Body), pal::kBody);  // shows the rename field
  } else {
    std::string extra;
    if (!isWorld && b.parent == kWorldId) extra = b.role == Role::Model ? "   model" : "   environment";
    std::string label = std::string("      ") + (isWorld ? "World" : (b.name.empty() ? "(unnamed body)" : b.name));
    open = ImGui::TreeNodeEx("##body", fl, "%s", label.c_str());
    ImVec2 mn = ImGui::GetItemRectMin();
    ImVec2 mx = ImGui::GetItemRectMax();
    iconAt(mn, ImGui::GetTreeNodeToLabelSpacing() - h * 0.2f, isWorld ? Icon::World : Icon::Body,
           b.hidden ? pal::kTextDim : (isWorld ? pal::kTeal : pal::kBody));
    if (!extra.empty()) {
      ImVec2 ts = ImGui::CalcTextSize(label.c_str());
      ImGui::GetWindowDrawList()->AddText(ImVec2(mn.x + ImGui::GetTreeNodeToLabelSpacing() + ts.x, mn.y + ImGui::GetStyle().FramePadding.y),
                                          b.role == Role::Model ? pal::kAccentDim : IM_COL32(110, 160, 150, 200),
                                          extra.c_str());
    }
    if (ImGui::IsItemClicked(0) && !ImGui::IsItemToggledOpen()) select(b.id, ImGui::GetIO().KeyShift, ImGui::GetIO().KeyCtrl);
    if (ImGui::IsItemFocused() && ImGui::IsKeyPressed(ImGuiKey_F2) && !isWorld) {
      renameId_ = b.id;
      renameBuf_ = b.name;
      focusRename_ = true;
    }
    if (!isWorld && ImGui::BeginDragDropSource()) {
      std::vector<Id> payload = sel_.contains(b.id) ? sel_.ids : std::vector<Id>{b.id};
      ImGui::SetDragDropPayload(kPayload, payload.data(), payload.size() * sizeof(Id));
      ImGui::Text("Move %s", b.name.c_str());
      ImGui::EndDragDropSource();
    }
    outlinerDragDrop(b.id);
    if (ImGui::BeginPopupContextItem("##ctx")) {
      drawElementContextMenu(b.id);
      ImGui::EndPopup();
    }
    // Visibility toggle on the right edge.
    if (!isWorld) {
      ImVec2 save = ImGui::GetCursorScreenPos();
      float bs = ImGui::GetFrameHeight();
      ImGui::SetCursorScreenPos(ImVec2(mx.x - bs, mn.y));
      Body* mb = doc_.body(b.id);
      if (iconButton("##vis", mb->hidden ? Icon::EyeClosed : Icon::EyeOpen, false, mb->hidden ? "Show" : "Hide", bs,
                     mb->hidden ? pal::kTextDim : IM_COL32(150, 156, 168, 160)))
        mb->hidden = !mb->hidden;
      ImGui::SetCursorScreenPos(save);
    }
    if (!isWorld) doc_.body(b.id)->expanded = open;
  }
  if (open) {
    for (uint32_t ji : ix.bodyJoints[bi]) {
      const Joint& j = doc_.joints[ji];
      outlinerLeaf(j.id, j.name.c_str(), int(jointIcon(j.type)), pal::kJoint);
    }
    for (uint32_t gi : ix.bodyGeoms[bi]) {
      const Geom& g = doc_.geoms[gi];
      outlinerLeaf(g.id, g.name.c_str(), int(geomIcon(g.type)), g.hidden ? pal::kTextDim : pal::kGeom);
    }
    for (uint32_t si : ix.bodySites[bi]) outlinerLeaf(doc_.sites[si].id, doc_.sites[si].name.c_str(), int(Icon::Site), pal::kSite);
    for (uint32_t ci : ix.bodyCameras[bi]) outlinerLeaf(doc_.cameras[ci].id, doc_.cameras[ci].name.c_str(), int(Icon::Camera), pal::kCamera);
    for (uint32_t li : ix.bodyLights[bi]) outlinerLeaf(doc_.lights[li].id, doc_.lights[li].name.c_str(), int(Icon::Light), pal::kLight);
    for (uint32_t ci : ix.childBodies[bi]) outlinerBody(ci, filter);
    ImGui::TreePop();
  }
  ImGui::PopID();
}

void Editor::drawOutliner() {
  if (!ImGui::Begin("Outliner")) {
    ImGui::End();
    return;
  }
  static std::string filter;
  float addW = ImGui::GetFrameHeight();
  ImGui::SetNextItemWidth(-addW - ImGui::GetStyle().ItemSpacing.x);
  ImGui::InputTextWithHint("##filter", "Filter by name...", &filter);
  ImGui::SameLine();
  if (iconButton("##addmenu", Icon::Plus, false, "Add element")) ImGui::OpenPopup("##outliner_add");
  if (ImGui::BeginPopup("##outliner_add")) {
    drawAddMenuItems(targetBody());
    ImGui::EndPopup();
  }
  ImGui::BeginChild("##tree", ImVec2(0, 0), ImGuiChildFlags_None);
  ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, ImVec2(4, 2));
  ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, ImVec2(4, 1));
  if (!filter.empty()) {
    // Flat search results.
    std::string f = filter;
    std::transform(f.begin(), f.end(), f.begin(), ::tolower);
    auto match = [&](const std::string& n) {
      std::string l = n;
      std::transform(l.begin(), l.end(), l.begin(), ::tolower);
      return l.find(f) != std::string::npos;
    };
    for (const Body& b : doc_.bodies) if (b.id != kWorldId && match(b.name)) outlinerLeaf(b.id, b.name.c_str(), int(Icon::Body), pal::kBody);
    for (const Joint& j : doc_.joints) if (match(j.name)) outlinerLeaf(j.id, j.name.c_str(), int(jointIcon(j.type)), pal::kJoint);
    for (const Geom& g : doc_.geoms) if (match(g.name)) outlinerLeaf(g.id, g.name.c_str(), int(geomIcon(g.type)), pal::kGeom);
    for (const Site& s : doc_.sites) if (match(s.name)) outlinerLeaf(s.id, s.name.c_str(), int(Icon::Site), pal::kSite);
    for (const Actuator& a : doc_.actuators) if (match(a.name)) outlinerLeaf(a.id, a.name.c_str(), int(Icon::Actuator), pal::kActuator);
    for (const Sensor& s : doc_.sensors) if (match(s.name)) outlinerLeaf(s.id, s.name.c_str(), int(Icon::Sensor), pal::kSensor);
  } else {
    outlinerBody(0, filter);
    ImGui::Spacing();
    auto group = [&](const char* title, size_t count, auto&& body) {
      std::string t = std::string(title) + " (" + std::to_string(count) + ")";
      if (count == 0) return;
      if (ImGui::TreeNodeEx(title, ImGuiTreeNodeFlags_SpanAvailWidth | ImGuiTreeNodeFlags_FramePadding, "%s", t.c_str())) {
        body();
        ImGui::TreePop();
      }
    };
    group("Actuators", doc_.actuators.size(), [&] {
      for (const Actuator& a : doc_.actuators) outlinerLeaf(a.id, a.name.c_str(), int(Icon::Actuator), pal::kActuator);
    });
    group("Sensors", doc_.sensors.size(), [&] {
      for (const Sensor& s : doc_.sensors) outlinerLeaf(s.id, s.name.c_str(), int(Icon::Sensor), pal::kSensor);
    });
    group("Contact exclusions", doc_.excludes.size(), [&] {
      for (const Exclude& e : doc_.excludes) {
        std::string n = doc_.nameOf(e.body1) + " / " + doc_.nameOf(e.body2);
        outlinerLeaf(e.id, n.c_str(), int(Icon::Exclude), pal::kTextDim);
      }
    });
    group("Materials", doc_.materials.size(), [&] {
      for (const Material& m : doc_.materials) outlinerLeaf(m.id, m.name.c_str(), int(Icon::Material), pal::kTextDim);
    });
    group("Textures", doc_.textures.size(), [&] {
      for (const Texture& t : doc_.textures) outlinerLeaf(t.id, t.name.c_str(), int(Icon::Material), pal::kTextDim);
    });
  }
  ImGui::PopStyleVar(2);
  // Dropping on empty space moves things to the world.
  ImGui::Dummy(ImVec2(ImGui::GetContentRegionAvail().x, std::max(40.0f, ImGui::GetContentRegionAvail().y)));
  outlinerDragDrop(kWorldId);
  if (ImGui::IsItemClicked(0)) sel_.clear();
  ImGui::EndChild();
  ImGui::End();
}

}  // namespace mjf
