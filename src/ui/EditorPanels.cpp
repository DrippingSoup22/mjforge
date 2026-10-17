#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unordered_set>

#include <imgui.h>
#include <imgui_stdlib.h>

#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/MjcfDocs.h"
#include "ui/Theme.h"

namespace mjf {

namespace {
const char* D(const char* key) { return mjcfDoc(key); }
props::Opt O(const char* tip, const char* unit = nullptr, double speed = 0.01, double mn = -1e300, double mx = 1e300,
             const char* fmt = "%.4g") {
  props::Opt o;
  o.tip = tip;
  o.unit = unit;
  o.speed = speed;
  o.min = mn;
  o.max = mx;
  o.fmt = fmt;
  return o;
}
std::string rangeStr(bool limited, const Vec2& r) {
  if (!limited) return "-";
  char b[64];
  std::snprintf(b, sizeof b, "[%.3g, %.3g]", r.x, r.y);
  return b;
}
}  // namespace

// ------------------------------------------------------------------------------------ actuators

void Editor::drawActuatorsPanel() {
  if (!ImGui::Begin("Actuators")) {
    ImGui::End();
    return;
  }
  std::unordered_set<Id> actuated;
  for (const Actuator& a : doc_.actuators) if (a.trn == TrnType::Joint) actuated.insert(a.target);
  std::vector<Id> passive;
  for (const Joint& j : doc_.joints) if (j.type != JointType::Free && !actuated.count(j.id)) passive.push_back(j.id);
  ImGui::BeginDisabled(passive.empty());
  if (ImGui::Button("Actuate all passive joints")) ImGui::OpenPopup("actall");
  ImGui::EndDisabled();
  if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
    ImGui::SetTooltip("Adds one actuator to each of the %zu joints that have none (free joints excluded).", passive.size());
  if (ImGui::BeginPopup("actall")) {
    for (const auto& e : kActuatorTypes) if (ImGui::MenuItem(e.label)) addActuatorsFor(passive, e.value);
    ImGui::EndPopup();
  }
  ImGui::SameLine();
  ImGui::TextDisabled("%zu actuators, %zu passive joints", doc_.actuators.size(), passive.size());

  ImGuiTableFlags fl = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable |
                       ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
  if (ImGui::BeginTable("##acts", 8, fl)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Name", 0, 1.6f);
    ImGui::TableSetupColumn("Type", 0, 0.9f);
    ImGui::TableSetupColumn("Target", 0, 1.3f);
    ImGui::TableSetupColumn("Gear", 0, 0.6f);
    ImGui::TableSetupColumn("Control range", 0, 1.1f);
    ImGui::TableSetupColumn("Force range", 0, 1.1f);
    ImGui::TableSetupColumn("kp", 0, 0.6f);
    ImGui::TableSetupColumn("kv / damping ratio", 0, 0.9f);
    ImGui::TableHeadersRow();
    for (Actuator& a : doc_.actuators) {
      ImGui::TableNextRow();
      ImGui::PushID(int(a.id));
      ImGui::TableSetColumnIndex(0);
      bool sel = sel_.contains(a.id);
      if (ImGui::Selectable(doc_.displayName(a.id).c_str(), sel, ImGuiSelectableFlags_SpanAllColumns))
        select(a.id, ImGui::GetIO().KeyShift, ImGui::GetIO().KeyCtrl);
      if (ImGui::BeginPopupContextItem("##ctx")) {
        if (ImGui::MenuItem("Delete")) {
          sel_.set(a.id);
          deleteSelection();
        }
        ImGui::EndPopup();
      }
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(toXml(a.type));
      ImGui::TableSetColumnIndex(2);
      std::string t = doc_.nameOf(a.target);
      if (a.target == kNoId || !doc_.exists(a.target)) ImGui::TextColored(pal::v4(pal::kError), "(missing)");
      else ImGui::TextUnformatted(t.c_str());
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%.3g", a.gear[0]);
      ImGui::TableSetColumnIndex(4);
      ImGui::TextUnformatted(rangeStr(a.ctrllimited, a.ctrlrange).c_str());
      ImGui::TableSetColumnIndex(5);
      ImGui::TextUnformatted(rangeStr(a.forcelimited, a.forcerange).c_str());
      ImGui::TableSetColumnIndex(6);
      if (a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity) ImGui::Text("%.4g", a.kp);
      else ImGui::TextDisabled("-");
      ImGui::TableSetColumnIndex(7);
      if (a.dampratio > 0) ImGui::Text("ratio %.3g", a.dampratio);
      else if (a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity || a.type == ActuatorType::Velocity ||
               a.type == ActuatorType::Damper)
        ImGui::Text("%.4g", a.kv);
      else ImGui::TextDisabled("-");
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

// ------------------------------------------------------------------------------------ sensors

void Editor::drawSensorsPanel() {
  if (!ImGui::Begin("Sensors")) {
    ImGui::End();
    return;
  }
  if (ImGui::Button("Add joint position + velocity sensors")) {
    checkpoint("Add joint sensors");
    int n = 0;
    for (const Joint& j : std::vector<Joint>(doc_.joints)) {
      if (j.type == JointType::Free) continue;
      addSensor(doc_, SensorType::JointPos, j.id);
      addSensor(doc_, SensorType::JointVel, j.id);
      n += 2;
    }
    notify("Added " + std::to_string(n) + " sensors", pal::kOk, 2);
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Policies often read joint state directly from qpos/qvel; sensors make it explicit and add noise options.");
  ImGui::SameLine();
  ImGui::TextDisabled("%zu sensors", doc_.sensors.size());
  int total = 0;
  for (const Sensor& s : doc_.sensors) total += sensorDim(s);
  ImGui::SameLine();
  ImGui::TextDisabled("(%d values in sensordata)", total);
  ImGuiTableFlags fl = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_Resizable |
                       ImGuiTableFlags_ScrollY | ImGuiTableFlags_SizingStretchProp;
  if (ImGui::BeginTable("##sens", 5, fl)) {
    ImGui::TableSetupScrollFreeze(0, 1);
    ImGui::TableSetupColumn("Name", 0, 1.6f);
    ImGui::TableSetupColumn("Type", 0, 1.2f);
    ImGui::TableSetupColumn("Attached to", 0, 1.4f);
    ImGui::TableSetupColumn("Dim", 0, 0.4f);
    ImGui::TableSetupColumn("Noise", 0, 0.5f);
    ImGui::TableHeadersRow();
    for (const Sensor& s : doc_.sensors) {
      const SensorInfo& info = sensorInfo(s.type);
      ImGui::TableNextRow();
      ImGui::PushID(int(s.id));
      ImGui::TableSetColumnIndex(0);
      if (ImGui::Selectable(doc_.displayName(s.id).c_str(), sel_.contains(s.id), ImGuiSelectableFlags_SpanAllColumns))
        select(s.id, ImGui::GetIO().KeyShift, ImGui::GetIO().KeyCtrl);
      if (ImGui::BeginPopupContextItem("##ctx")) {
        if (ImGui::MenuItem("Delete")) {
          sel_.set(s.id);
          deleteSelection();
        }
        ImGui::EndPopup();
      }
      ImGui::TableSetColumnIndex(1);
      ImGui::TextUnformatted(info.xml);
      ImGui::TableSetColumnIndex(2);
      if (info.obj == SensorObj::None) ImGui::TextDisabled("-");
      else if (info.obj == SensorObj::Contact && !s.obj && !s.ref) ImGui::TextDisabled("(any contact)");
      else if (s.obj && !doc_.exists(s.obj)) ImGui::TextColored(pal::v4(pal::kError), "(missing)");
      else if (s.ref) ImGui::Text("%s / %s", s.obj ? doc_.displayName(s.obj).c_str() : "any", doc_.displayName(s.ref).c_str());
      else ImGui::TextUnformatted(doc_.displayName(s.obj).c_str());
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%d", sensorDim(s));
      ImGui::TableSetColumnIndex(4);
      ImGui::Text("%.3g", s.noise);
      ImGui::PopID();
    }
    ImGui::EndTable();
  }
  ImGui::End();
}

// ------------------------------------------------------------------------------------ issues

void Editor::drawIssuesPanel() {
  bool upToDate = compiledRevision_ == editRevision_;
  int nerr = report_.counts[int(Severity::Error)] + ((upToDate && !compiled_.ok) ? 1 : 0);
  int nwarn = report_.counts[int(Severity::Warning)];
  char title[64];
  if (nerr + nwarn > 0) std::snprintf(title, sizeof title, "Issues (%d)###Issues", nerr + nwarn);
  else std::snprintf(title, sizeof title, "Issues###Issues");
  if (!ImGui::Begin(title)) {
    ImGui::End();
    return;
  }
  // Filter toggles.
  const Icon icons[4] = {Icon::Error, Icon::Warning, Icon::Info, Icon::Info};
  const ImU32 cols[4] = {pal::kError, pal::kWarning, pal::kInfo, pal::kTeal};
  const char* names[4] = {"Errors", "Warnings", "Info", "Tips"};
  for (int i = 0; i < 4; ++i) {
    if (i) ImGui::SameLine();
    int n = report_.counts[i] + (i == 0 ? ((upToDate && !compiled_.ok) ? 1 : 0) : 0);
    char lbl[48];
    std::snprintf(lbl, sizeof lbl, "%s %d##f%d", names[i], n, i);
    ImGui::PushStyleColor(ImGuiCol_Button, issueFilter_[i] ? ImGui::GetStyleColorVec4(ImGuiCol_ButtonActive) : ImVec4(0, 0, 0, 0));
    ImVec2 p = ImGui::GetCursorScreenPos();
    float h = ImGui::GetFrameHeight();
    ImGui::SetCursorScreenPos(ImVec2(p.x + h * 0.9f, p.y));
    if (ImGui::Button(lbl)) issueFilter_[i] = !issueFilter_[i];
    ImGui::PopStyleColor();
    drawIcon(ImGui::GetWindowDrawList(), ImVec2(p.x + h * 0.45f, p.y + h * 0.5f), h * 0.6f, icons[i], cols[i]);
  }
  ImGui::SameLine(0, 20);
  if (upToDate && compiled_.ok) {
    const mjModel* m = compiled_.model.get();
    ImGui::TextDisabled("MuJoCo %s: %d bodies, %d DoF, %d actuators, %.3g kg.  %d legs detected.  %d contact pairs (%d self), "
                        "%d MJWarp broadphase pairs at rest.",
                        mj_versionString(), int(m->nbody) - 1, int(m->nv), int(m->nu), m->body_subtreemass[0],
                        report_.legCount, report_.collisionPairs, report_.selfCollisionPairs, report_.broadphaseAtRest);
  } else if (!upToDate) {
    ImGui::TextDisabled("Checking with MuJoCo...");
  }
  ImGui::Separator();
  ImGui::BeginChild("##issues");
  if (upToDate && !compiled_.ok) {
    iconInline(Icon::Error, pal::kError);
    ImGui::SameLine();
    ImGui::TextColored(pal::v4(pal::kError), "MuJoCo cannot compile this model");
    ImGui::Indent(ImGui::GetTextLineHeight() + 8);
    std::string hint = explainCompileError(compiled_.error);
    if (!hint.empty()) ImGui::TextWrapped("%s", hint.c_str());
    ImGui::PushStyleColor(ImGuiCol_Text, pal::v4(pal::kTextDim));
    ImGui::TextWrapped("%s", compiled_.error.c_str());
    ImGui::PopStyleColor();
    ImGui::Unindent(ImGui::GetTextLineHeight() + 8);
    ImGui::Separator();
  }
  int shown = 0;
  for (size_t idx = 0; idx < report_.issues.size(); ++idx) {
    const Issue& is = report_.issues[idx];
    int sv = int(is.severity);
    if (!issueFilter_[sv]) continue;
    ++shown;
    ImGui::PushID(int(idx));
    iconInline(icons[sv], cols[sv]);
    ImGui::SameLine();
    ImGui::TextUnformatted(is.title.c_str());
    ImGui::SameLine();
    ImGui::TextDisabled("  %s", areaName(is.area));
    if (is.fix) {
      ImGui::SameLine();
      float w = ImGui::CalcTextSize(is.fixLabel.c_str()).x + ImGui::GetStyle().FramePadding.x * 2;
      float x = ImGui::GetWindowContentRegionMax().x - w;
      if (ImGui::GetCursorPosX() < x) ImGui::SetCursorPosX(x);
      ImGui::PushStyleColor(ImGuiCol_Button, pal::v4(pal::kAccentDim));
      if (ImGui::SmallButton(is.fixLabel.c_str())) {
        checkpoint(is.fixLabel.c_str());
        auto fix = is.fix;  // the report is rebuilt after the change
        fix(doc_);
        doc_.touch();
        markDirty();
        notify("Applied: " + is.fixLabel, pal::kOk, 2.5);
        ImGui::PopStyleColor();
        ImGui::PopID();
        break;
      }
      ImGui::PopStyleColor();
    }
    ImGui::Indent(ImGui::GetTextLineHeight() + 8);
    if (!is.detail.empty()) {
      ImGui::PushStyleColor(ImGuiCol_Text, pal::v4(pal::kTextDim));
      ImGui::TextWrapped("%s", is.detail.c_str());
      ImGui::PopStyleColor();
    }
    if (!is.elements.empty()) {
      int n = 0;
      for (Id id : is.elements) {
        if (!doc_.exists(id)) continue;
        if (n++ > 0) ImGui::SameLine();
        if (n > 8) {
          ImGui::TextDisabled("+%zu more", is.elements.size() - 8);
          break;
        }
        std::string nm = doc_.nameOf(id);
        std::string lbl = nm.empty() ? doc_.displayName(id) : std::string(kindName(doc_.kindOf(id))) + ": " + nm;
        ImGui::PushID(int(id));
        if (ImGui::SmallButton(lbl.c_str())) {
          sel_.set(id);
          focusSelection();
        }
        ImGui::PopID();
      }
    }
    ImGui::Unindent(ImGui::GetTextLineHeight() + 8);
    ImGui::Spacing();
    ImGui::PopID();
  }
  if (shown == 0 && (compiled_.ok || !upToDate)) {
    iconInline(Icon::Ok, pal::kOk);
    ImGui::SameLine();
    ImGui::TextDisabled("No issues to show.");
  }
  ImGui::EndChild();
  ImGui::End();
}

void Editor::drawStrength(Id jointId) {
  const JointReport* r = report_.joint(jointId);
  if (!r) return;
  ImGui::Spacing();
  ImGui::SeparatorText("Physics check");
  const char* tu = r->angular ? "N*m" : "N";
  double need = r->required();
  if (!r->actuated) {
    ImGui::TextDisabled("Passive joint: no actuator. Load to lift its subtree: %.3g %s.", r->liftLoad, tu);
  } else {
    // Capacity vs need bar.
    ImGui::Text("Actuator capacity");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.45f);
    if (r->unbounded) ImGui::TextColored(pal::v4(pal::kWarning), "unbounded");
    else ImGui::Text("%.3g %s", r->capacity, tu);
    ImGui::Text("Needed (estimate)");
    ImGui::SameLine(ImGui::GetContentRegionAvail().x * 0.45f);
    ImGui::Text("%.3g %s", need, tu);
    if (ImGui::IsItemHovered()) {
      ImGui::SetTooltip("Lift: %.3g %s to raise the %.3g kg subtree against gravity (worst case).\n"
                        "Support: %.3g %s to carry the body while walking (legs only; ~half the feet on the ground).",
                        r->liftLoad, tu, r->movedMass, r->supportLoad, tu);
    }
    if (need > 0 && !r->unbounded) {
      double ratio = r->capacity / need;
      float w = ImGui::GetContentRegionAvail().x, h = ImGui::GetTextLineHeight() * 0.7f;
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImDrawList* dl = ImGui::GetWindowDrawList();
      float fill = float(std::min(ratio / 3.0, 1.0));
      ImU32 c = ratio < 0.25 ? pal::kError : ratio < 1.0 ? pal::kWarning : pal::kOk;
      dl->AddRectFilled(p, ImVec2(p.x + w, p.y + h), IM_COL32(255, 255, 255, 20), 3);
      dl->AddRectFilled(p, ImVec2(p.x + w * fill, p.y + h), c, 3);
      float one = p.x + w / 3.0f;
      dl->AddLine(ImVec2(one, p.y - 2), ImVec2(one, p.y + h + 2), IM_COL32(230, 230, 240, 200), 2);
      ImGui::Dummy(ImVec2(w, h + 2));
      if (ratio < 0.001) ImGui::TextColored(pal::v4(c), "under 0.1%% of the estimated need");
      else if (ratio < 1.0) ImGui::TextColored(pal::v4(c), "%.0f%% of the estimated need", ratio * 100);
      else ImGui::TextColored(pal::v4(c), "%.1fx the estimated need", ratio);
    }
    if (r->holdLoad > 0 && !r->unbounded && r->capacity < r->holdLoad)
      ImGui::TextColored(pal::v4(pal::kError), "Saturates: cannot even hold the body at rest");
    else if (r->sag > 0 && r->stiffness > 0)
      ImGui::TextDisabled("Servo sag under body weight: %.2f %s", r->angular ? rad2deg(r->sag) : r->sag * 100, r->angular ? "deg" : "cm");
  }
  if (r->stiffness > 0) {
    ImU32 c = r->omegaDt > 2 ? pal::kError : r->omegaDt > 0.6 ? pal::kWarning : pal::kOk;
    ImGui::TextColored(pal::v4(c), "Stiffness margin: w*dt = %.2f", r->omegaDt);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Natural frequency times timestep. Below ~0.5 is comfortable; above 2 explicit integration "
                        "diverges. Raise armature or lower kp if it is high.");
  }
  ImGui::TextDisabled("Moves %.3g kg, effective inertia %.3g %s", r->movedMass, r->inertia, r->angular ? "kg*m^2" : "kg");
}

// ------------------------------------------------------------------------------------ MJCF

void Editor::drawXmlPanel() {
  if (!ImGui::Begin("MJCF")) {
    ImGui::End();
    return;
  }
  if (xmlPreviewRevision_ != editRevision_) {
    xmlPreview_ = writeMjcf(doc_);
    xmlPreviewRevision_ = editRevision_;
  }
  if (ImGui::Button("Copy")) {
    ImGui::SetClipboardText(xmlPreview_.c_str());
    notify("MJCF copied to clipboard", pal::kOk, 1.5);
  }
  ImGui::SameLine();
  if (ImGui::Button("Export...")) exportDialog();
  ImGui::SameLine();
  ImGui::TextDisabled("%zu lines, %.1f KB - live preview of the single-file export", size_t(std::count(xmlPreview_.begin(), xmlPreview_.end(), '\n')),
                      xmlPreview_.size() / 1024.0);
  ImGui::PushFont(codeFont(), 0.0f);
  ImGui::InputTextMultiline("##xml", &xmlPreview_, ImVec2(-1, -1), ImGuiInputTextFlags_ReadOnly);
  ImGui::PopFont();
  ImGui::End();
}

// ------------------------------------------------------------------------------------ scene

void Editor::drawScenePanel() {
  if (!ImGui::Begin("Scene")) {
    ImGui::End();
    return;
  }
  OptionSettings& o = doc_.option;
  const OptionSettings od;
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("Model name");
  ImGui::SameLine();
  ImGui::SetNextItemWidth(-1);
  std::string name = doc_.modelName;
  if (ImGui::InputText("##modelname", &name)) {
    change();
    doc_.modelName = name;
  }
  if (props::begin("##scene")) {
    if (props::section("Simulation")) {
      props::real("Timestep", &o.timestep, od.timestep, O(D("option.timestep"), "s", 0.0001, 1e-5, 0.1, "%.4g"));
      props::combo("Integrator", &o.integrator, kIntegrators, od.integrator, D("option.integrator"));
      props::combo("Solver", &o.solver, kSolvers, od.solver, D("option.solver"));
      props::integer("Iterations", &o.iterations, od.iterations, O(D("option.iterations"), nullptr, 0.2, 1, 10000));
      props::integer("Line-search iter.", &o.ls_iterations, od.ls_iterations, O(D("option.ls_iterations"), nullptr, 0.2, 0, 10000));
      props::real("Tolerance", &o.tolerance, od.tolerance, O(D("option.tolerance"), nullptr, 1e-9, 0, 1, "%.2g"));
      props::combo("Friction cone", &o.cone, kCones, od.cone, D("option.cone"));
      props::real("Impedance ratio", &o.impratio, od.impratio, O(D("option.impratio"), nullptr, 0.05, 1e-4, 1000));
      props::combo("Jacobian", &o.jacobian, kJacobians, od.jacobian, D("option.jacobian"));
      if (o.sleep) props::real("Sleep tolerance", &o.sleep_tolerance, od.sleep_tolerance, O(D("option.sleep_tolerance"), "m/s", 1e-4, 0, 10, "%.3g"));
      props::text("Arena memory", &o.memory, D("size.memory"));
    }
    if (props::section("Solver details", false)) {
      props::real("Line-search tol.", &o.ls_tolerance, od.ls_tolerance, O(D("option.ls_tolerance"), nullptr, 1e-4, 0, 1, "%.3g"));
      props::integer("CCD iterations", &o.ccd_iterations, od.ccd_iterations, O(D("option.ccd_iterations"), nullptr, 0.2, 0, 10000));
      props::real("CCD tolerance", &o.ccd_tolerance, od.ccd_tolerance, O(D("option.ccd_tolerance"), nullptr, 1e-7, 0, 1, "%.3g"));
      props::integer("Noslip iterations", &o.noslip_iterations, od.noslip_iterations,
                     O(D("option.noslip_iterations"), nullptr, 0.2, 0, 10000));
      if (o.noslip_iterations > 0)
        props::real("Noslip tolerance", &o.noslip_tolerance, od.noslip_tolerance, O(D("option.noslip_tolerance"), nullptr, 1e-7, 0, 1, "%.3g"));
      props::vec3("Magnetic field", &o.magnetic, od.magnetic, O(D("option.magnetic"), "T", 0.01));
      props::checkbox("Override contacts", &o.override_, false, D("flag.override"));
      if (o.override_) {
        props::real("Override margin", &o.o_margin, od.o_margin, O(D("option.o_margin"), "m", 0.0005, 0, 10));
        props::vec2("Override solref", &o.o_solref, od.o_solref, O(D("option.o_solref"), nullptr, 0.001));
        props::vec5("Override solimp", &o.o_solimp, od.o_solimp, O(D("option.o_solimp"), nullptr, 0.001));
        props::vec5("Override friction", &o.o_friction, od.o_friction, O(D("option.o_friction"), nullptr, 0.001, 0, 100));
      }
    }
    if (props::section("Actuator groups", o.disableactuator != 0)) {
      for (int g = 0; g < 6; ++g) {
        bool on = (o.disableactuator & (1 << g)) == 0;
        std::string label = "Group " + std::to_string(g) + " active";
        if (props::checkbox(label.c_str(), &on, true, D("option.actuatorgroupdisable")))
          o.disableactuator = on ? o.disableactuator & ~(1 << g) : o.disableactuator | (1 << g);
      }
    }
    if (props::section("Environment")) {
      props::vec3("Gravity", &o.gravity, od.gravity, O(D("option.gravity"), "m/s^2", 0.01));
      props::vec3("Wind", &o.wind, od.wind, O(D("option.wind"), "m/s", 0.01));
      props::real("Medium density", &o.density, od.density, O(D("option.density"), "kg/m^3", 0.01, 0, 1e5));
      props::real("Medium viscosity", &o.viscosity, od.viscosity, O(D("option.viscosity"), "Pa*s", 0.0001, 0, 1e5));
    }
    if (props::section("Flags", false)) {
      props::checkbox("Contacts", &o.contact, true, D("flag.contact"));
      props::checkbox("Gravity", &o.gravityFlag, true, D("flag.gravity"));
      props::checkbox("Joint limits", &o.limit, true, D("flag.limit"));
      props::checkbox("Equality constraints", &o.equality, true, D("flag.equality"));
      props::checkbox("Friction loss", &o.frictionloss, true, D("flag.frictionloss"));
      props::checkbox("Springs", &o.spring, true, D("flag.spring"));
      props::checkbox("Dampers", &o.damper, true, D("flag.damper"));
      props::checkbox("Actuation", &o.actuation, true, D("flag.actuation"));
      props::checkbox("Clamp controls", &o.clampctrl, true, D("flag.clampctrl"));
      props::checkbox("Warm start", &o.warmstart, true, D("flag.warmstart"));
      props::checkbox("Filter parent contacts", &o.filterparent, true, D("flag.filterparent"));
      props::checkbox("Euler implicit damping", &o.eulerdamp, true, D("flag.eulerdamp"));
      props::checkbox("Multi-contact CCD", &o.multiccd, true, D("flag.multiccd"));
      props::checkbox("Native CCD", &o.nativeccd, true, D("flag.nativeccd"));
      props::checkbox("Energy", &o.energy, false, D("flag.energy"));
      props::checkbox("Sleep", &o.sleep, false, D("flag.sleep"));
    }
    if (props::section("Compiler", false)) {
      CompilerSettings& c = doc_.compiler;
      props::checkbox("Auto limits", &c.autolimits, true, D("compiler.autolimits"));
      props::combo("Inertia from geoms", &c.inertiafromgeom, kInertiaFromGeom, InertiaFromGeom::Auto, D("compiler.inertiafromgeom"));
      props::real("Bound mass", &c.boundmass, 0, O(D("compiler.boundmass"), "kg", 0.001, 0, 1e6));
      props::real("Bound inertia", &c.boundinertia, 0, O(D("compiler.boundinertia"), "kg*m^2", 1e-5, 0, 1e6));
      props::real("Set total mass", &c.settotalmass, -1, O(D("compiler.settotalmass"), "kg", 0.1, -1, 1e6));
      props::checkbox("Balance inertia", &c.balanceinertia, false, D("compiler.balanceinertia"));
    }
    if (props::section("Visual (MuJoCo viewers)", false)) {
      VisualSettings& v = doc_.visual;
      const VisualSettings vd;
      props::color3("Headlight ambient", &v.headlightAmbient, vd.headlightAmbient, D("headlight.ambient"));
      props::color3("Headlight diffuse", &v.headlightDiffuse, vd.headlightDiffuse, D("headlight.diffuse"));
      props::color3("Headlight specular", &v.headlightSpecular, vd.headlightSpecular, D("headlight.specular"));
      props::real("Camera azimuth", &v.azimuth, vd.azimuth, O(D("global.azimuth"), "deg", 0.5, -360, 360));
      props::real("Camera elevation", &v.elevation, vd.elevation, O(D("global.elevation"), "deg", 0.5, -90, 90));
      props::integer("Offscreen width", &v.offwidth, vd.offwidth, O(D("global.offwidth"), "px", 1, 1, 16384));
      props::integer("Offscreen height", &v.offheight, vd.offheight, O(D("global.offheight"), "px", 1, 1, 16384));
      props::integer("Shadow map size", &v.shadowsize, vd.shadowsize, O(D("quality.shadowsize"), "px", 16, 0, 16384));
      props::color("Haze", &v.haze, vd.haze, "Color of the horizon haze in MuJoCo's renderer.");
    }
    props::end();
  }
  ImGui::Spacing();
  ImGui::SeparatorText("Presets");
  if (ImGui::Button("Stable default")) {
    checkpoint("Physics preset");
    o.timestep = 0.002;
    o.integrator = Integrator::ImplicitFast;
    o.solver = Solver::Newton;
    o.iterations = 100;
    o.ls_iterations = 50;
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("2 ms step, implicitfast integrator, Newton solver (MuJoCo defaults otherwise).");
  ImGui::SameLine();
  if (ImGui::Button("RL training (MJWarp)")) {
    checkpoint("Physics preset");
    o.timestep = 0.004;
    o.integrator = Integrator::ImplicitFast;
    o.solver = Solver::Newton;
    o.iterations = 20;
    o.ls_iterations = 20;
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("4 ms step, implicitfast, Newton with modest iteration caps. MJWarp exits early once all worlds "
                      "converge, so caps matter less than nconmax/njmax. Check stability in the preview.");

  ImGui::SeparatorText("Keyframes");
  if (doc_.keyframes.empty()) {
    ImGui::TextWrapped("None: simulations start from the reference pose. Set a start pose in pose mode (P), or "
                       "simulate and press 'Save pose as keyframe' in the Simulation panel.");
    if (ImGui::Button("Pose mode")) setPoseMode(true);
  }
  for (size_t k = 0; k < doc_.keyframes.size(); ++k) {
    ImGui::PushID(int(k));
    Keyframe& kf = doc_.keyframes[k];
    ImGui::SetNextItemWidth(150 * scale_);
    std::string kname = kf.name;
    if (ImGui::InputText("##kname", &kname)) {
      change();
      kf.name = kname;
    }
    ImGui::SameLine();
    if (k == 0) {
      ImGui::TextColored(pal::v4(pal::kOk), "start pose");
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Simulations (and the analysis) start here. Training code usually resets to keyframe 0 too:\n"
                          "mujoco.mj_resetDataKeyframe(m, d, 0)");
    } else if (ImGui::SmallButton("Make start pose")) {
      checkpoint("Reorder keyframes");
      std::rotate(doc_.keyframes.begin(), doc_.keyframes.begin() + k, doc_.keyframes.begin() + k + 1);
      ImGui::PopID();
      break;
    }
    ImGui::SameLine();
    if (ImGui::SmallButton("Delete")) {
      checkpoint("Delete keyframe");
      doc_.keyframes.erase(doc_.keyframes.begin() + k);
      ImGui::PopID();
      break;
    }
    ImGui::SameLine();
    ImGui::TextDisabled("%zu joints%s", kf.qpos.size(), kf.ctrl.empty() ? "" : ", controls");
    ImGui::PopID();
  }
  ImGui::End();
}

}  // namespace mjf
