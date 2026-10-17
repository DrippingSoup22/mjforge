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
      if (ImGui::Selectable(a.name.empty() ? "(unnamed)" : a.name.c_str(), sel, ImGuiSelectableFlags_SpanAllColumns))
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
  for (const Sensor& s : doc_.sensors) total += sensorInfo(s.type).dim;
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
      if (ImGui::Selectable(s.name.empty() ? "(unnamed)" : s.name.c_str(), sel_.contains(s.id), ImGuiSelectableFlags_SpanAllColumns))
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
      else if (!doc_.exists(s.obj)) ImGui::TextColored(pal::v4(pal::kError), "(missing)");
      else ImGui::TextUnformatted(doc_.nameOf(s.obj).c_str());
      ImGui::TableSetColumnIndex(3);
      ImGui::Text("%d", info.dim);
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
  std::string title = "Issues###Issues";
  if (upToDate && !compiled_.ok) title = "Issues (1)###Issues";
  if (!ImGui::Begin(title.c_str())) {
    ImGui::End();
    return;
  }
  if (!upToDate) {
    ImGui::TextDisabled("Checking the model with MuJoCo...");
  } else if (!compiled_.ok) {
    iconInline(Icon::Error, pal::kError);
    ImGui::SameLine();
    ImGui::TextColored(pal::v4(pal::kError), "MuJoCo cannot compile this model");
    ImGui::PushTextWrapPos(0);
    ImGui::TextUnformatted(compiled_.error.c_str());
    ImGui::PopTextWrapPos();
  } else {
    iconInline(Icon::Ok, pal::kOk);
    ImGui::SameLine();
    const mjModel* m = compiled_.model.get();
    ImGui::Text("Compiles in MuJoCo %s: %d bodies, %d DoF (nq %d), %d actuators, %d sensor values, total mass %.3g kg",
                mj_versionString(), int(m->nbody) - 1, int(m->nv), int(m->nq), int(m->nu), int(m->nsensordata),
                m->body_subtreemass[0]);
    for (const std::string& w : compiled_.warnings) {
      iconInline(Icon::Warning, pal::kWarning);
      ImGui::SameLine();
      ImGui::TextWrapped("%s", w.c_str());
    }
    int pen = 0;
    double worst = 0;
    for (const InitialContact& c : compiled_.initialContacts)
      if (c.dist < -1e-4) {
        ++pen;
        worst = std::min(worst, c.dist);
      }
    if (pen > 0) {
      iconInline(Icon::Warning, pal::kWarning);
      ImGui::SameLine();
      ImGui::TextWrapped("%d contact(s) already penetrating at the start pose (deepest %.1f mm). Overlapping geoms of "
                         "different bodies push each other apart violently when the simulation starts. They are "
                         "marked with red crosses in the viewport.",
                         pen, -worst * 1000);
    }
  }
  ImGui::End();
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
    o.cone = Cone::Pyramidal;
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("4 ms step, implicitfast, Newton with modest iteration caps. MJWarp exits early once all worlds "
                      "converge, so caps matter less than nconmax/njmax. Check stability in the preview.");
  ImGui::End();
}

}  // namespace mjf
