// Simulation preview: toolbar controls, simulated poses in the viewport, mouse perturbation and
// the Simulation panel (control modes, actuator states, plots, MuJoCo Warp buffer sizing).
#include <algorithm>
#include <cmath>
#include <cstdio>

#include "ui/Editor.h"  // first: brings in the OpenGL loader before GLFW

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <implot.h>

#include "sim/WarpLoader.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace mjf {

namespace {

double now() { return glfwGetTime(); }

// Docks a window that was never shown (no saved settings) next to an existing window, so panels
// added in newer versions appear in the layout of existing users.
void dockNextTo(const char* window, const char* existing) {
  if (ImGui::FindWindowByName(window) || ImGui::FindWindowSettingsByID(ImHashStr(window))) return;
  ImGuiWindow* w = ImGui::FindWindowByName(existing);
  if (w && w->DockNode) ImGui::DockBuilderDockWindow(window, w->DockNode->ID);
}

// Display units of an actuator: joint transmissions show joint angles (degrees) or positions (m)
// and joint torques / forces instead of raw actuator lengths and forces.
struct ActUnits {
  double scale = 1;       // actuator length -> displayed position
  const char* unit = "";
  double forceScale = 1;  // actuator force -> displayed joint torque / force
  const char* forceUnit = "";
  bool angular = false;
};

ActUnits unitsFor(const mjModel* m, int i) {
  ActUnits u;
  int trn = m->actuator_trntype[i], j = m->actuator_trnid[2 * i];
  double gear = m->actuator_gear[6 * i];
  if ((trn == mjTRN_JOINT || trn == mjTRN_JOINTINPARENT) && j >= 0 && gear != 0) {
    if (m->jnt_type[j] == mjJNT_HINGE) {
      u.scale = rad2deg(1.0) / gear;
      u.unit = "\xC2\xB0";
      u.forceUnit = " N\xC2\xB7m";
      u.angular = true;
    } else if (m->jnt_type[j] == mjJNT_SLIDE) {
      u.scale = 1.0 / gear;
      u.unit = " m";
      u.forceUnit = " N";
    }
    u.forceScale = gear;
  }
  return u;
}

// Largest force the actuator can produce (actuator space), infinity if unbounded.
double actuatorCapacity(const mjModel* m, const ActuatorDrive& a, int i) {
  double cap = INFINITY;
  if (m->actuator_forcelimited[i])
    cap = std::max(std::fabs(m->actuator_forcerange[2 * i]), std::fabs(m->actuator_forcerange[2 * i + 1]));
  bool gainOnly = m->actuator_gaintype[i] == mjGAIN_FIXED && m->actuator_biastype[i] == mjBIAS_NONE;
  if (gainOnly && a.hi > a.lo)
    cap = std::min(cap, std::fabs(m->actuator_gainprm[i * mjNGAIN]) * std::max(std::fabs(a.lo), std::fabs(a.hi)));
  return cap;
}

std::string fmtValue(double v, const char* unit, bool angular) {
  char b[64];
  if (angular) std::snprintf(b, sizeof b, "%.1f%s", std::fabs(v) < 0.05 ? 0.0 : v, unit);
  else std::snprintf(b, sizeof b, "%.3g%s", std::fabs(v) < 1e-9 ? 0.0 : v, unit);
  return b;
}

// Force with a precision suited to the actuator's capacity (no 3.2e-16 noise).
std::string fmtForce(double v, double capacity) {
  int decimals = capacity >= 100 ? 0 : capacity >= 10 ? 1 : capacity >= 1 ? 2 : 3;
  double q = std::pow(10.0, -decimals) * 0.5;
  char b[48];
  std::snprintf(b, sizeof b, "%.*f", decimals, std::fabs(v) < q ? 0.0 : v);
  return b;
}

// Axis range covering the finite values, at least `minSpan` wide (no zooming into float noise).
void plotRange(const std::vector<float>& a, const std::vector<float>& b, double minSpan, double& lo, double& hi) {
  double mn = INFINITY, mx = -INFINITY;
  for (const auto* v : {&a, &b})
    for (float x : *v)
      if (std::isfinite(x)) mn = std::min(mn, double(x)), mx = std::max(mx, double(x));
  if (mn > mx) mn = mx = 0;
  double c = 0.5 * (mn + mx), half = std::max(0.5 * (mx - mn) * 1.12, 0.5 * minSpan);
  lo = c - half;
  hi = c + half;
}

const char* kModeNames[] = {"Hold pose", "Manual", "Sine test", "Limp"};
const char* kModeTips[] = {
    "Keep the start pose. Position servos target it; torque motors use a built-in PD controller limited by their "
    "control range.\nShows whether the actuators can carry the model.",
    "Set each actuator's control signal yourself, like a policy would: the target position for servos, the "
    "torque command for motors.",
    "Sweep every actuator through its range with a sine wave: shows range, speed and tracking under load.",
    "Actuators produce no force: shows how the model falls and settles passively."};

}  // namespace

// ------------------------------------------------------------------------------------ control

void Editor::startSimulation(bool paused) {
  if (sim_.active()) {
    sim_.setPaused(paused);
    lastFrameTime_ = now();
    return;
  }
  poseMode_ = false;  // the simulation starts from the pose and shows it moving
  simStartPaused_ = paused;
  if (compileValid()) {
    launchSimulation();
    return;
  }
  if (compiledRevision_ == editRevision_ && !compiled_.ok) {
    notify("Cannot simulate: the model does not compile (see Issues)", pal::kError, 5);
    return;
  }
  // Start as soon as the compile of the current edit arrives.
  simWanted_ = true;
  compileDueTime_ = 0;
}

void Editor::launchSimulation() {
  simWanted_ = false;
  if (!sim_.start(compiled_.model, compiled_.map)) {
    notify("Could not start the simulation", pal::kError, 4);
    return;
  }
  sim_.setPaused(simStartPaused_);
  simRevision_ = compiledRevision_;
  lastFrameTime_ = now();
  if (simPlotActuator_ >= sim_.model()->nu) simPlotActuator_ = 0;
  if (cliSimSeconds_ > 0) {
    int steps = int(std::lround(cliSimSeconds_ / sim_.model()->opt.timestep));
    for (int i = 0; i < steps && !sim_.diverged(); ++i) sim_.stepOnce();
    cliSimSeconds_ = 0;
  }
  pendingFocus_.push_back("Simulation");
}

void Editor::stopSimulation() {
  simWanted_ = false;
  simDrag_.active = false;
  if (sim_.active() && sim_.sizing().peakContacts + sim_.sizing().peakBroadphase > 0) {
    lastSizing_ = sim_.sizing();
    lastSizingRevision_ = editRevision_;
  }
  sim_.stop();
}

std::string Editor::mjwarpLoader(const std::string& xmlFile, bool* measured) const {
  // Measured sizes belong to the model that was simulated: any edit (or another document) since
  // then falls back to the estimate.
  WarpSizing ws = sim_.active() ? sim_.sizing() : lastSizingRevision_ == editRevision_ ? lastSizing_ : WarpSizing{};
  bool fromSim = ws.peakContacts + ws.peakBroadphase > 0;
  if (measured) *measured = fromSim;
  return mjwarpLoaderScript(doc_, xmlFile, fromSim ? &ws : nullptr);
}

void Editor::toggleSimulation() {
  if (!sim_.active()) {
    if (simWanted_) simWanted_ = false;
    else startSimulation(false);
    return;
  }
  if (sim_.diverged()) {
    sim_.reset();
    sim_.setPaused(false);
  } else {
    sim_.setPaused(!sim_.paused());
  }
  lastFrameTime_ = now();
}

void Editor::resetSimulation() {
  if (!sim_.active()) return;
  sim_.reset();
  lastFrameTime_ = now();
}

void Editor::stepSimulation() {
  if (!sim_.active()) {
    startSimulation(true);
    return;
  }
  sim_.setPaused(true);
  sim_.stepOnce();
}

void Editor::captureKeyframe() {
  if (!sim_.active()) return;
  const mjModel* m = sim_.model();
  const mjData* d = sim_.data();
  const ExportMap& map = sim_.map();
  Keyframe k;
  int n = 1;
  auto taken = [&](const std::string& name) {
    for (const Keyframe& x : doc_.keyframes) if (x.name == name) return true;
    return false;
  };
  do k.name = "pose" + std::to_string(n++); while (taken(k.name));
  for (int j = 0; j < m->njnt && j < int(map.joints.size()); ++j) {
    if (!doc_.joint(map.joints[j])) continue;
    int t = m->jnt_type[j];
    int nq = t == mjJNT_FREE ? 7 : t == mjJNT_BALL ? 4 : 1;
    const mjtNum* q = d->qpos + m->jnt_qposadr[j];
    k.qpos.push_back({map.joints[j], std::vector<double>(q, q + nq)});
  }
  // Position servos aim at the captured pose.
  for (int i = 0; i < m->nu && i < int(map.actuators.size()); ++i)
    if (sim_.drive(i).kind == ActuatorDrive::Position && doc_.actuator(map.actuators[i]))
      k.ctrl.push_back({map.actuators[i], d->actuator_length[i]});
  checkpoint("Save keyframe");
  doc_.keyframes.insert(doc_.keyframes.begin(), k);
  notify("Saved '" + k.name + "' as the start pose (Scene panel lists keyframes)", pal::kOk, 4);
}

bool Editor::movesInSimulation(Id element) const {
  Id b = doc_.kindOf(element) == Kind::Body ? element : doc_.ownerBody(element);
  while (b != kNoId && b != kWorldId) {
    const Body* body = doc_.body(b);
    if (!body) break;
    if (body->mocap || doc_.bodyDofs(b) > 0) return true;
    b = body->parent;
  }
  return false;
}

// ------------------------------------------------------------------------------------ viewport

double Editor::simJointOffset(Id joint) const {
  if (!sim_.active()) return 0;
  int j = sim_.jointIndex(joint);
  if (j < 0) return 0;
  const mjModel* m = sim_.model();
  if (m->jnt_type[j] != mjJNT_HINGE && m->jnt_type[j] != mjJNT_SLIDE) return 0;
  int adr = m->jnt_qposadr[j];
  return sim_.data()->qpos[adr] - m->qpos0[adr];
}

void Editor::applySimPoses() {
  const mjModel* m = sim_.model();
  const mjData* d = sim_.data();
  const ExportMap& map = sim_.map();
  // 2: pose from the simulation, 1: follows a simulated parent, 0: document pose.
  std::vector<uint8_t> dynamic(doc_.bodies.size(), 0);
  int n = std::min(int(m->nbody), int(map.bodies.size()));
  for (int b = 1; b < n; ++b) {
    // Static bodies keep their document pose, so they stay editable while the simulation runs.
    if (!sim_.bodyMoves(b) && m->body_mocapid[b] < 0) continue;
    int bi = doc_.bodyIndex(map.bodies[b]);
    if (bi <= 0) continue;
    poses_[bi] = {{d->xpos[3 * b], d->xpos[3 * b + 1], d->xpos[3 * b + 2]},
                  {d->xquat[4 * b], d->xquat[4 * b + 1], d->xquat[4 * b + 2], d->xquat[4 * b + 3]}};
    dynamic[bi] = 2;
  }
  // Bodies added after the model was compiled follow their simulated parent until the next compile.
  for (uint32_t bi : doc_.index().dfsOrder) {
    if (bi == 0 || dynamic[bi] == 2) continue;
    const Body& b = doc_.bodies[bi];
    int pi = doc_.bodyIndex(b.parent);
    if (pi > 0 && dynamic[pi]) {
      poses_[bi] = poses_[pi] * Pose{b.pos, b.quat};
      dynamic[bi] = 1;
    }
  }
}

void Editor::buildSimOverlays() {
  const mjModel* m = sim_.model();
  if (vp_.showContacts) {
    sim_.contacts(simContacts_);
    double g = mju_norm3(m->opt.gravity);
    double weight = std::max(1e-9, m->body_subtreemass[0] * (g > 0 ? g : 9.81));
    double scale = vp_.cam.distance * 0.25 / weight;
    for (const SimContact& c : simContacts_) {
      bool active = c.normalForce > 0;
      Color col = active ? Color{1.0f, 0.6f, 0.15f, 1} : Color{1.0f, 0.85f, 0.5f, 0.6f};
      dbg_.cross3(c.pos, std::max(0.006, vp_.cam.distance * 0.006), col, 2.0f, DebugDraw::Overlay);
      if (!simShowForces_ || !active) continue;
      // Draw the force acting on the moving body (MuJoCo's normal points from geom1 to geom2).
      Vec3 n = c.normal;
      int b1 = c.geom1 >= 0 ? m->geom_bodyid[c.geom1] : 0, b2 = c.geom2 >= 0 ? m->geom_bodyid[c.geom2] : 0;
      if (!sim_.bodyMoves(b2) && sim_.bodyMoves(b1)) n = -n;
      double L = std::min(vp_.cam.distance * 0.3, c.normalForce * scale);
      dbg_.arrow(c.pos, c.pos + n * L, col, 2.0f, DebugDraw::Overlay, 0.25);
    }
  }
  if (sim_.perturbBody() > 0) {
    Vec3 a = sim_.perturbPointWorld(), b = sim_.perturbTarget();
    Color white{1, 1, 1, 0.95f};
    dbg_.line(a, b, white, 2.0f, DebugDraw::Overlay);
    dbg_.circle(a, vp_.cam.forward(), vp_.cam.distance * 0.008, white, 2.0f, DebugDraw::Overlay, 16);
    dbg_.cross3(b, vp_.cam.distance * 0.012, white, 2.0f, DebugDraw::Overlay);
  }
}

bool Editor::simMouseDown(const Ray& ray) {
  double t = 0;
  Id hit = pick(ray, &t);
  if (!hit || t >= 1e299) return false;
  Id body = doc_.kindOf(hit) == Kind::Body ? hit : doc_.ownerBody(hit);
  int b = sim_.bodyIndex(body);
  if (b <= 0 || !sim_.bodyMoves(b)) return false;
  const mjData* d = sim_.data();
  Vec3 p = ray.at(t);
  const double* x = d->xpos + 3 * b;
  const double* R = d->xmat + 9 * b;
  Vec3 r{p.x - x[0], p.y - x[1], p.z - x[2]};
  Vec3 local{R[0] * r.x + R[3] * r.y + R[6] * r.z, R[1] * r.x + R[4] * r.y + R[7] * r.z,
             R[2] * r.x + R[5] * r.y + R[8] * r.z};
  sim_.setPerturb(b, local, p);
  simDrag_.active = true;
  simDrag_.planePoint = p;
  simDrag_.planeNormal = vp_.cam.forward();
  return true;
}

void Editor::simMouseDrag(const Ray& ray) {
  // The grabbed point follows the mouse on a plane facing the camera.
  double denom = dot(ray.dir, simDrag_.planeNormal);
  if (std::fabs(denom) < 1e-9) return;
  double t = dot(simDrag_.planePoint - ray.origin, simDrag_.planeNormal) / denom;
  if (t > 0) sim_.setPerturbTarget(ray.at(t));
}

void Editor::drawSimBadge() {
  ImDrawList* dl = ImGui::GetWindowDrawList();
  bool diverged = sim_.diverged(), paused = sim_.paused();
  ImU32 col = diverged ? pal::kError : paused ? pal::kWarning : pal::kOk;
  dl->AddRect(ImVec2(vp_.x + 1, vp_.y + 1), ImVec2(vp_.x + vp_.w - 1, vp_.y + vp_.h - 1),
              (col & 0x00FFFFFF) | (ImU32(150) << 24), 0, 0, 2.0f * scale_);
  char line[160];
  if (diverged) {
    std::snprintf(line, sizeof line, "UNSTABLE at %.3f s - %s", sim_.divergeTime(), sim_.divergeReason().c_str());
  } else {
    char rate[64] = "";
    double rtf = sim_.realtimeFactor();
    if (!paused && rtf > 0 && rtf < 0.9 * sim_.speed) std::snprintf(rate, sizeof rate, "   %.2fx real time", rtf);
    else if (std::fabs(sim_.speed - 1.0) > 1e-3) std::snprintf(rate, sizeof rate, "   %.2fx speed", sim_.speed);
    std::snprintf(line, sizeof line, "%s   %.2f s%s", paused ? "PAUSED" : "SIMULATING", sim_.simTime(), rate);
  }
  ImVec2 ts = ImGui::CalcTextSize(line);
  ImVec2 pad(10 * scale_, 5 * scale_);
  ImVec2 p(vp_.x + vp_.w * 0.5f - ts.x * 0.5f, vp_.y + 10 * scale_);
  dl->AddRectFilled(ImVec2(p.x - pad.x, p.y - pad.y), ImVec2(p.x + ts.x + pad.x, p.y + ts.y + pad.y),
                    IM_COL32(20, 22, 27, 210), 5 * scale_);
  dl->AddRect(ImVec2(p.x - pad.x, p.y - pad.y), ImVec2(p.x + ts.x + pad.x, p.y + ts.y + pad.y),
              (col & 0x00FFFFFF) | (ImU32(200) << 24), 5 * scale_);
  dl->AddText(p, col, line);
  if (diverged) {
    const char* hint = "Ctrl+R resets. See the Simulation panel for likely causes.";
    ImVec2 hs = ImGui::CalcTextSize(hint);
    ImVec2 hp(vp_.x + vp_.w * 0.5f - hs.x * 0.5f, p.y + ts.y + pad.y * 2 + 4 * scale_);
    dl->AddRectFilled(ImVec2(hp.x - pad.x, hp.y - pad.y * 0.6f), ImVec2(hp.x + hs.x + pad.x, hp.y + hs.y + pad.y * 0.6f),
                      IM_COL32(20, 22, 27, 190), 4 * scale_);
    dl->AddText(hp, IM_COL32(220, 224, 232, 255), hint);
  }
}

// ------------------------------------------------------------------------------------ toolbar

void Editor::drawSimToolbarButtons() {
  bool active = sim_.active();
  bool running = active && !sim_.paused() && !sim_.diverged();
  const char* tip = running ? "Pause (Space)" : active ? "Resume (Space)" : "Simulate with MuJoCo (Space)";
  if (iconButton("##simplay", running ? Icon::Pause : Icon::Play, active || simWanted_, tip, 0, running ? 0 : pal::kOk))
    toggleSimulation();
  ImGui::SameLine();
  if (iconButton("##simstep", Icon::Step, false, "Advance one timestep (Right arrow while paused)")) stepSimulation();
  ImGui::SameLine();
  ImGui::BeginDisabled(!active);
  if (iconButton("##simreset", Icon::Reset, false, "Back to the start pose (Ctrl+R)")) resetSimulation();
  ImGui::SameLine();
  if (iconButton("##simstop", Icon::Stop, false, "Stop and return to editing (Shift+Space)", 0, active ? pal::kError : 0))
    stopSimulation();
  ImGui::EndDisabled();
  if (active || simWanted_) {
    ImGui::SameLine();
    ImGui::AlignTextToFramePadding();
    if (simWanted_) ImGui::TextDisabled("compiling...");
    else if (sim_.diverged()) ImGui::TextColored(pal::v4(pal::kError), "unstable");
    else ImGui::TextColored(pal::v4(running ? pal::kOk : pal::kWarning), "%.2f s", sim_.simTime());
  }
}

// ------------------------------------------------------------------------------------ panel

void Editor::drawSimulationPanel() {
  dockNextTo("Simulation", "Issues");
  if (!ImGui::Begin("Simulation")) {
    ImGui::End();
    return;
  }
  const float s = scale_;
  if (!sim_.active()) {
    ImGui::Spacing();
    ImGui::PushStyleColor(ImGuiCol_Button, IM_COL32(52, 120, 72, 255));
    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, IM_COL32(64, 145, 88, 255));
    if (ImGui::Button(simWanted_ ? "Compiling..." : "Play", ImVec2(110 * s, 0))) toggleSimulation();
    ImGui::PopStyleColor(2);
    ImGui::SameLine();
    ImGui::TextWrapped("Run the model in MuJoCo, drawn by MJForge's own renderer. Keep editing while it runs: masses, "
                       "gains, friction, joint limits or the timestep are picked up immediately.");
    ImGui::Spacing();
    ImGui::TextDisabled("Space  play / pause     Shift+Space  stop     Ctrl+R  reset     Right arrow  step while paused");
    ImGui::TextDisabled("Drag a body in the viewport to push it. Hold mode shows whether the actuators can carry the model;");
    ImGui::TextDisabled("the peak contact counts give the buffer sizes MuJoCo Warp needs.");
    ImGui::End();
    return;
  }

  const mjModel* m = sim_.model();
  const mjData* d = sim_.data();
  const ExportMap& map = sim_.map();
  const int nu = m->nu;
  if (simPlotActuator_ >= nu) simPlotActuator_ = 0;
  auto actuatorName = [&](int i) {
    Id id = i < int(map.actuators.size()) ? map.actuators[i] : kNoId;
    return id && doc_.exists(id) ? doc_.nameOf(id) : "actuator " + std::to_string(i);
  };

  float h = ImGui::GetContentRegionAvail().y;
  ImGuiTableFlags lf = ImGuiTableFlags_Resizable | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_NoPadOuterX;
  if (!ImGui::BeginTable("##simlayout", 2, lf, ImVec2(0, h))) {
    ImGui::End();
    return;
  }
  ImGui::TableSetupColumn("##controls", ImGuiTableColumnFlags_WidthStretch, 0.57f);
  ImGui::TableSetupColumn("##plots", ImGuiTableColumnFlags_WidthStretch, 0.43f);
  ImGui::TableNextRow();
  float childH = std::max(40.0f, h - ImGui::GetStyle().CellPadding.y * 2);

  // ---------------------------------------------------------------- left: controls + actuators
  ImGui::TableSetColumnIndex(0);
  ImGui::BeginChild("##simleft", ImVec2(0, childH));
  for (int k = 0; k < 4; ++k) {
    if (k) ImGui::SameLine();
    if (ImGui::RadioButton(kModeNames[k], int(sim_.mode) == k)) sim_.mode = CtrlMode(k);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", kModeTips[k]);
  }
  ImGui::SameLine(0, 18 * s);
  ImGui::SetNextItemWidth(120 * s);
  float speed = float(sim_.speed);
  if (ImGui::SliderFloat("##speed", &speed, 0.05f, 2.0f, "speed %.2fx", ImGuiSliderFlags_Logarithmic))
    sim_.speed = speed;
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Playback speed (slow motion below 1). Double-click to type.");
  double rtf = sim_.realtimeFactor();
  if (!sim_.paused() && rtf > 0 && rtf < 0.9 * sim_.speed) {
    ImGui::SameLine();
    ImGui::TextColored(pal::v4(pal::kWarning), "%.2fx real time", rtf);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("The editor can't simulate this model in real time on the CPU. Training speed on the GPU "
                        "with MuJoCo Warp is unrelated; smaller timesteps, many contacts and long chains cost most.");
  }
  if (sim_.mode == CtrlMode::Sine) {
    float f = float(sim_.sine.frequency), a = float(sim_.sine.amplitude * 100);
    ImGui::SetNextItemWidth(130 * s);
    if (ImGui::SliderFloat("##freq", &f, 0.05f, 5.0f, "%.2f Hz", ImGuiSliderFlags_Logarithmic)) sim_.sine.frequency = f;
    ImGui::SameLine();
    ImGui::SetNextItemWidth(150 * s);
    if (ImGui::SliderFloat("##amp", &a, 0.0f, 100.0f, "amplitude %.0f%%")) sim_.sine.amplitude = a / 100.0;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Fraction of each actuator's half range (joint range or control range).");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(140 * s);
    ImGui::Combo("##pattern", &sim_.sine.pattern, "All in phase\0Alternating\0Traveling wave\0");
  }
  if (ImGui::SmallButton("Save pose as keyframe")) captureKeyframe();
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Stores the current pose as a keyframe and makes it the start pose of every simulation (and of "
                      "the exported model: key 0). Position servos get matching targets.");
  if (sim_.mode == CtrlMode::Manual) {
    ImGui::SameLine();
    if (ImGui::SmallButton("Reset sliders to the start pose"))
      for (int i = 0; i < nu; ++i) sim_.manual[i] = sim_.drive(i).holdCtrl;
    ImGui::SameLine();
    ImGui::TextDisabled("right-click a slider to reset it");
  }

  if (sim_.diverged()) {
    ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(90, 32, 32, 110));
    ImGui::BeginChild("##diverged", ImVec2(0, 0), ImGuiChildFlags_AutoResizeY | ImGuiChildFlags_Borders);
    int j = sim_.divergeJoint();
    Id jointId = j >= 0 && j < int(map.joints.size()) ? map.joints[j] : kNoId;
    std::string where = jointId && doc_.exists(jointId) ? " near joint '" + doc_.nameOf(jointId) + "'" : "";
    ImGui::PushStyleColor(ImGuiCol_Text, pal::kError);
    ImGui::TextWrapped("The simulation became unstable at t = %.3f s: %s%s.", sim_.divergeTime(),
                       sim_.divergeReason().c_str(), where.c_str());
    ImGui::PopStyleColor();
    ImGui::TextWrapped("Usual causes: a timestep too large for stiff servos or light bodies (see the Stability items "
                       "in Issues), geoms that start inside each other, huge gains, or tiny masses and inertias. "
                       "Remedies: a smaller timestep, joint armature, the implicitfast integrator, lower kp.");
    if (ImGui::Button("Reset")) resetSimulation();
    ImGui::SameLine();
    if (ImGui::Button("Show Issues")) pendingFocus_.push_back("Issues");
    if (jointId && doc_.exists(jointId)) {
      ImGui::SameLine();
      if (ImGui::Button("Select the joint")) {
        sel_.set(jointId);
        pendingFocus_.push_back("Inspector");
      }
    }
    ImGui::EndChild();
    ImGui::PopStyleColor();
  }

  if (nu == 0) {
    ImGui::TextDisabled("The model has no actuators: it moves passively. Add actuators to joints to drive it.");
  } else {
    ImGuiTableFlags tf = ImGuiTableFlags_RowBg | ImGuiTableFlags_BordersInnerV | ImGuiTableFlags_ScrollY |
                         ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable;
    if (ImGui::BeginTable("##simacts", 5, tf, ImVec2(0, ImGui::GetContentRegionAvail().y))) {
      ImGui::TableSetupScrollFreeze(0, 1);
      ImGui::TableSetupColumn("Actuator", 0, 1.25f);
      ImGui::TableSetupColumn("Control", 0, 1.25f);
      ImGui::TableSetupColumn("Position", 0, 0.65f);
      ImGui::TableSetupColumn("Error", 0, 0.6f);
      ImGui::TableSetupColumn("Force / capacity", 0, 1.45f);
      ImGui::TableHeadersRow();
      for (int i = 0; i < nu; ++i) {
        const ActuatorDrive& a = sim_.drive(i);
        ActUnits un = unitsFor(m, i);
        bool pos = a.kind == ActuatorDrive::Position;
        ImGui::TableNextRow();
        ImGui::PushID(i);
        ImGui::TableSetColumnIndex(0);
        std::string name = actuatorName(i);
        if (ImGui::Selectable(name.c_str(), simPlotActuator_ == i,
                              ImGuiSelectableFlags_SpanAllColumns | ImGuiSelectableFlags_AllowOverlap)) {
          simPlotActuator_ = i;
          Id id = i < int(map.actuators.size()) ? map.actuators[i] : kNoId;
          if (id && doc_.exists(id)) sel_.set(id);
        }
        if (ImGui::IsItemHovered())
          ImGui::SetTooltip("%s actuator. Click to plot it and edit it in the Inspector (changes apply live).",
                            pos ? "Position servo" : a.kind == ActuatorDrive::Motor ? "Torque motor" : "Other");

        // Control signal: a slider in Manual mode. Servo targets are shown as joint positions.
        ImGui::TableSetColumnIndex(1);
        double cs = pos ? un.scale : 1.0;
        const char* cu = pos ? un.unit : "";
        if (sim_.mode == CtrlMode::Manual) {
          char fmt[24];
          std::snprintf(fmt, sizeof fmt, pos && un.angular ? "%%.1f%s" : "%%.3g%s", cu);
          float v = float(sim_.manual[i] * cs);
          bool changed;
          ImGui::SetNextItemWidth(-FLT_MIN);
          if (a.hi > a.lo) {
            float lo = float(a.lo * cs), hi = float(a.hi * cs);
            if (lo > hi) std::swap(lo, hi);
            changed = ImGui::SliderFloat("##u", &v, lo, hi, fmt);
          } else {
            changed = ImGui::DragFloat("##u", &v, float(0.005 * std::max(1.0, std::fabs(cs))), 0, 0, fmt);
          }
          if (changed) sim_.manual[i] = v / cs;
          if (ImGui::IsItemClicked(ImGuiMouseButton_Right)) sim_.manual[i] = a.holdCtrl;
        } else {
          ImGui::TextUnformatted(fmtValue(d->ctrl[i] * cs, cu, pos && un.angular).c_str());
        }

        ImGui::TableSetColumnIndex(2);
        ImGui::TextUnformatted(fmtValue(d->actuator_length[i] * un.scale, un.unit, un.angular).c_str());

        ImGui::TableSetColumnIndex(3);
        double target = sim_.target(i);
        if (std::isfinite(target)) {
          double err = target - d->actuator_length[i];
          double rel = a.halfRange > 0 ? std::fabs(err) / a.halfRange : 0;
          ImU32 ec = rel > 0.3 ? pal::kError : rel > 0.1 ? pal::kWarning : pal::kTextDim;
          ImGui::TextColored(pal::v4(ec), "%s", fmtValue(err * un.scale, un.unit, un.angular).c_str());
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Target minus actual position. A large error while holding means the actuator is too "
                              "weak or too soft (kp) for the load.");
        } else {
          ImGui::TextDisabled("-");
        }

        ImGui::TableSetColumnIndex(4);
        double cap = actuatorCapacity(m, a, i);
        double f = d->actuator_force[i] * un.forceScale;
        bool sat = sim_.saturated(i);
        if (std::isfinite(cap) && cap > 0) {
          double frac = std::min(1.0, std::fabs(d->actuator_force[i]) / cap);
          double capShown = cap * std::fabs(un.forceScale);
          std::string label = fmtForce(std::fabs(f), capShown) + " / " + fmtForce(capShown, capShown) + un.forceUnit;
          ImU32 bc = sat || frac > 0.98 ? pal::kError : frac > 0.8 ? pal::kWarning : pal::kTeal;
          ImGui::PushStyleColor(ImGuiCol_PlotHistogram, bc);
          ImGui::ProgressBar(float(frac), ImVec2(-FLT_MIN, 0), label.c_str());
          ImGui::PopStyleColor();
          if (ImGui::IsItemHovered())
            ImGui::SetTooltip("Force the actuator applies now vs. the most it can apply (force range or control range "
                              "x gain).%s",
                              frac > 0.98 ? "\nAt the limit: it can't do more, so it may fail to hold or track." : "");
        } else {
          ImGui::Text("%s%s", fmtForce(f, std::fabs(f)).c_str(), un.forceUnit);
          ImGui::SameLine();
          ImGui::TextDisabled("(unlimited)");
        }
        ImGui::PopID();
      }
      ImGui::EndTable();
    }
  }
  ImGui::EndChild();

  // ---------------------------------------------------------------- right: plot + MJWarp sizing
  ImGui::TableSetColumnIndex(1);
  ImGui::BeginChild("##simright", ImVec2(0, childH));
  float warpH = ImGui::GetFrameHeightWithSpacing() + 4 * s;
  if (nu > 0) {
    int i = simPlotActuator_;
    const ActuatorDrive& a = sim_.drive(i);
    ActUnits un = unitsFor(m, i);
    ImGui::SetNextItemWidth(-FLT_MIN);
    if (ImGui::BeginCombo("##plotact", actuatorName(i).c_str())) {
      for (int k = 0; k < nu; ++k)
        if (ImGui::Selectable(actuatorName(k).c_str(), k == i)) simPlotActuator_ = k;
      ImGui::EndCombo();
    }
    const auto& hist = sim_.history();
    int n = int(hist.size());
    std::vector<float> xs(n), actual(n), target(n), force(n);
    bool hasTarget = false;
    for (int k = 0; k < n; ++k) {
      const SimSample& smp = hist[k];
      xs[k] = float(smp.t);
      bool ok = size_t(i) < smp.length.size();
      actual[k] = ok ? float(smp.length[i] * un.scale) : 0.f;
      target[k] = ok ? float(smp.target[i] * un.scale) : NAN;
      force[k] = ok ? float(smp.force[i] * un.forceScale) : 0.f;
      hasTarget |= std::isfinite(target[k]);
    }
    float plotH = std::max(90.0f * s, ImGui::GetContentRegionAvail().y - warpH);
    if (ImPlot::BeginPlot("##simplot", ImVec2(-1, plotH), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
      double t1 = std::max(10.0, sim_.simTime());
      const char* ylabel = un.angular ? "angle (deg)" : un.unit[0] ? "position (m)" : "length";
      const char* y2label = un.angular ? "torque (N m)" : un.unit[0] ? "force (N)" : "force";
      double lo, hi, flo, fhi;
      double posSpan = un.angular ? 4.0 : un.unit[0] ? 0.02 : std::max(0.02, 0.2 * a.halfRange);
      plotRange(actual, target, posSpan, lo, hi);
      double cap = actuatorCapacity(m, a, i);
      double fSpan = std::isfinite(cap) ? 0.1 * cap * std::fabs(un.forceScale) : 0.1;
      plotRange(force, force, std::max(1e-3, fSpan), flo, fhi);
      ImPlot::SetupAxes("time (s)", ylabel, 0, 0);
      ImPlot::SetupAxis(ImAxis_Y2, y2label, ImPlotAxisFlags_AuxDefault);
      ImPlot::SetupAxisLimits(ImAxis_X1, t1 - 10.0, t1, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y1, lo, hi, ImPlotCond_Always);
      ImPlot::SetupAxisLimits(ImAxis_Y2, flo, fhi, ImPlotCond_Always);
      ImPlot::SetupLegend(ImPlotLocation_NorthWest, ImPlotLegendFlags_Horizontal);
      if (n > 0) {
        ImPlotSpec ts;
        ts.LineColor = ImVec4(0.75f, 0.77f, 0.82f, 0.9f);
        ts.LineWeight = 1.5f;
        ts.Flags = ImPlotLineFlags_SkipNaN;
        if (hasTarget) ImPlot::PlotLine("target", xs.data(), target.data(), n, ts);
        ImPlotSpec as;
        as.LineColor = pal::v4(pal::kTeal);
        as.LineWeight = 2.0f;
        ImPlot::PlotLine("actual", xs.data(), actual.data(), n, as);
        ImPlot::SetAxes(ImAxis_X1, ImAxis_Y2);
        ImPlotSpec fs;
        fs.LineColor = ImVec4(0.91f, 0.51f, 0.23f, 0.85f);
        fs.LineWeight = 1.5f;
        ImPlot::PlotLine(un.angular ? "torque" : "force", xs.data(), force.data(), n, fs);
      }
      ImPlot::EndPlot();
    }
  } else {
    ImGui::Dummy(ImVec2(0, std::max(0.0f, ImGui::GetContentRegionAvail().y - warpH)));
  }

  // MuJoCo Warp buffer sizes measured in this run.
  const WarpSizing& ws = sim_.sizing();
  int nconmax = ws.recommendedNconmax(), njmax = ws.recommendedNjmax();
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("MJWarp");
  ImGui::SameLine();
  ImGui::TextColored(pal::v4(pal::kAccent), "nconmax=%d  njmax=%d", nconmax, njmax);
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Buffer sizes for mujoco_warp.make_data, from the peaks of this run:\n"
                      "  %d contacts, %d broadphase candidate pairs, %d constraint rows.\n\n"
                      "MJWarp preallocates contact and constraint buffers per world. Too small and contacts are "
                      "dropped (overflow warning).\nIts broadphase needs room for every candidate geom pair, which can "
                      "exceed the number of actual contacts.\nPush the model around and run the sine test for "
                      "realistic peaks; terrains and randomization need more headroom.",
                      ws.peakContacts, ws.peakBroadphase, ws.peakConstraints);
  ImGui::SameLine();
  ImGui::TextDisabled("(peaks: %d contacts, %d pairs)", ws.peakContacts, ws.peakBroadphase);
  ImGui::SameLine();
  if (ImGui::SmallButton("Copy Python")) {
    std::string name = doc_.modelName.empty() ? "model" : doc_.modelName;
    char buf[512];
    std::snprintf(buf, sizeof buf,
                  "import mujoco\nimport mujoco_warp as mjw\n\n"
                  "mjm = mujoco.MjModel.from_xml_path(\"%s.xml\")\n"
                  "m = mjw.put_model(mjm)\n"
                  "# sizes measured in MJForge (peaks: %d contacts, %d broadphase pairs, %d constraint rows)\n"
                  "d = mjw.make_data(mjm, nworld=4096, nconmax=%d, njmax=%d)\n",
                  name.c_str(), ws.peakContacts, ws.peakBroadphase, ws.peakConstraints, nconmax, njmax);
    ImGui::SetClipboardText(buf);
    notify("Copied the MuJoCo Warp setup to the clipboard", pal::kOk, 2);
  }
  ImGui::EndChild();
  ImGui::EndTable();
  ImGui::End();
}

}  // namespace mjf
