#include "ui/Editor.h"

#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

#include <imgui.h>
#include <imgui_internal.h>
#include <nlohmann/json.hpp>

#include "core/Generators.h"
#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Ops.h"
#include "core/ProjectIO.h"
#include "core/Templates.h"
#include "sim/MjcfImport.h"
#include "ui/Icons.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

static double nowSeconds() { return glfwGetTime(); }
static std::string quoteName(const std::string& s) { return "'" + s + "'"; }

// ------------------------------------------------------------------------------------ Selection

bool Selection::contains(Id id) const { return std::find(ids.begin(), ids.end(), id) != ids.end(); }
void Selection::set(Id id) {
  ids.clear();
  if (id != kNoId) ids.push_back(id);
}
void Selection::add(Id id) {
  if (id == kNoId) return;
  auto it = std::find(ids.begin(), ids.end(), id);
  if (it != ids.end()) ids.erase(it);
  ids.push_back(id);
}
void Selection::toggle(Id id) {
  auto it = std::find(ids.begin(), ids.end(), id);
  if (it != ids.end()) ids.erase(it);
  else if (id != kNoId) ids.push_back(id);
}
void Selection::prune(const Document& doc) {
  ids.erase(std::remove_if(ids.begin(), ids.end(), [&](Id id) { return !doc.exists(id); }), ids.end());
}

// ------------------------------------------------------------------------------------ Editor

Editor::Editor() = default;
Editor::~Editor() = default;

void Editor::init(GLFWwindow* window, float scale, const std::string& configDir) {
  window_ = window;
  scale_ = scale;
  configDir_ = configDir;
  if (!renderer_.init()) std::fprintf(stderr, "Renderer initialization failed\n");
  compiler_.setResultCallback([] { glfwPostEmptyEvent(); });
  dbg_.init();
  propCtx_.change = [this] { change(); };
  props::setContext(&propCtx_);
  loadPrefs();
  std::error_code ec;
  if (lastDir_.empty()) lastDir_ = fs::current_path(ec).string();
  newDocument("quadruped");
  initAutosave();
}

void Editor::applyCli(const CliOptions& cli) {
  if (!cli.screenshot.empty() || !cli.actions.empty()) showRecover_ = false;  // scripted runs never block
  if (!cli.templateName.empty()) newDocument(cli.templateName);
  if (!cli.input.empty()) openPath(cli.input);
  for (const std::string& name : cli.select) {
    auto findName = [&](const auto& vec) -> Id {
      for (const auto& e : vec) if (e.name == name) return e.id;
      return kNoId;
    };
    Id id = findName(doc_.bodies);
    if (!id) id = findName(doc_.geoms);
    if (!id) id = findName(doc_.joints);
    if (!id) id = findName(doc_.sites);
    if (!id) id = findName(doc_.actuators);
    if (!id) id = findName(doc_.sensors);
    if (!id) id = findName(doc_.lights);
    if (!id) id = findName(doc_.cameras);
    if (!id) id = findName(doc_.tendons);
    if (!id) id = findName(doc_.equalities);
    if (!id) id = findName(doc_.pairs);
    if (id) sel_.add(id);
  }
  for (const std::string& a : cli.actions) {
    if (a == "frame") focusSelection();
    else if (a == "xray") vp_.xray = true;
    else if (a == "com") vp_.showCom = true;
    else if (a == "labels") vp_.showLabels = true;
    else if (a == "rotate") gizmoOp_ = GizmoOp::Rotate;
    else if (a == "scale") gizmoOp_ = GizmoOp::Scale;
    else if (a == "export") showExport_ = true;
    else if (a == "about") showAbout_ = true;
    else if (a == "demo") showDemo_ = true;
    else if (a.rfind("focus:", 0) == 0) pendingFocus_.push_back(a.substr(6));
    else if (a == "play") startSimulation(false);
    else if (a.rfind("import:", 0) == 0) beginImport(a.substr(7));
    else if (a.rfind("gen:", 0) == 0) addGeneratedEnvironment(a.substr(4));
    else if (a == "addmodel") addModelRoot();
    else if (a.rfind("addmesh:", 0) == 0) addMeshFromFile(targetBody(), a.substr(8));
    else if (a == "notoasts") toasts_.clear();
    else if (a == "shortcuts") showShortcuts_ = true;
    else if (a.rfind("export:", 0) == 0) exportTo(a.substr(7), ExportOptions::Scope::All, false);
    else if (a == "limbtool") limbTool_ = true;
    else if (a == "posemode") setPoseMode(true);
    else if (a == "quit") quit_ = true;
    else if (a == "recover") showRecover_ = !recoverPath_.empty();
    else if (a.rfind("pose:", 0) == 0) {
      // pose:<joint>=<value> (degrees for hinges), as if dragged in pose mode.
      size_t eq = a.find('=');
      std::string jn = a.substr(5, eq == std::string::npos ? std::string::npos : eq - 5);
      for (const Joint& j : doc_.joints)
        if (j.name == jn && eq != std::string::npos && (j.type == JointType::Hinge || j.type == JointType::Slide)) {
          double v = std::atof(a.c_str() + eq + 1);
          checkpoint("Pose");
          setKeyJointQpos(startKeyframe(doc_), j.id, {j.type == JointType::Hinge ? deg2rad(v) : v});
          poseEdited();
          break;
        }
    }
    else if (a.rfind("sim:", 0) == 0) {
      // Start paused and fast-forward: deterministic screenshots of the simulation.
      cliSimSeconds_ = std::max(1e-9, std::atof(a.c_str() + 4));
      startSimulation(true);
    } else if (a.rfind("simmode:", 0) == 0) {
      std::string m = a.substr(8);
      sim_.mode = m == "manual" ? CtrlMode::Manual : m == "sine" ? CtrlMode::Sine : m == "limp" ? CtrlMode::Limp : CtrlMode::Hold;
    }
  }
}

bool Editor::wantsContinuousRendering() const {
  return vp_.gizmoUsing || vp_.dragButton >= 0 || compiler_.busy() || requestedRevision_ != editRevision_ ||
         compiledRevision_ != requestedRevision_ || !toasts_.empty() || fileDialog_.isOpen() || simWanted_ ||
         (sim_.active() && !sim_.paused() && !sim_.diverged());
}

bool Editor::hasPendingWork() {
  // A finished compile, a scheduled compile that is due, or a pending window focus request.
  if (requestedRevision_ != editRevision_) return true;
  if (compiledRevision_ != requestedRevision_ && !compiler_.busy()) return true;
  if (autosaveDue()) return true;
  return !pendingFocus_.empty() || focusInspector_ > 0;
}

void Editor::change() {
  ImGuiID id = ImGui::GetItemID();
  if (id == 0 || id != activeEditId_) {
    history_.push(doc_, "Edit");
    activeEditId_ = id;
  }
  markDirty();
}

void Editor::checkpoint(const char* label) {
  history_.push(doc_, label);
  activeEditId_ = 0;
  markDirty();
}

void Editor::markDirty() {
  modified_ = true;
  ++editRevision_;
  compileDueTime_ = nowSeconds() + 0.12;
  setTitle();
}

void Editor::undo() {
  if (history_.undo(doc_)) {
    sel_.prune(doc_);
    ++editRevision_;
    modified_ = true;
    compileDueTime_ = nowSeconds();
    notify("Undo", pal::kTextDim, 1.2);
    setTitle();
  }
}

void Editor::redo() {
  if (history_.redo(doc_)) {
    sel_.prune(doc_);
    ++editRevision_;
    modified_ = true;
    compileDueTime_ = nowSeconds();
    notify("Redo", pal::kTextDim, 1.2);
    setTitle();
  }
}

void Editor::select(Id id, bool additive, bool toggle) {
  if (toggle) sel_.toggle(id);
  else if (additive) sel_.add(id);
  else sel_.set(id);
}

void Editor::notify(const std::string& text, unsigned int color, double seconds) {
  toasts_.push_back({text, color ? color : IM_COL32(220, 224, 232, 255), nowSeconds() + seconds});
  if (toasts_.size() > 5) toasts_.erase(toasts_.begin());
}

void Editor::setTitle() {
  if (!window_) return;
  std::string name = path_.empty() ? "untitled" : fs::path(path_).filename().string();
  std::string title = (modified_ ? "* " : "") + name + " - MJForge";
  glfwSetWindowTitle(window_, title.c_str());
}

// ------------------------------------------------------------------------------------ files

void Editor::newDocument(const std::string& templateId) {
  Document d;
  if (!makeTemplate(templateId, d)) {
    notify("Unknown template: " + templateId, pal::kError);
    return;
  }
  stopSimulation();
  removeAutosave();
  doc_ = std::move(d);
  history_.clear();
  sel_.clear();
  path_.clear();
  modified_ = false;
  ++editRevision_;
  compileDueTime_ = 0;
  frameModels();
  setTitle();
}

bool Editor::openPath(const std::string& path) {
  Document d;
  std::string editorState;
  std::vector<std::string> notes;
  if (!loadAnyFile(path, d, &notes, &editorState)) return false;
  bool mjcf = fs::path(path).extension() != ".mjforge";
  stopSimulation();
  removeAutosave();
  doc_ = std::move(d);
  history_.clear();
  sel_.clear();
  path_ = mjcf ? std::string() : path;  // MJCF opens as a new, unsaved project
  modified_ = false;
  ++editRevision_;
  compileDueTime_ = 0;
  frameModels();
  if (!editorState.empty()) applyEditorState(editorState);
  lastDir_ = fs::path(path).parent_path().string();
  addRecent(path);
  setTitle();
  notify("Opened " + fs::path(path).filename().string(), pal::kOk);
  for (const std::string& n : notes) notify(n, pal::kWarning, 8);
  return true;
}

bool Editor::savePath(const std::string& path) {
  std::string err;
  if (!saveProject(doc_, path, &err, editorStateJson())) {
    notify("Save failed: " + err, pal::kError, 6);
    return false;
  }
  path_ = path;
  modified_ = false;
  removeAutosave();
  lastDir_ = fs::path(path).parent_path().string();
  addRecent(path);
  setTitle();
  notify("Saved " + fs::path(path).filename().string(), pal::kOk);
  if (afterSave_) {
    auto next = std::move(afterSave_);
    afterSave_ = nullptr;
    next();
  }
  return true;
}

void Editor::save() {
  if (path_.empty()) saveAs();
  else savePath(path_);
}

void Editor::saveAs() {
  std::string def = path_.empty() ? doc_.modelName + ".mjforge" : fs::path(path_).filename().string();
  fileDialog_.open(FileDialog::Mode::Save, "Save project", {".mjforge"}, lastDir_, def, kPurposeSaveAs);
}

void Editor::openDialog() {
  fileDialog_.open(FileDialog::Mode::Open, "Open project or MJCF", {".mjforge", ".xml"}, lastDir_, "", kPurposeOpen);
}

void Editor::exportDialog() { showExport_ = true; }

bool Editor::exportTo(const std::string& path, ExportOptions::Scope scope, bool split) {
  // Big meshes and texture images are written into a folder next to the XML; mention it.
  auto meshNote = [&](const std::string& stem) -> std::string {
    bool assets = false;
    for (const MeshAsset& m : doc_.meshes) assets |= m.vertexCount() > 1000;
    for (const Texture& t : doc_.textures) assets |= t.builtin == TextureBuiltin::None && !t.file.empty();
    return assets ? " (assets in " + stem + "_assets/)" : "";
  };
  ExportOptions opt;
  opt.angle = exportAngle_ == 1 ? AngleUnit::Degree : AngleUnit::Radian;
  std::string err;
  fs::path p(path);
  if (split) {
    // model file next to the scene file; the scene includes it.
    std::string stem = p.stem().string();
    fs::path modelPath = p.parent_path() / (stem + "_model.xml");
    ExportOptions mo = opt;
    mo.scope = ExportOptions::Scope::Models;
    ExportOptions so = opt;
    so.scope = ExportOptions::Scope::Environment;
    so.includeFile = modelPath.filename().string();
    // Both files share one mesh folder named after the scene.
    if (!exportMjcfFile(doc_, modelPath.string(), mo, &err, stem) || !exportMjcfFile(doc_, path, so, &err, stem)) {
      notify("Export failed: " + err, pal::kError, 6);
      return false;
    }
    notify("Exported " + p.filename().string() + " + " + modelPath.filename().string() + meshNote(stem), pal::kOk, 5);
  } else {
    opt.scope = scope;
    if (!exportMjcfFile(doc_, path, opt, &err)) {
      notify("Export failed: " + err, pal::kError, 6);
      return false;
    }
    notify("Exported " + p.filename().string() + meshNote(p.stem().string()), pal::kOk, 5);
  }
  if (exportLoader_) {
    bool measured = false;
    fs::path py = p.parent_path() / (p.stem().string() + "_mjwarp.py");
    if (writeTextFile(py.string(), mjwarpLoader(p.filename().string(), &measured), &err))
      notify(std::string("Wrote ") + py.filename().string() + (measured ? " (sizes from the simulation)" : " (estimated sizes: simulate first for measured ones)"),
             pal::kTextDim, 5);
  }
  lastDir_ = p.parent_path().string();
  return true;
}

// ------------------------------------------------------------------------------------ prefs

std::string Editor::editorStateJson() const {
  nlohmann::json j;
  j["camera"] = {{"target", {vp_.cam.target.x, vp_.cam.target.y, vp_.cam.target.z}},
                 {"distance", vp_.cam.distance},
                 {"azimuth", vp_.cam.azimuth},
                 {"elevation", vp_.cam.elevation}};
  return j.dump();
}

void Editor::applyEditorState(const std::string& s) {
  auto j = nlohmann::json::parse(s, nullptr, false);
  if (j.is_discarded() || !j.contains("camera")) return;
  const auto& c = j["camera"];
  if (c.contains("target") && c["target"].size() == 3)
    vp_.cam.target = {c["target"][0].get<double>(), c["target"][1].get<double>(), c["target"][2].get<double>()};
  vp_.cam.distance = c.value("distance", vp_.cam.distance);
  vp_.cam.azimuth = c.value("azimuth", vp_.cam.azimuth);
  vp_.cam.elevation = c.value("elevation", vp_.cam.elevation);
}

void Editor::loadPrefs() {
  if (configDir_.empty()) return;
  std::string text;
  if (!readTextFile(configDir_ + "/prefs.json", text)) return;
  auto j = nlohmann::json::parse(text, nullptr, false);
  if (j.is_discarded()) return;
  if (j.contains("recent")) for (const auto& r : j["recent"]) recent_.push_back(r.get<std::string>());
  lastDir_ = j.value("lastDir", std::string());
  snap_ = j.value("snap", false);
  snapMove_ = j.value("snapMove", 0.05f);
  snapAngle_ = j.value("snapAngle", 15.0f);
  vp_.showShadows = j.value("shadows", true);
  vp_.showGrid = j.value("grid", true);
}

void Editor::savePrefs() {
  if (configDir_.empty()) return;
  nlohmann::json j;
  j["recent"] = recent_;
  j["lastDir"] = lastDir_;
  j["snap"] = snap_;
  j["snapMove"] = snapMove_;
  j["snapAngle"] = snapAngle_;
  j["shadows"] = vp_.showShadows;
  j["grid"] = vp_.showGrid;
  writeTextFile(configDir_ + "/prefs.json", j.dump(2));
}

void Editor::addRecent(const std::string& path) {
  std::error_code ec;
  std::string abs = fs::absolute(path, ec).string();
  recent_.erase(std::remove(recent_.begin(), recent_.end(), abs), recent_.end());
  recent_.insert(recent_.begin(), abs);
  if (recent_.size() > 10) recent_.resize(10);
  savePrefs();
}

// ------------------------------------------------------------------------------------ validation

void Editor::scheduleCompile() {
  if (requestedRevision_ == editRevision_) return;
  if (nowSeconds() < compileDueTime_) return;
  ExportMap map;
  std::string xml = writeMjcf(doc_, {}, &map);
  compiler_.request(std::move(xml), std::move(map), editRevision_);
  requestedRevision_ = editRevision_;
  runAnalysis();  // immediate feedback; refined when the compile result arrives
}

void Editor::pollCompile() {
  CompileResult r;
  if (!compiler_.poll(r)) return;
  compiledRevision_ = r.revision;
  compiled_ = std::move(r);
  if (compiledRevision_ != editRevision_) return;
  runAnalysis();
  if (simWanted_) {
    if (compiled_.ok) launchSimulation();
    else {
      simWanted_ = false;
      notify("Cannot simulate: the model does not compile (see Issues)", pal::kError, 5);
    }
  } else if (sim_.active() && compiled_.ok && simRevision_ != compiledRevision_) {
    // Live tuning: the running simulation picks up the edited model.
    bool kept = sim_.update(compiled_.model, compiled_.map);
    simRevision_ = compiledRevision_;
    if (!kept) notify("The structure changed: the simulation restarted from the start pose", pal::kTextDim, 3);
  }
}

void Editor::runAnalysis() {
  bool valid = compiledRevision_ == editRevision_;
  report_ = analyzeModel(doc_, &compiled_, valid);
  reportRevision_ = editRevision_;
  reportHasCompile_ = valid;
}

// ------------------------------------------------------------------------------------ frame

void Editor::buildDefaultLayout(unsigned int dockId) {
  ImGui::DockBuilderRemoveNode(dockId);
  ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockId, ImGui::GetMainViewport()->WorkSize);
  ImGuiID center = dockId;
  ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.17f, nullptr, &center);
  ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.27f, nullptr, &center);
  ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.27f, nullptr, &center);
  ImGui::DockBuilderDockWindow("Outliner", left);
  ImGui::DockBuilderDockWindow("Inspector", right);
  ImGui::DockBuilderDockWindow("Scene", right);
  focusInspector_ = 3;
  ImGui::DockBuilderDockWindow("Viewport", center);
  ImGui::DockBuilderDockWindow("Issues", bottom);
  ImGui::DockBuilderDockWindow("Actuators", bottom);
  ImGui::DockBuilderDockWindow("Sensors", bottom);
  ImGui::DockBuilderDockWindow("MJCF", bottom);
  ImGui::DockBuilderDockWindow("Simulation", bottom);
  ImGui::DockBuilderFinish(dockId);
}

void Editor::frame() {
  ++frameCount_;
  pollCompile();
  scheduleCompile();
  double t = nowSeconds();
  double dt = lastFrameTime_ > 0 ? t - lastFrameTime_ : 0;
  lastFrameTime_ = t;
  if (sim_.active()) sim_.advance(dt);
  maybeAutosave();
  handleShortcuts();
  drawMenuBar();
  drawToolbar();
  drawStatusBar();

  ImGuiID dock = ImGui::GetID("MainDock");
  if (!layoutBuilt_ && ImGui::DockBuilderGetNode(dock) == nullptr) buildDefaultLayout(dock);
  layoutBuilt_ = true;
  ImGui::DockSpaceOverViewport(dock, ImGui::GetMainViewport(), ImGuiDockNodeFlags_PassthruCentralNode);

  drawOutliner();
  drawScenePanel();
  drawInspector();
  // Deferred focus requests wait while a modal is open: focusing a window would close it.
  bool modalOpen = ImGui::GetTopMostPopupModal() != nullptr;
  if (modalOpen) {
  } else if (focusInspector_ > 0 && --focusInspector_ == 0) ImGui::SetWindowFocus("Inspector");
  else if (focusInspector_ == 0 && frameCount_ > 3 && !pendingFocus_.empty()) {
    // One window per frame: tab selection only follows the last focused window of a frame.
    ImGui::SetWindowFocus(pendingFocus_.front().c_str());
    pendingFocus_.erase(pendingFocus_.begin());
  }
  drawActuatorsPanel();
  drawSensorsPanel();
  drawXmlPanel();
  drawIssuesPanel();
  drawSimulationPanel();
  drawViewport();

  drawUnsavedModal();
  drawRecoverModal();
  if (!fileDialog_.isOpen() && afterSave_ && !ImGui::IsPopupOpen("Unsaved changes")) afterSave_ = nullptr;  // save cancelled
  if (fileDialog_.draw()) {
    const std::string& p = fileDialog_.result();
    switch (fileDialog_.purpose()) {
      case kPurposeOpen: openPath(p); break;
      case kPurposeSaveAs: savePath(p); break;
      case kPurposeImport: beginImport(p); break;
      case kPurposeMesh: addMeshFromFile(meshTarget_, p); break;
      case kPurposeExport:
        exportTo(p, exportScope_ == 1 ? ExportOptions::Scope::Models
                    : exportScope_ == 2 ? ExportOptions::Scope::Environment
                                        : ExportOptions::Scope::All,
                 exportSplit_);
        break;
      default: break;
    }
  }
  if (showExport_) {
    ImGui::OpenPopup("Export MJCF");
    showExport_ = false;
  }
  ImGui::SetNextWindowSize(ImVec2(520 * scale_, 0), ImGuiCond_Appearing);
  if (ImGui::BeginPopupModal("Export MJCF", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextWrapped("Writes MuJoCo XML that loads with mujoco.MjModel.from_xml_path() and MuJoCo Warp "
                       "(mjw.put_model).");
    ImGui::Separator();
    ImGui::TextDisabled("What to export");
    ImGui::RadioButton("Everything (environment + models) in one file", &exportScope_, 0);
    ImGui::RadioButton("Models only (creatures / robots)", &exportScope_, 1);
    ImGui::RadioButton("Environment only (floor, obstacles, lights, options)", &exportScope_, 2);
    ImGui::Checkbox("Split: scene.xml includes scene_model.xml (MuJoCo Menagerie layout)", &exportSplit_);
    if (exportSplit_) ImGui::TextDisabled("  The scene file gets the environment and <include>s the model file.");
    ImGui::Checkbox("Also write a MuJoCo Warp loader script (name_mjwarp.py)", &exportLoader_);
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("A ready-to-run Python file that loads the XML with mujoco_warp and sets nconmax / njmax from\n"
                        "the contact peaks of your last simulation preview (or estimates from the start pose).");
    ImGui::TextDisabled("Angle unit in the file");
    ImGui::RadioButton("Radian", &exportAngle_, 0);
    ImGui::SameLine();
    ImGui::RadioButton("Degree", &exportAngle_, 1);
    ImGui::Separator();
    if (!compiled_.ok && compiledRevision_ == editRevision_)
      ImGui::TextColored(pal::v4(pal::kError), "Warning: the model currently does not compile in MuJoCo.");
    if (ImGui::Button("Choose file...", ImVec2(160 * scale_, 0))) {
      std::string def = doc_.modelName.empty() ? "model.xml" : doc_.modelName + ".xml";
      if (exportSplit_) def = "scene.xml";
      fileDialog_.open(FileDialog::Mode::Save, "Export MJCF", {".xml"}, lastDir_, def, kPurposeExport);
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button("Cancel", ImVec2(100 * scale_, 0))) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  drawImportModal();
  drawArrayModal();
  drawAboutWindow();
  drawShortcutsWindow();
  if (showDemo_) ImGui::ShowDemoWindow(&showDemo_);
  drawToasts();

  if (!ImGui::IsAnyItemActive()) activeEditId_ = 0;
}

// ------------------------------------------------------------------------------------ menus

void Editor::drawAddMenuItems(Id ctxBody) {
  Id body = ctxBody;
  bool inWorld = body == kWorldId;
  if (ImGui::MenuItem("Model (creature root)", nullptr, false, true)) addModelRoot();
  if (ImGui::MenuItem(inWorld ? "Body" : "Child body")) addBodyTo(body);
  if (ImGui::BeginMenu("Geom")) {
    for (const auto& e : kGeomTypes) {
      if (e.value == GeomType::Hfield || e.value == GeomType::Mesh) continue;
      if (ImGui::MenuItem(e.label)) addGeomTo(body, e.value);
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Mesh from file...")) openMeshDialog(body);
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Joint", !inWorld)) {
    for (const auto& e : kJointTypes)
      if (ImGui::MenuItem(e.label)) addJointTo(body, e.value);
    ImGui::EndMenu();
  }
  if (ImGui::MenuItem("Site")) addSiteTo(body);
  if (ImGui::MenuItem("Camera")) addCameraTo(body);
  if (ImGui::MenuItem("Light")) addLightTo(body);
  ImGui::Separator();
  if (ImGui::BeginMenu("Environment")) {
    for (const GeneratorInfo& g : generatorList()) {
      if (ImGui::MenuItem(g.label)) addGeneratedEnvironment(g.type);
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s\nParameters can be changed later in the Inspector.", g.description);
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Environment prop")) {
    if (ImGui::MenuItem("Static box")) addEnvironmentProp(GeomType::Box, false);
    if (ImGui::MenuItem("Static cylinder")) addEnvironmentProp(GeomType::Cylinder, false);
    if (ImGui::MenuItem("Static sphere")) addEnvironmentProp(GeomType::Sphere, false);
    ImGui::Separator();
    if (ImGui::MenuItem("Dynamic crate (free body)")) addEnvironmentProp(GeomType::Box, true);
    if (ImGui::MenuItem("Dynamic ball (free body)")) addEnvironmentProp(GeomType::Sphere, true);
    ImGui::EndMenu();
  }
}

void Editor::drawMenuBar() {
  if (!ImGui::BeginMainMenuBar()) return;
  if (ImGui::BeginMenu("File")) {
    if (ImGui::BeginMenu("New")) {
      for (const auto& t : templateList()) {
        if (ImGui::MenuItem(t.name)) {
          std::string id = t.id;
          requestDiscard([this, id] { newDocument(id); });
        }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", t.description);
      }
      ImGui::EndMenu();
    }
    if (ImGui::MenuItem("Open...", "Ctrl+O")) requestDiscard([this] { openDialog(); });
    if (ImGui::BeginMenu("Open recent", !recent_.empty())) {
      for (const auto& r : recent_)
        if (ImGui::MenuItem(r.c_str())) {
          std::string p = r;
          requestDiscard([this, p] { openPath(p); });
          break;
        }
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Save", "Ctrl+S")) save();
    if (ImGui::MenuItem("Save as...", "Ctrl+Shift+S")) saveAs();
    ImGui::Separator();
    if (ImGui::MenuItem("Import into scene...", "Ctrl+I")) importDialog();
    if (ImGui::IsItemHovered())
      ImGui::SetTooltip("Bring the models and/or the environment of another project or MJCF file into this one.");
    if (ImGui::MenuItem("Export MJCF...", "Ctrl+E")) exportDialog();
    ImGui::Separator();
    if (ImGui::MenuItem("Quit", "Ctrl+Q")) quit_ = true;
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Edit")) {
    std::string u = history_.canUndo() ? "Undo " + history_.undoLabel() : "Undo";
    if (ImGui::MenuItem(u.c_str(), "Ctrl+Z", false, history_.canUndo())) undo();
    if (ImGui::MenuItem("Redo", "Ctrl+Y", false, history_.canRedo())) redo();
    ImGui::Separator();
    if (ImGui::MenuItem("Duplicate", "Ctrl+D", false, !sel_.empty())) duplicateSelection();
    if (ImGui::MenuItem("Array...", nullptr, false, !sel_.empty())) showArray_ = true;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Linear or radial copies of the selected body / geom (e.g. legs around a torso).");
    if (ImGui::MenuItem("Delete", "Del", false, !sel_.empty())) deleteSelection();
    if (ImGui::BeginMenu("Mirror", !sel_.empty())) {
      if (ImGui::MenuItem("Across XZ plane (left/right)")) mirrorSelection(1);
      if (ImGui::MenuItem("Across YZ plane (front/back)")) mirrorSelection(0);
      if (ImGui::MenuItem("Across XY plane (up/down)")) mirrorSelection(2);
      ImGui::EndMenu();
    }
    // Movable elements, without those that move with a selected ancestor body.
    std::vector<Id> movable;
    for (Id id : sel_.ids) {
      Kind k = doc_.kindOf(id);
      if (id == kWorldId || !(k == Kind::Body || k == Kind::Geom || k == Kind::Site || k == Kind::Camera || k == Kind::Light))
        continue;
      Id owner = k == Kind::Body ? doc_.body(id)->parent : doc_.ownerBody(id);
      bool covered = false;
      for (Id o : sel_.ids) covered |= o != id && doc_.kindOf(o) == Kind::Body && doc_.isAncestor(o, owner);
      if (!covered) movable.push_back(id);
    }
    if (ImGui::BeginMenu("Align", movable.size() >= 2)) {
      const char* sides[3][3] = {{"Left (min X)", "Center X", "Right (max X)"},
                                 {"Front (min Y)", "Center Y", "Back (max Y)"},
                                 {"Bottom (min Z)", "Center Z", "Top (max Z)"}};
      for (int axis = 0; axis < 3; ++axis) {
        if (axis) ImGui::Separator();
        for (int side = -1; side <= 1; ++side)
          if (ImGui::MenuItem(sides[axis][side + 1])) {
            checkpoint("Align");
            alignElements(doc_, movable, axis, side);
          }
      }
      ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      ImGui::SetTooltip("Line up the selected elements' bounding boxes (world axes). Select at least two.");
    if (ImGui::BeginMenu("Distribute", movable.size() >= 3)) {
      for (int axis = 0; axis < 3; ++axis) {
        const char* label[3] = {"Evenly along X", "Evenly along Y", "Evenly along Z"};
        if (ImGui::MenuItem(label[axis])) {
          checkpoint("Distribute");
          distributeElements(doc_, movable, axis);
        }
      }
      ImGui::EndMenu();
    }
    if (ImGui::IsItemHovered(ImGuiHoveredFlags_AllowWhenDisabled))
      ImGui::SetTooltip("Equal spacing between the outermost selected elements. Select at least three.");
    ImGui::Separator();
    if (ImGui::MenuItem("Select none", "Esc")) sel_.clear();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Add")) {
    drawAddMenuItems(targetBody());
    ImGui::Separator();
    std::vector<Id> joints;
    for (Id id : sel_.ids) if (doc_.kindOf(id) == Kind::Joint) joints.push_back(id);
    if (ImGui::BeginMenu("Actuator for selected joints", !joints.empty())) {
      for (const auto& e : kActuatorTypes)
        if (ImGui::MenuItem(e.label)) addActuatorsFor(joints, e.value);
      ImGui::EndMenu();
    }
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Tools")) {
    drawToolsMenu();
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("View")) {
    ImGui::MenuItem("Grid", nullptr, &vp_.showGrid);
    ImGui::MenuItem("Shadows", nullptr, &vp_.showShadows);
    ImGui::MenuItem("X-ray (see through bodies)", "X", &vp_.xray);
    ImGui::Separator();
    ImGui::MenuItem("Joints", "J", &vp_.showJoints);
    ImGui::MenuItem("Centers of mass", "C", &vp_.showCom);
    ImGui::MenuItem("Inertia boxes", nullptr, &vp_.showInertia);
    ImGui::MenuItem("Body frames", nullptr, &vp_.showFrames);
    ImGui::MenuItem("Sites", nullptr, &vp_.showSites);
    ImGui::MenuItem("Lights", nullptr, &vp_.showLights);
    ImGui::MenuItem("Cameras", nullptr, &vp_.showCameras);
    ImGui::MenuItem("Names", "L", &vp_.showLabels);
    ImGui::MenuItem("Contacts", nullptr, &vp_.showContacts);
    if (ImGui::BeginMenu("Geom groups")) {
      ImGui::TextDisabled("Like MuJoCo's viewer: groups 0-2 shown, 3-5 hidden.");
      ImGui::TextDisabled("Imported robots often keep collision geoms in group 3.");
      for (int g = 0; g < 6; ++g) {
        char label[32];
        std::snprintf(label, sizeof label, "Group %d", g);
        ImGui::MenuItem(label, nullptr, &vp_.groupVisible[g]);
      }
      ImGui::EndMenu();
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Frame selection", "F")) focusSelection();
    if (ImGui::MenuItem("Frame all", "Home")) frameAll();
    if (ImGui::MenuItem("Orthographic", "Numpad 5", vp_.cam.ortho)) vp_.cam.ortho = !vp_.cam.ortho;
    ImGui::Separator();
    if (ImGui::MenuItem("Reset panel layout")) layoutBuilt_ = false, ImGui::DockBuilderRemoveNode(ImGui::GetID("MainDock"));
    ImGui::EndMenu();
  }
  if (ImGui::BeginMenu("Help")) {
    if (ImGui::MenuItem("Keyboard & mouse", "F1")) showShortcuts_ = true;
    if (ImGui::MenuItem("About MJForge")) showAbout_ = true;
    ImGui::MenuItem("ImGui demo (developer)", nullptr, &showDemo_);
    ImGui::EndMenu();
  }
  ImGui::EndMainMenuBar();
}

void Editor::drawToolbar() {
  ImGuiViewport* vp = ImGui::GetMainViewport();
  float h = ImGui::GetFrameHeight() + 10 * scale_;
  ImGuiWindowFlags fl = ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoSavedSettings;
  if (ImGui::BeginViewportSideBar("##toolbar", vp, ImGuiDir_Up, h, fl)) {
    ImGui::SetCursorPosY(ImGui::GetCursorPosY() + 1);
    auto sep = [&] {
      ImGui::SameLine(0, 10 * scale_);
      ImVec2 p = ImGui::GetCursorScreenPos();
      ImGui::GetWindowDrawList()->AddLine(ImVec2(p.x, p.y + 3), ImVec2(p.x, p.y + ImGui::GetFrameHeight() - 3),
                                          IM_COL32(70, 75, 88, 255));
      ImGui::Dummy(ImVec2(1, 1));
      ImGui::SameLine(0, 10 * scale_);
    };
    if (iconButton("##sel", Icon::Select, gizmoOp_ == GizmoOp::Select, "Select (Q)")) gizmoOp_ = GizmoOp::Select;
    ImGui::SameLine();
    if (iconButton("##mov", Icon::Move, gizmoOp_ == GizmoOp::Translate, "Move (W)")) gizmoOp_ = GizmoOp::Translate;
    ImGui::SameLine();
    if (iconButton("##rot", Icon::Rotate, gizmoOp_ == GizmoOp::Rotate, "Rotate (E)")) gizmoOp_ = GizmoOp::Rotate;
    ImGui::SameLine();
    if (iconButton("##scl", Icon::Scale, gizmoOp_ == GizmoOp::Scale, "Resize geom (R)")) gizmoOp_ = GizmoOp::Scale;
    sep();
    ImGui::SetNextItemWidth(78 * scale_);
    int space = gizmoLocal_ ? 1 : 0;
    const char* spaces[] = {"World", "Local"};
    if (ImGui::Combo("##space", &space, spaces, 2)) gizmoLocal_ = space == 1;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Gizmo orientation (Tab toggles)");
    ImGui::SameLine();
    if (iconButton("##snap", Icon::Magnet, snap_, "Snap to grid / angle increments (hold Ctrl to invert)")) snap_ = !snap_;
    if (snap_) {
      ImGui::SameLine();
      ImGui::SetNextItemWidth(70 * scale_);
      ImGui::DragFloat("##snapm", &snapMove_, 0.005f, 0.001f, 10.0f, "%.3g m");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(60 * scale_);
      ImGui::DragFloat("##snapa", &snapAngle_, 0.5f, 1.0f, 90.0f, "%.0f\xC2\xB0");
    }
    sep();
    ImGui::AlignTextToFramePadding();
    ImGui::TextDisabled("Pick");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(84 * scale_);
    int pm = pickMode_ == PickMode::Body ? 0 : 1;
    const char* picks[] = {"Bodies", "Geoms"};
    if (ImGui::Combo("##pick", &pm, picks, 2)) pickMode_ = pm == 0 ? PickMode::Body : PickMode::Geom;
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("What a click in the viewport selects (Alt+click picks the other)");
    sep();
    if (iconButton("##addmodel", Icon::Body, false, "Add a model root (free-floating creature body)")) addModelRoot();
    ImGui::SameLine();
    if (iconButton("##addbody", Icon::Plus, false, "Add a child body to the selection")) addBodyTo(targetBody());
    ImGui::SameLine();
    if (iconButton("##addbox", Icon::Box, false, "Add a box geom")) addGeomTo(targetBody(), GeomType::Box);
    ImGui::SameLine();
    if (iconButton("##addsph", Icon::Sphere, false, "Add a sphere geom")) addGeomTo(targetBody(), GeomType::Sphere);
    ImGui::SameLine();
    if (iconButton("##addcap", Icon::Capsule, false, "Add a capsule geom")) addGeomTo(targetBody(), GeomType::Capsule);
    ImGui::SameLine();
    if (iconButton("##addcyl", Icon::Cylinder, false, "Add a cylinder geom")) addGeomTo(targetBody(), GeomType::Cylinder);
    ImGui::SameLine();
    if (iconButton("##addhinge", Icon::Hinge, false, "Add a hinge joint to the selected body")) {
      Id b = targetBody();
      if (b != kWorldId) addJointTo(b, JointType::Hinge);
      else notify("Select a body first: joints connect a body to its parent", pal::kWarning);
    }
    ImGui::SameLine();
    if (iconButton("##addslide", Icon::Slide, false, "Add a slide joint to the selected body")) {
      Id b = targetBody();
      if (b != kWorldId) addJointTo(b, JointType::Slide);
      else notify("Select a body first: joints connect a body to its parent", pal::kWarning);
    }
    ImGui::SameLine();
    if (iconButton("##addsite", Icon::Site, false, "Add a site (sensor / attachment point)")) addSiteTo(targetBody());
    ImGui::SameLine();
    if (iconButton("##limbtool", Icon::Limb, limbTool_, "Add limb tool (G): click a body to grow a jointed limb")) {
      limbTool_ = !limbTool_;
      if (limbTool_) setPoseMode(false);
    }
    ImGui::SameLine();
    if (iconButton("##posemode", Icon::Pose, poseMode_, "Pose mode (P): set the start pose by moving joints"))
      setPoseMode(!poseMode_);
    sep();
    if (iconButton("##mirror", Icon::Mirror, false, "Mirror selected body left/right (Ctrl+M)")) mirrorSelection(1);
    sep();
    drawSimToolbarButtons();

    // Validation status on the right.
    bool upToDate = compiledRevision_ == editRevision_;
    std::string statusStr;
    ImU32 col;
    Icon icon;
    int nerr = report_.counts[int(Severity::Error)] + ((upToDate && !compiled_.ok) ? 1 : 0);
    int nwarn = report_.counts[int(Severity::Warning)];
    if (!upToDate) { statusStr = "Checking..."; col = pal::kTextDim; icon = Icon::Info; }
    else if (nerr > 0) { statusStr = std::to_string(nerr) + (nerr == 1 ? " error" : " errors"); col = pal::kError; icon = Icon::Error; }
    else if (nwarn > 0) { statusStr = "Compiles, " + std::to_string(nwarn) + (nwarn == 1 ? " warning" : " warnings"); col = pal::kWarning; icon = Icon::Warning; }
    else { statusStr = "Compiles in MuJoCo"; col = pal::kOk; icon = Icon::Ok; }
    const char* status = statusStr.c_str();
    float w = ImGui::CalcTextSize(status).x + ImGui::GetFrameHeight() + 20 * scale_;
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - w);
    iconInline(icon, col);
    ImGui::SameLine();
    ImGui::TextColored(pal::v4(col), "%s", status);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", !compiled_.ok && upToDate ? compiled_.error.c_str() : "Click to open the Issues panel");
    if (ImGui::IsItemClicked()) ImGui::SetWindowFocus("Issues");
  }
  ImGui::End();
}

void Editor::drawStatusBar() {
  ImGuiViewport* vp = ImGui::GetMainViewport();
  float h = ImGui::GetFrameHeight();
  if (ImGui::BeginViewportSideBar("##status", vp, ImGuiDir_Down, h, ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_MenuBar)) {
    if (ImGui::BeginMenuBar()) {
      int dofs = 0, nbody = int(doc_.bodies.size()) - 1;
      for (const Body& b : doc_.bodies) dofs += doc_.bodyDofs(b.id);
      // Mass of the models only: static environment bodies have a mass in MuJoCo but it never matters.
      double mass = report_.modelMass;
      ImGui::TextDisabled("%s", path_.empty() ? "unsaved project" : path_.c_str());
      ImGui::SameLine(0, 24);
      ImGui::Text("%d bodies  %zu geoms  %d DoF  %zu actuators  %zu sensors", nbody, doc_.geoms.size(), dofs,
                  doc_.actuators.size(), doc_.sensors.size());
      if (mass > 0) {
        ImGui::SameLine(0, 24);
        ImGui::Text("model mass %.3g kg", mass);
      }
      if (compiled_.ok && compiledRevision_ == editRevision_) {
        ImGui::SameLine(0, 24);
        ImGui::TextDisabled("compiled in %.1f ms", compiled_.compileMs);
      }
      if (sim_.active()) {
        ImGui::SameLine(0, 24);
        ImGui::TextColored(pal::v4(pal::kOk), "simulating: %d contacts, %d constraint rows", int(sim_.data()->ncon),
                           int(sim_.data()->nefc));
      }
      ImGui::EndMenuBar();
    }
  }
  ImGui::End();
}

void Editor::drawToasts() {
  double t = nowSeconds();
  toasts_.erase(std::remove_if(toasts_.begin(), toasts_.end(), [t](const Toast& x) { return x.until < t; }), toasts_.end());
  if (toasts_.empty()) return;
  ImGuiViewport* vp = ImGui::GetMainViewport();
  float y = vp->WorkPos.y + vp->WorkSize.y - 16 * scale_;
  for (int i = int(toasts_.size()) - 1; i >= 0; --i) {
    const Toast& to = toasts_[i];
    ImVec2 ts = ImGui::CalcTextSize(to.text.c_str());
    ImVec2 pad(14 * scale_, 9 * scale_);
    ImVec2 size(ts.x + pad.x * 2, ts.y + pad.y * 2);
    ImVec2 pos(vp->WorkPos.x + vp->WorkSize.x * 0.5f - size.x * 0.5f, y - size.y);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    float alpha = float(std::min(1.0, (to.until - t) * 3.0));
    dl->AddRectFilled(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(32, 35, 43, int(235 * alpha)), 6 * scale_);
    dl->AddRect(pos, ImVec2(pos.x + size.x, pos.y + size.y), IM_COL32(70, 76, 90, int(255 * alpha)), 6 * scale_);
    ImU32 c = (to.color & 0x00FFFFFF) | (ImU32(255 * alpha) << 24);
    dl->AddText(ImVec2(pos.x + pad.x, pos.y + pad.y), c, to.text.c_str());
    y -= size.y + 6 * scale_;
  }
}

void Editor::drawAboutWindow() {
  if (!showAbout_) return;
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(520 * scale_, 0), ImGuiCond_Appearing);
  if (ImGui::Begin("About MJForge", &showAbout_, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::Text("MJForge %s", MJFORGE_VERSION);
    ImGui::TextDisabled("Visual model & environment editor for MuJoCo / MuJoCo Warp");
    ImGui::Separator();
    ImGui::Text("MuJoCo library: %s (built against %s)", mj_versionString(), MJFORGE_MUJOCO_VERSION);
    ImGui::TextWrapped("Exports MJCF that is validated live with the MuJoCo compiler. MuJoCo Warp loads the same "
                       "XML through mujoco.MjModel.from_xml_path() + mujoco_warp.put_model().");
    ImGui::Separator();
    ImGui::TextDisabled("Mouse: right-drag orbit, middle-drag or Shift+right-drag pan, wheel zoom, left-click select.");
    ImGui::TextDisabled("Keys: W/E/R move/rotate/resize, F frame, Ctrl+D duplicate, Del delete, Ctrl+Z undo.");
    ImGui::TextDisabled("Simulation: Space play/pause, Shift+Space stop, Ctrl+R reset, drag a body to push it.");
  }
  ImGui::End();
}

void Editor::drawShortcutsWindow() {
  if (!showShortcuts_) return;
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  ImGui::SetNextWindowSize(ImVec2(560 * scale_, 0), ImGuiCond_Appearing);
  if (ImGui::Begin("Keyboard & mouse", &showShortcuts_, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
    auto table = [&](const char* id, std::initializer_list<std::pair<const char*, const char*>> rows) {
      if (!ImGui::BeginTable(id, 2, ImGuiTableFlags_RowBg | ImGuiTableFlags_SizingFixedFit)) return;
      for (const auto& [k, v] : rows) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(0);
        ImGui::TextColored(pal::v4(pal::kAccent), "%s", k);
        ImGui::TableSetColumnIndex(1);
        ImGui::TextUnformatted(v);
      }
      ImGui::EndTable();
    };
    ImGui::SeparatorText("Mouse");
    table("##mouse", {{"Left click", "select (Shift: add, Ctrl: toggle, Alt: geoms instead of bodies)"},
                      {"Double click", "frame the clicked element"},
                      {"Right drag / click", "orbit / context menu"},
                      {"Middle drag, Shift+right drag", "pan"},
                      {"Wheel", "zoom toward the cursor"},
                      {"Left drag (simulating)", "push the body under the cursor"}});
    ImGui::SeparatorText("Editing");
    table("##edit", {{"Q  W  E  R", "select, move, rotate, resize"},
                     {"Tab", "gizmo in world / local frame (hold Ctrl to invert snapping)"},
                     {"G", "add-limb tool (Esc leaves it)"},
                     {"P", "pose mode: set the start pose by moving joints (Esc leaves it)"},
                     {"Ctrl+D  Ctrl+M  Del", "duplicate, mirror left/right, delete"},
                     {"Ctrl+Z  Ctrl+Y", "undo, redo"},
                     {"F2", "rename (outliner)"}});
    ImGui::SeparatorText("View");
    table("##view", {{"F  Home", "frame selection, frame everything"},
                     {"Numpad 1 3 7 5", "front, side, top, orthographic"},
                     {"X  J  C  L", "x-ray, joints, centers of mass, names"}});
    ImGui::SeparatorText("Simulation");
    table("##sim", {{"Space  Shift+Space", "play / pause, stop"},
                    {"Ctrl+R  Right arrow", "reset, single step while paused"}});
    ImGui::SeparatorText("Files");
    table("##files", {{"Ctrl+O  Ctrl+S  Ctrl+Shift+S", "open (project or MJCF), save, save as"},
                      {"Ctrl+I  Ctrl+E  Ctrl+Q", "import into scene, export MJCF, quit"}});
  }
  ImGui::End();
}

void Editor::handleShortcuts() {
  ImGuiIO& io = ImGui::GetIO();
  if (io.WantTextInput) return;
  bool ctrl = io.KeyCtrl, shift = io.KeyShift;
  auto pressed = [](ImGuiKey k) { return ImGui::IsKeyPressed(k, false); };
  if (ctrl && pressed(ImGuiKey_Z)) { shift ? redo() : undo(); return; }
  if (ctrl && pressed(ImGuiKey_Y)) { redo(); return; }
  if (ctrl && pressed(ImGuiKey_S)) { shift ? saveAs() : save(); return; }
  if (ctrl && pressed(ImGuiKey_O)) { requestDiscard([this] { openDialog(); }); return; }
  if (ctrl && pressed(ImGuiKey_I)) { importDialog(); return; }
  if (ctrl && pressed(ImGuiKey_E)) { exportDialog(); return; }
  if (ctrl && pressed(ImGuiKey_Q)) { quit_ = true; return; }
  if (ctrl && pressed(ImGuiKey_D)) { duplicateSelection(); return; }
  if (ctrl && pressed(ImGuiKey_M)) { mirrorSelection(1); return; }
  if (ctrl && pressed(ImGuiKey_R)) { resetSimulation(); return; }
  if (ctrl) return;
  if (pressed(ImGuiKey_F1)) showShortcuts_ = !showShortcuts_;
  // Single-key shortcuts only when the mouse is not over a text field.
  if (pressed(ImGuiKey_Delete) || (pressed(ImGuiKey_Backspace) && !io.WantCaptureKeyboard)) deleteSelection();
  if (pressed(ImGuiKey_Escape)) {
    if (limbTool_) limbTool_ = false;
    else if (poseMode_) setPoseMode(false);
    else sel_.clear();
  }
  if (io.WantCaptureKeyboard && !vp_.hovered) return;
  if (pressed(ImGuiKey_Space)) shift ? stopSimulation() : toggleSimulation();
  if (ImGui::IsKeyPressed(ImGuiKey_RightArrow, true) && sim_.active() && sim_.paused()) stepSimulation();
  if (pressed(ImGuiKey_G)) {
    limbTool_ = !limbTool_;
    if (limbTool_) setPoseMode(false);
  }
  if (pressed(ImGuiKey_P)) setPoseMode(!poseMode_);
  if (pressed(ImGuiKey_Q)) gizmoOp_ = GizmoOp::Select;
  if (pressed(ImGuiKey_W)) gizmoOp_ = GizmoOp::Translate;
  if (pressed(ImGuiKey_E)) gizmoOp_ = GizmoOp::Rotate;
  if (pressed(ImGuiKey_R)) gizmoOp_ = GizmoOp::Scale;
  if (pressed(ImGuiKey_Tab)) gizmoLocal_ = !gizmoLocal_;
  if (pressed(ImGuiKey_F)) focusSelection();
  if (pressed(ImGuiKey_Home)) frameAll();
  if (pressed(ImGuiKey_X)) vp_.xray = !vp_.xray;
  if (pressed(ImGuiKey_J)) vp_.showJoints = !vp_.showJoints;
  if (pressed(ImGuiKey_C)) vp_.showCom = !vp_.showCom;
  if (pressed(ImGuiKey_L)) vp_.showLabels = !vp_.showLabels;
  if (pressed(ImGuiKey_Keypad1)) { vp_.cam.azimuth = 0; vp_.cam.elevation = 0; }
  if (pressed(ImGuiKey_Keypad3)) { vp_.cam.azimuth = 90; vp_.cam.elevation = 0; }
  if (pressed(ImGuiKey_Keypad7)) { vp_.cam.elevation = 89; }
  if (pressed(ImGuiKey_Keypad5)) vp_.cam.ortho = !vp_.cam.ortho;
}

// ------------------------------------------------------------------------------------ tools

std::vector<Id> Editor::selectedModelRoots() const {
  std::vector<Id> roots;
  for (Id id : sel_.ids) {
    Id b = doc_.kindOf(id) == Kind::Body ? id : doc_.ownerBody(id);
    if (b == kWorldId || b == kNoId) continue;
    Id r = doc_.rootOf(b);
    if (r != kWorldId && std::find(roots.begin(), roots.end(), r) == roots.end()) roots.push_back(r);
  }
  if (roots.empty())
    for (const Body& b : doc_.bodies)
      if (b.id != kWorldId && b.parent == kWorldId && b.role == Role::Model) roots.push_back(b.id);
  return roots;
}

void Editor::drawToolsMenu() {
  std::vector<Id> roots = selectedModelRoots();
  std::string target = roots.size() == 1 ? quoteName(doc_.nameOf(roots[0])) : std::to_string(roots.size()) + " models";
  ImGui::TextDisabled("Applies to: %s (select a model to target it)", target.c_str());
  ImGui::Separator();
  if (ImGui::BeginMenu("Collisions", !roots.empty())) {
    auto preset = [&](const char* label, const char* tip, CollisionPreset p) {
      if (ImGui::MenuItem(label)) {
        checkpoint(label);
        int n = 0;
        for (Id r : roots) n += applyCollisionPreset(doc_, r, p);
        notify(std::string(label) + ": " + std::to_string(n) + " geoms changed", pal::kOk, 3);
      }
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tip);
    };
    preset("Disable self-collisions (keep world contacts)",
           "Model geoms get contype=2, conaffinity=1: they collide with the environment (contype/conaffinity 1) but "
           "not with each other. Fewer contact pairs = faster MJWarp training.",
           CollisionPreset::NoSelfCollision);
    preset("Only the feet touch the world",
           "The lowest geoms of each model keep contacts with the environment; every other model geom gets "
           "contype=conaffinity=0. The cheapest setup for locomotion on open terrain.",
           CollisionPreset::FeetOnly);
    preset("Everything collides (MuJoCo default)", "contype = conaffinity = 1 for all model geoms.",
           CollisionPreset::AllCollide);
    ImGui::EndMenu();
  }
  std::vector<Id> bodies;
  for (Id id : sel_.ids) if (doc_.kindOf(id) == Kind::Body && id != kWorldId) bodies.push_back(id);
  if (ImGui::MenuItem("Exclude contacts between selected bodies", nullptr, false, bodies.size() >= 2)) {
    checkpoint("Exclude contacts");
    int n = 0;
    for (size_t i = 0; i < bodies.size(); ++i)
      for (size_t k = i + 1; k < bodies.size(); ++k, ++n) addExclude(doc_, bodies[i], bodies[k]);
    notify("Added " + std::to_string(n) + " contact exclusion(s)", pal::kOk, 2);
  }
  if (ImGui::BeginMenu("Constraints & tendons")) {
    drawConstraintMenu();
    ImGui::EndMenu();
  }
  if (ImGui::MenuItem("Place models on the floor", nullptr, false, !roots.empty())) {
    checkpoint("Place on floor");
    for (Id r : roots) placeOnFloor(doc_, r);
  }
  ImGui::Separator();
  if (ImGui::BeginMenu("Add sensors", !roots.empty())) {
    if (ImGui::MenuItem("IMU at the model root (accelerometer, gyro, orientation)")) {
      checkpoint("Add IMU");
      for (Id r : roots) {
        Id s = addSite(doc_, r, {}, doc_.nameOf(r) + "_imu");
        addSensor(doc_, SensorType::Accelerometer, s);
        addSensor(doc_, SensorType::Gyro, s);
        addSensor(doc_, SensorType::FrameQuat, s, doc_.nameOf(r) + "_orientation");
      }
    }
    if (ImGui::MenuItem("Foot touch sensors (sites on the lowest geoms)")) {
      checkpoint("Add foot sensors");
      int n = 0;
      for (Id r : roots)
        for (Id g : lowestGeoms(doc_, r)) {
          const Geom* gg = doc_.geom(g);
          Id s = addSite(doc_, gg->body, {gg->pos, gg->quat}, gg->name + "_touch_site");
          Site* site = doc_.site(s);
          site->type = gg->type == GeomType::Plane ? GeomType::Sphere : gg->type;
          site->size = gg->size * 1.15;
          site->rgba = {0.2f, 0.9f, 0.5f, 0.3f};
          addSensor(doc_, SensorType::Touch, s, gg->name + "_touch");
          ++n;
        }
      notify("Added " + std::to_string(n) + " touch sensors", pal::kOk, 2);
    }
    ImGui::EndMenu();
  }
}

// ------------------------------------------------------------------------------------ actions

Id Editor::targetBody() const {
  Id p = sel_.primary();
  switch (doc_.kindOf(p)) {
    case Kind::Body: return p;
    case Kind::Geom:
    case Kind::Joint:
    case Kind::Site:
    case Kind::Camera:
    case Kind::Light: return doc_.ownerBody(p);
    default: return kWorldId;
  }
}

void Editor::deleteSelection() {
  std::vector<Id> ids;
  for (Id id : sel_.ids) if (id != kWorldId && doc_.exists(id)) ids.push_back(id);
  if (ids.empty()) return;
  checkpoint("Delete");
  deleteElements(doc_, ids);
  sel_.clear();
  notify("Deleted " + std::to_string(ids.size()) + " element(s)", pal::kTextDim, 1.5);
}

void Editor::duplicateSelection() {
  std::vector<Id> ids;
  for (Id id : sel_.ids) {
    Kind k = doc_.kindOf(id);
    if (k == Kind::Body || k == Kind::Geom || k == Kind::Joint || k == Kind::Site || k == Kind::Camera || k == Kind::Light)
      if (id != kWorldId) ids.push_back(id);
  }
  if (ids.empty()) return;
  checkpoint("Duplicate");
  double off = std::max(0.05, vp_.cam.distance * 0.06);
  auto tops = duplicate(doc_, ids, {off, off, 0});
  sel_.ids = tops;
}

void Editor::mirrorSelection(int axis) {
  std::vector<Id> bodies;
  for (Id id : sel_.ids) if (doc_.kindOf(id) == Kind::Body && id != kWorldId) bodies.push_back(id);
  if (bodies.empty()) {
    notify("Select a body to mirror (its whole subtree is copied)", pal::kWarning);
    return;
  }
  checkpoint("Mirror");
  std::vector<Id> out;
  for (Id b : bodies) {
    Id m = mirrorSubtree(doc_, b, axis);
    if (m) out.push_back(m);
  }
  sel_.ids = out;
  notify("Mirrored " + std::to_string(out.size()) + " subtree(s)", pal::kOk, 2);
}

void Editor::focusSelection() {
  if (sel_.empty()) {
    frameAll();
    return;
  }
  auto poses = computeBodyPoses(doc_);
  Aabb box;
  for (Id id : sel_.ids) {
    Kind k = doc_.kindOf(id);
    if (k == Kind::Body && id != kWorldId) box.add(subtreeAabb(doc_, poses, id));
    if (k == Kind::Geom) {
      const Geom* g = doc_.geom(id);
      if (g->type == GeomType::Plane) box.add(elementWorldPose(doc_, poses, id).pos);
      else box.add(geomWorldAabb(doc_, *g, elementWorldPose(doc_, poses, id)));
    }
    if (k == Kind::Joint || k == Kind::Site || k == Kind::Camera || k == Kind::Light) {
      // Frame the element together with the body it belongs to.
      Id owner = doc_.ownerBody(id);
      if (owner != kWorldId && owner != kNoId) box.add(subtreeAabb(doc_, poses, owner));
      Vec3 p = elementWorldPose(doc_, poses, id).pos;
      box.add(p - Vec3{0.15, 0.15, 0.15});
      box.add(p + Vec3{0.15, 0.15, 0.15});
    }
  }
  if (!box.valid()) return;
  vp_.cam.frame(box.center(), std::max(0.1, length(box.extent()) * 0.5));
}

void Editor::frameModels() {
  // Documents open framed on their creatures; terrains can be much larger (Home frames all).
  Aabb box = modelsBounds(doc_);
  if (!box.valid()) {
    frameAll();
    return;
  }
  vp_.cam.frame(box.center(), std::max(0.4, length(box.extent()) * 0.5));
}

void Editor::frameAll() {
  auto poses = computeBodyPoses(doc_);
  Aabb box;
  for (const Body& b : doc_.bodies)
    if (b.parent == kWorldId) box.add(subtreeAabb(doc_, poses, b.id));
  for (const Geom& g : doc_.geoms)
    if (g.body == kWorldId && g.type != GeomType::Plane) box.add(geomWorldAabb(doc_, g, {g.pos, g.quat}));
  if (!box.valid()) {
    vp_.cam.target = {0, 0, 0.3};
    vp_.cam.distance = 3;
    return;
  }
  vp_.cam.frame(box.center(), std::max(0.4, length(box.extent()) * 0.5));
}

static Vec3 groundPointUnderCamera(const OrbitCamera& cam) { return {cam.target.x, cam.target.y, 0}; }

Id Editor::addModelRoot() {
  checkpoint("Add model");
  Vec3 p = groundPointUnderCamera(vp_.cam);
  Id b = addBody(doc_, kWorldId, {{p.x, p.y, 0.5}, {}}, "creature", Role::Model);
  addJoint(doc_, b, JointType::Free, {}, {0, 0, 1}, doc_.body(b)->name + "_root");
  Id g = addGeom(doc_, b, GeomType::Sphere, {0.1, 0, 0}, {}, doc_.body(b)->name + "_torso");
  doc_.geom(g)->rgba = {0.92f, 0.55f, 0.20f, 1};
  sel_.set(b);
  notify("Added a free-floating model root. Add child bodies and joints to build limbs.", pal::kTextDim, 4);
  return b;
}

Id Editor::addBodyTo(Id parent) {
  checkpoint("Add body");
  Vec3 offset{};
  if (parent == kWorldId) {
    Vec3 p = groundPointUnderCamera(vp_.cam);
    offset = {p.x, p.y, 0.5};
  } else {
    auto poses = computeBodyPoses(doc_);
    Aabb box = subtreeAabb(doc_, poses, parent);
    double s = box.valid() ? std::max(0.05, length(box.extent()) * 0.25) : 0.1;
    offset = {s, 0, 0};
  }
  Id b = addBody(doc_, parent, {offset, {}}, parent == kWorldId ? "body" : doc_.body(parent)->name + "_child",
                 parent == kWorldId ? Role::Model : Role::Model);
  if (parent != kWorldId) {
    Id g = addGeom(doc_, b, GeomType::Capsule, {0.03, 0.1, 0}, {}, doc_.body(b)->name + "_geom");
    setGeomFromTo(*doc_.geom(g), {0, 0, 0}, {0.2, 0, 0});
    addJoint(doc_, b, JointType::Hinge, {}, {0, 1, 0});
  } else {
    addGeom(doc_, b, GeomType::Box, {0.1, 0.1, 0.1}, {});
  }
  sel_.set(b);
  return b;
}

Id Editor::addGeomTo(Id body, GeomType type) {
  checkpoint("Add geom");
  Vec3 size{0.1, 0.1, 0.1};
  switch (type) {
    case GeomType::Sphere: size = {0.08, 0, 0}; break;
    case GeomType::Capsule: size = {0.04, 0.12, 0}; break;
    case GeomType::Cylinder: size = {0.06, 0.08, 0}; break;
    case GeomType::Ellipsoid: size = {0.12, 0.08, 0.06}; break;
    case GeomType::Plane: size = {1, 1, 0.05}; break;
    default: break;
  }
  Pose local;
  if (body == kWorldId) {
    Vec3 p = groundPointUnderCamera(vp_.cam);
    double half = type == GeomType::Sphere ? size.x : (type == GeomType::Plane ? 0 : (type == GeomType::Box ? size.z : size.y + (type == GeomType::Capsule ? size.x : 0)));
    local.pos = {p.x, p.y, half};
  }
  Id g = addGeom(doc_, body, type, size, local);
  if (body == kWorldId) doc_.geom(g)->rgba = {0.62f, 0.64f, 0.68f, 1};
  sel_.set(g);
  return g;
}

void Editor::openMeshDialog(Id body) {
  meshTarget_ = body;
  fileDialog_.open(FileDialog::Mode::Open, "Add a mesh (STL, OBJ or MSH)", {".stl", ".obj", ".msh"}, lastDir_, "", kPurposeMesh);
}

Id Editor::addMeshFromFile(Id body, const std::string& path) {
  if (!doc_.body(body)) body = kWorldId;
  Document tmp;
  Pose frame;
  std::string err;
  Id mid = loadMeshFile(tmp, path, 1.0, &frame, &err);
  bool mm = false;
  if (mid && length(meshBounds(*tmp.mesh(mid)).extent()) > 20) {
    // CAD files are usually in millimeters; a part over 20 m across is unlikely.
    Document scaled;
    Pose f;
    if (Id m = loadMeshFile(scaled, path, 0.001, &f, &err)) {
      tmp = std::move(scaled);
      mid = m;
      frame = f;
      mm = true;
    }
  }
  if (!mid) {
    notify("Could not load the mesh: " + err, pal::kError, 6);
    return kNoId;
  }
  checkpoint("Add mesh");
  MeshAsset a = *tmp.mesh(mid);
  a.id = doc_.newId();
  a.name = doc_.uniqueName(Kind::Mesh, a.name);
  doc_.meshes.push_back(a);
  Id g = addGeom(doc_, body, GeomType::Mesh, {}, frame, body == kWorldId ? a.name : doc_.body(body)->name + "_" + a.name);
  Geom& geom = *doc_.geom(g);
  geom.mesh = a.id;
  if (body == kWorldId) {
    // Props stand on the floor under the camera.
    Aabb box = geomWorldAabb(doc_, geom, {geom.pos, geom.quat});
    Vec3 p = groundPointUnderCamera(vp_.cam);
    geom.pos += Vec3{p.x - box.center().x, p.y - box.center().y, -box.lo.z};
    geom.rgba = {0.62f, 0.64f, 0.68f, 1};
  }
  doc_.touch();
  sel_.set(g);
  lastDir_ = fs::path(path).parent_path().string();
  char msg[256];
  std::snprintf(msg, sizeof msg, "Added mesh %s (%zu vertices, %zu triangles)%s", a.name.c_str(), a.vertexCount(), a.faceCount(),
                mm ? ", scaled from millimeters to meters" : "");
  notify(msg, pal::kOk, 5);
  return g;
}

Id Editor::addJointTo(Id body, JointType type) {
  if (body == kWorldId || !doc_.body(body)) return kNoId;
  if (type == JointType::Free && doc_.body(body)->parent != kWorldId) {
    notify("Free joints are only allowed on bodies attached directly to the world", pal::kWarning, 4);
    return kNoId;
  }
  checkpoint("Add joint");
  Id j = addJoint(doc_, body, type, {}, type == JointType::Slide ? Vec3{1, 0, 0} : Vec3{0, 1, 0});
  sel_.set(j);
  return j;
}

Id Editor::addSiteTo(Id body) {
  checkpoint("Add site");
  Pose local;
  if (body == kWorldId) local.pos = groundPointUnderCamera(vp_.cam) + Vec3{0, 0, 0.1};
  Id s = addSite(doc_, body, local);
  sel_.set(s);
  return s;
}

Id Editor::addCameraTo(Id body) {
  checkpoint("Add camera");
  Pose local;
  if (body == kWorldId) {
    // Camera looking at the current view target from the current viewpoint.
    Vec3 eye = vp_.cam.eye();
    Vec3 f = normalize(vp_.cam.target - eye);
    Vec3 r = normalize(cross(f, {0, 0, 1}), {1, 0, 0});
    Vec3 u = cross(r, f);
    Mat3 m;
    for (int i = 0; i < 3; ++i) {
      m(i, 0) = r[i];
      m(i, 1) = u[i];
      m(i, 2) = -f[i];
    }
    local = {eye, fromMat3(m)};
  }
  Id c = addCamera(doc_, body, local);
  sel_.set(c);
  return c;
}

Id Editor::addLightTo(Id body) {
  checkpoint("Add light");
  Vec3 pos = body == kWorldId ? groundPointUnderCamera(vp_.cam) + Vec3{0, 0, 3} : Vec3{0, 0, 0.3};
  Id l = addLight(doc_, body, pos, {0, 0, -1});
  sel_.set(l);
  return l;
}

void Editor::addActuatorsFor(const std::vector<Id>& joints, ActuatorType type) {
  if (joints.empty()) return;
  checkpoint("Add actuators");
  std::vector<Id> made;
  for (Id j : joints) {
    const Joint* jj = doc_.joint(j);
    if (!jj || jj->type == JointType::Free) continue;
    made.push_back(addActuator(doc_, type, j));
  }
  if (!made.empty()) sel_.ids = made;
  notify("Added " + std::to_string(made.size()) + " " + toXml(type) + " actuator(s)", pal::kOk, 2);
}

Id Editor::addGeneratedEnvironment(const std::string& type) {
  checkpoint("Add environment");
  // In front of the models (+X is "forward" for the templates), or at the view center.
  Vec3 origin = groundPointUnderCamera(vp_.cam);
  Aabb models = modelsBounds(doc_);
  bool centered = type == "rough" || type == "waves" || type == "arena" || type == "obstacles";
  if (models.valid()) origin = centered ? Vec3{models.center().x, models.center().y, 0}
                                        : Vec3{models.hi.x + 0.4, models.center().y, 0};
  Id b = addGenerated(doc_, type, origin);
  if (b) sel_.set(b);
  return b;
}

Id Editor::addEnvironmentProp(GeomType type, bool dynamic) {
  checkpoint("Add prop");
  Vec3 p = groundPointUnderCamera(vp_.cam);
  Vec3 size = type == GeomType::Box ? Vec3{0.25, 0.25, 0.25}
            : type == GeomType::Sphere ? Vec3{0.15, 0, 0}
                                        : Vec3{0.2, 0.3, 0};
  double half = type == GeomType::Box ? size.z : (type == GeomType::Sphere ? size.x : size.y);
  if (!dynamic) {
    Id g = addGeom(doc_, kWorldId, type, size, {{p.x, p.y, half}, {}}, "obstacle");
    doc_.geom(g)->rgba = {0.55f, 0.58f, 0.62f, 1};
    sel_.set(g);
    return g;
  }
  Id b = addBody(doc_, kWorldId, {{p.x, p.y, half + 0.01}, {}}, type == GeomType::Box ? "crate" : "ball", Role::Environment);
  addJoint(doc_, b, JointType::Free, {}, {0, 0, 1}, doc_.body(b)->name + "_free");
  Id g = addGeom(doc_, b, type, size, {});
  doc_.geom(g)->rgba = type == GeomType::Box ? Color{0.62f, 0.45f, 0.28f, 1} : Color{0.3f, 0.55f, 0.85f, 1};
  doc_.geom(g)->density = 300;
  sel_.set(b);
  return b;
}

}  // namespace mjf
