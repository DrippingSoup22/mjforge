// Editor: owns the document, history and selection, and draws all panels.
#pragma once

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "analysis/Analysis.h"
#include "app/Cli.h"
#include "core/Document.h"
#include "core/History.h"
#include "core/MjcfWriter.h"
#include "render/Camera.h"
#include "render/DebugDraw.h"
#include "render/Renderer.h"
#include "sim/MujocoBridge.h"
#include "sim/Simulator.h"
#include "ui/FileDialog.h"
#include "ui/Props.h"

struct GLFWwindow;

namespace mjf {

struct Selection {
  std::vector<Id> ids;  // the last one is the primary selection
  bool empty() const { return ids.empty(); }
  bool contains(Id id) const;
  Id primary() const { return ids.empty() ? kNoId : ids.back(); }
  void set(Id id);
  void add(Id id);
  void toggle(Id id);
  void clear() { ids.clear(); }
  void prune(const Document& doc);
};

enum class GizmoOp { Select, Translate, Rotate, Scale };
enum class PickMode { Body, Geom };

// Everything the 3D viewport needs between frames.
struct ViewportState {
  OrbitCamera cam;
  RenderTarget target;
  bool hovered = false;
  float x = 0, y = 0, w = 1, h = 1;  // screen rect of the image
  Id hoverId = kNoId;
  bool gizmoUsing = false;
  // camera drag
  int dragButton = -1;
  bool dragMoved = false;
  float pressX = 0, pressY = 0;
  // display toggles
  bool showGrid = true, showShadows = true, showJoints = true, showCom = false, showSites = true;
  bool showFrames = false, showLabels = false, showLights = true, showCameras = true, xray = false;
  bool showInertia = false, showContacts = true;
};

struct Toast {
  std::string text;
  unsigned int color;
  double until;
};

class Editor {
 public:
  Editor();
  ~Editor();

  void init(GLFWwindow* window, float scale, const std::string& configDir);
  void applyCli(const CliOptions& cli);
  void frame();
  bool wantsContinuousRendering() const;
  // True when something changed without user input (e.g. a validation result arrived).
  bool hasPendingWork();
  bool quitRequested() const { return quit_; }
  bool confirmQuit();

  // ----- document & transactions -----
  Document& doc() { return doc_; }
  // Records an undo step for an upcoming change (coalesced while a widget stays active).
  void change();
  void checkpoint(const char* label);  // explicit undo step for discrete operations
  void markDirty();                    // schedule validation, set modified flag
  void undo();
  void redo();

  // ----- selection -----
  Selection& selection() { return sel_; }
  void select(Id id, bool additive = false, bool toggle = false);

  // ----- actions -----
  void newDocument(const std::string& templateId);
  bool openPath(const std::string& path);
  bool savePath(const std::string& path);
  void save();
  void saveAs();
  void openDialog();
  void exportDialog();
  bool exportTo(const std::string& path, ExportOptions::Scope scope, bool split);
  void deleteSelection();
  void duplicateSelection();
  void mirrorSelection(int axis);
  void focusSelection();
  void frameAll();
  void notify(const std::string& text, unsigned int color = 0, double seconds = 3.5);

  // Creation helpers used by menus and context menus.
  Id addModelRoot();
  Id addBodyTo(Id parent);
  Id addGeomTo(Id body, GeomType type);
  Id addJointTo(Id body, JointType type);
  Id addSiteTo(Id body);
  Id addCameraTo(Id body);
  Id addLightTo(Id body);
  void addActuatorsFor(const std::vector<Id>& joints, ActuatorType type);
  Id addEnvironmentProp(GeomType type, bool dynamic);
  // The body new things get attached to: the selected body, or the owner of the selected element.
  Id targetBody() const;

  // ----- simulation preview -----
  bool simulating() const { return sim_.active(); }
  void startSimulation(bool paused = false);
  void stopSimulation();
  void toggleSimulation();  // play / pause; starts when stopped
  void resetSimulation();
  void stepSimulation();
  Simulator& simulator() { return sim_; }

  const CompileResult& compiled() const { return compiled_; }
  const AnalysisReport& report() const { return report_; }
  bool compileValid() const { return compiled_.ok && compiledRevision_ == editRevision_; }

 private:
  // panels
  void buildDefaultLayout(unsigned int dockId);
  void drawMenuBar();
  void drawToolbar();
  void drawViewport();
  void drawOutliner();
  void drawInspector();
  void drawActuatorsPanel();
  void drawSensorsPanel();
  void drawIssuesPanel();
  void drawXmlPanel();
  void drawScenePanel();
  void drawStatusBar();
  void drawSimulationPanel();
  void drawSimToolbarButtons();
  void launchSimulation();  // starts from the current compile result
  void drawToasts();
  void drawAboutWindow();
  void handleShortcuts();
  void drawAddMenuItems(Id contextBody);
  void drawToolsMenu();
  std::vector<Id> selectedModelRoots() const;
  void drawElementContextMenu(Id id);

  // inspector sections
  void inspectBody(Body& b);
  void inspectGeom(Geom& g);
  void inspectJoint(Joint& j);
  void inspectSite(Site& s);
  void inspectCamera(Camera& c);
  void inspectLight(Light& l);
  void inspectActuator(Actuator& a);
  void inspectSensor(Sensor& s);
  void inspectMaterial(Material& m);
  void inspectTexture(Texture& t);
  void inspectExclude(Exclude& e);
  void inspectWorld();
  template <class T, class F> void applyToSelected(F&& f);

  // viewport internals
  void buildScene(std::vector<DrawItem>& items);
  void buildOverlays();
  Id pick(const Ray& ray, double* tHit = nullptr);
  void viewportInput();
  void drawGizmo();
  void drawViewportOverlayUI();
  // simulation view
  void applySimPoses();
  void buildSimOverlays();
  void drawSimBadge();
  bool simMouseDown(const Ray& ray);  // starts a perturbation drag; true if a moving body was grabbed
  void simMouseDrag(const Ray& ray);
  bool movesInSimulation(Id element) const;
  // Simulated joint position relative to the reference (0 when not simulating).
  double simJointOffset(Id joint) const;

  // outliner internals
  void outlinerBody(uint32_t bodyIndex, const std::string& filter);
  void outlinerLeaf(Id id, const char* name, int icon, unsigned int color);
  bool outlinerDragDrop(Id targetBody);

  // validation
  void scheduleCompile();
  void pollCompile();

  void setTitle();
  std::string editorStateJson() const;
  void applyEditorState(const std::string& json);
  void loadPrefs();
  void savePrefs();
  void addRecent(const std::string& path);

  GLFWwindow* window_ = nullptr;
  float scale_ = 1;
  std::string configDir_;
  bool quit_ = false;
  bool layoutBuilt_ = false;
  int focusInspector_ = 0;
  int frameCount_ = 0;
  std::vector<std::string> pendingFocus_;  // windows to bring to front after the layout settles
    // frames until the Inspector tab is brought to front

  Document doc_;
  History history_;
  Selection sel_;
  std::string path_;
  bool modified_ = false;
  uint64_t editRevision_ = 1;  // bumps on every change (structure or value)
  unsigned int activeEditId_ = 0;
  bool pendingUndo_ = false;

  // rendering
  Renderer renderer_;
  DebugDraw dbg_;
  ViewportState vp_;
  GizmoOp gizmoOp_ = GizmoOp::Translate;
  bool gizmoLocal_ = false;
  bool snap_ = false;
  float snapMove_ = 0.05f, snapAngle_ = 15.0f, snapScale_ = 0.01f;
  PickMode pickMode_ = PickMode::Body;
  std::vector<Pose> poses_;  // body world poses for the current frame
  struct GizmoDrag {
    bool active = false;
    Pose primaryStart;
    std::vector<std::pair<Id, Pose>> starts;  // world poses at drag start
    Vec3 sizeStart;
  } gizmoDrag_;

  // validation
  BackgroundCompiler compiler_;
  CompileResult compiled_;
  uint64_t compiledRevision_ = 0;
  uint64_t requestedRevision_ = 0;
  double compileDueTime_ = 0;
  AnalysisReport report_;
  uint64_t reportRevision_ = 0;
  bool reportHasCompile_ = false;
  bool issueFilter_[4] = {true, true, true, true};
  void runAnalysis();
  void drawStrength(Id joint);
  std::string xmlPreview_;
  uint64_t xmlPreviewRevision_ = 0;

  // simulation
  Simulator sim_;
  bool simWanted_ = false;     // start (paused if simStartPaused_) when the next valid compile arrives
  bool simStartPaused_ = false;
  uint64_t simRevision_ = 0;   // edit revision of the model being simulated
  double lastFrameTime_ = 0;
  double cliSimSeconds_ = 0;   // --action sim:<seconds>: fast-forward after starting
  int simPlotActuator_ = 0;    // compiled actuator index shown in the plot
  bool simShowForces_ = true;
  std::vector<SimContact> simContacts_;
  struct SimDrag {
    bool active = false;
    Vec3 planePoint, planeNormal;
  } simDrag_;

  // dialogs
  FileDialog fileDialog_;
  bool showAbout_ = false;
  bool showDemo_ = false;
  bool showExport_ = false;
  int exportScope_ = 0;
  bool exportSplit_ = false;
  int exportAngle_ = 0;
  std::string lastDir_;
  std::vector<std::string> recent_;
  std::vector<Toast> toasts_;
  PropContext propCtx_;
  Id renameId_ = kNoId;
  std::string renameBuf_;
  bool focusRename_ = false;
};

enum FileDialogPurpose { kPurposeOpen = 1, kPurposeSaveAs = 2, kPurposeExport = 3 };

}  // namespace mjf
