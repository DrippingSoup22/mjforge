#include "app/App.h"

#include <glad/gl.h>
// GLFW after glad.
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <vector>

#include <imgui.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <implot.h>

#include "ui/Editor.h"
#include "ui/Theme.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

extern const unsigned char mjf_font_ui[];
extern const unsigned int mjf_font_ui_size;
extern const unsigned char mjf_font_mono[];
extern const unsigned int mjf_font_mono_size;

namespace mjf {

namespace {

void glfwErrorCallback(int code, const char* desc) { std::fprintf(stderr, "GLFW error %d: %s\n", code, desc); }

// Set by any window/input callback: the next loop iteration must render.
bool g_inputEvent = true;
void markEvent() { g_inputEvent = true; }
void installEventTrackers(GLFWwindow* w) {
  // Installed before ImGui's backend, which chains to these.
  glfwSetCursorPosCallback(w, [](GLFWwindow*, double, double) { markEvent(); });
  glfwSetMouseButtonCallback(w, [](GLFWwindow*, int, int, int) { markEvent(); });
  glfwSetScrollCallback(w, [](GLFWwindow*, double, double) { markEvent(); });
  glfwSetKeyCallback(w, [](GLFWwindow*, int, int, int, int) { markEvent(); });
  glfwSetCharCallback(w, [](GLFWwindow*, unsigned int) { markEvent(); });
  glfwSetWindowFocusCallback(w, [](GLFWwindow*, int) { markEvent(); });
  glfwSetCursorEnterCallback(w, [](GLFWwindow*, int) { markEvent(); });
  glfwSetWindowSizeCallback(w, [](GLFWwindow*, int, int) { markEvent(); });
  glfwSetFramebufferSizeCallback(w, [](GLFWwindow*, int, int) { markEvent(); });
  glfwSetWindowRefreshCallback(w, [](GLFWwindow*) { markEvent(); });
  glfwSetDropCallback(w, [](GLFWwindow*, int, const char**) { markEvent(); });
}

std::string configDir() {
  namespace fs = std::filesystem;
  fs::path base;
#ifdef _WIN32
  if (const char* a = std::getenv("APPDATA")) base = a;
#elif defined(__APPLE__)
  if (const char* h = std::getenv("HOME")) base = fs::u8path(h) / "Library" / "Application Support";
#else
  if (const char* x = std::getenv("XDG_CONFIG_HOME")) base = x;
  else if (const char* h = std::getenv("HOME")) base = fs::u8path(h) / ".config";
#endif
  if (base.empty()) return {};
  fs::path dir = base / "mjforge";
  std::error_code ec;
  fs::create_directories(dir, ec);
  return dir.u8string();
}

}  // namespace

App::App() = default;
App::~App() { shutdown(); }

bool App::init(const CliOptions& cli) {
  glfwSetErrorCallback(glfwErrorCallback);
  if (!glfwInit()) {
    std::fprintf(stderr, "Failed to initialize GLFW (is a display available?)\n");
    return false;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
  glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif
  glfwWindowHint(GLFW_SCALE_TO_MONITOR, GLFW_TRUE);
  window_ = glfwCreateWindow(cli.width, cli.height, "MJForge", nullptr, nullptr);
  if (!window_) {
    std::fprintf(stderr, "Failed to create an OpenGL 3.3 window\n");
    return false;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);
  if (!gladLoadGL(reinterpret_cast<GLADloadfunc>(glfwGetProcAddress))) {
    std::fprintf(stderr, "Failed to load OpenGL functions\n");
    return false;
  }

  float sx = 1, sy = 1;
  glfwGetWindowContentScale(window_, &sx, &sy);
  scale_ = std::max(1.0f, std::max(sx, sy));

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable | ImGuiConfigFlags_NavEnableKeyboard;
  io.ConfigWindowsMoveFromTitleBarOnly = true;
  bool testing = !cli.screenshot.empty();
  if (testing) {
    io.IniFilename = nullptr;  // deterministic default layout for screenshots
  } else {
    std::string dir = configDir();
    iniPath_ = dir.empty() ? "mjforge_layout.ini" : dir + "/layout.ini";
    io.IniFilename = iniPath_.c_str();
  }
  applyTheme(scale_);
  setupFonts(scale_);
  installEventTrackers(window_);
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

  editor_ = std::make_unique<Editor>();
  editor_->init(window_, scale_, configDir());
  editor_->applyCli(cli);
  return true;
}

void App::setupFonts(float scale) {
  ImGuiIO& io = ImGui::GetIO();
  ImFontConfig cfg;
  cfg.FontDataOwnedByAtlas = false;
  cfg.OversampleH = 2;
  std::snprintf(cfg.Name, sizeof cfg.Name, "Inter");
  ImFont* ui = io.Fonts->AddFontFromMemoryTTF((void*)mjf_font_ui, int(mjf_font_ui_size), 16.0f * scale, &cfg);
  ImFontConfig mono;
  mono.FontDataOwnedByAtlas = false;
  std::snprintf(mono.Name, sizeof mono.Name, "JetBrains Mono");
  ImFont* code = io.Fonts->AddFontFromMemoryTTF((void*)mjf_font_mono, int(mjf_font_mono_size), 15.0f * scale, &mono);
  if (!ui) ui = io.Fonts->AddFontDefault();
  io.FontDefault = ui;
  setFonts(ui, code ? code : ui);
}

bool App::saveScreenshot(const std::string& path) {
  int w = 0, h = 0;
  glfwGetFramebufferSize(window_, &w, &h);
  std::vector<unsigned char> px(size_t(w) * h * 3);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glReadBuffer(GL_BACK);
  glReadPixels(0, 0, w, h, GL_RGB, GL_UNSIGNED_BYTE, px.data());
  stbi_flip_vertically_on_write(1);
  bool ok = stbi_write_png(path.c_str(), w, h, 3, px.data(), w * 3) != 0;
  std::printf(ok ? "Saved screenshot %s (%dx%d)\n" : "Failed to save %s\n", path.c_str(), w, h);
  return ok;
}

int App::run(const CliOptions& cli) {
  if (!init(cli)) return 1;
  int frame = 0;
  const bool trace = std::getenv("MJFORGE_TRACE") != nullptr;
  double lastTime = glfwGetTime();
  int busyFrames = 3;  // frames to render after the last event so ImGui settles (hover, trickled input)
  while (!glfwWindowShouldClose(window_)) {
    bool continuous = editor_->wantsContinuousRendering() || !cli.screenshot.empty();
    if (continuous || busyFrames > 0) {
      glfwPollEvents();
    } else {
      // Idle: sleep until input arrives or a worker wakes us (glfwPostEmptyEvent). No rendering.
      glfwWaitEventsTimeout(2.0);
    }
    if (g_inputEvent) {
      g_inputEvent = false;
      busyFrames = 4;
    } else if (!continuous && busyFrames == 0) {
      if (!editor_->hasPendingWork()) continue;
      busyFrames = 2;
    }
    if (busyFrames > 0) --busyFrames;
    if (glfwGetWindowAttrib(window_, GLFW_ICONIFIED)) {
      glfwWaitEventsTimeout(0.1);
      continue;
    }

    double workStart = glfwGetTime();
    ImGui_ImplOpenGL3_NewFrame();
    ImGui_ImplGlfw_NewFrame();
    ImGui::NewFrame();
    editor_->frame();
    ImGui::Render();

    int fw = 0, fh = 0;
    glfwGetFramebufferSize(window_, &fw, &fh);
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glViewport(0, 0, fw, fh);
    glClearColor(0.08f, 0.085f, 0.095f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT);
    ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

    ++frame;
    if (trace) {
      double t = glfwGetTime();
      std::fprintf(stderr, "frame %d  %.1f ms (work %.1f ms)\n", frame, (t - lastTime) * 1000, (t - workStart) * 1000);
      lastTime = t;
    }
    if (!cli.screenshot.empty() && frame >= cli.frames) {
      saveScreenshot(cli.screenshot);
      break;
    }
    glfwSwapBuffers(window_);
    if (editor_->quitRequested()) glfwSetWindowShouldClose(window_, GLFW_TRUE);
    if (glfwWindowShouldClose(window_) && !editor_->confirmQuit()) glfwSetWindowShouldClose(window_, GLFW_FALSE);
  }
  return 0;
}

void App::shutdown() {
  if (!window_) return;
  editor_.reset();
  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();
  glfwDestroyWindow(window_);
  glfwTerminate();
  window_ = nullptr;
}

}  // namespace mjf
