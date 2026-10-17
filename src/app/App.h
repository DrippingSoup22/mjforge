// Platform layer: window, OpenGL context, Dear ImGui setup and the main loop.
#pragma once

#include <memory>
#include <string>

#include "app/Cli.h"

struct GLFWwindow;

namespace mjf {

class Editor;

class App {
 public:
  App();
  ~App();
  int run(const CliOptions& cli);

 private:
  bool init(const CliOptions& cli);
  void shutdown();
  void setupFonts(float scale);
  bool saveScreenshot(const std::string& path);

  GLFWwindow* window_ = nullptr;
  std::unique_ptr<Editor> editor_;
  float scale_ = 1.0f;
  std::string iniPath_;
};

}  // namespace mjf
