// Editor: owns the document, history and selection, and draws all panels.
#pragma once

#include <string>

#include "app/Cli.h"
#include "core/Document.h"
#include "core/History.h"

struct GLFWwindow;

namespace mjf {

class Editor {
 public:
  Editor();
  ~Editor();

  void init(GLFWwindow* window, float scale, const std::string& configDir);
  void applyCli(const CliOptions& cli);
  void frame();
  bool wantsContinuousRendering() const { return false; }
  bool quitRequested() const { return quit_; }
  bool confirmQuit() { return true; }

 private:
  void buildDefaultLayout(unsigned int dockId);
  void drawMenuBar();

  GLFWwindow* window_ = nullptr;
  float scale_ = 1;
  std::string configDir_;
  Document doc_;
  History history_;
  bool quit_ = false;
  bool layoutBuilt_ = false;
};

}  // namespace mjf
