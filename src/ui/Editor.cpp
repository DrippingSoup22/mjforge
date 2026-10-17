#include "ui/Editor.h"

#include <imgui.h>
#include <imgui_internal.h>

#include "ui/Theme.h"

namespace mjf {

Editor::Editor() = default;
Editor::~Editor() = default;

void Editor::init(GLFWwindow* window, float scale, const std::string& configDir) {
  window_ = window;
  scale_ = scale;
  configDir_ = configDir;
}

void Editor::applyCli(const CliOptions&) {}

void Editor::buildDefaultLayout(unsigned int dockId) {
  ImGui::DockBuilderRemoveNode(dockId);
  ImGui::DockBuilderAddNode(dockId, ImGuiDockNodeFlags_DockSpace);
  ImGui::DockBuilderSetNodeSize(dockId, ImGui::GetMainViewport()->WorkSize);
  ImGuiID center = dockId;
  ImGuiID left = ImGui::DockBuilderSplitNode(center, ImGuiDir_Left, 0.18f, nullptr, &center);
  ImGuiID right = ImGui::DockBuilderSplitNode(center, ImGuiDir_Right, 0.26f, nullptr, &center);
  ImGuiID bottom = ImGui::DockBuilderSplitNode(center, ImGuiDir_Down, 0.28f, nullptr, &center);
  ImGui::DockBuilderDockWindow("Outliner", left);
  ImGui::DockBuilderDockWindow("Inspector", right);
  ImGui::DockBuilderDockWindow("Viewport", center);
  ImGui::DockBuilderDockWindow("Issues", bottom);
  ImGui::DockBuilderDockWindow("Actuators", bottom);
  ImGui::DockBuilderDockWindow("MJCF", bottom);
  ImGui::DockBuilderFinish(dockId);
}

void Editor::drawMenuBar() {
  if (ImGui::BeginMainMenuBar()) {
    if (ImGui::BeginMenu("File")) {
      if (ImGui::MenuItem("Quit", "Ctrl+Q")) quit_ = true;
      ImGui::EndMenu();
    }
    ImGui::EndMainMenuBar();
  }
}

void Editor::frame() {
  drawMenuBar();
  ImGuiID dock = ImGui::GetID("MainDock");
  if (!layoutBuilt_ && ImGui::DockBuilderGetNode(dock) == nullptr) buildDefaultLayout(dock);
  layoutBuilt_ = true;
  ImGui::DockSpaceOverViewport(dock, ImGui::GetMainViewport());

  const char* panels[] = {"Outliner", "Inspector", "Viewport", "Issues", "Actuators", "MJCF"};
  for (const char* p : panels) {
    ImGui::Begin(p);
    ImGui::TextDisabled("%s", p);
    ImGui::End();
  }
}

}  // namespace mjf
