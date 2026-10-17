// Preferences: settings that outlive a project, stored in prefs.json in the settings folder.
#include <imgui.h>

#include "ui/Editor.h"
#include "ui/Theme.h"

namespace mjf {

void Editor::drawPreferences() {
  if (!showPrefs_) return;
  ImGui::SetNextWindowSize(ImVec2(440 * scale_, 0), ImGuiCond_Appearing);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  bool open = true;
  if (!ImGui::Begin("Preferences", &open, ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::End();
    return;
  }
  bool changed = false;
  ImGui::PushItemWidth(160 * scale_);
  ImGui::SeparatorText("Projects");
  float autosave = float(autosaveSeconds_);
  if (ImGui::DragFloat("Autosave every", &autosave, 1.0f, 0.0f, 3600.0f, autosave > 0 ? "%.0f s" : "off")) {
    autosaveSeconds_ = std::max(0.0f, autosave);
    changed = true;
  }
  if (ImGui::IsItemHovered())
    ImGui::SetTooltip("Modified projects are saved in the background to the settings folder and offered for recovery "
                      "after a crash. 0 turns it off.");
  ImGui::SeparatorText("Snapping (toolbar magnet; hold Ctrl to invert while dragging)");
  changed |= ImGui::Checkbox("Snap by default", &snap_);
  changed |= ImGui::DragFloat("Move step", &snapMove_, 0.001f, 0.001f, 10.0f, "%.3f m");
  changed |= ImGui::DragFloat("Angle step", &snapAngle_, 0.5f, 0.5f, 90.0f, "%.1f deg");
  changed |= ImGui::DragFloat("Resize step", &snapScale_, 0.001f, 0.001f, 10.0f, "%.3f m");
  ImGui::SeparatorText("Viewport");
  changed |= ImGui::Checkbox("Shadows", &vp_.showShadows);
  ImGui::SameLine();
  changed |= ImGui::Checkbox("Grid", &vp_.showGrid);
  ImGui::SeparatorText("Export defaults");
  changed |= ImGui::Combo("Angle unit", &exportAngle_, "radian\0degree\0");
  changed |= ImGui::Checkbox("Write the MuJoCo Warp loader (name_mjwarp.py)", &exportLoader_);
  changed |= ImGui::Checkbox("Split into scene + model files", &exportSplit_);
  ImGui::PopItemWidth();
  ImGui::Spacing();
  ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 420 * scale_);
  ImGui::TextColored(pal::v4(pal::kTextDim), "Settings folder: %s", configDir_.empty() ? "(none)" : configDir_.c_str());
  ImGui::PopTextWrapPos();
  if (changed) savePrefs();
  ImGui::End();
  if (!open) showPrefs_ = false;
}

}  // namespace mjf
