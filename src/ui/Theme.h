// Visual style: ImGui theme, fonts and the shared color palette.
#pragma once

#include <imgui.h>

namespace mjf {

void applyTheme(float scale);
void setFonts(ImFont* ui, ImFont* code);
ImFont* uiFont();
ImFont* codeFont();

namespace pal {
inline constexpr ImU32 kAccent = IM_COL32(232, 131, 58, 255);    // forge orange
inline constexpr ImU32 kAccentDim = IM_COL32(232, 131, 58, 110);
inline constexpr ImU32 kTeal = IM_COL32(63, 182, 168, 255);
inline constexpr ImU32 kError = IM_COL32(235, 87, 87, 255);
inline constexpr ImU32 kWarning = IM_COL32(242, 184, 64, 255);
inline constexpr ImU32 kInfo = IM_COL32(102, 163, 235, 255);
inline constexpr ImU32 kOk = IM_COL32(92, 196, 120, 255);
inline constexpr ImU32 kTextDim = IM_COL32(140, 147, 160, 255);
inline constexpr ImU32 kBody = IM_COL32(120, 170, 240, 255);
inline constexpr ImU32 kGeom = IM_COL32(170, 178, 190, 255);
inline constexpr ImU32 kJoint = IM_COL32(240, 140, 80, 255);
inline constexpr ImU32 kSite = IM_COL32(242, 200, 60, 255);
inline constexpr ImU32 kActuator = IM_COL32(230, 100, 120, 255);
inline constexpr ImU32 kSensor = IM_COL32(110, 200, 140, 255);
inline constexpr ImU32 kCamera = IM_COL32(170, 140, 230, 255);
inline constexpr ImU32 kLight = IM_COL32(250, 230, 120, 255);
inline ImVec4 v4(ImU32 c) { return ImGui::ColorConvertU32ToFloat4(c); }
}  // namespace pal

}  // namespace mjf
