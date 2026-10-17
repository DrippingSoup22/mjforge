#include "ui/Theme.h"

namespace mjf {

namespace {
ImFont* g_ui = nullptr;
ImFont* g_code = nullptr;
}  // namespace

void setFonts(ImFont* ui, ImFont* code) {
  g_ui = ui;
  g_code = code;
}
ImFont* uiFont() { return g_ui; }
ImFont* codeFont() { return g_code ? g_code : g_ui; }

void applyTheme(float scale) {
  ImGuiStyle& s = ImGui::GetStyle();
  s = ImGuiStyle();
  s.WindowPadding = ImVec2(10, 8);
  s.FramePadding = ImVec2(7, 4);
  s.CellPadding = ImVec2(6, 3);
  s.ItemSpacing = ImVec2(8, 5);
  s.ItemInnerSpacing = ImVec2(6, 4);
  s.IndentSpacing = 16;
  s.ScrollbarSize = 12;
  s.GrabMinSize = 10;
  s.WindowBorderSize = 1;
  s.ChildBorderSize = 1;
  s.PopupBorderSize = 1;
  s.FrameBorderSize = 0;
  s.TabBorderSize = 0;
  s.WindowRounding = 6;
  s.ChildRounding = 5;
  s.FrameRounding = 4;
  s.PopupRounding = 5;
  s.ScrollbarRounding = 6;
  s.GrabRounding = 3;
  s.TabRounding = 4;
  s.WindowTitleAlign = ImVec2(0.0f, 0.5f);
  s.SeparatorTextBorderSize = 1;
  s.DockingSeparatorSize = 2;

  ImVec4* c = s.Colors;
  auto rgb = [](int r, int g, int b, float a = 1.0f) { return ImVec4(r / 255.f, g / 255.f, b / 255.f, a); };
  const ImVec4 bg = rgb(22, 24, 29);
  const ImVec4 panel = rgb(28, 31, 38);
  const ImVec4 frame = rgb(38, 42, 51);
  const ImVec4 frameHover = rgb(47, 52, 64);
  const ImVec4 frameActive = rgb(58, 64, 80);
  const ImVec4 accent = rgb(232, 131, 58);
  const ImVec4 accentSoft = rgb(232, 131, 58, 0.45f);
  const ImVec4 text = rgb(228, 231, 236);

  c[ImGuiCol_Text] = text;
  c[ImGuiCol_TextDisabled] = rgb(124, 130, 141);
  c[ImGuiCol_WindowBg] = panel;
  c[ImGuiCol_ChildBg] = rgb(0, 0, 0, 0);
  c[ImGuiCol_PopupBg] = rgb(30, 33, 41, 0.98f);
  c[ImGuiCol_Border] = rgb(48, 52, 62);
  c[ImGuiCol_BorderShadow] = rgb(0, 0, 0, 0);
  c[ImGuiCol_FrameBg] = frame;
  c[ImGuiCol_FrameBgHovered] = frameHover;
  c[ImGuiCol_FrameBgActive] = frameActive;
  c[ImGuiCol_TitleBg] = bg;
  c[ImGuiCol_TitleBgActive] = bg;
  c[ImGuiCol_TitleBgCollapsed] = bg;
  c[ImGuiCol_MenuBarBg] = bg;
  c[ImGuiCol_ScrollbarBg] = rgb(0, 0, 0, 0);
  c[ImGuiCol_ScrollbarGrab] = rgb(58, 63, 75);
  c[ImGuiCol_ScrollbarGrabHovered] = rgb(72, 78, 92);
  c[ImGuiCol_ScrollbarGrabActive] = rgb(86, 92, 108);
  c[ImGuiCol_CheckMark] = accent;
  c[ImGuiCol_SliderGrab] = rgb(200, 112, 50);
  c[ImGuiCol_SliderGrabActive] = accent;
  c[ImGuiCol_Button] = rgb(44, 49, 60);
  c[ImGuiCol_ButtonHovered] = rgb(58, 64, 79);
  c[ImGuiCol_ButtonActive] = rgb(72, 79, 97);
  c[ImGuiCol_Header] = rgb(232, 131, 58, 0.22f);
  c[ImGuiCol_HeaderHovered] = rgb(232, 131, 58, 0.32f);
  c[ImGuiCol_HeaderActive] = rgb(232, 131, 58, 0.42f);
  c[ImGuiCol_Separator] = rgb(48, 52, 62);
  c[ImGuiCol_SeparatorHovered] = accentSoft;
  c[ImGuiCol_SeparatorActive] = accent;
  c[ImGuiCol_ResizeGrip] = rgb(0, 0, 0, 0);
  c[ImGuiCol_ResizeGripHovered] = accentSoft;
  c[ImGuiCol_ResizeGripActive] = accent;
  c[ImGuiCol_Tab] = bg;
  c[ImGuiCol_TabHovered] = frameHover;
  c[ImGuiCol_TabSelected] = panel;
  c[ImGuiCol_TabSelectedOverline] = accent;
  c[ImGuiCol_TabDimmed] = bg;
  c[ImGuiCol_TabDimmedSelected] = panel;
  c[ImGuiCol_TabDimmedSelectedOverline] = rgb(110, 110, 120);
  c[ImGuiCol_DockingPreview] = accentSoft;
  c[ImGuiCol_DockingEmptyBg] = bg;
  c[ImGuiCol_PlotLines] = rgb(160, 170, 190);
  c[ImGuiCol_PlotLinesHovered] = accent;
  c[ImGuiCol_PlotHistogram] = accent;
  c[ImGuiCol_PlotHistogramHovered] = rgb(250, 160, 90);
  c[ImGuiCol_TableHeaderBg] = rgb(34, 37, 46);
  c[ImGuiCol_TableBorderStrong] = rgb(48, 52, 62);
  c[ImGuiCol_TableBorderLight] = rgb(40, 44, 52);
  c[ImGuiCol_TableRowBg] = rgb(0, 0, 0, 0);
  c[ImGuiCol_TableRowBgAlt] = rgb(255, 255, 255, 0.025f);
  c[ImGuiCol_TextSelectedBg] = rgb(232, 131, 58, 0.35f);
  c[ImGuiCol_DragDropTarget] = accent;
  c[ImGuiCol_NavCursor] = accent;
  c[ImGuiCol_ModalWindowDimBg] = rgb(0, 0, 0, 0.5f);

  s.ScaleAllSizes(scale);
}

}  // namespace mjf
