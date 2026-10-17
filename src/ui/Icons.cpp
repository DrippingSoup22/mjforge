#include "ui/Icons.h"

#include <algorithm>
#include <cmath>

#include "ui/Theme.h"

namespace mjf {

namespace {
inline ImVec2 off(ImVec2 c, float s, float x, float y) { return ImVec2(c.x + x * s, c.y + y * s); }
}  // namespace

void drawIcon(ImDrawList* dl, ImVec2 c, float size, Icon icon, ImU32 col) {
  const float s = size * 0.5f;  // half size
  const float t = std::max(1.0f, size * 0.09f);  // stroke
  switch (icon) {
    case Icon::World: {
      dl->AddCircle(c, s * 0.85f, col, 20, t);
      dl->AddLine(off(c, s, -0.85f, 0), off(c, s, 0.85f, 0), col, t);
      dl->AddEllipse(c, ImVec2(s * 0.38f, s * 0.85f), col, 0, 20, t);
      break;
    }
    case Icon::Body: {
      dl->AddCircleFilled(c, s * 0.42f, col, 16);
      dl->AddCircle(c, s * 0.8f, col, 20, t);
      break;
    }
    case Icon::Box:
      dl->AddRectFilled(off(c, s, -0.62f, -0.62f), off(c, s, 0.62f, 0.62f), col, s * 0.12f);
      break;
    case Icon::Sphere:
      dl->AddCircleFilled(c, s * 0.7f, col, 20);
      break;
    case Icon::Capsule:
      dl->AddRectFilled(off(c, s, -0.42f, -0.8f), off(c, s, 0.42f, 0.8f), col, s * 0.42f);
      break;
    case Icon::Cylinder:
      dl->AddRectFilled(off(c, s, -0.55f, -0.5f), off(c, s, 0.55f, 0.6f), col, s * 0.1f);
      dl->AddEllipseFilled(off(c, s, 0, -0.5f), ImVec2(s * 0.55f, s * 0.22f), col);
      dl->AddEllipseFilled(off(c, s, 0, 0.6f), ImVec2(s * 0.55f, s * 0.22f), col);
      break;
    case Icon::Ellipsoid:
      dl->AddEllipseFilled(c, ImVec2(s * 0.85f, s * 0.5f), col);
      break;
    case Icon::Plane: {
      ImVec2 p[4] = {off(c, s, -0.9f, 0.45f), off(c, s, -0.35f, -0.45f), off(c, s, 0.9f, -0.45f), off(c, s, 0.35f, 0.45f)};
      dl->AddConvexPolyFilled(p, 4, col);
      break;
    }
    case Icon::Hfield: {
      ImVec2 p[6] = {off(c, s, -0.9f, 0.6f), off(c, s, -0.5f, -0.1f), off(c, s, -0.1f, 0.3f), off(c, s, 0.3f, -0.6f),
                     off(c, s, 0.9f, 0.2f), off(c, s, 0.9f, 0.6f)};
      dl->AddConvexPolyFilled(p, 3, col);
      dl->AddPolyline(p, 6, col, 0, t * 1.3f);
      break;
    }
    case Icon::Hinge: {
      dl->PathArcTo(c, s * 0.62f, 0.6f, 5.3f, 20);
      dl->PathStroke(col, 0, t * 1.4f);
      ImVec2 tip = ImVec2(c.x + std::cos(0.6f) * s * 0.62f, c.y + std::sin(0.6f) * s * 0.62f);
      dl->AddTriangleFilled(tip + ImVec2(-s * 0.38f, -s * 0.02f), tip + ImVec2(s * 0.18f, -s * 0.30f),
                            tip + ImVec2(s * 0.14f, s * 0.26f), col);
      dl->AddCircleFilled(c, s * 0.16f, col);
      break;
    }
    case Icon::Slide: {
      dl->AddLine(off(c, s, -0.6f, 0), off(c, s, 0.6f, 0), col, t * 1.4f);
      dl->AddTriangleFilled(off(c, s, -0.9f, 0), off(c, s, -0.5f, -0.35f), off(c, s, -0.5f, 0.35f), col);
      dl->AddTriangleFilled(off(c, s, 0.9f, 0), off(c, s, 0.5f, -0.35f), off(c, s, 0.5f, 0.35f), col);
      break;
    }
    case Icon::Ball:
      dl->AddCircle(c, s * 0.7f, col, 20, t * 1.3f);
      dl->AddCircleFilled(c, s * 0.3f, col, 12);
      break;
    case Icon::Free: {
      for (int i = 0; i < 4; ++i) {
        float a = i * 1.5707963f;
        ImVec2 d(std::cos(a), std::sin(a));
        ImVec2 tip = ImVec2(c.x + d.x * s * 0.9f, c.y + d.y * s * 0.9f);
        dl->AddLine(c, ImVec2(c.x + d.x * s * 0.5f, c.y + d.y * s * 0.5f), col, t * 1.3f);
        ImVec2 n(-d.y, d.x);
        dl->AddTriangleFilled(tip, ImVec2(c.x + d.x * s * 0.5f + n.x * s * 0.28f, c.y + d.y * s * 0.5f + n.y * s * 0.28f),
                              ImVec2(c.x + d.x * s * 0.5f - n.x * s * 0.28f, c.y + d.y * s * 0.5f - n.y * s * 0.28f), col);
      }
      break;
    }
    case Icon::Site: {
      ImVec2 p[4] = {off(c, s, 0, -0.75f), off(c, s, 0.6f, 0), off(c, s, 0, 0.75f), off(c, s, -0.6f, 0)};
      dl->AddConvexPolyFilled(p, 4, col);
      break;
    }
    case Icon::Camera:
      dl->AddRectFilled(off(c, s, -0.8f, -0.45f), off(c, s, 0.3f, 0.45f), col, s * 0.1f);
      dl->AddTriangleFilled(off(c, s, 0.3f, 0), off(c, s, 0.85f, -0.4f), off(c, s, 0.85f, 0.4f), col);
      break;
    case Icon::Light: {
      dl->AddCircleFilled(c, s * 0.38f, col, 16);
      for (int i = 0; i < 8; ++i) {
        float a = i * 0.785398f;
        dl->AddLine(ImVec2(c.x + std::cos(a) * s * 0.55f, c.y + std::sin(a) * s * 0.55f),
                    ImVec2(c.x + std::cos(a) * s * 0.85f, c.y + std::sin(a) * s * 0.85f), col, t);
      }
      break;
    }
    case Icon::Actuator: {  // lightning bolt
      ImVec2 p[6] = {off(c, s, 0.15f, -0.9f), off(c, s, -0.5f, 0.15f), off(c, s, -0.05f, 0.15f),
                     off(c, s, -0.2f, 0.9f), off(c, s, 0.5f, -0.2f), off(c, s, 0.05f, -0.2f)};
      dl->AddConvexPolyFilled(p, 3, col);
      ImVec2 q[3] = {p[2], p[3], p[4]};
      dl->AddConvexPolyFilled(q, 3, col);
      ImVec2 r[4] = {p[0], p[2], p[4], p[5]};
      dl->AddPolyline(r, 4, col, ImDrawFlags_Closed, t);
      break;
    }
    case Icon::Sensor: {  // signal wave
      ImVec2 p[7];
      for (int i = 0; i < 7; ++i) {
        float x = -0.9f + i * 0.3f;
        float y = (i % 2 ? -0.55f : 0.55f) * (i == 0 || i == 6 ? 0.0f : 1.0f);
        p[i] = off(c, s, x, y);
      }
      dl->AddPolyline(p, 7, col, 0, t * 1.4f);
      break;
    }
    case Icon::Material:
      dl->AddCircleFilled(c, s * 0.75f, col, 20);
      dl->AddCircleFilled(off(c, s, -0.25f, -0.25f), s * 0.18f, IM_COL32(20, 22, 26, 255), 10);
      dl->AddCircleFilled(off(c, s, 0.25f, -0.15f), s * 0.14f, IM_COL32(20, 22, 26, 255), 10);
      break;
    case Icon::Exclude:
      dl->AddCircle(c, s * 0.7f, col, 20, t * 1.3f);
      dl->AddLine(off(c, s, -0.5f, 0.5f), off(c, s, 0.5f, -0.5f), col, t * 1.3f);
      break;
    case Icon::EyeOpen:
    case Icon::EyeClosed: {
      dl->PathArcTo(off(c, s, 0, 0.75f), s * 1.05f, -2.35f, -0.79f, 14);
      dl->PathArcTo(off(c, s, 0, -0.75f), s * 1.05f, 0.79f, 2.35f, 14);
      dl->PathStroke(col, ImDrawFlags_Closed, t);
      if (icon == Icon::EyeOpen) dl->AddCircleFilled(c, s * 0.25f, col, 10);
      else dl->AddLine(off(c, s, -0.8f, 0.6f), off(c, s, 0.8f, -0.6f), col, t * 1.2f);
      break;
    }
    case Icon::Play:
      dl->AddTriangleFilled(off(c, s, -0.5f, -0.7f), off(c, s, 0.7f, 0), off(c, s, -0.5f, 0.7f), col);
      break;
    case Icon::Pause:
      dl->AddRectFilled(off(c, s, -0.55f, -0.65f), off(c, s, -0.15f, 0.65f), col, 1);
      dl->AddRectFilled(off(c, s, 0.15f, -0.65f), off(c, s, 0.55f, 0.65f), col, 1);
      break;
    case Icon::Stop:
      dl->AddRectFilled(off(c, s, -0.55f, -0.55f), off(c, s, 0.55f, 0.55f), col, 1.5f);
      break;
    case Icon::Step:
      dl->AddTriangleFilled(off(c, s, -0.6f, -0.65f), off(c, s, 0.35f, 0), off(c, s, -0.6f, 0.65f), col);
      dl->AddRectFilled(off(c, s, 0.4f, -0.65f), off(c, s, 0.65f, 0.65f), col, 1);
      break;
    case Icon::Reset: {
      dl->PathArcTo(c, s * 0.6f, -2.6f, 2.2f, 20);
      dl->PathStroke(col, 0, t * 1.4f);
      ImVec2 tip = ImVec2(c.x + std::cos(-2.6f) * s * 0.6f, c.y + std::sin(-2.6f) * s * 0.6f);
      dl->AddTriangleFilled(tip + ImVec2(-s * 0.3f, -s * 0.05f), tip + ImVec2(s * 0.25f, -s * 0.35f),
                            tip + ImVec2(s * 0.2f, s * 0.25f), col);
      break;
    }
    case Icon::Select: {
      ImVec2 p[7] = {off(c, s, -0.5f, -0.85f), off(c, s, 0.55f, 0.15f), off(c, s, 0.08f, 0.2f), off(c, s, 0.35f, 0.8f),
                     off(c, s, 0.12f, 0.9f), off(c, s, -0.15f, 0.3f), off(c, s, -0.5f, 0.6f)};
      dl->AddPolyline(p, 7, col, ImDrawFlags_Closed, t * 1.2f);
      break;
    }
    case Icon::Move:
      drawIcon(dl, c, size, Icon::Free, col);
      break;
    case Icon::Rotate: {
      dl->PathArcTo(c, s * 0.68f, -0.4f, 4.4f, 24);
      dl->PathStroke(col, 0, t * 1.4f);
      ImVec2 tip = ImVec2(c.x + std::cos(-0.4f) * s * 0.68f, c.y + std::sin(-0.4f) * s * 0.68f);
      dl->AddTriangleFilled(tip + ImVec2(-s * 0.32f, -s * 0.12f), tip + ImVec2(s * 0.3f, -s * 0.2f),
                            tip + ImVec2(s * 0.05f, s * 0.38f), col);
      break;
    }
    case Icon::Scale:
      dl->AddRect(off(c, s, -0.8f, -0.1f), off(c, s, 0.1f, 0.8f), col, 0, 0, t * 1.2f);
      dl->AddLine(off(c, s, -0.1f, 0.1f), off(c, s, 0.75f, -0.75f), col, t * 1.3f);
      dl->AddTriangleFilled(off(c, s, 0.85f, -0.85f), off(c, s, 0.25f, -0.75f), off(c, s, 0.75f, -0.25f), col);
      break;
    case Icon::Warning: {
      dl->AddTriangleFilled(off(c, s, 0, -0.85f), off(c, s, 0.9f, 0.75f), off(c, s, -0.9f, 0.75f), col);
      ImU32 dark = IM_COL32(25, 25, 28, 255);
      dl->AddRectFilled(off(c, s, -0.08f, -0.35f), off(c, s, 0.08f, 0.25f), dark);
      dl->AddCircleFilled(off(c, s, 0, 0.48f), s * 0.1f, dark);
      break;
    }
    case Icon::Error: {
      dl->AddCircleFilled(c, s * 0.85f, col, 20);
      ImU32 dark = IM_COL32(25, 25, 28, 255);
      dl->AddLine(off(c, s, -0.38f, -0.38f), off(c, s, 0.38f, 0.38f), dark, t * 1.6f);
      dl->AddLine(off(c, s, -0.38f, 0.38f), off(c, s, 0.38f, -0.38f), dark, t * 1.6f);
      break;
    }
    case Icon::Info: {
      dl->AddCircleFilled(c, s * 0.85f, col, 20);
      ImU32 dark = IM_COL32(25, 25, 28, 255);
      dl->AddRectFilled(off(c, s, -0.09f, -0.15f), off(c, s, 0.09f, 0.55f), dark);
      dl->AddCircleFilled(off(c, s, 0, -0.42f), s * 0.11f, dark);
      break;
    }
    case Icon::Ok: {
      dl->AddCircleFilled(c, s * 0.85f, col, 20);
      ImU32 dark = IM_COL32(25, 25, 28, 255);
      ImVec2 p[3] = {off(c, s, -0.42f, 0.0f), off(c, s, -0.1f, 0.35f), off(c, s, 0.45f, -0.3f)};
      dl->AddPolyline(p, 3, dark, 0, t * 1.6f);
      break;
    }
    case Icon::Magnet: {
      dl->PathArcTo(c, s * 0.55f, 0.0f, 3.14159f, 16);
      dl->PathStroke(col, 0, t * 2.4f);
      dl->AddLine(off(c, s, -0.55f, 0), off(c, s, -0.55f, -0.7f), col, t * 2.4f);
      dl->AddLine(off(c, s, 0.55f, 0), off(c, s, 0.55f, -0.7f), col, t * 2.4f);
      break;
    }
    case Icon::Limb: {
      dl->AddCircleFilled(off(c, s, -0.55f, -0.45f), s * 0.3f, col, 12);
      dl->AddLine(off(c, s, -0.55f, -0.45f), off(c, s, 0.2f, 0.1f), col, t * 2.6f);
      dl->AddLine(off(c, s, 0.2f, 0.1f), off(c, s, 0.65f, 0.8f), col, t * 2.6f);
      dl->AddCircleFilled(off(c, s, 0.2f, 0.1f), s * 0.16f, IM_COL32(25, 25, 28, 255), 10);
      break;
    }
    case Icon::Mirror: {
      dl->AddLine(off(c, s, 0, -0.9f), off(c, s, 0, 0.9f), col, t);
      dl->AddTriangleFilled(off(c, s, -0.2f, -0.5f), off(c, s, -0.85f, 0.5f), off(c, s, -0.2f, 0.5f), col);
      dl->AddTriangle(off(c, s, 0.2f, -0.5f), off(c, s, 0.85f, 0.5f), off(c, s, 0.2f, 0.5f), col, t);
      break;
    }
    case Icon::Plus:
      dl->AddLine(off(c, s, -0.7f, 0), off(c, s, 0.7f, 0), col, t * 1.8f);
      dl->AddLine(off(c, s, 0, -0.7f), off(c, s, 0, 0.7f), col, t * 1.8f);
      break;
  }
}

void iconInline(Icon icon, ImU32 color) {
  float h = ImGui::GetTextLineHeight();
  ImVec2 p = ImGui::GetCursorScreenPos();
  ImGui::Dummy(ImVec2(h, h));
  drawIcon(ImGui::GetWindowDrawList(), ImVec2(p.x + h * 0.5f, p.y + h * 0.5f), h * 0.9f, icon, color);
}

bool iconButton(const char* id, Icon icon, bool active, const char* tooltip, float size, ImU32 color) {
  if (size <= 0) size = ImGui::GetFrameHeight();
  ImVec2 p = ImGui::GetCursorScreenPos();
  bool clicked = ImGui::InvisibleButton(id, ImVec2(size, size));
  bool hovered = ImGui::IsItemHovered();
  ImDrawList* dl = ImGui::GetWindowDrawList();
  ImU32 bg = active ? pal::kAccentDim : (hovered ? IM_COL32(255, 255, 255, 28) : IM_COL32(0, 0, 0, 0));
  if (bg) dl->AddRectFilled(p, ImVec2(p.x + size, p.y + size), bg, 4.0f);
  ImU32 c = color ? color : (active ? IM_COL32(255, 255, 255, 255) : IM_COL32(205, 210, 220, 255));
  drawIcon(dl, ImVec2(p.x + size * 0.5f, p.y + size * 0.5f), size * 0.62f, icon, c);
  if (tooltip && hovered) ImGui::SetTooltip("%s", tooltip);
  return clicked;
}

}  // namespace mjf
