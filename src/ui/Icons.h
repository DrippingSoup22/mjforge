// Small vector icons drawn with ImDrawList (crisp at any DPI, no icon font needed).
#pragma once

#include <imgui.h>

namespace mjf {

enum class Icon {
  World, Body, Box, Sphere, Capsule, Cylinder, Ellipsoid, Plane, Hfield, Hinge, Slide, Ball, Free, Site, Camera,
  Light, Actuator, Sensor, Material, Exclude, EyeOpen, EyeClosed, Play, Pause, Step, Reset, Stop, Select, Move, Rotate,
  Scale, Warning, Error, Info, Ok, Magnet, Limb, Mirror, Plus, Pose
};

void drawIcon(ImDrawList* dl, ImVec2 center, float size, Icon icon, ImU32 color);
// Inline icon occupying a square of the current line height.
void iconInline(Icon icon, ImU32 color);
// Toolbar-style toggle button with an icon; returns true when clicked.
bool iconButton(const char* id, Icon icon, bool active, const char* tooltip, float size = 0, ImU32 color = 0);

}  // namespace mjf
