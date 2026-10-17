// Property-grid widgets for the inspector.
//
// Each row is "label | widget". Labels turn accent-colored when the value differs from MuJoCo's
// default and a reset button appears. Widgets edit a temporary copy and call
// PropContext::change() *before* writing, so the undo snapshot always holds the old value and a
// whole drag produces a single undo step.
#pragma once

#include <functional>
#include <string>
#include <vector>

#include <imgui.h>

#include "core/Document.h"

namespace mjf {

struct PropContext {
  std::function<void()> change;  // called right before a value is written
  float labelWidth = 0;          // 0 = automatic
};

namespace props {

void setContext(PropContext* ctx);

// Table wrapper. Returns false if the table isn't visible (then don't call end()).
bool begin(const char* id);
void end();
// Collapsible section header spanning both columns (inside a table).
bool section(const char* title, bool defaultOpen = true);
// A label-only row with a value text (read-only information).
void info(const char* label, const char* fmt, ...);
void separator();

struct Opt {
  const char* tip = nullptr;   // tooltip text
  const char* unit = nullptr;  // e.g. "m", "kg"
  double speed = 0.01;
  double min = -1e300, max = 1e300;
  const char* fmt = "%.4g";
  bool disabled = false;
};

bool real(const char* label, double* v, double def, const Opt& o = {});
bool angle(const char* label, double* radians, double defRad, const Opt& o = {});  // shown in degrees
bool integer(const char* label, int* v, int def, const Opt& o = {});
bool vec2(const char* label, Vec2* v, const Vec2& def, const Opt& o = {});
bool vec3(const char* label, Vec3* v, const Vec3& def, const Opt& o = {});
bool vec5(const char* label, Vec5* v, const Vec5& def, const Opt& o = {});
bool angleRange(const char* label, Vec2* radians, const Opt& o = {});
bool checkbox(const char* label, bool* v, bool def, const char* tip = nullptr);
bool color(const char* label, Color* c, const Color& def, const char* tip = nullptr);
bool color3(const char* label, Vec3* c, const Vec3& def, const char* tip = nullptr);
bool text(const char* label, std::string* s, const char* tip = nullptr);
bool orientation(const char* label, Quat* q, const char* tip = nullptr);  // Euler XYZ degrees
bool bits(const char* label, int* mask, int def, const char* tip = nullptr);  // 32-bit collision mask
// Combo over an enum table.
template <class E, size_t N>
bool combo(const char* label, E* v, const EnumEntry<E> (&table)[N], E def, const char* tip = nullptr);
// Combo over arbitrary labels; returns true if *index changed.
bool comboIndex(const char* label, int* index, const std::vector<std::string>& items, int def, const char* tip = nullptr);
// Element picker: lists (id, name) pairs. kNoId means "none" when allowNone.
bool element(const char* label, Id* id, const std::vector<std::pair<Id, std::string>>& items, bool allowNone,
             const char* tip = nullptr);

// Internal helpers used by the templates.
bool beginRow(const char* label, bool modified, const char* tip, bool* resetClicked, const char* unit = nullptr);
void endRow();
void notifyChange();

}  // namespace props

template <class E, size_t N>
bool props::combo(const char* label, E* v, const EnumEntry<E> (&table)[N], E def, const char* tip) {
  bool reset = false;
  if (!beginRow(label, *v != def, tip, &reset)) return false;
  bool changed = false;
  if (reset) {
    notifyChange();
    *v = def;
    changed = true;
  }
  const char* cur = enumLabel(table, *v);
  if (ImGui::BeginCombo("##v", cur)) {
    for (const auto& e : table) {
      bool sel = e.value == *v;
      if (ImGui::Selectable(e.label, sel) && !sel) {
        notifyChange();
        *v = e.value;
        changed = true;
      }
      if (sel) ImGui::SetItemDefaultFocus();
    }
    ImGui::EndCombo();
  }
  endRow();
  return changed;
}

}  // namespace mjf
