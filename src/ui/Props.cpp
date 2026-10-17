#include "ui/Props.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdarg>
#include <cstdio>
#include <unordered_map>

#include <imgui_internal.h>
#include <imgui_stdlib.h>

#include "ui/Theme.h"

namespace mjf {
namespace props {

namespace {
PropContext* g_ctx = nullptr;

bool nearlyEq(double a, double b) { return std::fabs(a - b) <= 1e-12 * std::max(1.0, std::fabs(b)); }

std::string withUnit(const char* fmt, const char* unit) {
  std::string f = fmt ? fmt : "%.4g";
  if (unit && *unit) {
    f += " ";
    f += unit;
  }
  return f;
}
}  // namespace

void setContext(PropContext* ctx) { g_ctx = ctx; }

void notifyChange() {
  if (g_ctx && g_ctx->change) g_ctx->change();
}

namespace {
int g_tableSeq = 0;
bool g_inTable = false;
bool beginTableInternal() {
  char id[32];
  std::snprintf(id, sizeof id, "##props%d", g_tableSeq++);
  ImGuiTableFlags flags = ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_PadOuterX;
  if (!ImGui::BeginTable(id, 2, flags)) return false;
  float lw = (g_ctx && g_ctx->labelWidth > 0) ? g_ctx->labelWidth : 0.0f;
  if (lw > 0) {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthFixed, lw);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch);
  } else {
    ImGui::TableSetupColumn("label", ImGuiTableColumnFlags_WidthStretch, 0.42f);
    ImGui::TableSetupColumn("value", ImGuiTableColumnFlags_WidthStretch, 0.58f);
  }
  return true;
}
}  // namespace

bool begin(const char* id) {
  ImGui::PushID(id);
  g_tableSeq = 0;
  g_inTable = beginTableInternal();
  if (!g_inTable) ImGui::PopID();
  return g_inTable;
}

void end() {
  if (g_inTable) ImGui::EndTable();
  g_inTable = false;
  ImGui::PopID();
}

bool section(const char* title, bool defaultOpen) {
  // Headers span the full width: close the current table, draw the header, start a new table
  // (same column proportions, so rows stay aligned across sections).
  if (g_inTable) ImGui::EndTable();
  ImGui::PushStyleColor(ImGuiCol_Header, ImVec4(1, 1, 1, 0.045f));
  ImGui::PushStyleColor(ImGuiCol_HeaderHovered, ImVec4(1, 1, 1, 0.08f));
  ImGui::PushStyleColor(ImGuiCol_HeaderActive, ImVec4(1, 1, 1, 0.10f));
  bool open = ImGui::CollapsingHeader(title, defaultOpen ? ImGuiTreeNodeFlags_DefaultOpen : 0);
  ImGui::PopStyleColor(3);
  g_inTable = beginTableInternal();
  return open && g_inTable;
}

void info(const char* label, const char* fmt, ...) {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("%s", label);
  ImGui::TableSetColumnIndex(1);
  ImGui::AlignTextToFramePadding();
  char buf[512];
  va_list args;
  va_start(args, fmt);
  std::vsnprintf(buf, sizeof buf, fmt, args);
  va_end(args);
  ImGui::TextWrapped("%s", buf);
}

void separator() {
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::Dummy(ImVec2(1, 2));
}

bool beginRow(const char* label, bool modified, const char* tip, bool* reset, const char* unit) {
  *reset = false;
  ImGui::TableNextRow();
  ImGui::TableSetColumnIndex(0);
  ImGui::AlignTextToFramePadding();
  if (modified) ImGui::PushStyleColor(ImGuiCol_Text, pal::v4(pal::kAccent));
  ImGui::TextUnformatted(label);
  if (modified) ImGui::PopStyleColor();
  if (unit && *unit) {
    ImGui::SameLine(0, 4);
    ImGui::TextDisabled("(%s)", unit);
  }
  if (tip && ImGui::IsItemHovered(ImGuiHoveredFlags_DelayShort)) {
    ImGui::SetNextWindowSize(ImVec2(420, 0));
    ImGui::BeginTooltip();
    ImGui::TextWrapped("%s", tip);
    if (modified) ImGui::TextDisabled("Modified from MuJoCo's default (click the reset button to restore).");
    ImGui::EndTooltip();
  }
  ImGui::TableSetColumnIndex(1);
  ImGui::PushID(label);
  float avail = ImGui::GetContentRegionAvail().x;
  float btn = ImGui::GetFrameHeight();
  if (modified) {
    float x0 = ImGui::GetCursorPosX();
    ImGui::SetCursorPosX(x0 + avail - btn);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    ImGui::PushStyleColor(ImGuiCol_Text, pal::v4(pal::kTextDim));
    if (ImGui::Button("\xE2\x86\xBA", ImVec2(btn, btn))) *reset = true;  // ↺
    ImGui::PopStyleColor(2);
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Reset to MuJoCo default");
    ImGui::SameLine();
    ImGui::SetCursorPosX(x0);
    ImGui::SetNextItemWidth(avail - btn - ImGui::GetStyle().ItemSpacing.x);
  } else {
    ImGui::SetNextItemWidth(avail);
  }
  return true;
}

void endRow() { ImGui::PopID(); }

bool real(const char* label, double* v, double def, const Opt& o) {
  bool reset = false;
  beginRow(label, !nearlyEq(*v, def), o.tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *v = def; changed = true; }
  double tmp = std::fabs(*v) < 1e-12 ? 0.0 : *v;
  std::string fmt = withUnit(o.fmt, o.unit);
  ImGui::BeginDisabled(o.disabled);
  if (ImGui::DragScalar("##v", ImGuiDataType_Double, &tmp, float(o.speed), &o.min, &o.max, fmt.c_str(),
                        ImGuiSliderFlags_AlwaysClamp) && tmp != *v) {
    notifyChange();
    *v = tmp;
    changed = true;
  }
  ImGui::EndDisabled();
  endRow();
  return changed;
}

bool angle(const char* label, double* rad, double defRad, const Opt& o) {
  bool reset = false;
  beginRow(label, !nearlyEq(*rad, defRad), o.tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *rad = defRad; changed = true; }
  double deg = rad2deg(*rad);
  double mn = o.min > -1e299 ? rad2deg(o.min) : -1e300, mx = o.max < 1e299 ? rad2deg(o.max) : 1e300;
  ImGui::BeginDisabled(o.disabled);
  if (ImGui::DragScalar("##v", ImGuiDataType_Double, &deg, 0.5f, &mn, &mx, "%.2f\xC2\xB0", ImGuiSliderFlags_AlwaysClamp)) {
    double r = deg2rad(deg);
    if (r != *rad) {
      notifyChange();
      *rad = r;
      changed = true;
    }
  }
  ImGui::EndDisabled();
  endRow();
  return changed;
}

bool integer(const char* label, int* v, int def, const Opt& o) {
  bool reset = false;
  beginRow(label, *v != def, o.tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *v = def; changed = true; }
  int tmp = *v;
  int mn = o.min > -1e9 ? int(o.min) : INT32_MIN, mx = o.max < 1e9 ? int(o.max) : INT32_MAX;
  ImGui::BeginDisabled(o.disabled);
  if (ImGui::DragInt("##v", &tmp, float(std::max(0.05, o.speed)), mn, mx, "%d", ImGuiSliderFlags_AlwaysClamp) && tmp != *v) {
    notifyChange();
    *v = tmp;
    changed = true;
  }
  ImGui::EndDisabled();
  endRow();
  return changed;
}

template <int N>
static bool dragN(const char* label, double* data, const double* def, const Opt& o) {
  bool modified = false;
  for (int i = 0; i < N; ++i) modified |= !nearlyEq(data[i], def[i]);
  bool reset = false;
  beginRow(label, modified, o.tip, &reset, o.unit);
  bool changed = false;
  if (reset) {
    notifyChange();
    for (int i = 0; i < N; ++i) data[i] = def[i];
    changed = true;
  }
  double tmp[N];
  for (int i = 0; i < N; ++i) tmp[i] = std::fabs(data[i]) < 1e-12 ? 0.0 : data[i];  // hide float noise
  std::string fmt = o.fmt ? o.fmt : "%.4g";
  ImGui::BeginDisabled(o.disabled);
  if (ImGui::DragScalarN("##v", ImGuiDataType_Double, tmp, N, float(o.speed), &o.min, &o.max, fmt.c_str(),
                         ImGuiSliderFlags_AlwaysClamp)) {
    bool diff = false;
    for (int i = 0; i < N; ++i) diff |= tmp[i] != (std::fabs(data[i]) < 1e-12 ? 0.0 : data[i]);
    if (diff) {
      notifyChange();
      for (int i = 0; i < N; ++i) data[i] = tmp[i];
      changed = true;
    }
  }
  ImGui::EndDisabled();
  endRow();
  return changed;
}

bool vec2(const char* label, Vec2* v, const Vec2& def, const Opt& o) {
  double d[2] = {v->x, v->y}, dd[2] = {def.x, def.y};
  bool c = dragN<2>(label, d, dd, o);
  if (c) *v = {d[0], d[1]};
  return c;
}

bool vec3(const char* label, Vec3* v, const Vec3& def, const Opt& o) {
  double d[3] = {v->x, v->y, v->z}, dd[3] = {def.x, def.y, def.z};
  bool c = dragN<3>(label, d, dd, o);
  if (c) *v = {d[0], d[1], d[2]};
  return c;
}

bool vec5(const char* label, Vec5* v, const Vec5& def, const Opt& o) { return dragN<5>(label, v->v, def.v, o); }

bool angleRange(const char* label, Vec2* r, const Opt& o) {
  double d[2] = {rad2deg(r->x), rad2deg(r->y)}, dd[2] = {0, 0};
  Opt oo = o;
  oo.speed = 0.5;
  oo.fmt = "%.1f\xC2\xB0";
  oo.unit = nullptr;
  bool c = dragN<2>(label, d, dd, oo);
  if (c) *r = {deg2rad(d[0]), deg2rad(d[1])};
  return c;
}

bool checkbox(const char* label, bool* v, bool def, const char* tip) {
  bool reset = false;
  beginRow(label, *v != def, tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *v = def; changed = true; }
  bool tmp = *v;
  if (ImGui::Checkbox("##v", &tmp) && tmp != *v) {
    notifyChange();
    *v = tmp;
    changed = true;
  }
  endRow();
  return changed;
}

bool color(const char* label, Color* c, const Color& def, const char* tip) {
  bool reset = false;
  beginRow(label, *c != def, tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *c = def; changed = true; }
  float tmp[4] = {c->r, c->g, c->b, c->a};
  if (ImGui::ColorEdit4("##v", tmp, ImGuiColorEditFlags_AlphaBar | ImGuiColorEditFlags_AlphaPreviewHalf)) {
    Color n{tmp[0], tmp[1], tmp[2], tmp[3]};
    if (n != *c) {
      notifyChange();
      *c = n;
      changed = true;
    }
  }
  endRow();
  return changed;
}

bool color3(const char* label, Vec3* c, const Vec3& def, const char* tip) {
  bool reset = false;
  bool mod = !(nearlyEq(c->x, def.x) && nearlyEq(c->y, def.y) && nearlyEq(c->z, def.z));
  beginRow(label, mod, tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *c = def; changed = true; }
  float tmp[3] = {float(c->x), float(c->y), float(c->z)};
  if (ImGui::ColorEdit3("##v", tmp, ImGuiColorEditFlags_Float)) {
    notifyChange();
    *c = {tmp[0], tmp[1], tmp[2]};
    changed = true;
  }
  endRow();
  return changed;
}

bool text(const char* label, std::string* s, const char* tip) {
  bool reset = false;
  beginRow(label, false, tip, &reset);
  std::string tmp = *s;
  bool changed = false;
  if (ImGui::InputText("##v", &tmp) && tmp != *s) {
    notifyChange();
    *s = tmp;
    changed = true;
  }
  endRow();
  return changed;
}

bool orientation(const char* label, Quat* q, const char* tip) {
  // Euler angles are derived from the quaternion; a per-widget cache keeps the displayed angles
  // stable while editing (Euler extraction is not unique).
  static std::unordered_map<ImGuiID, std::pair<Quat, Vec3>> cache;
  ImGuiID key = ImGui::GetID(label);
  Vec3 e;
  auto it = cache.find(key);
  Quat qc = canonical(*q);
  auto same = [](const Quat& a, const Quat& b) {
    return std::fabs(a.w - b.w) + std::fabs(a.x - b.x) + std::fabs(a.y - b.y) + std::fabs(a.z - b.z) < 1e-9;
  };
  if (it != cache.end() && same(it->second.first, qc)) e = it->second.second;
  else e = toEulerXYZ(qc);
  bool identity = same(qc, Quat{});
  bool reset = false;
  beginRow(label, !identity, tip, &reset);
  bool changed = false;
  if (reset) {
    notifyChange();
    *q = Quat{};
    cache.erase(key);
    endRow();
    return true;
  }
  double deg[3] = {rad2deg(e.x), rad2deg(e.y), rad2deg(e.z)};
  for (double& d : deg)
    if (std::fabs(d) < 5e-4) d = 0;  // avoid "-0.0"
  if (ImGui::DragScalarN("##v", ImGuiDataType_Double, deg, 3, 0.5f, nullptr, nullptr, "%.1f\xC2\xB0")) {
    Vec3 ne{deg2rad(deg[0]), deg2rad(deg[1]), deg2rad(deg[2])};
    Quat nq = canonical(fromEulerXYZ(ne));
    notifyChange();
    *q = nq;
    cache[key] = {nq, ne};
    changed = true;
  } else if (it == cache.end() || !same(it->second.first, qc)) {
    cache[key] = {qc, e};
  }
  endRow();
  return changed;
}

bool bits(const char* label, int* mask, int def, const char* tip) {
  bool reset = false;
  beginRow(label, *mask != def, tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *mask = def; changed = true; }
  float w = ImGui::CalcItemWidth();
  float btn = ImGui::GetFrameHeight() * 1.6f;
  ImGui::SetNextItemWidth(w - btn - ImGui::GetStyle().ItemSpacing.x);
  int tmp = *mask;
  if (ImGui::InputInt("##v", &tmp, 0, 0) && tmp != *mask) {
    notifyChange();
    *mask = tmp;
    changed = true;
  }
  ImGui::SameLine();
  if (ImGui::Button("bits", ImVec2(btn, 0))) ImGui::OpenPopup("bits");
  if (ImGui::BeginPopup("bits")) {
    ImGui::TextDisabled("Bit mask (bit 0 = value 1)");
    for (int b = 0; b < 32; ++b) {
      if (b % 8) ImGui::SameLine();
      bool on = (*mask >> b) & 1;
      char id[8];
      std::snprintf(id, sizeof id, "%d", b);
      ImGui::PushStyleColor(ImGuiCol_Button, on ? pal::v4(pal::kAccentDim) : ImGui::GetStyleColorVec4(ImGuiCol_FrameBg));
      if (ImGui::Button(id, ImVec2(ImGui::GetFrameHeight() * 1.3f, 0))) {
        notifyChange();
        *mask ^= (1 << b);
        changed = true;
      }
      ImGui::PopStyleColor();
    }
    ImGui::EndPopup();
  }
  endRow();
  return changed;
}

bool comboIndex(const char* label, int* index, const std::vector<std::string>& items, int def, const char* tip) {
  bool reset = false;
  beginRow(label, *index != def, tip, &reset);
  bool changed = false;
  if (reset) { notifyChange(); *index = def; changed = true; }
  const char* cur = (*index >= 0 && *index < int(items.size())) ? items[*index].c_str() : "";
  if (ImGui::BeginCombo("##v", cur)) {
    for (int i = 0; i < int(items.size()); ++i) {
      bool sel = i == *index;
      if (ImGui::Selectable(items[i].c_str(), sel) && !sel) {
        notifyChange();
        *index = i;
        changed = true;
      }
    }
    ImGui::EndCombo();
  }
  endRow();
  return changed;
}

bool element(const char* label, Id* id, const std::vector<std::pair<Id, std::string>>& items, bool allowNone, const char* tip,
             const char* noneLabel) {
  bool reset = false;
  beginRow(label, false, tip, &reset);
  bool changed = false;
  std::string cur = noneLabel;
  for (const auto& [i, n] : items) if (i == *id) cur = n.empty() ? "(unnamed)" : n;
  if (ImGui::BeginCombo("##v", cur.c_str(), ImGuiComboFlags_HeightLarge)) {
    static ImGuiTextFilter filter;
    if (ImGui::IsWindowAppearing()) {
      ImGui::SetKeyboardFocusHere();
      filter.Clear();
    }
    filter.Draw("##filter", -1);
    if (allowNone && ImGui::Selectable(noneLabel, *id == kNoId) && *id != kNoId) {
      notifyChange();
      *id = kNoId;
      changed = true;
    }
    for (const auto& [i, n] : items) {
      if (!filter.PassFilter(n.c_str())) continue;
      ImGui::PushID(int(i));
      if (ImGui::Selectable(n.empty() ? "(unnamed)" : n.c_str(), i == *id) && i != *id) {
        notifyChange();
        *id = i;
        changed = true;
      }
      ImGui::PopID();
    }
    ImGui::EndCombo();
  }
  endRow();
  return changed;
}

}  // namespace props
}  // namespace mjf
