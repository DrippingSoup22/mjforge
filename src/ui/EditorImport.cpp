// Opening MJCF files and importing projects / MJCF into the current scene.
#include <algorithm>
#include <filesystem>

#include <imgui.h>

#include "core/Merge.h"
#include "core/ProjectIO.h"
#include "sim/MjcfImport.h"
#include "ui/Editor.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {

bool isMjcfPath(const std::string& path) {
  std::string ext = fs::path(path).extension().string();
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return char(std::tolower(c)); });
  return ext == ".xml" || ext == ".mjcf";
}

}  // namespace

bool Editor::loadAnyFile(const std::string& path, Document& out, std::vector<std::string>* notes,
                         std::string* editorState) {
  std::string err;
  if (isMjcfPath(path)) {
    ImportReport rep;
    if (!importMjcfFile(path, out, &err, &rep)) {
      notify("MuJoCo could not load " + fs::path(path).filename().string() + ": " + err, pal::kError, 8);
      return false;
    }
    if (notes) *notes = rep.notes;
    return true;
  }
  if (!loadProject(out, path, &err, editorState)) {
    notify("Open failed: " + err, pal::kError, 6);
    return false;
  }
  return true;
}

void Editor::importDialog() {
  fileDialog_.open(FileDialog::Mode::Open, "Import into the scene", {".mjforge", ".xml"}, lastDir_, "", kPurposeImport);
}

void Editor::beginImport(const std::string& path) {
  auto doc = std::make_unique<Document>();
  std::vector<std::string> notes;
  if (!loadAnyFile(path, *doc, &notes, nullptr)) return;
  DocSummary s = summarize(*doc);
  bool hasEnv = s.worldGeoms > 0 || s.environmentBodies > 0;
  importOpt_ = MergeOptions{};
  importOpt_.models = !s.models.empty();
  // A creature file brings its creature; an environment file replaces the current environment.
  importOpt_.environment = s.models.empty() && hasEnv;
  importOpt_.replaceEnvironment = importOpt_.environment;
  importOpt_.settings = false;
  // Put imported models next to the existing ones.
  Aabb mine = modelsBounds(doc_), theirs = modelsBounds(*doc);
  if (mine.valid() && theirs.valid()) importOpt_.modelOffset = {mine.hi.x - theirs.lo.x + 0.3, 0, 0};
  importDoc_ = std::move(doc);
  importPath_ = path;
  importNotes_ = notes;
  showImport_ = true;
  lastDir_ = fs::path(path).parent_path().string();
}

void Editor::drawImportModal() {
  if (showImport_) {
    ImGui::OpenPopup("Import into the scene");
    showImport_ = false;
  }
  ImGui::SetNextWindowSize(ImVec2(560 * scale_, 0), ImGuiCond_Appearing);
  if (!ImGui::BeginPopupModal("Import into the scene", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  if (!importDoc_) {
    ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
    return;
  }
  DocSummary s = summarize(*importDoc_);
  ImGui::Text("From %s", fs::path(importPath_).filename().string().c_str());
  ImGui::Separator();

  std::string models;
  for (size_t i = 0; i < s.models.size() && i < 4; ++i) models += (i ? ", " : "") + s.models[i];
  if (s.models.size() > 4) models += ", ...";
  ImGui::BeginDisabled(s.models.empty());
  ImGui::Checkbox("Models", &importOpt_.models);
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (s.models.empty()) ImGui::TextDisabled("(none in this file)");
  else ImGui::TextDisabled("%zu: %s", s.models.size(), models.c_str());
  if (importOpt_.models) {
    ImGui::Indent();
    float off[3] = {float(importOpt_.modelOffset.x), float(importOpt_.modelOffset.y), float(importOpt_.modelOffset.z)};
    ImGui::SetNextItemWidth(260 * scale_);
    if (ImGui::DragFloat3("Offset (m)", off, 0.01f, -100, 100, "%.3f"))
      importOpt_.modelOffset = {off[0], off[1], off[2]};
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("Moves the imported models so they don't overlap yours.");
    ImGui::Unindent();
  }

  bool hasEnv = s.worldGeoms > 0 || s.environmentBodies > 0;
  ImGui::BeginDisabled(!hasEnv);
  ImGui::Checkbox("Environment", &importOpt_.environment);
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (!hasEnv) ImGui::TextDisabled("(none in this file)");
  else
    ImGui::TextDisabled("%d world geom%s%s, %d bod%s, %d light%s", s.worldGeoms, s.worldGeoms == 1 ? "" : "s",
                        s.hasFloor ? " incl. floor" : "", s.environmentBodies, s.environmentBodies == 1 ? "y" : "ies",
                        s.lights, s.lights == 1 ? "" : "s");
  if (importOpt_.environment) {
    ImGui::Indent();
    ImGui::Checkbox("Replace my current environment (floor, obstacles, lights, sky)", &importOpt_.replaceEnvironment);
    if (!importOpt_.replaceEnvironment)
      ImGui::TextDisabled("A second floor plane is skipped if you already have one.");
    ImGui::Unindent();
  }
  ImGui::Checkbox("Also use its simulation settings (timestep, gravity, solver, ...)", &importOpt_.settings);

  if (!importNotes_.empty()) {
    ImGui::Separator();
    ImGui::TextColored(pal::v4(pal::kWarning), "Imported from MJCF with these limitations:");
    for (const std::string& n : importNotes_) ImGui::BulletText("%s", n.c_str());
  }
  ImGui::Separator();
  bool any = importOpt_.models || importOpt_.environment || importOpt_.settings;
  ImGui::BeginDisabled(!any);
  if (ImGui::Button("Import", ImVec2(120 * scale_, 0))) {
    checkpoint("Import");
    MergeResult r = mergeDocument(doc_, *importDoc_, importOpt_);
    sel_.ids = r.modelRoots.empty() ? r.topLevel : r.modelRoots;
    std::string msg = "Imported " + std::to_string(r.elements) + " elements";
    if (r.renamed) msg += ", " + std::to_string(r.renamed) + " renamed to keep names unique";
    if (r.skippedFloors) msg += ", kept your floor";
    notify(msg, pal::kOk, 5);
    importDoc_.reset();
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndDisabled();
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(100 * scale_, 0))) {
    importDoc_.reset();
    ImGui::CloseCurrentPopup();
  }
  ImGui::EndPopup();
}

}  // namespace mjf
