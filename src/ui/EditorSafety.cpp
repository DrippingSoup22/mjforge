// Not losing work: the "save changes?" prompt before replacing or closing a modified project, a
// background autosave of modified projects, and recovery of the autosave after a crash.
#include <chrono>
#include <cstdlib>
#include <filesystem>

#include <imgui.h>
#include <nlohmann/json.hpp>

#include "core/ProjectIO.h"
#include "ui/Editor.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {
// Seconds between autosaves of a modified project (MJFORGE_AUTOSAVE_INTERVAL overrides, for tests).
double autosaveInterval() {
  static double v = [] {
    const char* e = std::getenv("MJFORGE_AUTOSAVE_INTERVAL");
    return e && std::atof(e) > 0 ? std::atof(e) : 30.0;
  }();
  return v;
}
const char* const kAutosavePrefix = "autosave-";
double secondsNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

// ------------------------------------------------------------------------------------ unsaved

void Editor::requestDiscard(std::function<void()> action) {
  if (!modified_) {
    action();
    return;
  }
  pendingDiscard_ = std::move(action);
  showUnsaved_ = true;
  quit_ = false;  // the prompt decides
}

void Editor::drawUnsavedModal() {
  if (showUnsaved_) {
    ImGui::OpenPopup("Unsaved changes");
    showUnsaved_ = false;
  }
  if (!ImGui::BeginPopupModal("Unsaved changes", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  std::string name = path_.empty() ? "This project" : fs::path(path_).filename().string();
  ImGui::Text("%s has unsaved changes.", name.c_str());
  ImGui::TextDisabled("Save them first?");
  ImGui::Spacing();
  float w = 110 * scale_;
  if (ImGui::Button("Save", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();
    if (!path_.empty()) {
      if (savePath(path_) && pendingDiscard_) pendingDiscard_();
      pendingDiscard_ = nullptr;
    } else {
      afterSave_ = std::move(pendingDiscard_);  // runs once the file dialog saved the project
      pendingDiscard_ = nullptr;
      saveAs();
    }
  }
  ImGui::SameLine();
  if (ImGui::Button("Don't save", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();
    removeAutosave();
    modified_ = false;
    if (pendingDiscard_) pendingDiscard_();
    pendingDiscard_ = nullptr;
  }
  ImGui::SameLine();
  if (ImGui::Button("Cancel", ImVec2(w, 0)) || ImGui::IsKeyPressed(ImGuiKey_Escape, false)) {
    ImGui::CloseCurrentPopup();
    pendingDiscard_ = nullptr;
  }
  ImGui::EndPopup();
}

bool Editor::confirmQuit() {
  savePrefs();
  if (!modified_ || quitConfirmed_) {
    finishAutosave();
    removeAutosave();
    return true;
  }
  requestDiscard([this] {
    quitConfirmed_ = true;
    quit_ = true;
  });
  return false;
}

// ------------------------------------------------------------------------------------ autosave

void Editor::initAutosave() {
  if (configDir_.empty()) return;
  long long stamp = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
  autosavePath_ = (fs::path(configDir_) / (std::string(kAutosavePrefix) + std::to_string(stamp) + ".mjforge")).string();
  // A previous session that ended without saving left its autosave behind: offer the newest.
  std::error_code ec;
  fs::file_time_type newest{};
  for (const auto& e : fs::directory_iterator(configDir_, ec)) {
    std::string n = e.path().filename().string();
    if (n.rfind(kAutosavePrefix, 0) != 0 || e.path().extension() != ".mjforge") continue;
    auto t = fs::last_write_time(e.path(), ec);
    if (recoverPath_.empty() || t > newest) {
      recoverPath_ = e.path().string();
      newest = t;
    }
  }
  showRecover_ = !recoverPath_.empty();
}

bool Editor::autosaveDue() const {
  return !autosavePath_.empty() && modified_ && editRevision_ != autosavedRevision_ && !autosaveJob_.valid() &&
         secondsNow() - lastAutosaveTime_ >= autosaveInterval();
}

void Editor::maybeAutosave() {
  if (autosaveJob_.valid() && autosaveJob_.wait_for(std::chrono::seconds(0)) == std::future_status::ready) autosaveJob_.get();
  if (!autosaveDue()) return;
  lastAutosaveTime_ = secondsNow();
  autosavedRevision_ = editRevision_;
  // Serialize a copy on a worker thread: big projects (meshes) take a moment to encode.
  auto copy = std::make_shared<Document>(doc_);
  nlohmann::json state = nlohmann::json::parse(editorStateJson(), nullptr, false);
  if (state.is_discarded()) state = nlohmann::json::object();
  state["autosaveOf"] = path_;
  std::string stateText = state.dump(), path = autosavePath_;
  autosaveJob_ = std::async(std::launch::async, [copy, stateText, path] {
    std::string tmp = path + ".tmp", err;
    if (!saveProject(*copy, tmp, &err, stateText)) return false;
    std::error_code ec;
    fs::rename(tmp, path, ec);  // atomic replace: a crash mid-write never leaves a broken file
    return !ec;
  });
}

void Editor::finishAutosave() {
  if (autosaveJob_.valid()) autosaveJob_.get();
}

void Editor::removeAutosave() {
  finishAutosave();
  if (autosavePath_.empty()) return;
  std::error_code ec;
  fs::remove(autosavePath_, ec);
  autosavedRevision_ = editRevision_;
}

void Editor::drawRecoverModal() {
  if (showRecover_) {
    ImGui::OpenPopup("Recover unsaved work?");
    showRecover_ = false;
  }
  if (!ImGui::BeginPopupModal("Recover unsaved work?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  std::error_code ec;
  auto age = fs::file_time_type::clock::now() - fs::last_write_time(recoverPath_, ec);
  long long minutes = std::chrono::duration_cast<std::chrono::minutes>(age).count();
  ImGui::TextUnformatted("MJForge closed before the last changes were saved.");
  ImGui::TextDisabled("An autosave from %s ago is available.",
                      minutes < 1 ? "less than a minute" : minutes < 120 ? (std::to_string(minutes) + " minutes").c_str()
                                                                         : (std::to_string(minutes / 60) + " hours").c_str());
  ImGui::Spacing();
  float w = 110 * scale_;
  if (ImGui::Button("Recover", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();
    Document d;
    std::string err, state;
    if (loadProject(d, recoverPath_, &err, &state)) {
      stopSimulation();
      doc_ = std::move(d);
      history_.clear();
      sel_.clear();
      auto j = nlohmann::json::parse(state, nullptr, false);
      path_ = !j.is_discarded() && j.contains("autosaveOf") && j["autosaveOf"].is_string() ? j["autosaveOf"].get<std::string>() : "";
      applyEditorState(state);
      markDirty();  // recovered work still has to be saved
      fs::remove(recoverPath_, ec);
      notify("Recovered the autosaved project. Save it to keep it.", pal::kOk, 6);
    } else {
      notify("Could not recover: " + err, pal::kError, 6);
    }
    recoverPath_.clear();
  }
  ImGui::SameLine();
  if (ImGui::Button("Discard", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();
    fs::remove(recoverPath_, ec);
    recoverPath_.clear();
  }
  ImGui::SameLine();
  if (ImGui::Button("Later", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();  // keep the file; asked again next time
    recoverPath_.clear();
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Keep the autosave and decide at the next start");
  ImGui::EndPopup();
}

}  // namespace mjf
