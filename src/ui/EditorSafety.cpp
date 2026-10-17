// Not losing work: the "save changes?" prompt before replacing or closing a modified project, a
// background autosave of modified projects, and recovery of the autosave after a crash.
#include <chrono>
#include <cerrno>
#include <cstdlib>
#include <filesystem>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#else
#include <signal.h>
#include <unistd.h>
#endif

#include <imgui.h>
#include <nlohmann/json.hpp>

#include "core/ProjectIO.h"
#include "ui/Editor.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {
// MJFORGE_AUTOSAVE_INTERVAL overrides the preference (for tests).
double autosaveOverride() {
  const char* e = std::getenv("MJFORGE_AUTOSAVE_INTERVAL");
  return e && std::atof(e) > 0 ? std::atof(e) : 0.0;
}
const char* const kAutosavePrefix = "autosave-";
double secondsNow() {
  return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

long processId() {
#if defined(_WIN32)
  return long(GetCurrentProcessId());
#else
  return long(getpid());
#endif
}

// Whether another MJForge that owns an autosave is still running (then it is not a crash leftover).
bool processAlive(long pid) {
  if (pid <= 0 || pid == processId()) return false;
#if defined(_WIN32)
  HANDLE h = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, FALSE, DWORD(pid));
  if (!h) return GetLastError() == ERROR_ACCESS_DENIED;
  DWORD code = 0;
  bool alive = GetExitCodeProcess(h, &code) && code == STILL_ACTIVE;
  CloseHandle(h);
  return alive;
#else
  return kill(pid_t(pid), 0) == 0 || errno == EPERM;
#endif
}

// "autosave-<time>-<pid>.mjforge" -> pid (0 for older files without one).
long autosaveOwner(const std::string& name) {
  size_t dash = name.rfind('-'), dot = name.rfind('.');
  if (dash == std::string::npos || dot == std::string::npos || dash < std::string(kAutosavePrefix).size() || dot <= dash + 1) return 0;
  return std::strtol(name.substr(dash + 1, dot - dash - 1).c_str(), nullptr, 10);
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
  std::string name = path_.empty() ? "This project" : fs::u8path(path_).filename().u8string();
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
    // The action itself drops the changes when it succeeds (new / open / quit reset the modified
    // flag and remove the autosave); an Open dialog that gets cancelled keeps them protected.
    ImGui::CloseCurrentPopup();
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
  autosavePath_ = (fs::u8path(configDir_) /
                   (std::string(kAutosavePrefix) + std::to_string(stamp) + "-" + std::to_string(processId()) + ".mjforge"))
                      .u8string();
  // A previous session that ended without saving left its autosave behind: offer the newest. An
  // MJForge that is still running owns its autosave: leave that one alone.
  std::error_code ec;
  fs::file_time_type newest{};
  for (const auto& e : fs::directory_iterator(fs::u8path(configDir_), ec)) {
    std::string n = e.path().filename().u8string();
    if (n.rfind(kAutosavePrefix, 0) != 0 || e.path().extension() != ".mjforge") continue;
    if (processAlive(autosaveOwner(n))) continue;
    auto t = fs::last_write_time(e.path(), ec);
    if (recoverPath_.empty() || t > newest) {
      recoverPath_ = e.path().u8string();
      newest = t;
    }
  }
  showRecover_ = !recoverPath_.empty();
}

bool Editor::autosaveDue() const {
  double interval = autosaveOverride() > 0 ? autosaveOverride() : autosaveSeconds_;
  return interval > 0 && !autosavePath_.empty() && modified_ && editRevision_ != autosavedRevision_ && !autosaveJob_.valid() &&
         secondsNow() - lastAutosaveTime_ >= interval;
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
  std::string stateText = state.dump(-1, ' ', false, nlohmann::json::error_handler_t::replace), path = autosavePath_;
  autosaveJob_ = std::async(std::launch::async, [copy, stateText, path] {
    try {  // an exception here would surface on the UI thread at get()
      std::string err;
      return saveProject(*copy, path, &err, stateText);  // written to a temporary file and renamed
    } catch (const std::exception&) {
      return false;
    }
  });
}

void Editor::finishAutosave() {
  if (autosaveJob_.valid()) autosaveJob_.get();
}

void Editor::removeAutosave() {
  finishAutosave();
  if (autosavePath_.empty()) return;
  std::error_code ec;
  fs::remove(fs::u8path(autosavePath_), ec);
  autosavedRevision_ = editRevision_;
}

void Editor::drawRecoverModal() {
  if (showRecover_) {
    ImGui::OpenPopup("Recover unsaved work?");
    showRecover_ = false;
  }
  if (!ImGui::BeginPopupModal("Recover unsaved work?", nullptr, ImGuiWindowFlags_AlwaysAutoResize)) return;
  std::error_code ec;
  auto age = fs::file_time_type::clock::now() - fs::last_write_time(fs::u8path(recoverPath_), ec);
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
      fs::remove(fs::u8path(recoverPath_), ec);
      notify("Recovered the autosaved project. Save it to keep it.", pal::kOk, 6);
    } else {
      notify("Could not recover: " + err, pal::kError, 6);
    }
    recoverPath_.clear();
  }
  ImGui::SameLine();
  if (ImGui::Button("Discard", ImVec2(w, 0))) {
    ImGui::CloseCurrentPopup();
    fs::remove(fs::u8path(recoverPath_), ec);
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
