#include "ui/FileDialog.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <filesystem>

#include <imgui.h>
#include <imgui_stdlib.h>

#include "ui/Icons.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {
std::string lower(std::string s) {
  for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
  return s;
}
std::string homeDir() {
#ifdef _WIN32
  const char* h = std::getenv("USERPROFILE");
#else
  const char* h = std::getenv("HOME");
#endif
  return h ? h : ".";
}
}  // namespace

void FileDialog::open(Mode mode, const std::string& title, const std::vector<std::string>& extensions,
                      const std::string& startDir, const std::string& defaultName, int purpose) {
  mode_ = mode;
  title_ = title;
  exts_ = extensions;
  std::error_code ec;
  dir_ = (!startDir.empty() && fs::is_directory(fs::u8path(startDir), ec)) ? fs::absolute(fs::u8path(startDir), ec).u8string()
                                                                : fs::current_path(ec).u8string();
  name_ = defaultName;
  result_.clear();
  error_.clear();
  purpose_ = purpose;
  selected_ = -1;
  confirmOverwrite_ = false;
  refresh();
  open_ = true;
  requestOpen_ = true;
}

void FileDialog::refresh() {
  entries_.clear();
  pathEdit_ = dir_;
  std::error_code ec;
  for (fs::directory_iterator it(dir_, fs::directory_options::skip_permission_denied, ec), end; !ec && it != end;
       it.increment(ec)) {
    const fs::directory_entry& e = *it;
    std::string name = e.path().filename().u8string();
    if (name.empty() || name[0] == '.') continue;
    std::error_code ec2;
    bool isDir = e.is_directory(ec2);
    if (!isDir) {
      if (!exts_.empty()) {
        std::string ext = lower(e.path().extension().u8string());
        if (std::find(exts_.begin(), exts_.end(), ext) == exts_.end()) continue;
      }
    }
    entries_.push_back({name, isDir, isDir ? 0 : e.file_size(ec2)});
  }
  std::sort(entries_.begin(), entries_.end(), [](const Entry& a, const Entry& b) {
    if (a.dir != b.dir) return a.dir;
    return lower(a.name) < lower(b.name);
  });
  selected_ = -1;
}

bool FileDialog::accept(const std::string& path) {
  if (mode_ == Mode::Save) {
    std::string p = path;
    if (!exts_.empty() && fs::u8path(p).extension().empty()) p += exts_.front();
    std::error_code ec;
    if (fs::exists(fs::u8path(p), ec) && !confirmOverwrite_) {
      confirmOverwrite_ = true;
      result_ = p;
      return false;
    }
    result_ = p;
  } else {
    result_ = path;
  }
  open_ = false;
  return true;
}

bool FileDialog::draw() {
  if (!open_) return false;
  bool done = false;
  if (requestOpen_) {
    ImGui::OpenPopup("##filedialog");
    requestOpen_ = false;
  }
  ImVec2 vs = ImGui::GetMainViewport()->WorkSize;
  ImGui::SetNextWindowSize(ImVec2(std::min(760.0f, vs.x * 0.9f), std::min(520.0f, vs.y * 0.85f)), ImGuiCond_Appearing);
  ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, ImVec2(0.5f, 0.5f));
  bool keepOpen = true;
  std::string label = title_ + "###filedialog";
  if (ImGui::BeginPopupModal("##filedialog", &keepOpen, ImGuiWindowFlags_NoSavedSettings)) {
    ImGui::TextUnformatted(title_.c_str());
    ImGui::Separator();
    // Navigation bar.
    if (ImGui::Button("Up")) {
      fs::path p(dir_);
      if (p.has_parent_path() && p.parent_path() != p) {
        dir_ = p.parent_path().u8string();
        refresh();
      }
    }
    ImGui::SameLine();
    if (ImGui::Button("Home")) {
      dir_ = homeDir();
      refresh();
    }
    ImGui::SameLine();
    ImGui::SetNextItemWidth(-1);
    if (ImGui::InputText("##path", &pathEdit_, ImGuiInputTextFlags_EnterReturnsTrue)) {
      std::error_code ec;
      if (fs::is_directory(fs::u8path(pathEdit_), ec)) {
        dir_ = fs::absolute(fs::u8path(pathEdit_), ec).u8string();
        refresh();
      } else if (fs::is_regular_file(fs::u8path(pathEdit_), ec)) {
        done = accept(pathEdit_);
      }
    }
    float footer = ImGui::GetFrameHeightWithSpacing() * (mode_ == Mode::Save ? 2.4f : 1.4f);
    ImGui::BeginChild("##entries", ImVec2(0, -footer), ImGuiChildFlags_Borders);
    for (int i = 0; i < int(entries_.size()); ++i) {
      const Entry& e = entries_[i];
      ImGui::PushID(i);
      iconInline(e.dir ? Icon::Box : Icon::Plane, e.dir ? pal::kInfo : pal::kTextDim);
      ImGui::SameLine();
      bool sel = selected_ == i;
      if (ImGui::Selectable(e.name.c_str(), sel, ImGuiSelectableFlags_AllowDoubleClick)) {
        selected_ = i;
        if (!e.dir) name_ = e.name;
        if (ImGui::IsMouseDoubleClicked(0)) {
          if (e.dir) {
            dir_ = (fs::u8path(dir_) / e.name).u8string();
            refresh();
            ImGui::PopID();
            break;
          } else {
            done = accept((fs::u8path(dir_) / e.name).u8string());
          }
        }
      }
      if (!e.dir) {
        ImGui::SameLine(ImGui::GetContentRegionAvail().x - 60);
        ImGui::TextDisabled("%.1f KB", double(e.size) / 1024.0);
      }
      ImGui::PopID();
    }
    if (entries_.empty()) ImGui::TextDisabled("(no matching files)");
    ImGui::EndChild();

    if (mode_ == Mode::Save) {
      ImGui::AlignTextToFramePadding();
      ImGui::TextUnformatted("File name");
      ImGui::SameLine();
      ImGui::SetNextItemWidth(-1);
      if (ImGui::InputText("##name", &name_, ImGuiInputTextFlags_EnterReturnsTrue) && !name_.empty())
        done = accept((fs::u8path(dir_) / name_).u8string());
    }
    if (confirmOverwrite_) {
      ImGui::TextColored(pal::v4(pal::kWarning), "File exists. Press Save again to overwrite.");
      ImGui::SameLine();
    }
    float bw = 100;
    ImGui::SetCursorPosX(ImGui::GetWindowContentRegionMax().x - bw * 2 - ImGui::GetStyle().ItemSpacing.x);
    if (ImGui::Button("Cancel", ImVec2(bw, 0))) {
      open_ = false;
    }
    ImGui::SameLine();
    const char* ok = mode_ == Mode::Save ? "Save" : "Open";
    bool canOk = mode_ == Mode::Save ? !name_.empty() : (selected_ >= 0 && !entries_[selected_].dir);
    ImGui::BeginDisabled(!canOk);
    if (ImGui::Button(ok, ImVec2(bw, 0))) {
      if (mode_ == Mode::Save) done = accept((fs::u8path(dir_) / name_).u8string());
      else if (selected_ >= 0) done = accept((fs::u8path(dir_) / entries_[selected_].name).u8string());
    }
    ImGui::EndDisabled();
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) open_ = false;
    if (!open_) ImGui::CloseCurrentPopup();
    ImGui::EndPopup();
  }
  if (!keepOpen) open_ = false;
  return done;
}

}  // namespace mjf
