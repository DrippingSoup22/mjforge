// Portable in-app file browser (no native dialog dependencies).
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace mjf {

class FileDialog {
 public:
  enum class Mode { Open, Save };

  void open(Mode mode, const std::string& title, const std::vector<std::string>& extensions,
            const std::string& startDir, const std::string& defaultName = "", int purpose = 0);
  // Draw every frame. Returns true once when the user confirmed a path.
  bool draw();
  bool isOpen() const { return open_; }
  const std::string& result() const { return result_; }
  int purpose() const { return purpose_; }
  const std::string& directory() const { return dir_; }

 private:
  void refresh();
  bool accept(const std::string& path);

  struct Entry {
    std::string name;
    bool dir;
    uintmax_t size;
  };
  bool open_ = false, requestOpen_ = false;
  Mode mode_ = Mode::Open;
  std::string title_;
  std::vector<std::string> exts_;
  std::string dir_, pathEdit_, name_, result_, error_;
  std::vector<Entry> entries_;
  int selected_ = -1;
  int purpose_ = 0;
  bool confirmOverwrite_ = false;
};

}  // namespace mjf
