// Undo/redo by whole-document snapshots. Documents are small and heavy payloads (height field
// data) are shared copy-on-write, so snapshots are cheap.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

class History {
 public:
  explicit History(size_t limit = 300) : limit_(limit) {}

  // Records `before` (the state prior to an edit) so that the edit can be undone.
  void push(const Document& before, std::string label) {
    undo_.push_back({before, std::move(label)});
    if (undo_.size() > limit_) undo_.erase(undo_.begin());
    redo_.clear();
  }
  bool canUndo() const { return !undo_.empty(); }
  bool canRedo() const { return !redo_.empty(); }
  const std::string& undoLabel() const { return undo_.back().label; }
  const std::string& redoLabel() const { return redo_.back().label; }

  bool undo(Document& doc) {
    if (undo_.empty()) return false;
    redo_.push_back({doc, undo_.back().label});
    doc = std::move(undo_.back().doc);
    undo_.pop_back();
    doc.touch();
    return true;
  }
  bool redo(Document& doc) {
    if (redo_.empty()) return false;
    undo_.push_back({doc, redo_.back().label});
    doc = std::move(redo_.back().doc);
    redo_.pop_back();
    doc.touch();
    return true;
  }
  void clear() {
    undo_.clear();
    redo_.clear();
  }
  size_t undoCount() const { return undo_.size(); }

 private:
  struct Entry {
    Document doc;
    std::string label;
  };
  std::vector<Entry> undo_, redo_;
  size_t limit_;
};

}  // namespace mjf
