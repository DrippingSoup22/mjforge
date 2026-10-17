// Project files (.mjforge): the full document as versioned, human-readable JSON.
#pragma once

#include <string>

#include "core/Document.h"

namespace mjf {

constexpr int kProjectFormatVersion = 1;

std::string projectToJson(const Document& doc, const std::string& editorStateJson = "");
// Replaces `doc` on success. `editorStateJson` (optional) receives the "editor" object.
bool projectFromJson(Document& doc, const std::string& json, std::string* error, std::string* editorStateJson = nullptr);

bool saveProject(const Document& doc, const std::string& path, std::string* error,
                 const std::string& editorStateJson = "");
bool loadProject(Document& doc, const std::string& path, std::string* error, std::string* editorStateJson = nullptr);

bool readTextFile(const std::string& path, std::string& out, std::string* error = nullptr);
bool writeTextFile(const std::string& path, const std::string& content, std::string* error = nullptr);

}  // namespace mjf
