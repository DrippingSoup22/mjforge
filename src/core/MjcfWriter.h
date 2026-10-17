// Writes a Document as MJCF XML.
//
// The output is deterministic and minimal: attributes equal to MuJoCo's defaults are omitted.
// Orientations are always written as quaternions, and <compiler angle=...> is always explicit, so
// the files behave the same when they are included into other models.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

struct ExportOptions {
  enum class Scope {
    All,          // everything: environment + all models
    Models,       // only bodies whose top-level role is Model (and what references them)
    Environment,  // world geoms/sites/lights/cameras + environment bodies + scene settings
  };
  Scope scope = Scope::All;
  Id modelRoot = kNoId;             // Models scope: restrict to this top-level body
  AngleUnit angle = AngleUnit::Radian;
  std::string includeFile;          // emit <include file="..."/> (split scene files)
  bool writeOption = true;          // <option> block
  bool writeVisual = true;          // <visual> block
  bool header = true;               // generator comment
  int precision = 9;                // significant digits for numbers
  // Only for files written to disk (writeMjcf with `files`): meshes with more vertices than this
  // become binary STL files in `assetDir` (relative to the MJCF file) instead of inline vertex data,
  // and texture images are copied there, so the export is self-contained.
  size_t externalMeshVertices = size_t(-1);
  std::string assetDir = "assets";
};

struct ExportFile {
  std::string path;     // relative to the MJCF file
  std::string content;  // binary
};

// Maps compiled MuJoCo indices to document IDs. Only exact for single-file exports.
struct ExportMap {
  std::vector<Id> bodies, geoms, joints, sites, cameras, lights, actuators, sensors, tendons;
};

std::string writeMjcf(const Document& doc, const ExportOptions& opt = {}, ExportMap* map = nullptr,
                      std::vector<ExportFile>* files = nullptr);

// Binary STL of a triangle mesh.
std::string meshToStl(const MeshAsset& mesh);

// Writes an MJCF file plus its asset files into `<assetStem>_assets/` next to the XML (assetStem
// defaults to the file's stem): meshes with more than 1000 vertices as binary STL (smaller ones
// stay inline) and copies of texture images. `written` (optional) receives the written paths.
bool exportMjcfFile(const Document& doc, const std::string& path, const ExportOptions& opt, std::string* error,
                    const std::string& assetStem = "", std::vector<std::string>* written = nullptr);

// Number formatting used by the writer (exposed for tests and the UI).
std::string formatNumber(double v, int precision = 9);

}  // namespace mjf
