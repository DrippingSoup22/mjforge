// Writes a Document as MJCF XML.
//
// The output is deterministic and minimal: attributes equal to MuJoCo's defaults are omitted.
// Orientations are always written as quaternions and <compiler angle=...> is always explicit.
// Note that MuJoCo applies the last <compiler angle> to the whole model, so an exported file
// included into a scene with another angle unit changes meaning; <attach> keeps both intact.
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
  int precision = 0;                // significant digits for numbers; 0: shortest exact text
  // Only for files written to disk (writeMjcf with `files`): meshes with more vertices than this
  // become binary STL files in `assetDir` (relative to the MJCF file) instead of inline vertex data,
  // and texture images are copied there, so the export is self-contained.
  size_t externalMeshVertices = size_t(-1);
  // Same for height fields with more samples than this: MuJoCo's binary format (exact values).
  size_t externalHfieldSamples = size_t(-1);
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
// MuJoCo's binary height field file: int32 nrow, int32 ncol, then float32 rows from the -Y edge.
std::string hfieldToBin(const Hfield& hfield);

// Writes an MJCF file plus its asset files into `<assetStem>_assets/` next to the XML (assetStem
// defaults to the file's stem): meshes with more than 1000 vertices as binary STL (smaller ones
// stay inline), height fields over 64x64 samples as binary files, and copies of texture images.
// `written` (optional) receives the written paths.
bool exportMjcfFile(const Document& doc, const std::string& path, const ExportOptions& opt, std::string* error,
                    const std::string& assetStem = "", std::vector<std::string>* written = nullptr);

// Writes a scene for training setups that swap robots or terrains: <stem>_model.xml holds the
// models (bodies with the Model role) and scene.xml the environment, including the model file.
// Assets go to <stem>_assets/ once; elements that tie models to the environment (contact pairs,
// exclusions, constraints, tendons, actuators, sensors) and the keyframes live in the scene.
bool exportSplitMjcf(const Document& doc, const std::string& scenePath, const ExportOptions& opt, std::string* error,
                     std::vector<std::string>* written = nullptr);

// Number formatting used by the writer (exposed for tests and the UI).
// precision <= 0 gives the shortest text that reads back as exactly v.
std::string formatNumber(double v, int precision = 9);

}  // namespace mjf
