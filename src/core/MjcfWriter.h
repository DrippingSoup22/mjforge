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
};

// Maps compiled MuJoCo indices to document IDs. Only exact for single-file exports.
struct ExportMap {
  std::vector<Id> bodies, geoms, joints, sites, cameras, lights, actuators, sensors;
};

std::string writeMjcf(const Document& doc, const ExportOptions& opt = {}, ExportMap* map = nullptr);

// Number formatting used by the writer (exposed for tests and the UI).
std::string formatNumber(double v, int precision = 9);

}  // namespace mjf
