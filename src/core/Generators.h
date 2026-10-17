// Procedural environment pieces for locomotion training: stairs, ramps, rough terrain, waves,
// stepping stones, obstacle fields and arenas.
//
// A generator owns one top-level environment body (static: no joints) and its geoms. Its type and
// parameters are stored on the body, so the piece can be moved as a unit and regenerated when a
// parameter changes. "Bake" turns it into plain geometry.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

struct GenParam {
  const char* key;
  const char* label;
  double def;
  double min, max;
  double speed;       // drag speed in the UI
  const char* unit;   // nullptr: unitless
  bool integer;
  const char* tip;
};

struct GeneratorInfo {
  const char* type;   // stable id stored in projects, e.g. "stairs"
  const char* label;  // menu label
  const char* description;
  std::vector<GenParam> params;
};

const std::vector<GeneratorInfo>& generatorList();
const GeneratorInfo* findGenerator(const std::string& type);

// Creates a generated environment body at the given world position/yaw. Returns its ID.
Id addGenerated(Document& doc, const std::string& type, const Vec3& origin, double yaw = 0,
                const std::vector<double>* params = nullptr);
// Rebuilds the geometry of a generated body from its stored parameters.
bool regenerate(Document& doc, Id body);
// Value of a parameter of a generated body (default if absent).
double genParam(const Body& b, const char* key);

}  // namespace mjf
