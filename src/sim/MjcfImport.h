// Imports MJCF into an editor document with MuJoCo's own parser and compiler, so defaults,
// <include>s, alternative orientations (euler, axisangle, fromto, ...) and units behave exactly
// as in MuJoCo. Geometry and kinematics come from the compiled model; authoring intent that the
// compiler would lose (geom densities, explicit inertials, actuator shortcut types, damping ratios)
// comes from the parsed spec.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

struct ImportReport {
  std::vector<std::string> notes;  // features that were approximated or not imported
  int approximatedGeoms = 0;       // SDF geoms replaced by boxes
};

// Loads an MJCF file (includes and asset files resolved relative to it).
bool importMjcfFile(const std::string& path, Document& out, std::string* error, ImportReport* report = nullptr);
// Loads MJCF text (no external files).
bool importMjcfString(const std::string& xml, Document& out, std::string* error, ImportReport* report = nullptr);

// Loads an STL, OBJ or MSH file with MuJoCo (vertices multiplied by `scale`) and adds it to `doc` as
// a mesh asset. Like MuJoCo, the vertices are stored relative to the mesh's center of volume and
// principal axes; `frame` receives that frame in the file's coordinates, the pose a geom needs to
// show the mesh where the file puts it.
Id loadMeshFile(Document& doc, const std::string& path, double scale, Pose* frame, std::string* error);

}  // namespace mjf
