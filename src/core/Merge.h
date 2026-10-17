// Combining documents: bringing the models and/or the environment of another project (or of an
// imported MJCF file) into the current one.
#pragma once

#include <string>
#include <vector>

#include "core/Document.h"

namespace mjf {

struct MergeOptions {
  bool models = true;               // top-level bodies with role Model, with their actuators/sensors
  bool environment = true;          // world geoms/sites/lights/cameras, environment bodies, skybox
  bool replaceEnvironment = false;  // remove the destination's environment first
  bool settings = false;            // take the simulation and visual settings from the source (the
                                    // compiler's mass settings stay: imported bodies keep their masses)
  Vec3 modelOffset;                 // added to the positions of imported model roots (world frame)
};

struct MergeResult {
  std::vector<Id> modelRoots;  // imported top-level model bodies
  std::vector<Id> topLevel;    // every imported top-level element (for selection)
  int renamed = 0;             // elements renamed because the name was taken
  int skippedFloors = 0;       // floor planes not imported because the destination has one
  int elements = 0;            // number of elements imported
};

MergeResult mergeDocument(Document& dst, const Document& src, const MergeOptions& opt);

// What a document contains, for the import dialog.
struct DocSummary {
  std::vector<std::string> models;  // names of top-level model bodies
  int environmentBodies = 0;
  int worldGeoms = 0;
  int lights = 0;
  bool hasFloor = false;
};
DocSummary summarize(const Document& doc);

// World-space bounding box of all models (invalid if there are none); used to place imports.
Aabb modelsBounds(const Document& doc);

}  // namespace mjf
