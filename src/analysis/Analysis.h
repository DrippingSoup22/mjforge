// Model analysis: structural checks, physics plausibility (actuator strength vs. loads, servo
// sag, timestep stability, mass conditioning), initial-state checks, MuJoCo Warp compatibility
// rules and performance hints. Every finding is an Issue with an explanation and, when possible,
// a one-click fix.
#pragma once

#include <functional>
#include <limits>
#include <string>
#include <vector>

#include "core/Document.h"
#include "sim/MujocoBridge.h"

namespace mjf {

enum class Severity : uint8_t { Error, Warning, Info, Tip };
enum class IssueArea : uint8_t { Model, Physics, Stability, Start, MJWarp, Performance };
const char* severityName(Severity s);
const char* areaName(IssueArea a);

struct Issue {
  Severity severity = Severity::Info;
  IssueArea area = IssueArea::Model;
  std::string code;    // stable identifier, e.g. "weak-actuator"
  std::string title;   // one line
  std::string detail;  // explanation with numbers
  std::vector<Id> elements;
  std::string fixLabel;
  std::function<void(Document&)> fix;
};

// Strength / dynamics summary for a hinge or slide joint.
struct JointReport {
  Id joint = kNoId;
  bool angular = true;
  bool actuated = false;
  bool unbounded = false;       // some actuator has no control/force bound
  double capacity = 0;          // max torque (N*m) or force (N) the actuators can apply here
  double liftLoad = 0;          // worst-case gravity load of the moved subtree
  double supportLoad = 0;       // estimated load while supporting the body in locomotion (legs)
  double holdLoad = 0;          // static load supporting the body at the reference pose (legs)
  double movedMass = 0;         // mass of the subtree this joint moves
  double inertia = 0;           // effective inertia about the axis incl. armature
  double stiffness = 0;         // servo + spring stiffness in joint space
  double damping = 0;           // actuator + joint damping in joint space
  double omegaDt = 0;           // natural frequency * timestep (stability: keep < 0.5)
  double sag = 0;               // steady-state deflection of position servos under holdLoad
  double axisTilt = 1;          // |axis x gravity direction|: 0 = vertical axis (gravity can't load it)
  double required() const { return std::max(liftLoad, supportLoad); }
};

struct PairTypeStat {
  GeomType a, b;
  int count = 0;
  bool ccd = false;  // MJWarp uses the general (slower) convex collider for this pair
};

struct AnalysisReport {
  std::vector<Issue> issues;
  std::vector<JointReport> joints;
  int counts[4] = {0, 0, 0, 0};  // per Severity
  double modelMass = 0;           // all bodies with role Model
  int legCount = 0;
  int collisionPairs = 0;         // geom pairs MuJoCo will test (after filtering)
  int selfCollisionPairs = 0;     // ... of which inside the same model
  int broadphaseAtRest = 0;       // MJWarp broadphase candidates at the reference pose
  std::vector<PairTypeStat> pairTypes;
  const JointReport* joint(Id id) const {
    for (const JointReport& j : joints) if (j.joint == id) return &j;
    return nullptr;
  }
};

// `compiled` may be null or stale; it is only used for start-state checks when `compiledValid`.
AnalysisReport analyzeModel(const Document& doc, const CompileResult* compiled, bool compiledValid);

// Maximum force an actuator can produce (in actuator space, before gear). Infinity if unbounded.
double actuatorMaxForce(const Document& doc, const Actuator& a);

// Translates a MuJoCo compile error into a friendlier explanation (empty if unknown).
std::string explainCompileError(const std::string& error);

// Whether MuJoCo will test these two geoms for contact (bodies, filtering, masks, excludes).
bool geomPairCollides(const Document& doc, const Geom& g1, const Geom& g2);

}  // namespace mjf
