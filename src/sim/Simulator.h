// Interactive simulation preview built on the MuJoCo C library.
//
// The simulator owns a private copy of the compiled model, so it can be swapped for a recompiled
// one while running (live parameter tuning) and its options can be changed without affecting the
// validator's copy.
#pragma once

#include <deque>
#include <string>
#include <vector>

#include "core/Math.h"
#include "core/MjcfWriter.h"
#include "sim/MujocoBridge.h"

namespace mjf {

enum class CtrlMode {
  Hold,    // keep the reference pose: position servos target it, torque motors use a built-in PD
  Manual,  // per-actuator sliders
  Sine,    // sine sweep through each actuator's range
  Limp,    // actuation disabled
};

struct SineTest {
  double frequency = 0.5;  // Hz
  double amplitude = 0.5;  // fraction of each actuator's half range
  int pattern = 1;         // 0 all in phase, 1 alternating, 2 traveling wave
};

// MuJoCo Warp buffer sizing measured during the preview.
struct WarpSizing {
  int peakContacts = 0;
  int peakConstraints = 0;
  int peakBroadphase = 0;  // MJWarp broadphase candidate pairs
  int recommendedNconmax() const;
  int recommendedNjmax() const;
};

struct SimSample {
  double t;
  std::vector<float> length;  // per actuator (joint position x gear for joint transmissions)
  std::vector<float> target;  // per actuator: position the drive aims for (NaN if none)
  std::vector<float> ctrl;    // per actuator
  std::vector<float> force;   // per actuator
};

struct SimContact {
  Vec3 pos, normal;  // normal points from geom1 to geom2
  double normalForce = 0;
  double dist = 0;
  int geom1 = -1, geom2 = -1;
};

// How an actuator is driven in the Hold and Sine modes.
struct ActuatorDrive {
  enum Kind { Position, Motor, Other };
  Kind kind = Other;
  double refLength = 0;     // actuator length at the reference pose
  double holdCtrl = 0;      // control that holds the reference pose (or keeps still)
  double halfRange = 0;     // half of the motion range used by the sine test (actuator length units)
  double kp = 0, kd = 0;    // built-in PD for torque motors (actuator space)
  double gain = 1;          // force per unit control (motors)
  double lo = 0, hi = 0;    // control range (lo >= hi: unlimited)
};

class Simulator {
 public:
  // Starts from a compiled model (a private copy is made). Returns false if invalid.
  bool start(const ModelPtr& model, const ExportMap& map);
  void stop();
  bool active() const { return model_ != nullptr; }
  // Swaps in a recompiled model. When the structure is unchanged the state (pose, velocities,
  // time) is kept, so parameters can be tuned while the simulation runs; otherwise it restarts
  // from the reference pose. Returns true if the state was kept.
  bool update(const ModelPtr& model, const ExportMap& map);

  void reset();
  void setPaused(bool p) { paused_ = p; }
  bool paused() const { return paused_; }
  void stepOnce();
  // Advances by wall-clock time (scaled by speed) within a compute budget.
  void advance(double wallDt, double budgetSeconds = 0.012);

  // Perturbation: spring pulling a point of a body (local coordinates) toward a world target.
  void setPerturb(int body, const Vec3& localPoint, const Vec3& target);
  void setPerturbTarget(const Vec3& target) { perturbTarget_ = target; }
  void clearPerturb() { perturbBody_ = -1; }
  int perturbBody() const { return perturbBody_; }
  Vec3 perturbPointWorld() const;
  Vec3 perturbTarget() const { return perturbTarget_; }

  const mjModel* model() const { return model_.get(); }
  const mjData* data() const { return data_.get(); }
  const ExportMap& map() const { return map_; }

  CtrlMode mode = CtrlMode::Hold;
  SineTest sine;
  std::vector<double> manual;  // per actuator (Manual mode)
  double speed = 1.0;

  // Status
  double realtimeFactor() const { return rtf_; }
  bool diverged() const { return diverged_; }
  const std::string& divergeReason() const { return divergeReason_; }
  double divergeTime() const { return divergeTime_; }
  // Compiled joint where the instability was detected (-1 if unknown).
  int divergeJoint() const { return divergeJoint_; }
  const WarpSizing& sizing() const { return sizing_; }
  double simTime() const { return data_ ? data_->time : 0.0; }
  const std::deque<SimSample>& history() const { return history_; }
  // Actuators whose force is at the force limit.
  bool saturated(int actuator) const;
  const ActuatorDrive& drive(int actuator) const { return drives_[actuator]; }
  // Actuator length the current control aims for (position servos, PD-driven motors), else NaN.
  double target(int actuator) const { return targets_[actuator]; }
  // Whether a body moves during simulation (it is not welded to the world).
  bool bodyMoves(int body) const;
  void contacts(std::vector<SimContact>& out) const;
  // Index of the compiled element mapped to a document ID (-1 if absent).
  int bodyIndex(Id id) const;
  int jointIndex(Id id) const;
  int actuatorIndex(Id id) const;

 private:
  void prepare();
  void computeControls();
  double pdControl(int i, double targetLength) const;
  void applyPerturb();
  void afterStep(double tBefore);
  void buildPairList();
  int countBroadphase() const;

  ModelPtr model_;
  DataPtr data_;
  ExportMap map_;
  int baseDisableFlags_ = 0;
  bool paused_ = false;
  double accum_ = 0;
  double rtf_ = 0;
  bool diverged_ = false;
  std::string divergeReason_;
  double divergeTime_ = 0;
  int divergeJoint_ = -1;
  WarpSizing sizing_;
  std::deque<SimSample> history_;
  double lastSampleT_ = -1;
  int stepCounter_ = 0;
  std::vector<ActuatorDrive> drives_;
  std::vector<double> targets_;
  std::vector<std::pair<int, int>> pairs_;  // geom pairs passing MuJoCo's static filters
  int perturbBody_ = -1;
  Vec3 perturbLocal_, perturbTarget_;
};

}  // namespace mjf
