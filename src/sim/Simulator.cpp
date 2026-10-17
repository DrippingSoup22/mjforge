#include "sim/Simulator.h"

#include <algorithm>
#include <chrono>
#include <cmath>

namespace mjf {

namespace {

int niceSize(double n) {
  // 16, 24, 32, 48, 64, 96, 128, ... like MJWarp's default size ladder.
  int size = 16;
  while (size < n) size = (size & (size - 1)) == 0 ? size * 3 / 2 : (size / 3) * 4;
  return size;
}

bool positionLike(const mjModel* m, int i) {
  return m->actuator_biastype[i] == mjBIAS_AFFINE && m->actuator_biasprm[i * mjNBIAS + 1] < 0 &&
         m->actuator_dyntype[i] != mjDYN_INTEGRATOR;
}

bool motorLike(const mjModel* m, int i) {
  return m->actuator_gaintype[i] == mjGAIN_FIXED && m->actuator_biastype[i] == mjBIAS_NONE &&
         m->actuator_dyntype[i] != mjDYN_INTEGRATOR && m->actuator_gainprm[i * mjNGAIN] != 0;
}

// Inertia an actuator sees along its own direction: 1 / (J M^-1 J^T) with J its moment row (which
// includes the gear). Everything is free to move, so this is the smallest, stability-relevant one.
double actuatorSpaceInertia(const mjModel* m, mjData* d, int i) {
  std::vector<mjtNum> J(m->nv, 0.0), x(m->nv, 0.0);
  int adr = d->moment_rowadr[i], nnz = d->moment_rownnz[i];
  for (int k = 0; k < nnz; ++k) J[d->moment_colind[adr + k]] = d->actuator_moment[adr + k];
  mj_solveM(m, d, x.data(), J.data(), 1);
  double s = 0;
  for (int k = 0; k < m->nv; ++k) s += J[k] * x[k];
  return s > 1e-12 ? 1.0 / s : 0.0;
}

// Smallest inertia a joint's actuator can see about its axis: the moved subtree swinging freely,
// or the rest of the robot rotating about a planted limb (stance). The armature always adds.
// Sizing a PD on the swing inertia alone makes it chatter in stance when the limb is heavy.
double effectiveJointInertia(const mjModel* m, const mjData* ref, int j) {
  int dof = m->jnt_dofadr[j], b = m->jnt_bodyid[j];
  double armature = m->dof_armature[dof];
  double distal = std::max(0.0, m->dof_M0[dof] - armature);
  int root = m->body_rootid[b];
  if (m->body_weldid[root] == 0) return distal + armature;  // fixed base: the rest can't move
  bool angular = m->jnt_type[j] == mjJNT_HINGE;
  const mjtNum* a = ref->xaxis + 3 * j;
  const mjtNum* p = ref->xanchor + 3 * j;
  double proximal = 0;
  for (int k = 1; k < m->nbody; ++k) {
    if (m->body_rootid[k] != root) continue;
    bool below = false;
    for (int x = k; x > 0; x = m->body_parentid[x])
      if (x == b) below = true;
    if (below) continue;
    double mass = m->body_mass[k];
    if (!angular) {
      proximal += mass;
      continue;
    }
    const mjtNum* R = ref->ximat + 9 * k;
    const mjtNum* I = m->body_inertia + 3 * k;
    double v[3] = {R[0] * a[0] + R[3] * a[1] + R[6] * a[2], R[1] * a[0] + R[4] * a[1] + R[7] * a[2],
                   R[2] * a[0] + R[5] * a[1] + R[8] * a[2]};
    const mjtNum* c = ref->xipos + 3 * k;
    double r[3] = {c[0] - p[0], c[1] - p[1], c[2] - p[2]};
    double ra = r[0] * a[0] + r[1] * a[1] + r[2] * a[2];
    double d2 = r[0] * r[0] + r[1] * r[1] + r[2] * r[2] - ra * ra;
    proximal += I[0] * v[0] * v[0] + I[1] * v[1] * v[1] + I[2] * v[2] * v[2] + mass * d2;
  }
  return std::min(distal, proximal) + armature;
}

int indexOf(const std::vector<Id>& v, Id id) {
  for (size_t i = 0; i < v.size(); ++i)
    if (v[i] == id) return int(i);
  return -1;
}

}  // namespace

int WarpSizing::recommendedNconmax() const {
  return niceSize(std::max({double(peakBroadphase), peakContacts * 1.5, 8.0}));
}

int WarpSizing::recommendedNjmax() const { return niceSize(std::max(peakConstraints * 1.5, 16.0)); }

bool Simulator::start(const ModelPtr& src, const ExportMap& map) {
  if (!src) return false;
  try {
    model_ = makeModelPtr(mj_copyModel(nullptr, src.get()));
    data_ = makeData(model_.get());
  } catch (const std::exception&) {
    data_.reset();
    model_.reset();
  }
  if (!model_ || !data_) {
    stop();
    return false;
  }
  map_ = map;
  baseDisableFlags_ = model_->opt.disableflags;
  buildPairList();
  prepare();
  reset();
  return true;
}

void Simulator::stop() {
  data_.reset();
  model_.reset();
  history_.clear();
  pairs_.clear();
  drives_.clear();
  perturbBody_ = -1;
  diverged_ = false;
  rtf_ = 0;
}

bool Simulator::update(const ModelPtr& src, const ExportMap& map) {
  if (!src) return false;
  if (!model_) {
    start(src, map);
    return false;
  }
  const mjModel* o = model_.get();
  const mjModel* n = src.get();
  bool same = o->nq == n->nq && o->nv == n->nv && o->na == n->na && o->nu == n->nu && o->nbody == n->nbody &&
              o->njnt == n->njnt && o->nmocap == n->nmocap;
  for (int j = 0; same && j < n->njnt; ++j) same = o->jnt_type[j] == n->jnt_type[j];
  if (!same) {
    bool p = paused_;
    start(src, map);
    paused_ = p;
    return false;
  }
  ModelPtr nm;
  DataPtr d;
  try {
    nm = makeModelPtr(mj_copyModel(nullptr, n));
    d = makeData(nm.get());
  } catch (const std::exception&) {
    return false;
  }
  if (!nm || !d) return false;
  // Keep the dynamic state; mocap bodies take their (possibly edited) positions from the new model.
  const mjData* od = data_.get();
  mju_copy(d->qpos, od->qpos, n->nq);
  mju_copy(d->qvel, od->qvel, n->nv);
  mju_copy(d->act, od->act, n->na);
  mju_copy(d->ctrl, od->ctrl, n->nu);
  mju_copy(d->qacc_warmstart, od->qacc_warmstart, n->nv);
  d->time = od->time;
  model_ = std::move(nm);
  data_ = std::move(d);
  map_ = map;
  baseDisableFlags_ = model_->opt.disableflags;
  try {
    mj_forward(model_.get(), data_.get());
  } catch (const std::exception&) {
  }
  buildPairList();
  prepare();
  return true;
}

void Simulator::prepare() {
  const mjModel* m = model_.get();
  DataPtr ref = makeData(m);
  if (ref) {
    mj_resetData(m, ref.get());
    if (m->nkey > 0) mj_resetDataKeyframe(m, ref.get(), 0);
    try {
      mj_forward(m, ref.get());
    } catch (const std::exception&) {
    }
  }
  const double dt = m->opt.timestep;
  drives_.assign(m->nu, {});
  targets_.assign(m->nu, std::nan(""));
  for (int i = 0; i < m->nu; ++i) {
    ActuatorDrive& a = drives_[i];
    a.refLength = ref ? ref->actuator_length[i] : 0;
    if (m->actuator_ctrllimited[i]) {
      a.lo = m->actuator_ctrlrange[2 * i];
      a.hi = m->actuator_ctrlrange[2 * i + 1];
    }
    bool lim = a.hi > a.lo;
    double gear = m->actuator_gear[6 * i];
    int trn = m->actuator_trntype[i], j = m->actuator_trnid[2 * i];
    bool jointTrn = (trn == mjTRN_JOINT || trn == mjTRN_JOINTINPARENT) && j >= 0 &&
                    (m->jnt_type[j] == mjJNT_HINGE || m->jnt_type[j] == mjJNT_SLIDE);
    // Motion range for the sine test, in actuator length units.
    double half = 0;
    if (jointTrn && m->jnt_limited[j]) half = 0.5 * std::fabs(gear) * (m->jnt_range[2 * j + 1] - m->jnt_range[2 * j]);
    else if (jointTrn) half = std::fabs(gear) * (m->jnt_type[j] == mjJNT_HINGE ? 0.5 : 0.05);
    if (positionLike(m, i)) {
      a.kind = ActuatorDrive::Position;
      a.holdCtrl = a.refLength;
      if (lim) half = half > 0 ? std::min(half, 0.5 * (a.hi - a.lo)) : 0.5 * (a.hi - a.lo);
    } else if (motorLike(m, i) && jointTrn && gear != 0 && ref &&
               effectiveJointInertia(m, ref.get(), j) > 0) {
      // Torque motor: hold / track positions with the stiffest PD that stays stable when the
      // control is computed explicitly once per step (omega * dt = 0.15, damping ratio 0.8).
      double inertia = effectiveJointInertia(m, ref.get(), j) / (gear * gear);
      double w = 0.15 / dt;
      a.kind = ActuatorDrive::Motor;
      a.gain = m->actuator_gainprm[i * mjNGAIN];
      a.kp = inertia * w * w;
      a.kd = 2.0 * 0.8 * inertia * w;
      // Keep PD targets a little inside the joint limits: a stiff PD pushing against a limit
      // constraint chatters (e.g. a leg whose reference pose is fully stretched).
      if (m->jnt_limited[j]) {
        double lo = m->jnt_range[2 * j], hi = m->jnt_range[2 * j + 1];
        double margin = std::min(0.05, 0.1 * (hi - lo));
        double l1 = gear * (lo + margin), l2 = gear * (hi - margin);
        a.targetLo = std::min(l1, l2);
        a.targetHi = std::max(l1, l2);
        a.refLength = clampv(a.refLength, a.targetLo, a.targetHi);
      }
    } else if (motorLike(m, i) && trn == mjTRN_TENDON && gear != 0 && ref && m->nv > 0 &&
               actuatorSpaceInertia(m, ref.get(), i) > 0) {
      // Torque motor on a tendon: the same PD, in tendon length units.
      double inertia = actuatorSpaceInertia(m, ref.get(), i);
      double w = 0.15 / dt;
      a.kind = ActuatorDrive::Motor;
      a.gain = m->actuator_gainprm[i * mjNGAIN];
      a.kp = inertia * w * w;
      a.kd = 2.0 * 0.8 * inertia * w;
      int t = m->actuator_trnid[2 * i];
      if (t >= 0 && m->tendon_limited[t]) {
        double lo = m->tendon_range[2 * t], hi = m->tendon_range[2 * t + 1];
        double margin = 0.1 * (hi - lo);
        double l1 = gear * (lo + margin), l2 = gear * (hi - margin);
        a.targetLo = std::min(l1, l2);
        a.targetHi = std::max(l1, l2);
        a.refLength = clampv(a.refLength, a.targetLo, a.targetHi);
        half = 0.4 * std::fabs(gear) * (hi - lo);  // sine test amplitude, in length units
      } else {
        half = 0.02 * std::fabs(gear);
      }
    } else if (lim) {
      half = 0.5 * (a.hi - a.lo);
    }
    if (half <= 0) half = lim ? 0.5 * (a.hi - a.lo) : 1.0;
    a.halfRange = half;
    if (lim) a.holdCtrl = clampv(a.holdCtrl, a.lo, a.hi);
  }
  if (manual.size() != size_t(m->nu)) {
    manual.assign(m->nu, 0.0);
    for (int i = 0; i < m->nu; ++i) manual[i] = drives_[i].holdCtrl;
  }
}

void Simulator::reset() {
  if (!model_) return;
  mjModel* m = model_.get();
  mjData* d = data_.get();
  m->opt.disableflags = baseDisableFlags_;
  mj_resetData(m, d);
  if (m->nkey > 0) mj_resetDataKeyframe(m, d, 0);
  try {
    mj_forward(m, d);
  } catch (const std::exception&) {
  }
  history_.clear();
  lastSampleT_ = -1;
  accum_ = 0;
  rtf_ = 0;
  diverged_ = false;
  divergeReason_.clear();
  divergeTime_ = 0;
  divergeJoint_ = -1;
  sizing_ = {};
  perturbBody_ = -1;
  for (int w = 0; w < mjNWARNING; ++w) d->warning[w].number = 0;
}

double Simulator::pdControl(int i, double target) const {
  const ActuatorDrive& a = drives_[i];
  const mjData* d = data_.get();
  target = clampv(target, a.targetLo, a.targetHi);
  double f = a.kp * (target - d->actuator_length[i]) - a.kd * d->actuator_velocity[i];
  return f / a.gain;
}

void Simulator::computeControls() {
  mjModel* m = model_.get();
  mjData* d = data_.get();
  m->opt.disableflags = mode == CtrlMode::Limp ? (baseDisableFlags_ | mjDSBL_ACTUATION) : baseDisableFlags_;
  const double none = std::nan("");
  for (int i = 0; i < m->nu; ++i) {
    const ActuatorDrive& a = drives_[i];
    bool position = a.kind == ActuatorDrive::Position, motor = a.kind == ActuatorDrive::Motor;
    double u = 0, target = none;
    switch (mode) {
      case CtrlMode::Limp: break;
      case CtrlMode::Manual: u = size_t(i) < manual.size() ? manual[i] : 0; break;
      case CtrlMode::Hold:
        if (motor) u = pdControl(i, target = a.refLength);
        else u = a.holdCtrl;
        break;
      case CtrlMode::Sine: {
        double phase = 0;
        if (sine.pattern == 1) phase = (i % 2) * kPi;
        else if (sine.pattern == 2) phase = 2 * kPi * i / std::max(1, int(m->nu));
        double s = sine.amplitude * a.halfRange * std::sin(2 * kPi * sine.frequency * d->time + phase);
        if (motor) u = pdControl(i, target = a.refLength + s);
        else if (position) u = a.holdCtrl + s;
        else u = (a.hi > a.lo ? 0.5 * (a.lo + a.hi) : 0.0) + s;
        break;
      }
    }
    if (a.hi > a.lo) u = clampv(u, a.lo, a.hi);
    if (position && mode != CtrlMode::Limp) target = u;
    d->ctrl[i] = u;
    targets_[i] = target;
  }
}

Vec3 Simulator::perturbPointWorld() const {
  if (perturbBody_ < 0 || !data_) return {};
  const mjData* d = data_.get();
  const double* x = d->xpos + 3 * perturbBody_;
  const double* R = d->xmat + 9 * perturbBody_;
  const Vec3& l = perturbLocal_;
  return {x[0] + R[0] * l.x + R[1] * l.y + R[2] * l.z, x[1] + R[3] * l.x + R[4] * l.y + R[5] * l.z,
          x[2] + R[6] * l.x + R[7] * l.y + R[8] * l.z};
}

void Simulator::setPerturb(int body, const Vec3& localPoint, const Vec3& target) {
  perturbBody_ = body;
  perturbLocal_ = localPoint;
  perturbTarget_ = target;
}

void Simulator::applyPerturb() {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  mju_zero(d->qfrc_applied, m->nv);
  if (perturbBody_ <= 0 || perturbBody_ >= m->nbody) return;
  // Spring-damper like MuJoCo's own mouse perturbation: stiffness scales with the dragged subtree.
  int b = perturbBody_;
  double mass = std::max(1e-6, m->body_subtreemass[b]);
  double k = mass * std::max(1.0, double(m->vis.map.stiffness));
  double c = 2.0 * 0.7 * std::sqrt(k * mass);
  Vec3 p = perturbPointWorld();
  double vel[6];
  mj_objectVelocity(m, d, mjOBJ_XBODY, b, vel, 0);
  Vec3 w{vel[0], vel[1], vel[2]}, v{vel[3], vel[4], vel[5]};
  Vec3 xb{d->xpos[3 * b], d->xpos[3 * b + 1], d->xpos[3 * b + 2]};
  Vec3 vp = v + cross(w, p - xb);
  Vec3 f = (perturbTarget_ - p) * k - vp * c;
  double g = mju_norm3(m->opt.gravity);
  if (g <= 0) g = 9.81;
  double fmax = (5.0 * m->body_subtreemass[m->body_rootid[b]] + 10.0 * mass) * g;
  double fl = length(f);
  if (fl > fmax) f = f * (fmax / fl);
  double force[3] = {f.x, f.y, f.z}, torque[3] = {0, 0, 0}, point[3] = {p.x, p.y, p.z};
  mj_applyFT(m, d, force, torque, point, b, d->qfrc_applied);
}

void Simulator::stepOnce() {
  if (!model_ || diverged_) return;
  double t = data_->time;
  computeControls();
  applyPerturb();
  try {
    mj_step(model_.get(), data_.get());
  } catch (const std::exception& e) {
    diverged_ = true;
    divergeReason_ = e.what();
    divergeTime_ = t;
    paused_ = true;
    return;
  }
  afterStep(t);
}

void Simulator::advance(double wallDt, double budget) {
  if (!model_ || paused_ || diverged_) {
    rtf_ = 0;
    return;
  }
  const double dt = model_->opt.timestep;
  wallDt = std::min(wallDt, 0.05);  // after a stall, slow down instead of jumping ahead
  accum_ += wallDt * speed;
  auto t0 = std::chrono::steady_clock::now();
  double simulated = 0;
  while (accum_ >= dt) {
    stepOnce();
    accum_ -= dt;
    simulated += dt;
    if (diverged_ || paused_) break;
    double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    if (elapsed > budget) {
      accum_ = 0;  // can't keep up: run slower than real time
      break;
    }
  }
  double inst = wallDt > 0 ? simulated / wallDt : 0;
  rtf_ = rtf_ <= 0 ? inst : 0.9 * rtf_ + 0.1 * inst;
}

bool Simulator::saturated(int i) const {
  if (!model_ || i < 0 || i >= model_->nu || !model_->actuator_forcelimited[i]) return false;
  if (mode == CtrlMode::Limp) return false;
  double lo = model_->actuator_forcerange[2 * i], hi = model_->actuator_forcerange[2 * i + 1];
  double f = data_->actuator_force[i];
  return (hi > 0 && f >= 0.995 * hi) || (lo < 0 && f <= 0.995 * lo);
}

bool Simulator::bodyMoves(int b) const {
  if (!model_ || b <= 0 || b >= model_->nbody) return false;
  int w = model_->body_weldid[b];
  return w != 0 && model_->body_mocapid[w] < 0;
}

int Simulator::bodyIndex(Id id) const { return indexOf(map_.bodies, id); }
int Simulator::jointIndex(Id id) const { return indexOf(map_.joints, id); }
int Simulator::actuatorIndex(Id id) const { return indexOf(map_.actuators, id); }

void Simulator::contacts(std::vector<SimContact>& out) const {
  out.clear();
  if (!model_) return;
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  for (int i = 0; i < d->ncon; ++i) {
    const mjContact& c = d->contact[i];
    SimContact s;
    s.pos = {c.pos[0], c.pos[1], c.pos[2]};
    s.normal = {c.frame[0], c.frame[1], c.frame[2]};
    s.dist = c.dist;
    s.geom1 = c.geom[0];
    s.geom2 = c.geom[1];
    if (c.efc_address >= 0) {
      mjtNum f[6];
      mj_contactForce(m, d, i, f);
      s.normalForce = f[0];
    }
    out.push_back(s);
  }
}

void Simulator::afterStep(double tBefore) {
  const mjModel* m = model_.get();
  mjData* d = data_.get();
  int bad = -1;
  for (int i = 0; i < m->nv && bad < 0; ++i)
    if (!std::isfinite(d->qvel[i]) || !std::isfinite(d->qacc[i])) bad = i;
  int warn = -1;
  for (int w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS})
    if (d->warning[w].number > 0) warn = w;
  if (bad >= 0 || warn >= 0) {
    diverged_ = true;
    paused_ = true;
    divergeTime_ = tBefore;
    int dof = warn >= 0 ? d->warning[warn].lastinfo : bad;
    divergeJoint_ = dof >= 0 && dof < m->nv ? m->dof_jntid[dof] : -1;
    divergeReason_ = warn == mjWARN_BADQPOS   ? "the positions became invalid"
                     : warn == mjWARN_BADQVEL ? "the velocities exploded"
                                              : "the accelerations exploded";
    for (int w = 0; w < mjNWARNING; ++w) d->warning[w].number = 0;
    return;
  }
  sizing_.peakContacts = std::max(sizing_.peakContacts, int(d->ncon));
  sizing_.peakConstraints = std::max(sizing_.peakConstraints, int(d->nefc));
  if (pairs_.size() < 4000 || (++stepCounter_ % 8) == 0)
    sizing_.peakBroadphase = std::max(sizing_.peakBroadphase, countBroadphase());

  // History for plots, ~100 samples per simulated second, last 10 s.
  if (lastSampleT_ < 0 || d->time - lastSampleT_ >= 0.01 - 1e-9) {
    lastSampleT_ = d->time;
    SimSample s;
    s.t = d->time;
    s.length.resize(m->nu);
    s.target.resize(m->nu);
    s.ctrl.resize(m->nu);
    s.force.resize(m->nu);
    for (int i = 0; i < m->nu; ++i) {
      s.length[i] = float(d->actuator_length[i]);
      s.target[i] = float(targets_[i]);
      s.ctrl[i] = float(d->ctrl[i]);
      s.force[i] = float(d->actuator_force[i]);
    }
    history_.push_back(std::move(s));
    while (!history_.empty() && history_.front().t < d->time - 10.0) history_.pop_front();
  }
}

void Simulator::buildPairList() {
  pairs_.clear();
  const mjModel* m = model_.get();
  bool filterParent = !(m->opt.disableflags & mjDSBL_FILTERPARENT);
  auto excluded = [m](int b1, int b2) {
    int s1 = (b1 << 16) + b2, s2 = (b2 << 16) + b1;
    for (int i = 0; i < m->nexclude; ++i)
      if (m->exclude_signature[i] == s1 || m->exclude_signature[i] == s2) return true;
    return false;
  };
  for (int g1 = 0; g1 < m->ngeom; ++g1) {
    for (int g2 = g1 + 1; g2 < m->ngeom; ++g2) {
      if (!((m->geom_contype[g1] & m->geom_conaffinity[g2]) || (m->geom_contype[g2] & m->geom_conaffinity[g1]))) continue;
      int b1 = m->geom_bodyid[g1], b2 = m->geom_bodyid[g2];
      int w1 = m->body_weldid[b1], w2 = m->body_weldid[b2];
      if (w1 == w2) continue;
      if (m->body_dofnum[w1] == 0 && m->body_dofnum[w2] == 0) continue;
      int wp1 = m->body_weldid[m->body_parentid[w1]], wp2 = m->body_weldid[m->body_parentid[w2]];
      if (filterParent && w1 != 0 && w2 != 0 && (w1 == wp2 || w2 == wp1)) continue;
      if (excluded(b1, b2)) continue;
      pairs_.push_back({g1, g2});
    }
  }
}

int Simulator::countBroadphase() const {
  const mjModel* m = model_.get();
  const mjData* d = data_.get();
  int n = 0;
  for (const auto& [g1, g2] : pairs_) {
    double rb1 = m->geom_rbound[g1], rb2 = m->geom_rbound[g2];
    double margin = m->geom_margin[g1] + m->geom_gap[g1] + m->geom_margin[g2] + m->geom_gap[g2];
    const double* p1 = d->geom_xpos + 3 * g1;
    const double* p2 = d->geom_xpos + 3 * g2;
    const double* R1 = d->geom_xmat + 9 * g1;
    const double* R2 = d->geom_xmat + 9 * g2;
    if (rb1 == 0 || rb2 == 0) {
      // Plane: signed distance of the other geom's center along the plane normal.
      const double* pp = rb1 == 0 ? p1 : p2;
      const double* Rp = rb1 == 0 ? R1 : R2;
      const double* po = rb1 == 0 ? p2 : p1;
      double rbo = rb1 == 0 ? rb2 : rb1;
      double dist = (po[0] - pp[0]) * Rp[2] + (po[1] - pp[1]) * Rp[5] + (po[2] - pp[2]) * Rp[8];
      if (dist <= rbo + margin) ++n;
      continue;
    }
    double dx = p2[0] - p1[0], dy = p2[1] - p1[1], dz = p2[2] - p1[2];
    double bound = rb1 + rb2 + margin;
    if (dx * dx + dy * dy + dz * dz > bound * bound) continue;
    // Oriented boxes (face normals only), as in MJWarp's _obb_filter.
    const double* a1 = m->geom_aabb + 6 * g1;
    const double* a2 = m->geom_aabb + 6 * g2;
    double c1[3], c2[3];
    for (int r = 0; r < 3; ++r) {
      c1[r] = p1[r] + R1[3 * r] * a1[0] + R1[3 * r + 1] * a1[1] + R1[3 * r + 2] * a1[2];
      c2[r] = p2[r] + R2[3 * r] * a2[0] + R2[3 * r + 1] * a2[1] + R2[3 * r + 2] * a2[2];
    }
    auto axis = [&](int box, int k, double out[3]) {
      const double* R = box == 0 ? R1 : R2;
      out[0] = R[k];
      out[1] = R[3 + k];
      out[2] = R[6 + k];
    };
    bool separated = false;
    for (int j = 0; j < 2 && !separated; ++j) {
      for (int k = 0; k < 3 && !separated; ++k) {
        double ax[3];
        axis(j, k, ax);
        double proj0 = c1[0] * ax[0] + c1[1] * ax[1] + c1[2] * ax[2];
        double proj1 = c2[0] * ax[0] + c2[1] * ax[1] + c2[2] * ax[2];
        double rad = 0;
        for (int i = 0; i < 2; ++i) {
          const double* half = i == 0 ? a1 + 3 : a2 + 3;
          for (int q = 0; q < 3; ++q) {
            double nq[3];
            axis(i, q, nq);
            rad += std::fabs(half[q] * (nq[0] * ax[0] + nq[1] * ax[1] + nq[2] * ax[2]));
          }
        }
        if (rad + margin < std::fabs(proj1 - proj0)) separated = true;
      }
    }
    if (!separated) ++n;
  }
  return n;
}

}  // namespace mjf
