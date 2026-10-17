#include "analysis/Analysis.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <filesystem>
#include <functional>
#include <map>
#include <set>
#include <unordered_map>
#include <unordered_set>

#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Ops.h"

namespace mjf {

const char* severityName(Severity s) {
  switch (s) {
    case Severity::Error: return "Error";
    case Severity::Warning: return "Warning";
    case Severity::Info: return "Info";
    case Severity::Tip: return "Tip";
  }
  return "";
}

const char* areaName(IssueArea a) {
  switch (a) {
    case IssueArea::Model: return "Model";
    case IssueArea::Physics: return "Physics";
    case IssueArea::Stability: return "Stability";
    case IssueArea::Start: return "Start pose";
    case IssueArea::MJWarp: return "MuJoCo Warp";
    case IssueArea::Performance: return "Performance";
  }
  return "";
}

namespace {

constexpr double kInf = std::numeric_limits<double>::infinity();

std::string fmt(const char* f, ...) {
  char buf[1024];
  va_list ap;
  va_start(ap, f);
  std::vsnprintf(buf, sizeof buf, f, ap);
  va_end(ap);
  return buf;
}

// 1, 2, 2.5, 5 x 10^k at or above v.
double niceCeil(double v) {
  if (v <= 0) return 0;
  double e = std::pow(10.0, std::floor(std::log10(v)));
  for (double m : {1.0, 2.0, 2.5, 5.0, 10.0})
    if (m * e >= v * (1 - 1e-9)) return m * e;
  return 10 * e;
}

std::string quote(const std::string& s) { return "'" + (s.empty() ? std::string("(unnamed)") : s) + "'"; }

struct Weld {
  std::vector<int> weld, weldParent, dof;
};

Weld computeWeld(const Document& doc) {
  const DocIndex& ix = doc.index();
  size_t n = doc.bodies.size();
  Weld w;
  w.weld.assign(n, 0);
  w.weldParent.assign(n, 0);
  w.dof.assign(n, 0);
  for (uint32_t bi : ix.dfsOrder) {
    if (bi == 0) continue;
    const Body& b = doc.bodies[bi];
    w.dof[bi] = doc.bodyDofs(b.id);
    int pi = doc.bodyIndex(b.parent);
    if (pi < 0) pi = 0;
    w.weld[bi] = (w.dof[bi] > 0 || b.mocap) ? int(bi) : w.weld[pi];
  }
  for (size_t bi = 1; bi < n; ++bi) {
    int wr = w.weld[bi];
    if (wr == 0) continue;
    int pi = doc.bodyIndex(doc.bodies[wr].parent);
    w.weldParent[bi] = pi < 0 ? 0 : w.weld[pi];
  }
  return w;
}

using ExclSet = std::set<std::pair<Id, Id>>;
ExclSet excludeSet(const Document& doc) {
  ExclSet s;
  for (const Exclude& e : doc.excludes) {
    s.insert({e.body1, e.body2});
    s.insert({e.body2, e.body1});
  }
  return s;
}

bool pairPasses(const Document& doc, const Weld& w, const ExclSet& excl, const Geom& g1, int b1, const Geom& g2, int b2) {
  if (b1 == b2) return false;
  int w1 = w.weld[b1], w2 = w.weld[b2];
  if (w1 == w2) return false;
  if (w.dof[w1] == 0 && w.dof[w2] == 0) return false;
  if (doc.option.filterparent && w1 != 0 && w2 != 0 && (w1 == w.weldParent[w2] || w2 == w.weldParent[w1])) return false;
  if (!((g1.contype & g2.conaffinity) || (g2.contype & g1.conaffinity))) return false;
  if (excl.count({doc.bodies[b1].id, doc.bodies[b2].id})) return false;
  return true;
}

bool isCcdPair(GeomType a, GeomType b) {
  if (int(a) > int(b)) std::swap(a, b);
  switch (a) {
    case GeomType::Plane: return false;
    case GeomType::Hfield: return true;
    case GeomType::Sphere:
      return !(b == GeomType::Sphere || b == GeomType::Capsule || b == GeomType::Cylinder || b == GeomType::Box);
    case GeomType::Capsule: return !(b == GeomType::Capsule || b == GeomType::Box);
    default: return true;
  }
}

double rbound(const Document& doc, const Geom& g) {
  const Vec3& s = g.size;
  switch (g.type) {
    case GeomType::Sphere: return s.x;
    case GeomType::Capsule: return s.x + s.y;
    case GeomType::Cylinder: return std::sqrt(s.x * s.x + s.y * s.y);
    case GeomType::Box: return length(s);
    case GeomType::Ellipsoid: return std::max({s.x, s.y, s.z});
    case GeomType::Hfield: {
      const Hfield* h = doc.hfield(g.hfield);
      return h ? std::sqrt(h->size.x * h->size.x + h->size.y * h->size.y + h->size.z * h->size.z) : 1.0;
    }
    case GeomType::Mesh: {
      double r = 0;
      if (const MeshAsset* m = doc.mesh(g.mesh))
        for (const Vec3& p : meshSupportPoints(*m)) r = std::max(r, length(p));
      return std::max(r, 1e-6);
    }
    default: return 0;  // planes
  }
}

// 2D convex hull (Andrew's monotone chain), counter-clockwise.
std::vector<Vec2> hull2d(std::vector<Vec2> p) {
  std::sort(p.begin(), p.end(), [](const Vec2& a, const Vec2& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); });
  if (p.size() < 3) return p;
  auto crossz = [](const Vec2& o, const Vec2& a, const Vec2& b) {
    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
  };
  std::vector<Vec2> h(2 * p.size());
  size_t k = 0;
  for (size_t i = 0; i < p.size(); ++i) {
    while (k >= 2 && crossz(h[k - 2], h[k - 1], p[i]) <= 0) --k;
    h[k++] = p[i];
  }
  for (size_t i = p.size() - 1, t = k + 1; i > 0; --i) {
    while (k >= t && crossz(h[k - 2], h[k - 1], p[i - 1]) <= 0) --k;
    h[k++] = p[i - 1];
  }
  h.resize(k - 1);
  return h;
}

// Signed distance from a point to a convex CCW polygon (negative inside).
double polygonDistance(const std::vector<Vec2>& poly, const Vec2& q) {
  if (poly.size() < 3) return kInf;
  double worst = -kInf;
  for (size_t i = 0; i < poly.size(); ++i) {
    const Vec2& a = poly[i];
    const Vec2& b = poly[(i + 1) % poly.size()];
    double ex = b.x - a.x, ey = b.y - a.y;
    double len = std::hypot(ex, ey);
    if (len < 1e-12) continue;
    // Outward normal of a CCW polygon edge is (ey, -ex).
    double d = ((q.x - a.x) * ey - (q.y - a.y) * ex) / len;
    worst = std::max(worst, d);
  }
  return worst;
}

// Control range after inheritrange: a servo can take its range from the joint it drives.
Vec2 servoCtrlRange(const Document& doc, const Actuator& a, bool* limited) {
  const Joint* j = (a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) ? doc.joint(a.target) : nullptr;
  if (a.type == ActuatorType::Position && a.inheritrange > 0 && j && j->limited) {
    double mid = 0.5 * (j->range.x + j->range.y), half = 0.5 * (j->range.y - j->range.x) * a.inheritrange;
    *limited = true;
    return {mid - half, mid + half};
  }
  *limited = a.ctrllimited;
  return a.ctrlrange;
}

// Largest position error a position-like servo can see (actuator space): between its farthest
// target and the farthest position the joint can reach, or its target span on an unlimited joint.
// Infinite when nothing bounds the targets.
double servoSpan(const Document& doc, const Actuator& a) {
  const Joint* j = (a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) ? doc.joint(a.target) : nullptr;
  bool limited = false;
  Vec2 t;
  if (a.type == ActuatorType::IntVelocity) {  // the integrated activation is the target
    limited = a.actlimited;
    t = a.actrange;
    if (a.inheritrange > 0 && j && j->limited) {
      double mid = 0.5 * (j->range.x + j->range.y), half = 0.5 * (j->range.y - j->range.x) * a.inheritrange;
      limited = true;
      t = {mid - half, mid + half};
    }
  } else {
    t = servoCtrlRange(doc, a, &limited);
  }
  if (!limited) return kInf;
  if (j && j->limited) {
    double l1 = a.gear[0] * j->range.x, l2 = a.gear[0] * j->range.y;
    return std::max(t.y - std::min(l1, l2), std::max(l1, l2) - t.x);
  }
  return t.y - t.x;
}

// A General actuator wired as a position servo (force = kp * (ctrl - length) ...).
bool positionLikeGeneral(const Actuator& a) {
  return a.type == ActuatorType::General && a.gaintype == GainType::Fixed && a.biastype == BiasType::Affine &&
         a.biasprm[1] < 0;
}

// Multiplies a servo's stiffness by s, rounded up to a nice value; an explicit kv grows with the
// square root so the damping ratio stays.
void stiffenServo(Document& d, Id id, double s) {
  Actuator* a = d.actuator(id);
  if (!a || !(s > 1)) return;
  if (a->type == ActuatorType::Position || a->type == ActuatorType::IntVelocity) {
    double kp = niceCeil(a->kp * s);
    if (a->dampratio <= 0 && a->kv > 0 && a->kp > 0) a->kv *= std::sqrt(kp / a->kp);
    a->kp = kp;
  } else if (positionLikeGeneral(*a)) {
    double k = niceCeil(-a->biasprm[1] * s), f = k / -a->biasprm[1];
    a->biasprm[1] = -k;
    a->gainprm[0] *= f;
    a->biasprm[2] *= std::sqrt(f);
  }
}

// Label for a list of (actuator, stiffness factor) changes: the new kp when they all agree.
std::string stiffenLabel(const Document& doc, const std::vector<std::pair<Id, double>>& scales) {
  double kp = -1;
  for (const auto& [id, s] : scales) {
    const Actuator* a = doc.actuator(id);
    if (!a || (a->type != ActuatorType::Position && a->type != ActuatorType::IntVelocity)) return "Stiffen the servos";
    double v = niceCeil(a->kp * s);
    if (kp >= 0 && std::fabs(v - kp) > 1e-9 * kp) return "Stiffen the servos";
    kp = v;
  }
  return kp > 0 ? fmt("Set kp to %.3g", kp) : "Stiffen the servos";
}

// Raises the actuators of a joint until together they reach `target` (joint space); each comes
// with its moment arm on the joint. Widens force ranges and the joint's actuator force cap that
// would clamp the result.
void strengthenJoint(Document& d, const std::vector<std::pair<Id, double>>& arms, double target, Id joint) {
  if (arms.empty()) return;
  double share = target / double(arms.size());
  for (const auto& [id, arm] : arms) {
    Actuator* a = d.actuator(id);
    if (!a) continue;
    double f = share / std::max(1e-12, std::fabs(arm));  // actuator-space force it must reach
    double fmax = actuatorMaxForce(d, *a);
    if (!(fmax < f)) continue;
    if (a->type == ActuatorType::Motor) {
      // The gear scales the arm, so it scales the torque at the joint without touching the ranges.
      double g = std::fabs(a->gear[0]) * f / std::max(fmax, 1e-12);
      a->gear[0] = std::copysign(niceCeil(g), a->gear[0] == 0 ? 1.0 : a->gear[0]);
      continue;
    }
    if (a->type == ActuatorType::DcMotor) {
      // A stronger motor of the same speed: the stall torque grows as the resistance falls.
      double s = f / std::max(fmax, 1e-12);
      if (a->resistance > 0) a->resistance /= s;
      else a->nominal.y *= s;
      if (a->saturation.x > 0) a->saturation.x = std::max(a->saturation.x, f);
      if (a->saturation.y > 0) a->saturation.y *= s;
      continue;
    }
    Actuator open = *a;
    open.forcelimited = false;
    double free = actuatorMaxForce(d, open);
    if (free < f) {
      double s = f / std::max(free, 1e-12);
      switch (a->type) {
        case ActuatorType::Position:
        case ActuatorType::IntVelocity: {
          double span = servoSpan(d, *a);
          a->kp = std::max(a->kp, niceCeil(std::isfinite(span) && span > 0 ? f / span : a->kp * s));
          break;
        }
        case ActuatorType::Velocity: a->kv = niceCeil(a->kv * s); break;
        case ActuatorType::Cylinder:
          if (a->diameter > 0) a->diameter *= std::sqrt(s);
          else a->area *= s;
          break;
        case ActuatorType::Muscle:
          if (a->muscleForce > 0) a->muscleForce = niceCeil(a->muscleForce * s);
          else a->muscleScale *= s;
          break;
        case ActuatorType::General:
          a->gainprm[0] *= s;
          if (positionLikeGeneral(*a)) {
            a->biasprm[1] *= s;
            a->biasprm[2] *= std::sqrt(s);
          }
          break;
        default: break;
      }
    }
    if (a->forcelimited && std::max(std::fabs(a->forcerange.x), std::fabs(a->forcerange.y)) < f) {
      a->forcerange.x = std::min(a->forcerange.x, -f);
      a->forcerange.y = std::max(a->forcerange.y, f);
    }
  }
  if (Joint* j = d.joint(joint))
    if (j->actuatorfrclimited && std::max(std::fabs(j->actuatorfrcrange.x), std::fabs(j->actuatorfrcrange.y)) < target) {
      j->actuatorfrcrange.x = std::min(j->actuatorfrcrange.x, -target);
      j->actuatorfrcrange.y = std::max(j->actuatorfrcrange.y, target);
    }
}

// Replaces compiler settotalmass (deprecated in MuJoCo 3.16) by scaling every mass source the way
// MuJoCo scales the compiled masses: the model stays the same.
void bakeTotalMass(Document& d) {
  double target = d.compiler.settotalmass;
  if (!(target > 0)) return;
  d.compiler.settotalmass = -1;
  CompileResult r = compileMjcf(writeMjcf(d));
  double total = 0;
  if (r.ok) {
    for (int b = 1; b < r.model->nbody; ++b) total += r.model->body_mass[b];
  } else {
    // A model MuJoCo refuses for another reason: MJForge's own estimate (exact for primitives).
    std::vector<Pose> poses = computeBodyPoses(d);
    for (size_t b = 1; b < d.bodies.size(); ++b) total += bodyMassProps(d, poses, d.bodies[b].id).mass;
  }
  if (!(total > 0)) return;  // no mass to scale: MuJoCo ignores settotalmass, so dropping it is exact
  double s = target / total;
  for (Geom& g : d.geoms) {
    if (g.mass >= 0) g.mass *= s;
    else g.density *= s;
  }
  for (Body& b : d.bodies)
    if (b.hasInertial) {
      b.mass *= s;
      b.diaginertia = b.diaginertia * s;
    }
  d.compiler.boundmass *= s;
  d.compiler.boundinertia *= s;
}

class Analyzer {
 public:
  Analyzer(const Document& d, const CompileResult* cr, bool crValid) : doc_(d), cr_(cr), crValid_(crValid) {}

  AnalysisReport run() {
    // Everything is evaluated at the start pose: the first keyframe if there is one.
    poses_ = computeBodyPoses(doc_, doc_.keyframes.empty() ? nullptr : &doc_.keyframes[0], &jointFrames_);
    weld_ = computeWeld(doc_);
    Vec3 g = doc_.option.gravity;
    gmag_ = doc_.option.gravityFlag ? length(g) : 0.0;
    gdir_ = normalize(g, {0, 0, -1});
    prepare();
    checkStructure();
    checkMass();
    analyzeJoints();
    analyzeLegs();
    checkJointFindings();
    checkStartState();
    checkMjwarp();
    analyzeCollisions();
    groupRepeated();
    std::stable_sort(rep_.issues.begin(), rep_.issues.end(),
                     [](const Issue& a, const Issue& b) { return int(a.severity) < int(b.severity); });
    for (const Issue& i : rep_.issues) rep_.counts[int(i.severity)]++;
    return std::move(rep_);
  }

 private:
  Issue& add(Severity s, IssueArea a, const char* code, std::string title, std::string detail, std::vector<Id> el = {}) {
    Issue i;
    i.severity = s;
    i.area = a;
    i.code = code;
    i.title = std::move(title);
    i.detail = std::move(detail);
    i.elements = std::move(el);
    rep_.issues.push_back(std::move(i));
    return rep_.issues.back();
  }

  // Collapses many findings of the same kind into one issue (with a combined fix).
  void groupRepeated() {
    struct G { const char* code; const char* title; };
    const G groups[] = {{"weak-actuator", "joints are too weak for the loads they carry"},
                        {"weak-for-motion", "joints may be too weak for some motions"},
                        {"servo-sag", "position servos are soft and sag under the body weight"},
                        {"stiff-servo", "joints are too stiff for the timestep"},
                        {"strong-actuator", "joints are much stronger than their loads"},
                        {"passive-joint", "joints are passive and undamped"},
                        {"explicit-damping", "joints have actuator damping that is unstable with this integrator"}};
    for (const G& g : groups) {
      std::vector<size_t> idx;
      for (size_t i = 0; i < rep_.issues.size(); ++i) if (rep_.issues[i].code == g.code) idx.push_back(i);
      if (idx.size() <= 3) continue;
      Issue merged;
      merged.code = g.code;
      merged.area = rep_.issues[idx[0]].area;
      merged.severity = Severity::Tip;
      std::vector<std::function<void(Document&)>> fixes;
      std::string detail;
      for (size_t k = 0; k < idx.size(); ++k) {
        Issue& is = rep_.issues[idx[k]];
        merged.severity = std::min(merged.severity, is.severity, [](Severity a, Severity b) { return int(a) < int(b); });
        if (k < 4) detail += (k ? "\n" : "") + is.title;
        for (Id id : is.elements) merged.elements.push_back(id);
        if (is.fix) fixes.push_back(is.fix);
      }
      if (idx.size() > 4) detail += fmt("\n... and %zu more. Select a joint to see its numbers in the inspector.", idx.size() - 4);
      merged.title = fmt("%zu %s", idx.size(), g.title);
      merged.detail = rep_.issues[idx[0]].detail + "\n\n" + detail;
      if (!fixes.empty()) {
        merged.fixLabel = fixes.size() == idx.size() ? fmt("Fix all %zu", fixes.size())
                                                     : fmt("Fix %zu of %zu", fixes.size(), idx.size());
        merged.fix = [fixes](Document& d) { for (const auto& f : fixes) f(d); };
      }
      for (auto it = idx.rbegin(); it != idx.rend(); ++it) rep_.issues.erase(rep_.issues.begin() + long(*it));
      rep_.issues.push_back(std::move(merged));
    }
  }

  // ---------------------------------------------------------------- structure
  void checkStructure() {
    // Unique names per kind.
    auto dupes = [&](const auto& vec, Kind kind) {
      std::unordered_map<std::string, std::vector<Id>> by;
      for (const auto& e : vec) if (!e.name.empty()) by[e.name].push_back(e.id);
      for (auto& [name, ids] : by) {
        if (ids.size() < 2) continue;
        add(Severity::Error, IssueArea::Model, "duplicate-name", fmt("Duplicate %s name '%s'", kindName(kind), name.c_str()),
            "MuJoCo requires unique names per element type. Rename one of them.", ids);
      }
    };
    dupes(doc_.bodies, Kind::Body);
    dupes(doc_.geoms, Kind::Geom);
    dupes(doc_.joints, Kind::Joint);
    dupes(doc_.sites, Kind::Site);
    dupes(doc_.cameras, Kind::Camera);
    dupes(doc_.lights, Kind::Light);
    dupes(doc_.actuators, Kind::Actuator);
    dupes(doc_.sensors, Kind::Sensor);
    dupes(doc_.tendons, Kind::Tendon);
    dupes(doc_.equalities, Kind::Equality);
    dupes(doc_.pairs, Kind::Pair);
    dupes(doc_.excludes, Kind::Exclude);
    dupes(doc_.materials, Kind::Material);
    dupes(doc_.textures, Kind::Texture);
    dupes(doc_.meshes, Kind::Mesh);
    dupes(doc_.hfields, Kind::Hfield);
    {
      std::unordered_map<std::string, int> keys;
      for (const Keyframe& k : doc_.keyframes)
        if (!k.name.empty() && ++keys[k.name] == 2)
          add(Severity::Error, IssueArea::Model, "duplicate-name", fmt("Duplicate keyframe name '%s'", k.name.c_str()),
              "MuJoCo requires unique keyframe names. Rename one of them in the Scene panel.", {});
    }

    for (const Texture& t : doc_.textures) {
      std::error_code ec;
      if (t.builtin != TextureBuiltin::None || t.file.empty() || std::filesystem::is_regular_file(std::filesystem::u8path(t.file), ec)) continue;
      add(Severity::Warning, IssueArea::Model, "texture-file", fmt("Texture image not found: %s", t.file.c_str()),
          fmt("Texture %s is exported as a flat color until the file exists again.", quote(t.name).c_str()), {});
    }

    for (const Actuator& a : doc_.actuators) {
      bool ok = false;
      switch (a.trn) {
        case TrnType::Joint:
        case TrnType::JointInParent: ok = doc_.joint(a.target) != nullptr; break;
        case TrnType::Site: ok = doc_.site(a.target) != nullptr; break;
        case TrnType::Body: ok = doc_.body(a.target) != nullptr && a.target != kWorldId; break;
        case TrnType::Tendon: ok = doc_.tendon(a.target) != nullptr; break;
        case TrnType::SliderCrank: ok = doc_.site(a.target) && doc_.site(a.slidersite); break;
      }
      if (!ok) {
        add(Severity::Error, IssueArea::Model, "actuator-target", fmt("Actuator %s has no valid target", quote(a.name).c_str()),
            "Pick the joint (or site/body) this actuator drives in the inspector.", {a.id});
        continue;
      }
      bool trnOk = a.type == ActuatorType::Adhesion ? a.trn == TrnType::Body
                   : a.type == ActuatorType::Muscle ? a.trn != TrnType::Body && a.trn != TrnType::Site
                   : a.type == ActuatorType::DcMotor ? a.trn != TrnType::Body
                                                     : true;
      if (!trnOk) {
        add(Severity::Error, IssueArea::Model, "actuator-transmission",
            fmt(a.type == ActuatorType::Adhesion ? "Adhesion actuator %s needs a body transmission"
                : a.type == ActuatorType::Muscle ? "Muscle %s needs a joint or tendon transmission"
                                                 : "DC motor %s needs a joint, tendon or site transmission", quote(a.name).c_str()),
            a.type == ActuatorType::Adhesion ? "Adhesion pulls on the contacts of a body. Pick the body in the inspector."
            : a.type == ActuatorType::Muscle ? "A muscle's force depends on its length, which only joints and tendons have."
                                             : "A motor turns something: pick a joint (or a tendon or site) in the inspector.",
            {a.id});
      }
      if (a.type == ActuatorType::DcMotor) {
        double K, R;
        dcMotorConstants(a, &K, &R);
        bool controller = a.dcinput != DcInput::Voltage;
        if (K <= 0 || R <= 0) {
          Issue& i = add(Severity::Error, IssueArea::Model, "dcmotor-constants",
                         fmt("DC motor %s needs its electrical data", quote(a.name).c_str()),
                         "MuJoCo needs a positive motor constant and resistance: enter the nominal voltage, stall torque and "
                         "no-load speed of a datasheet, or the resistance and motor constants.", {a.id});
          Id id = a.id;
          i.fixLabel = "Use 24 V, 5 N*m, 10 rad/s";
          i.fix = [id](Document& d) {
            if (Actuator* x = d.actuator(id)) {
              x->nominal = {24, 5, 10};
              x->resistance = 0;
              x->motorconst = {0, 0};
            }
          };
        } else if (!controller && (a.controller[0] != 0 || a.controller[1] != 0 || a.controller[2] != 0)) {
          Issue& i = add(Severity::Error, IssueArea::Model, "dcmotor-controller",
                         fmt("DC motor %s has controller gains but takes a voltage", quote(a.name).c_str()),
                         "The on-board controller works on position, velocity or torque inputs. Pick one as the input, or "
                         "clear the gains.", {a.id});
          Id id = a.id;
          i.fixLabel = "Take a position target";
          i.fix = [id](Document& d) { if (Actuator* x = d.actuator(id)) x->dcinput = DcInput::Pos; };
        } else if (a.controller[1] > 0 && a.dcinput != DcInput::Pos) {
          add(Severity::Error, IssueArea::Model, "dcmotor-integral", fmt("DC motor %s: the integral gain needs a position target", quote(a.name).c_str()),
              "The integrator accumulates position error, so ki works only with the position input.", {a.id});
        } else if (!controller && a.controller[3] > 0) {
          add(Severity::Error, IssueArea::Model, "dcmotor-slew", fmt("DC motor %s: a target rate limit needs the controller", quote(a.name).c_str()),
              "The rate limit applies to the controller's target: take a position, velocity or torque input.", {a.id});
        }
        // MuJoCo derives the missing one of thermal resistance, capacitance and time constant.
        const Vec6& th = a.thermal;
        if (th[0] > 0 || th[1] > 0 || th[2] > 0) {
          bool ok = (th[0] > 0 && th[1] > 0) || (th[2] > 0 && (th[0] > 0 || th[1] > 0));
          if (!ok)
            add(Severity::Error, IssueArea::Model, "dcmotor-thermal", fmt("DC motor %s: incomplete thermal model", quote(a.name).c_str()),
                "Give the thermal resistance and capacitance, or the time constant with one of them.", {a.id});
        }
        if (a.inductance.x < 0 || a.inductance.y < 0)
          add(Severity::Error, IssueArea::Model, "dcmotor-inductance", fmt("DC motor %s: negative inductance", quote(a.name).c_str()), "", {a.id});
      }
      // The delayed control is read from the last nsample controls (one per step), so the buffer must
      // reach back delay / timestep steps or the delay is cut short.
      int needSamples = actuatorHistoryNeeded(a, doc_.option.timestep);
      bool beyond = needSamples > kMaxHistorySamples;  // a delay longer than MuJoCo can hold
      if (a.nsample > kMaxHistorySamples) {
        Issue& i = add(Severity::Error, IssueArea::Model, "history-size",
                       fmt("Actuator %s keeps more history than MuJoCo allows", quote(a.name).c_str()),
                       fmt("MuJoCo keeps at most %d samples.", kMaxHistorySamples), {a.id});
        int n = std::min(needSamples, kMaxHistorySamples);
        Id id = a.id;
        i.fixLabel = fmt("Keep %d samples", n);
        i.fix = [id, n](Document& d) { if (Actuator* x = d.actuator(id)) x->nsample = n; };
      } else if (a.delay > 0 && a.nsample < needSamples) {
        bool none = a.nsample <= 0;
        std::string detail =
            beyond ? fmt("The %.3g s delay needs more than the %d samples MuJoCo keeps at this timestep: shorten the delay "
                         "or lengthen the timestep.", a.delay, kMaxHistorySamples)
            : none ? std::string("MuJoCo refuses a control delay without a history buffer: the delayed control is read "
                                 "from the actuator's last nsample controls.")
                   : fmt("The delayed control is read from the last %d controls, which reach back %.3g s, so the %.3g s "
                         "delay is cut short. It needs delay / timestep = %d samples.",
                         a.nsample, a.nsample * doc_.option.timestep, a.delay, needSamples);
        Issue& i = add(none ? Severity::Error : Severity::Warning, IssueArea::Model, "delay-history",
                       none ? fmt("Actuator %s has a delay but no history buffer", quote(a.name).c_str())
                            : fmt("Actuator %s keeps too few samples for its delay", quote(a.name).c_str()),
                       detail, {a.id});
        if (!beyond) {
          Id id = a.id;
          i.fixLabel = fmt("Keep %d samples", needSamples);
          i.fix = [id, needSamples](Document& d) { if (Actuator* x = d.actuator(id)) x->nsample = needSamples; };
        }
      }
      // inheritrange (on a servo: position, intvelocity, or a general actuator shaped like one) takes
      // the range of a hinge or slide joint or of a tendon, which must have one.
      bool servoShape = a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity ||
                        (a.type == ActuatorType::General && a.gaintype == GainType::Fixed && a.biastype == BiasType::Affine &&
                         a.gainprm[0] == -a.biasprm[1]);
      if (servoShape && a.inheritrange > 1e-12) {
        const Joint* tj = a.trn == TrnType::Joint ? doc_.joint(a.target) : nullptr;
        const Tendon* tt = a.trn == TrnType::Tendon ? doc_.tendon(a.target) : nullptr;
        bool dynOk = a.type != ActuatorType::General || a.dyntype == DynType::None || a.dyntype == DynType::FilterExact ||
                     a.dyntype == DynType::Integrator;
        const char* why = !dynOk ? "works only with position or integrator dynamics"
                          : a.trn != TrnType::Joint && a.trn != TrnType::Tendon
                              ? "needs a joint or tendon transmission (not a joint in its parent's frame, a site or a body)"
                          : tj && tj->type != JointType::Hinge && tj->type != JointType::Slide ? "needs a hinge or slide joint"
                          : (tj && tj->range.x == tj->range.y) || (tt && tt->range.x == tt->range.y)
                              ? "copies the range of its joint or tendon, which has none"
                              : nullptr;
        if (why) {
          Issue& i = add(Severity::Error, IssueArea::Model, "inherit-range",
                         fmt("Actuator %s cannot inherit its range", quote(a.name).c_str()),
                         std::string("MuJoCo refuses it: inheritrange ") + why + ".", {a.id, a.target});
          Id id = a.id;
          i.fixLabel = "Use its own range";
          i.fix = [id](Document& d) {
            if (Actuator* x = d.actuator(id)) {
              x->inheritrange = 0;
              if (!(x->ctrlrange.x < x->ctrlrange.y)) x->ctrllimited = false;
              if (!(x->actrange.x < x->actrange.y)) x->actlimited = false;
            }
          };
        }
      }
      // A muscle's length range is found by MuJoCo, which drives the transmission to its extremes
      // (mj_setLengthRange), unless a valid one is given: its length must be bounded. Measured with
      // MuJoCo 3.16: an unlimited hinge or slide, a ball or a free joint fails; a spatial tendon
      // fails when its ends can drift apart without limit (an unlimited slide or a free joint between
      // them, while rotations keep it bounded); a fixed tendon fails on any unlimited joint; a
      // limited tendon is always bounded.
      bool muscleLike = a.type == ActuatorType::Muscle ||
                        (a.type == ActuatorType::General && (a.gaintype == GainType::Muscle || a.biastype == BiasType::Muscle));
      bool lrGiven = (std::fabs(a.lengthrange.x) > 1e-12 || std::fabs(a.lengthrange.y) > 1e-12) && a.lengthrange.x < a.lengthrange.y;
      if (muscleLike && !lrGiven) {
        auto bounded = [](const Joint& j) { return j.limited && j.range.x < j.range.y; };
        const Joint* jt = a.trn == TrnType::Joint || a.trn == TrnType::JointInParent ? doc_.joint(a.target) : nullptr;
        const Tendon* tn = a.trn == TrnType::Tendon ? doc_.tendon(a.target) : nullptr;
        std::string why;
        if (jt && (jt->type == JointType::Ball || jt->type == JointType::Free))
          why = fmt("a %s joint has no single length to range over: muscles act on hinge and slide joints or tendons.",
                    jt->type == JointType::Ball ? "ball" : "free");
        else if (jt && !bounded(*jt))
          why = fmt("joint %s is not limited. Limit it, or set the muscle's length range (Other).", quote(jt->name).c_str());
        else if (tn && !(tn->limited && tn->range.x < tn->range.y)) {
          const Joint* loose = nullptr;
          if (tn->type == TendonType::Fixed) {
            for (const TendonJoint& tj : tn->joints)
              if (const Joint* j = doc_.joint(tj.joint); j && tj.coef != 0 && !bounded(*j) && !loose) loose = j;
          } else {
            // Joints between the bodies the path is attached to: below their common ancestor.
            std::vector<Id> ends;
            for (const TendonPoint& p : tn->path) {
              const Site* st = p.type == TendonPoint::Type::Site ? doc_.site(p.obj) : nullptr;
              const Geom* gm = p.type == TendonPoint::Type::Geom ? doc_.geom(p.obj) : nullptr;
              if (st) ends.push_back(st->body);
              if (gm) ends.push_back(gm->body);
            }
            const DocIndex& ix = doc_.index();
            auto up = [&](Id b) { const Body* x = doc_.body(b); return x ? x->parent : kWorldId; };
            auto depthOf = [&](Id b) { int i = doc_.bodyIndex(b); return i < 0 ? 0 : ix.depth[size_t(i)]; };
            Id common = ends.empty() ? kWorldId : ends[0];
            for (Id e : ends) {
              Id x = common, y = e;
              while (depthOf(x) > depthOf(y)) x = up(x);
              while (depthOf(y) > depthOf(x)) y = up(y);
              while (x != y && x != kWorldId) {
                x = up(x);
                y = up(y);
              }
              common = x == y ? x : kWorldId;
            }
            for (Id e : ends)
              for (Id b = e; b != common && b != kWorldId && doc_.bodyIndex(b) >= 0 && !loose; b = up(b))
                for (uint32_t ji : ix.bodyJoints[size_t(doc_.bodyIndex(b))]) {
                  const Joint& j = doc_.joints[ji];
                  if (j.type == JointType::Free || (j.type == JointType::Slide && !bounded(j))) loose = &j;
                }
          }
          if (loose)
            why = fmt("tendon %s can stretch without limit (joint %s %s freely). Limit the tendon or that joint, or set "
                      "the muscle's length range (Other).",
                      quote(tn->name).c_str(), quote(loose->name).c_str(),
                      loose->type == JointType::Hinge ? "turns" : loose->type == JointType::Slide ? "slides" : "moves");
        }
        if (!why.empty())
          add(Severity::Error, IssueArea::Model, "muscle-length",
              fmt("MuJoCo cannot find the length range of muscle %s", quote(a.name).c_str()),
              "It moves a muscle's joint or tendon to its extremes to find its operating lengths, but " + why, {a.id, a.target});
      }
      // Damping and armature act on the driven joint or tendon (the export writes them unless below
      // 1e-12; adhesion has neither).
      bool jointOrTendon = a.trn == TrnType::Joint || a.trn == TrnType::JointInParent || a.trn == TrnType::Tendon;
      if (a.type != ActuatorType::Adhesion && !jointOrTendon && (std::fabs(a.damping) > 1e-12 || std::fabs(a.armature) > 1e-12)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "actuator-reflected",
                       fmt("Actuator %s has damping or armature on a %s transmission", quote(a.name).c_str(),
                           enumLabel(kTrnTypes, a.trn)),
                       "MuJoCo adds actuator damping and armature to the joint or tendon it drives, and refuses them on "
                       "other transmissions. Put them on the joints instead.",
                       {a.id});
        Id id = a.id;
        i.fixLabel = "Remove them";
        i.fix = [id](Document& d) { if (Actuator* x = d.actuator(id)) x->damping = x->armature = 0; };
      }
      bool servo = a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity;
      if (servo && a.kv > 0 && a.dampratio > 0) {
        Issue& i = add(Severity::Error, IssueArea::Model, "kv-dampratio",
                       fmt("Actuator %s sets both kv and damping ratio", quote(a.name).c_str()),
                       "MuJoCo accepts only one of them. The damping ratio is usually easier to tune.", {a.id});
        Id id = a.id;
        i.fixLabel = "Keep damping ratio";
        i.fix = [id](Document& d) { if (Actuator* x = d.actuator(id)) x->kv = 0; };
      }
      if (a.type == ActuatorType::Damper && (!a.ctrllimited || a.ctrlrange.x < 0)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "damper-range", fmt("Damper %s needs a non-negative control range", quote(a.name).c_str()),
                       "Damper controls scale damping and must be in a range like [0, 1].", {a.id});
        Id id = a.id;
        i.fixLabel = "Set range to [0, 1]";
        i.fix = [id](Document& d) { if (Actuator* x = d.actuator(id)) { x->ctrllimited = true; x->ctrlrange = {0, 1}; } };
      }
      if (!a.ctrllimited && a.type != ActuatorType::Damper) {
        Issue& i = add(Severity::Warning, IssueArea::Model, "ctrl-unbounded", fmt("Actuator %s has an unbounded control range", quote(a.name).c_str()),
                       servo ? "Position targets are not clamped; RL policies need a bounded action space."
                             : "RL policies need a bounded action space; the force is not limited either unless a force range is set.",
                       {a.id});
        Id id = a.id;
        const Joint* j = a.trn == TrnType::Joint ? doc_.joint(a.target) : nullptr;
        bool useJoint = a.type == ActuatorType::Position && j && j->limited;  // an intvelocity control is a speed
        Vec2 r = useJoint ? j->range : Vec2{-1, 1};
        i.fixLabel = useJoint ? "Use the joint range" : "Limit to [-1, 1]";
        i.fix = [id, r](Document& d) { if (Actuator* x = d.actuator(id)) { x->ctrllimited = true; x->ctrlrange = r; } };
      }
      // MuJoCo refuses an empty or reversed range on a limited control, force or activation, as the
      // export writes them (an inherited range, a DC motor's force and a zero adhesion force range
      // are not written).
      bool integratorDyn = a.type == ActuatorType::IntVelocity || (a.type == ActuatorType::General && a.dyntype == DynType::Integrator);
      bool inherits = a.inheritrange > 0 && (a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity ||
                                             a.type == ActuatorType::General);
      bool ctrlWritten = a.ctrllimited && a.type != ActuatorType::Damper && a.type != ActuatorType::Adhesion && !(inherits && !integratorDyn);
      bool forceWritten = a.forcelimited && a.type != ActuatorType::DcMotor &&
                          (a.type != ActuatorType::Adhesion || std::fabs(a.forcerange.x) > 1e-12 || std::fabs(a.forcerange.y) > 1e-12);
      bool actWritten = a.actlimited && (a.type == ActuatorType::IntVelocity || a.type == ActuatorType::General) &&
                        !(inherits && integratorDyn);
      auto rangeOrder = [&](bool written, const Vec2& r, const char* code, const char* what, Vec2 Actuator::*field, bool Actuator::*flag,
                            bool silent = false) {
        if (!written || r.x < r.y) return;
        Issue& i = add(silent ? Severity::Warning : Severity::Error, IssueArea::Model, code,
                       fmt("Actuator %s: %s range lower limit is not below the upper limit", quote(a.name).c_str(), what),
                       silent ? "MuJoCo silently drops such a limit on an intvelocity servo, so its target is unlimited."
                              : "MuJoCo refuses an empty or reversed range on a limited actuator.",
                       {a.id});
        Id id = a.id;
        bool swap = r.x > r.y;
        i.fixLabel = swap ? "Swap the limits" : "Turn the limit off";
        i.fix = [id, swap, field, flag](Document& d) {
          if (Actuator* x = d.actuator(id)) {
            if (swap) std::swap((x->*field).x, (x->*field).y);
            else x->*flag = false;
          }
        };
      };
      rangeOrder(ctrlWritten, a.ctrlrange, "ctrl-range-order", "control", &Actuator::ctrlrange, &Actuator::ctrllimited);
      rangeOrder(forceWritten, a.forcerange, "force-range-order", "force", &Actuator::forcerange, &Actuator::forcelimited);
      rangeOrder(actWritten, a.actrange, "act-range-order", "activation", &Actuator::actrange, &Actuator::actlimited,
                 a.type == ActuatorType::IntVelocity);
      if (actWritten && a.type == ActuatorType::General && a.dyntype == DynType::None) {
        Issue& i = add(Severity::Error, IssueArea::Model, "act-range-nodyn",
                       fmt("Actuator %s limits an activation it does not have", quote(a.name).c_str()),
                       "An activation range needs activation dynamics (integrator, filter, muscle or user); MuJoCo refuses "
                       "it with dynamics 'none'.",
                       {a.id});
        Id id = a.id;
        i.fixLabel = "Turn the activation limit off";
        i.fix = [id](Document& d) { if (Actuator* x = d.actuator(id)) x->actlimited = false; };
      }
      // Muscle parameters (as gain and bias of a general actuator too): MuJoCo needs range[0] <
      // range[1], lmin < 1 < lmax and positive scale, vmax, fpmax and fvmax.
      auto muscleBad = [](double r0, double r1, double scale, double lmin, double lmax, double vmax, double fpmax, double fvmax) {
        return !(r0 < r1) || !(lmin < 1) || !(lmax > 1) || !(scale > 0) || !(vmax > 0) || !(fpmax > 0) || !(fvmax > 0);
      };
      bool badMuscle = false;
      if (a.type == ActuatorType::Muscle)
        badMuscle = muscleBad(a.muscleRange.x, a.muscleRange.y, a.muscleScale, a.lmin, a.lmax, a.vmax, a.fpmax, a.fvmax);
      if (a.type == ActuatorType::General)
        for (const Vec10* p : {a.gaintype == GainType::Muscle ? &a.gainprm : nullptr, a.biastype == BiasType::Muscle ? &a.biasprm : nullptr})
          if (p) badMuscle |= muscleBad((*p)[0], (*p)[1], (*p)[3], (*p)[4], (*p)[5], (*p)[6], (*p)[7], (*p)[8]);
      if (badMuscle) {
        Issue& i = add(Severity::Error, IssueArea::Model, "muscle-params", fmt("Muscle %s has invalid parameters", quote(a.name).c_str()),
                       "MuJoCo needs an operating range with lower < upper, lmin < 1 < lmax, and a positive scale, vmax, "
                       "fpmax and fvmax.",
                       {a.id});
        if (a.type == ActuatorType::Muscle) {
          Id id = a.id;
          i.fixLabel = "Reset the invalid ones";
          i.fix = [id](Document& d) {
            Actuator* x = d.actuator(id);
            if (!x) return;
            const Actuator def;
            if (!(x->muscleRange.x < x->muscleRange.y)) x->muscleRange = def.muscleRange;
            if (!(x->lmin < 1)) x->lmin = def.lmin;
            if (!(x->lmax > 1)) x->lmax = def.lmax;
            if (!(x->muscleScale > 0)) x->muscleScale = def.muscleScale;
            if (!(x->vmax > 0)) x->vmax = def.vmax;
            if (!(x->fpmax > 0)) x->fpmax = def.fpmax;
            if (!(x->fvmax > 0)) x->fvmax = def.fvmax;
          };
        }
      }
      const Joint* tj = a.trn == TrnType::Joint ? doc_.joint(a.target) : nullptr;
      // The range of joint angles a servo targets: the control of a position servo, the integrated
      // activation of an intvelocity servo (whose control is a speed).
      bool intvel = a.type == ActuatorType::IntVelocity;
      bool hasTargets = intvel ? a.actlimited : a.type == ActuatorType::Position && a.ctrllimited && a.inheritrange <= 0;
      Vec2 targets = intvel ? a.actrange : a.ctrlrange;
      bool degrees = hasTargets && tj && tj->type == JointType::Hinge &&
                     std::max(std::fabs(targets.x), std::fabs(targets.y)) > 2 * kPi + 0.1;
      if (degrees) {
        // MuJoCo converts joint ranges from degrees (compiler angle="degree") but never these.
        Issue& i = add(Severity::Warning, IssueArea::Physics, "ctrl-range-degrees",
                       fmt("Servo %s: %s range [%.4g, %.4g] looks like degrees", quote(a.name).c_str(),
                           intvel ? "activation" : "control", targets.x, targets.y),
                       "Servo targets are joint angles in radians. MuJoCo converts joint ranges written in degrees "
                       "(compiler angle=\"degree\") but never converts ctrlrange or actrange, a common mistake in "
                       "imported files.",
                       {a.id});
        Id id = a.id;
        i.fixLabel = "Convert to radians";
        i.fix = [id, intvel](Document& d) {
          if (Actuator* x = d.actuator(id)) {
            Vec2& r = intvel ? x->actrange : x->ctrlrange;
            r = {deg2rad(r.x), deg2rad(r.y)};
          }
        };
      }
      if (a.type == ActuatorType::Position && a.ctrllimited && a.trn == TrnType::Joint && !degrees) {
        const Joint* j = tj;
        double tol = deg2rad(1.0);
        if (j && j->limited && (a.ctrlrange.x < j->range.x - tol || a.ctrlrange.y > j->range.y + tol)) {
          Issue& i = add(Severity::Info, IssueArea::Physics, "target-beyond-limit",
                         fmt("Servo %s can target positions beyond the joint limits", quote(a.name).c_str()),
                         "Targets outside the joint range just press the joint against its limit.", {a.id, j->id});
          Id id = a.id;
          Vec2 r = j->range;
          i.fixLabel = "Match joint range";
          i.fix = [id, r](Document& d) { if (Actuator* x = d.actuator(id)) x->ctrlrange = r; };
        }
      }
    }
    auto isKind = [&](FrameObj t, Id id) {
      switch (t) {
        case FrameObj::Body:
        case FrameObj::XBody: return doc_.body(id) != nullptr;
        case FrameObj::Geom: return doc_.geom(id) != nullptr;
        case FrameObj::Site: return doc_.site(id) != nullptr;
        case FrameObj::Camera: return doc_.camera(id) != nullptr;
      }
      return false;
    };
    for (const Sensor& s : doc_.sensors) {
      SensorObj o = sensorInfo(s.type).obj;
      bool ok = true;
      bool bodyOrGeom1 = s.objtype == FrameObj::Body || s.objtype == FrameObj::Geom;
      bool bodyOrGeom2 = s.reftype == FrameObj::Body || s.reftype == FrameObj::Geom;
      switch (o) {
        case SensorObj::Joint: ok = doc_.joint(s.obj) != nullptr; break;
        case SensorObj::Actuator: ok = doc_.actuator(s.obj) != nullptr; break;
        case SensorObj::Site: ok = doc_.site(s.obj) != nullptr; break;
        case SensorObj::Body: ok = doc_.body(s.obj) != nullptr; break;
        case SensorObj::Frame: ok = isKind(s.objtype, s.obj) && (s.ref == kNoId || isKind(s.reftype, s.ref)); break;
        case SensorObj::Tendon: ok = doc_.tendon(s.obj) != nullptr; break;
        case SensorObj::GeomPair: ok = bodyOrGeom1 && bodyOrGeom2 && isKind(s.objtype, s.obj) && isKind(s.reftype, s.ref); break;
        case SensorObj::Contact:
          ok = (s.obj == kNoId || (s.objtype != FrameObj::Camera && isKind(s.objtype, s.obj))) &&
               (s.ref == kNoId || (s.reftype != FrameObj::Camera && s.reftype != FrameObj::Site && isKind(s.reftype, s.ref)));
          break;
        case SensorObj::Inside: ok = isKind(s.objtype, s.obj) && doc_.site(s.ref) && !(s.enclosed && s.objtype == FrameObj::Camera); break;
        case SensorObj::CamProj: ok = doc_.site(s.obj) && doc_.camera(s.ref); break;
        case SensorObj::None: break;
      }
      if (!ok) add(Severity::Error, IssueArea::Model, "sensor-target", fmt("Sensor %s has no valid target", quote(s.name).c_str()),
                   "Choose what this sensor is attached to in the inspector.", {s.id});
      // MuJoCo's rules on the kind of joint or tendon a sensor reads.
      const Joint* sj = o == SensorObj::Joint ? doc_.joint(s.obj) : nullptr;
      const Tendon* st = o == SensorObj::Tendon ? doc_.tendon(s.obj) : nullptr;
      const char* why = nullptr;
      switch (s.type) {
        case SensorType::JointPos:
        case SensorType::JointVel:
        case SensorType::JointActuatorFrc:
          if (sj && sj->type != JointType::Hinge && sj->type != JointType::Slide)
            why = "reads a single joint coordinate, so it needs a hinge or slide joint (ball joints have ballquat / "
                  "ballangvel sensors; a free body has frame sensors)";
          break;
        case SensorType::BallQuat:
        case SensorType::BallAngVel:
          if (sj && sj->type != JointType::Ball) why = "needs a ball joint (hinge and slide joints have jointpos / jointvel)";
          break;
        case SensorType::JointLimitPos:
        case SensorType::JointLimitVel:
        case SensorType::JointLimitFrc:
          if (sj && !sj->limited) why = "reports the joint's limit, so the joint must be limited";
          break;
        case SensorType::TendonLimitPos:
        case SensorType::TendonLimitVel:
        case SensorType::TendonLimitFrc:
          if (st && !st->limited) why = "reports the tendon's limit, so the tendon must be limited";
          break;
        default: break;
      }
      if (why)
        add(Severity::Error, IssueArea::Model, "sensor-kind", fmt("Sensor %s does not fit its target", quote(s.name).c_str()),
            std::string("MuJoCo refuses it: it ") + why + ".", {s.id, s.obj});
      if (s.noise < 0 || s.cutoff < 0)
        add(Severity::Error, IssueArea::Model, "sensor-negative", fmt("Sensor %s has a negative noise or cutoff", quote(s.name).c_str()),
            "", {s.id});
      bool unitData = s.type == SensorType::FrameXAxis || s.type == SensorType::FrameYAxis || s.type == SensorType::FrameZAxis ||
                      s.type == SensorType::FrameQuat || s.type == SensorType::BallQuat;
      if (unitData && s.cutoff > 0) {
        Issue& i = add(Severity::Error, IssueArea::Model, "sensor-cutoff",
                       fmt("Sensor %s: a cutoff cannot clip an axis or quaternion", quote(s.name).c_str()),
                       "MuJoCo refuses a cutoff on directions and orientations (clipping would break their unit length).",
                       {s.id});
        Id id = s.id;
        i.fixLabel = "Remove the cutoff";
        i.fix = [id](Document& d) { if (Sensor* x = d.sensor(id)) x->cutoff = 0; };
      }
      if (o == SensorObj::GeomPair) {
        auto hfield = [&](FrameObj t, Id g) {
          const Geom* x = t == FrameObj::Geom ? doc_.geom(g) : nullptr;
          return x && x->type == GeomType::Hfield;
        };
        if (hfield(s.objtype, s.obj) || hfield(s.reftype, s.ref))
          add(Severity::Error, IssueArea::Model, "sensor-hfield",
              fmt("Sensor %s measures a height field, which distance sensors do not support", quote(s.name).c_str()),
              "MuJoCo refuses height fields in distance, normal and fromto sensors (they are not convex). Measure "
              "against the body or another geom.",
              {s.id});
        if (s.obj != kNoId && s.obj == s.ref && s.objtype == s.reftype)
          add(Severity::Error, IssueArea::Model, "sensor-same", fmt("Sensor %s measures an object against itself", quote(s.name).c_str()),
              "MuJoCo refuses a distance, normal or fromto sensor whose two sides are the same geom or body. Choose "
              "another second object.",
              {s.id, s.obj});
      }
      if (o == SensorObj::CamProj)
        if (const Camera* c = doc_.camera(s.ref); c && (c->resolution[0] <= 0 || c->resolution[1] <= 0))
          add(Severity::Error, IssueArea::Model, "sensor-target",
              fmt("Sensor %s projects onto camera %s, which has no resolution", quote(s.name).c_str(), quote(c->name).c_str()), "",
              {s.id, c->id});
      // Update period and phase: MuJoCo takes a phase in (-period, 0], the first update being at
      // period + phase.
      if (s.interval.x < 0 || s.interval.y > 0 || (s.interval.x > 0 && s.interval.y <= -s.interval.x)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "sensor-interval",
                       fmt("Sensor %s has an invalid update period or phase", quote(s.name).c_str()),
                       "MuJoCo takes a period of zero or more and a phase between minus the period and 0 (the first "
                       "update is at period + phase).",
                       {s.id});
        Id id = s.id;
        i.fixLabel = "Update from the start";
        i.fix = [id](Document& d) {
          if (Sensor* x = d.sensor(id)) x->interval = {std::max(0.0, x->interval.x), 0};
        };
      }
      // Delayed and slower-rate values are read from the sensor's history buffer: one sample per step,
      // or per update period when there is one.
      int needSamples = sensorHistoryNeeded(s, doc_.option.timestep);
      bool beyond = needSamples > kMaxHistorySamples;
      bool noBuffer = s.nsample <= 0;
      if (s.nsample > kMaxHistorySamples) {
        Issue& i = add(Severity::Error, IssueArea::Model, "history-size",
                       fmt("Sensor %s keeps more history than MuJoCo allows", quote(s.name).c_str()),
                       fmt("MuJoCo keeps at most %d samples.", kMaxHistorySamples), {s.id});
        int n = std::min(needSamples, kMaxHistorySamples);
        Id id = s.id;
        i.fixLabel = fmt("Keep %d samples", n);
        i.fix = [id, n](Document& d) { if (Sensor* x = d.sensor(id)) x->nsample = n; };
      } else if ((s.delay > 0 && s.nsample < needSamples) || (s.interval.x > 0 && noBuffer)) {
        bool refused = s.delay > 0 && noBuffer;
        std::string detail =
            beyond     ? fmt("The %.3g s delay needs more than the %d samples MuJoCo keeps: shorten the delay, or "
                             "lengthen the timestep or update period.", s.delay, kMaxHistorySamples)
            : refused  ? std::string("MuJoCo refuses a sensor delay without a history buffer.")
            : noBuffer ? std::string("Without a history buffer MuJoCo ignores the update period and computes the sensor "
                                     "every step.")
                       : fmt("The buffer reaches back too little time, so the %.3g s delay is cut short.", s.delay);
        Issue& i = add(refused ? Severity::Error : Severity::Warning, IssueArea::Model, "sensor-history",
                       noBuffer ? fmt("Sensor %s has a %s but no history buffer", quote(s.name).c_str(),
                                      s.delay > 0 ? "delay" : "update period")
                                : fmt("Sensor %s keeps too few samples for its delay", quote(s.name).c_str()),
                       detail, {s.id});
        if (!beyond) {
          Id id = s.id;
          i.fixLabel = fmt("Keep %d samples", needSamples);
          i.fix = [id, needSamples](Document& d) { if (Sensor* x = d.sensor(id)) x->nsample = needSamples; };
        }
      }
      if (s.type == SensorType::Contact) {
        constexpr int kVel = (1 << 7) | (1 << 8);  // linvel, angvel
        if ((s.condata & kVel) && s.reduce == 3)
          add(Severity::Error, IssueArea::Model, "contact-sensor-netforce",
              fmt("Contact sensor %s: velocities cannot be combined with net force", quote(s.name).c_str()),
              "MuJoCo refuses relative velocities with reduce=\"netforce\".", {s.id});
        if (s.condata & kVel) {
          Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-contact-sensor-vel",
                         fmt("Contact sensor %s reports velocities, which MuJoCo Warp does not support", quote(s.name).c_str()),
                         "MuJoCo Warp raises NotImplementedError for contact sensor linvel / angvel.", {s.id});
          Id id = s.id;
          i.fixLabel = "Drop the velocity fields";
          i.fix = [id](Document& d) {
            if (Sensor* x = d.sensor(id)) x->condata = (x->condata & ~kVel) ? x->condata & ~kVel : 1;
          };
        }
      }
    }
    for (const Camera& c : doc_.cameras) {
      // MuJoCo refuses a fovy of 180 or more, also when it is an orthographic view height in meters
      // (written unless a lens replaces it).
      if (c.fovy >= 180 && !(c.sensorsize.x > 0 && c.sensorsize.y > 0)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "camera-fovy",
                       fmt(c.orthographic ? "Camera %s: view height of 180 m or more" : "Camera %s: field of view of 180 degrees or more",
                           quote(c.name).c_str()),
                       "MuJoCo refuses a fovy of 180 or more (for an orthographic camera it is the view height in meters).",
                       {c.id});
        Id id = c.id;
        i.fixLabel = c.orthographic ? "Use 179 m" : "Use 90 degrees";
        i.fix = [id](Document& d) { if (Camera* x = d.camera(id)) x->fovy = x->orthographic ? 179 : 90; };
      }
      bool sensor = c.sensorsize.x > 0 && c.sensorsize.y > 0;
      bool partial = !sensor && (c.sensorsize.x != 0 || c.sensorsize.y != 0 || c.focal.x != 0 || c.focal.y != 0 ||
                                 c.principal.x != 0 || c.principal.y != 0);
      bool noResolution = sensor && (c.resolution[0] <= 0 || c.resolution[1] <= 0);
      if (partial || noResolution) {
        // An incomplete lens is left out of the export (the field of view applies); a lens without
        // a resolution is refused by MuJoCo.
        Issue& i = add(noResolution ? Severity::Error : Severity::Warning, IssueArea::Model, "camera-intrinsics",
                       fmt("Camera %s: incomplete lens settings", quote(c.name).c_str()),
                       noResolution ? "A sensor size needs a resolution."
                                    : "The lens needs both sensor sizes (a focal length or principal point without them "
                                      "does nothing), so the export uses the field of view instead.",
                       {c.id});
        Id id = c.id;
        i.fixLabel = "Use the field of view";
        i.fix = [id](Document& d) {
          if (Camera* x = d.camera(id)) {
            x->sensorsize = x->focal = x->principal = Vec2{};
            x->resolution[0] = std::max(x->resolution[0], 1);
            x->resolution[1] = std::max(x->resolution[1], 1);
          }
        };
      }
    }
    for (const Light& l : doc_.lights)
      if (!(l.softness >= 0 && l.softness <= 1)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "light-softness", fmt("Light %s: softness must be between 0 and 1", quote(l.name).c_str()),
                       "", {l.id});
        Id id = l.id;
        i.fixLabel = "Clamp it";
        i.fix = [id](Document& d) { if (Light* x = d.light(id)) x->softness = std::isfinite(x->softness) ? clampv(x->softness, 0.0, 1.0) : 0.2; };
      }
    for (const Exclude& e : doc_.excludes)
      if (!doc_.body(e.body1) || !doc_.body(e.body2))
        add(Severity::Error, IssueArea::Model, "exclude-target", "Contact exclusion refers to a missing body", "", {e.id});
    checkPairsEqualitiesTendons();

    for (const Joint& j : doc_.joints) {
      const Body* b = doc_.body(j.body);
      if (!b) continue;
      if (j.type == JointType::Free && b->parent != kWorldId) {
        Issue& i = add(Severity::Error, IssueArea::Model, "free-not-top", fmt("Free joint %s is not on a top-level body", quote(j.name).c_str()),
                       "A free (6-DoF) joint must be on a body attached directly to the world. Move the body to the world, "
                       "or use hinge/slide/ball joints here.", {j.id});
        Id bid = b->id;
        i.fixLabel = "Move body to the world";
        i.fix = [bid](Document& d) { reparentBody(d, bid, kWorldId, true); };
      }
      if (b->mocap)
        add(Severity::Error, IssueArea::Model, "mocap-joint", fmt("Mocap body %s cannot have joints", quote(b->name).c_str()),
            "Mocap bodies are moved kinematically; remove the joint or turn off mocap.", {j.id, b->id});
      if (j.actuatorfrclimited && j.actuatorfrcrange.x >= j.actuatorfrcrange.y) {
        Issue& i = add(Severity::Error, IssueArea::Model, "joint-actfrcrange",
                       fmt("Joint %s: actuator force range lower limit is not below the upper limit", quote(j.name).c_str()),
                       "MuJoCo refuses an empty or reversed actuator force range on a joint.", {j.id});
        Id id = j.id;
        i.fixLabel = "Turn the limit off";
        i.fix = [id](Document& d) { if (Joint* x = d.joint(id)) x->actuatorfrclimited = false; };
      }
      if (j.limited && (j.type == JointType::Hinge || j.type == JointType::Slide) && j.range.x >= j.range.y) {
        Issue& i = add(Severity::Error, IssueArea::Model, "joint-range",
                       fmt("Joint %s: lower limit is not below the upper limit", quote(j.name).c_str()),
                       "MuJoCo refuses a limited joint with an empty range. Set range = [min, max], or turn the limit off.", {j.id});
        Id id = j.id;
        i.fixLabel = "Turn the limit off";
        i.fix = [id](Document& d) { if (Joint* x = d.joint(id)) x->limited = false; };
      }
      if (j.limited && j.type == JointType::Ball && j.range.x == 0 && j.range.y <= 0) {
        Issue& i = add(Severity::Warning, IssueArea::Physics, "ball-locked",
                       fmt("Ball joint %s is limited to a 0 degree cone", quote(j.name).c_str()),
                       "The limit constraint fights every rotation, so the joint is locked (and softly at that). Give it a "
                       "maximum angle, or remove the joint if the parts should be rigid.", {j.id});
        Id id = j.id;
        i.fixLabel = "Turn the limit off";
        i.fix = [id](Document& d) { if (Joint* x = d.joint(id)) x->limited = false; };
      }
      if (j.type == JointType::Ball && j.limited && j.range.x != 0) {
        Issue& i = add(Severity::Error, IssueArea::Model, "ball-range", fmt("Ball joint %s: range must start at 0", quote(j.name).c_str()),
                       "Ball joint ranges are [0, maximum cone angle].", {j.id});
        Id id = j.id;
        i.fixLabel = "Set lower bound to 0";
        i.fix = [id](Document& d) { if (Joint* x = d.joint(id)) x->range.x = 0; };
      }
    }
    // MuJoCo's rules for the joints of one body.
    for (size_t bi = 1; bi < doc_.bodies.size(); ++bi) {
      const Body& b = doc_.bodies[bi];
      const std::vector<uint32_t>& js = doc_.index().bodyJoints[bi];
      int dofs = 0;
      bool ball = false, rotationAfterBall = false;
      for (uint32_t ji : js) {
        const Joint& j = doc_.joints[ji];
        dofs += j.type == JointType::Free ? 6 : j.type == JointType::Ball ? 3 : 1;
        rotationAfterBall |= ball && (j.type == JointType::Ball || j.type == JointType::Hinge);
        ball |= j.type == JointType::Ball;
      }
      if (dofs > 6)
        add(Severity::Error, IssueArea::Model, "body-dofs", fmt("Body %s has %d degrees of freedom", quote(b.name).c_str(), dofs),
            "MuJoCo allows at most 6 per body. Remove joints, or chain them through more bodies.", {b.id});
      if (rotationAfterBall)
        add(Severity::Error, IssueArea::Model, "ball-then-rotation",
            fmt("Body %s has a hinge or ball joint after a ball joint", quote(b.name).c_str()),
            "MuJoCo refuses this order: a ball joint already turns the body every way, so any further rotation is "
            "redundant. Remove the extra joint, put the ball joint last, or move it to a body of its own.", {b.id});
      if (b.mocap && b.parent != kWorldId) {
        Issue& i = add(Severity::Error, IssueArea::Model, "mocap-not-top", fmt("Mocap body %s is not attached to the world", quote(b.name).c_str()),
                       "MuJoCo moves mocap bodies in world coordinates, so they must be children of the world.", {b.id});
        Id id = b.id;
        i.fixLabel = "Move body to the world";
        i.fix = [id](Document& d) { reparentBody(d, id, kWorldId, true); };
      }
    }
    for (const Body& b : doc_.bodies) {
      if (b.sleep == SleepPolicy::Auto) continue;
      if (!doc_.isTreeRoot(b.id)) {
        Issue& i = add(Severity::Warning, IssueArea::Model, "sleep-not-root",
                       fmt("Sleep policy of body %s is ignored", quote(b.name).c_str()),
                       "Only the first moving body of a kinematic tree (a body with joints whose parents are static) takes a "
                       "sleep policy; it applies to the whole tree. The export leaves it out here.", {b.id});
        Id id = b.id;
        i.fixLabel = "Reset to auto";
        i.fix = [id](Document& d) { if (Body* x = d.body(id)) x->sleep = SleepPolicy::Auto; };
      } else if (!doc_.option.sleep) {
        Issue& i = add(Severity::Info, IssueArea::Model, "sleep-disabled",
                       fmt("Body %s has a sleep policy but sleeping is off", quote(b.name).c_str()),
                       "Sleep policies only matter when the sleep flag is enabled (Scene > Flags > Sleep).", {b.id});
        i.fixLabel = "Enable sleeping";
        i.fix = [](Document& d) { d.option.sleep = true; };
      }
    }
    if (doc_.option.density <= 0 && doc_.option.viscosity <= 0) {
      std::vector<Id> fluid;
      for (const Geom& g : doc_.geoms)
        if (g.fluidEllipsoid) fluid.push_back(g.id);
      if (!fluid.empty()) {
        Issue& i = add(Severity::Info, IssueArea::Physics, "fluid-no-medium",
                       fmt("%zu geom(s) use the fluid model, but the medium has no density or viscosity", fluid.size()),
                       "The ellipsoid fluid forces scale with Scene > Environment > Medium density / viscosity, which are 0.",
                       fluid);
        i.fixLabel = "Use air";
        i.fix = [](Document& d) {
          d.option.density = 1.2;
          d.option.viscosity = 1.8e-5;
        };
      }
    }
    // MuJoCo's XML reader (tinyxml2) accepts at most 496 nested bodies.
    {
      const DocIndex& ix = doc_.index();
      size_t deepest = 0;
      for (size_t bi = 1; bi < ix.depth.size(); ++bi)
        if (ix.depth[bi] > ix.depth[deepest]) deepest = bi;
      if (ix.depth.size() > 1 && ix.depth[deepest] > 496)
        add(Severity::Error, IssueArea::Model, "body-depth", fmt("Bodies are nested %d deep", ix.depth[deepest]),
            "MuJoCo's XML reader accepts at most 496 nested bodies, so it cannot load this model. Attach long chains "
            "to a body higher up.",
            {doc_.bodies[deepest].id});
    }
    // Explicit inertials are checked by MuJoCo as written, after the compiler's lower bounds (with
    // inertiafromgeom="true" the geoms replace them).
    if (doc_.compiler.inertiafromgeom != InertiaFromGeom::True)
      for (size_t bi = 1; bi < doc_.bodies.size(); ++bi) {
        const Body& b = doc_.bodies[bi];
        if (!b.hasInertial) continue;
        double lo = doc_.compiler.boundinertia;
        Vec3 I = {std::max(b.diaginertia.x, lo), std::max(b.diaginertia.y, lo), std::max(b.diaginertia.z, lo)};
        if (std::max(b.mass, doc_.compiler.boundmass) < 0 || I.x < 0 || I.y < 0 || I.z < 0) {
          add(Severity::Error, IssueArea::Model, "inertial-negative", fmt("Body %s has a negative mass or inertia", quote(b.name).c_str()),
              "Mass and inertia must be zero or positive.", {b.id});
        } else if (!doc_.compiler.balanceinertia && (I.x + I.y < I.z || I.x + I.z < I.y || I.y + I.z < I.x)) {
          Issue& i = add(Severity::Error, IssueArea::Model, "inertia-triangle",
                         fmt("Body %s has an impossible inertia", quote(b.name).c_str()),
                         fmt("No rigid body has the principal inertias %.3g, %.3g, %.3g: each must be at most the sum of "
                             "the other two (A + B >= C), so MuJoCo refuses the model.",
                             I.x, I.y, I.z),
                         {b.id});
          Id id = b.id;
          i.fixLabel = "Average the three";
          i.fix = [id, I](Document& d) {
            if (Body* x = d.body(id)) x->diaginertia = Vec3{1, 1, 1} * ((I.x + I.y + I.z) / 3);
          };
        }
      }
    for (const Site& x : doc_.sites) {
      int n = x.type == GeomType::Sphere ? 1 : (x.type == GeomType::Capsule || x.type == GeomType::Cylinder) ? 2 : 3;
      bool bad = false;
      for (int i = 0; i < n; ++i) bad |= !(x.size[i] >= 1e-15);  // the export writes smaller values as 0
      if (bad) {
        Issue& i = add(Severity::Error, IssueArea::Model, "site-size", fmt("Site %s has a zero or negative size", quote(x.name).c_str()),
                       "", {x.id});
        Id id = x.id;
        i.fixLabel = "Use 5 mm";
        i.fix = [id](Document& d) {
          if (Site* s = d.site(id))
            for (int k = 0; k < 3; ++k)
              if (!(s->size[k] >= 1e-15)) s->size[k] = 0.005;
        };
      }
    }
    for (const Geom& g : doc_.geoms) {
      int n = g.type == GeomType::Sphere ? 1 : (g.type == GeomType::Capsule || g.type == GeomType::Cylinder) ? 2 : 3;
      if (g.type == GeomType::Plane || g.type == GeomType::Hfield || g.type == GeomType::Mesh) n = 0;
      bool bad = false;
      for (int i = 0; i < n; ++i) bad |= !(g.size[i] >= 1e-15);  // the export writes smaller values as 0
      if (g.type == GeomType::Plane) bad |= !(g.size[2] >= 1e-15);  // the grid spacing
      if (bad) add(Severity::Error, IssueArea::Model, "geom-size", fmt("Geom %s has a zero or negative size", quote(g.name).c_str()), "", {g.id});
      if (g.mass < 0 && g.density < 0)
        add(Severity::Error, IssueArea::Model, "geom-density", fmt("Geom %s has a negative density", quote(g.name).c_str()), "", {g.id});
      if (g.condim != 1 && g.condim != 3 && g.condim != 4 && g.condim != 6) {
        Issue& i = add(Severity::Error, IssueArea::Model, "geom-condim", fmt("Geom %s: contact dimension must be 1, 3, 4 or 6", quote(g.name).c_str()),
                       "", {g.id});
        Id id = g.id;
        i.fixLabel = "Use 3 (sliding friction)";
        i.fix = [id](Document& d) { if (Geom* x = d.geom(id)) x->condim = 3; };
      }
      if (g.type == GeomType::Plane && g.body != kWorldId && weld_.weld[doc_.bodyIndex(g.body)] != 0) {
        add(Severity::Error, IssueArea::Model, "plane-moving", fmt("Plane %s is on a moving body", quote(g.name).c_str()),
            "Planes are infinite and static. Use a thin box for moving parts.", {g.id});
      }
      if (g.type == GeomType::Hfield && !doc_.hfield(g.hfield))
        add(Severity::Error, IssueArea::Model, "hfield-missing", fmt("Height field geom %s has no height field data", quote(g.name).c_str()), "", {g.id});
      if (g.type == GeomType::Mesh && !doc_.mesh(g.mesh)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "mesh-missing", fmt("Mesh geom %s has no mesh", quote(g.name).c_str()),
                       "Pick a mesh in the inspector, load one with Add > Geom > Mesh from file, or use a primitive shape.", {g.id});
        Id id = g.id;
        i.fixLabel = "Make it a box";
        i.fix = [id](Document& d) {
          if (Geom* x = d.geom(id)) {
            x->type = GeomType::Box;
            x->size = {0.05, 0.05, 0.05};
          }
        };
      }
    }
  }

  void checkPairsEqualitiesTendons() {
    for (const ContactPair& c : doc_.pairs) {
      std::string n = quote(doc_.displayName(c.id));
      if (!doc_.geom(c.geom1) || !doc_.geom(c.geom2))
        add(Severity::Error, IssueArea::Model, "pair-target", fmt("Contact pair %s needs two geoms", n.c_str()),
            "Pick both geoms in the inspector.", {c.id});
      else if (c.geom1 == c.geom2) {
        Issue& i = add(Severity::Error, IssueArea::Model, "pair-same", fmt("Contact pair %s uses the same geom twice", n.c_str()),
                       "MuJoCo then makes contacts of the geom with itself that push on nothing, and MuJoCo Warp 3.16 "
                       "applies the pair to a different pair of geoms (or fails to load a model with a single geom). "
                       "Pick a second geom, or delete the pair.",
                       {c.id});
        Id id = c.id;
        i.fixLabel = "Delete the pair";
        i.fix = [id](Document& d) { deleteElements(d, {id}); };
      }
      if (c.condim != 1 && c.condim != 3 && c.condim != 4 && c.condim != 6)
        add(Severity::Error, IssueArea::Model, "pair-condim", fmt("Contact pair %s: contact dimension must be 1, 3, 4 or 6", n.c_str()), "", {c.id});
    }
    for (const Equality& e : doc_.equalities) {
      std::string n = quote(doc_.displayName(e.id));
      Kind k1 = doc_.kindOf(e.obj1), k2 = doc_.kindOf(e.obj2);
      bool ok = true;
      std::string why;
      switch (e.type) {
        case EqualityType::Connect:
        case EqualityType::Weld:
          if (k1 == Kind::Site) {
            ok = k2 == Kind::Site && e.obj1 != e.obj2;
            why = "Site constraints need two different sites.";
          } else {
            // The world may be either body, but not both.
            ok = k1 == Kind::Body && (e.obj2 == kNoId || k2 == Kind::Body) && e.obj1 != e.obj2 &&
                 !(e.obj1 == kWorldId && (e.obj2 == kNoId || e.obj2 == kWorldId));
            why = "Pick two different bodies (the second may be left empty for the world), or two sites.";
          }
          break;
        case EqualityType::Joint: {
          auto scalar = [&](Id j) {
            const Joint* x = doc_.joint(j);
            return x && (x->type == JointType::Hinge || x->type == JointType::Slide);
          };
          ok = scalar(e.obj1) && (e.obj2 == kNoId || scalar(e.obj2)) && e.obj1 != e.obj2;
          why = "Joint couplings take hinge or slide joints (the second may be left empty to hold the first in place).";
          break;
        }
        case EqualityType::Tendon:
          ok = doc_.tendon(e.obj1) && (e.obj2 == kNoId || doc_.tendon(e.obj2)) && e.obj1 != e.obj2;
          why = "Pick one or two tendons.";
          break;
      }
      if (!ok) add(Severity::Error, IssueArea::Model, "equality-target", fmt("Equality constraint %s is incomplete", n.c_str()), why, {e.id});
    }
    for (const Tendon& t : doc_.tendons) {
      std::string n = quote(doc_.displayName(t.id));
      if (t.type == TendonType::Fixed) {
        bool ok = !t.joints.empty();
        for (const TendonJoint& tj : t.joints) {
          const Joint* j = doc_.joint(tj.joint);
          ok &= j && (j->type == JointType::Hinge || j->type == JointType::Slide);
        }
        if (!ok)
          add(Severity::Error, IssueArea::Model, "tendon-joints", fmt("Fixed tendon %s needs hinge or slide joints", n.c_str()),
              "A fixed tendon's length is a weighted sum of joint positions. Add joints in the inspector.", {t.id});
      } else {
        std::string why = spatialPathError(doc_, t);  // MuJoCo's own path rules
        if (!why.empty())
          add(Severity::Error, IssueArea::Model, "tendon-path", fmt("Spatial tendon %s has an invalid path", n.c_str()),
              "MuJoCo refuses it: " + why + ". Paths run through sites; wrapping geoms (spheres or cylinders) sit between "
              "two sites; a pulley starts a new branch of sites.", {t.id});
        // An actuator's armature is added to the tendon it drives, and MuJoCo's tendon dynamics do
        // not support armature with geom wrapping: it stops with "geom wrapping not supported"
        // already while compiling (measured with MuJoCo 3.16; its compile check sees only the
        // tendon's own armature).
        bool wraps = std::any_of(t.path.begin(), t.path.end(), [](const TendonPoint& p) { return p.type == TendonPoint::Type::Geom; });
        if (wraps)
          for (const Actuator& a : doc_.actuators)
            if (a.trn == TrnType::Tendon && a.target == t.id && a.type != ActuatorType::Adhesion && std::fabs(a.armature) > 1e-12) {
              Issue& i = add(Severity::Error, IssueArea::Model, "tendon-wrap-armature",
                             fmt("Actuator %s adds armature to tendon %s, which wraps around a geom", quote(a.name).c_str(), n.c_str()),
                             "MuJoCo's tendon dynamics do not support armature with geom wrapping: it stops with \"geom "
                             "wrapping not supported\". Put the inertia on the joints instead.",
                             {a.id, t.id});
              Id aid = a.id;
              i.fixLabel = "Remove the actuator's armature";
              i.fix = [aid](Document& d) { if (Actuator* x = d.actuator(aid)) x->armature = 0; };
            }
      }
      if (t.limited && t.range.x >= t.range.y)
        add(Severity::Error, IssueArea::Model, "tendon-range", fmt("Tendon %s: lower limit is not below the upper limit", n.c_str()), "",
            {t.id});
      Id id = t.id;
      if (t.actfrclimited && (t.actuatorfrcrange.x >= t.actuatorfrcrange.y || t.actuatorfrcrange.x > 0 || t.actuatorfrcrange.y < 0)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "tendon-actfrcrange", fmt("Tendon %s: invalid actuator force range", n.c_str()),
                       "MuJoCo needs lower < upper, with 0 inside the range.", {t.id});
        i.fixLabel = "Turn the limit off";
        i.fix = [id](Document& d) { if (Tendon* x = d.tendon(id)) x->actfrclimited = false; };
      }
      if (t.springlength.x > t.springlength.y) {
        Issue& i = add(Severity::Error, IssueArea::Model, "tendon-springlength",
                       fmt("Tendon %s: spring length lower value above the upper one", n.c_str()),
                       "The two values bound the spring's dead band (-1 -1 uses the length at the reference pose). "
                       "MuJoCo refuses a lower value above the upper one.",
                       {t.id});
        i.fixLabel = "Use one rest length";
        i.fix = [id](Document& d) {
          if (Tendon* x = d.tendon(id)) {
            double l = x->springlength.y < 0 ? x->springlength.x : x->springlength.y;  // -1 alone: the value typed
            x->springlength = {l, l};
          }
        };
      }
      if (t.type == TendonType::Spatial && !(t.width >= 1e-15)) {
        Issue& i = add(Severity::Error, IssueArea::Model, "tendon-width", fmt("Spatial tendon %s needs a positive width", n.c_str()),
                       "MuJoCo refuses a spatial tendon of zero width (it only affects drawing).", {t.id});
        i.fixLabel = "Use 3 mm";
        i.fix = [id](Document& d) { if (Tendon* x = d.tendon(id)) x->width = 0.003; };
      }
    }
  }

  // Joints moved or held by tendons and equality constraints (not "passive" even without actuator).
  std::unordered_set<Id> tendonOrEqualityJoints() const {
    std::unordered_set<Id> out;
    auto chainJoints = [&](Id a, Id b) {
      // Joints of the bodies between a and b in the tree (up to their common ancestor).
      std::unordered_set<Id> up;
      for (Id x = a; x != kNoId; x = doc_.body(x) ? doc_.body(x)->parent : kNoId) up.insert(x);
      Id common = b;
      while (common != kNoId && !up.count(common)) common = doc_.body(common) ? doc_.body(common)->parent : kNoId;
      for (Id s : {a, b})
        for (Id x = s; x != kNoId && x != common; x = doc_.body(x) ? doc_.body(x)->parent : kNoId)
          for (uint32_t ji : doc_.index().bodyJoints[doc_.bodyIndex(x)]) out.insert(doc_.joints[ji].id);
    };
    for (const Tendon& t : doc_.tendons) {
      for (const TendonJoint& tj : t.joints) out.insert(tj.joint);
      Id prev = kNoId;
      for (const TendonPoint& p : t.path) {
        if (p.type != TendonPoint::Type::Site || !doc_.site(p.obj)) continue;
        Id b = doc_.site(p.obj)->body;
        if (prev != kNoId) chainJoints(prev, b);
        prev = b;
      }
    }
    for (const Equality& e : doc_.equalities) {
      if (e.type == EqualityType::Joint) {
        out.insert(e.obj1);
        out.insert(e.obj2);
      } else if (e.type == EqualityType::Connect || e.type == EqualityType::Weld) {
        Id b1 = doc_.kindOf(e.obj1) == Kind::Site ? doc_.site(e.obj1)->body : e.obj1;
        Id b2 = e.obj2 == kNoId ? kWorldId : (doc_.kindOf(e.obj2) == Kind::Site ? doc_.site(e.obj2)->body : e.obj2);
        if (doc_.body(b1) && doc_.body(b2)) chainJoints(b1, b2);
      }
    }
    return out;
  }

  // ---------------------------------------------------------------- mass
  // How a model's root moves: a free joint, a slide that lets it rise and fall (planar walkers), or
  // fixed to the world.
  enum class Base { Fixed, Free, Planar };
  struct Arm {
    Id joint;
    double arm;
  };

  bool compiledOk() const { return crValid_ && cr_ && cr_->ok && cr_->model; }
  const InertialTable* table() const { return inertials_.empty() ? nullptr : &inertials_; }

  // Per-body mass and center of mass (from the compiled model when there is one, so every compiler
  // option counts), the share of each body's weight that joints carry, how each model's root moves,
  // and which joints each actuator drives.
  void prepare() {
    inertials_ = crValid_ && cr_ ? compiledInertials(*cr_) : InertialTable{};
    size_t n = doc_.bodies.size();
    bodyMass_.assign(n, {});
    bodyWeight_.assign(n, 0);
    for (size_t bi = 1; bi < n; ++bi) {
      const Body& b = doc_.bodies[bi];
      bodyMass_[bi] = bodyMassProps(doc_, poses_, b.id, table());
      bodyWeight_[bi] = bodyMass_[bi].mass * (1.0 - b.gravcomp);  // gravity compensation carries the rest
    }
    for (const Body& b : doc_.bodies) {
      if (b.id == kWorldId || b.parent != kWorldId) continue;
      Base base = Base::Fixed;
      for (uint32_t ji : doc_.index().bodyJoints[doc_.bodyIndex(b.id)]) {
        const Joint& j = doc_.joints[ji];
        Vec3 a;
        jointWorldFrame(doc_, jointFrames_, j, nullptr, &a);
        if (j.type == JointType::Free) base = Base::Free;
        else if (j.type == JointType::Slide && std::fabs(a.z) > 0.5 && base == Base::Fixed) base = Base::Planar;
      }
      base_[b.id] = base;
      if (base != Base::Fixed) rootAcc_[b.id] = subtreeAcc(b.id);
    }
    // Transmissions: exact moment arms at the start state from MuJoCo, else joints and fixed tendons.
    const mjModel* m = compiledOk() ? cr_->model.get() : nullptr;
    if (m && cr_->map.actuators.size() == size_t(m->nu) && cr_->actuatorMoments.size() == size_t(m->nu) &&
        cr_->map.joints.size() == size_t(m->njnt)) {
      for (int a = 0; a < m->nu; ++a) {
        Id id = cr_->map.actuators[a];
        compiledActuator_[id] = a;
        const Actuator* act = doc_.actuator(id);
        if (!act || act->type == ActuatorType::Adhesion) continue;
        for (const CompileResult::Moment& mo : cr_->actuatorMoments[a]) {
          int jnt = m->dof_jntid[mo.dof];
          if (m->jnt_type[jnt] == mjJNT_HINGE || m->jnt_type[jnt] == mjJNT_SLIDE) arms_[id].push_back({cr_->map.joints[jnt], mo.arm});
        }
      }
      return;
    }
    for (const Actuator& a : doc_.actuators) {
      if (a.type == ActuatorType::Adhesion) continue;
      auto scalar = [&](Id j) {
        const Joint* x = doc_.joint(j);
        return x && (x->type == JointType::Hinge || x->type == JointType::Slide);
      };
      if ((a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) && scalar(a.target)) {
        arms_[a.id].push_back({a.target, a.gear[0]});
      } else if (a.trn == TrnType::Tendon) {
        const Tendon* t = doc_.tendon(a.target);
        if (t && t->type == TendonType::Fixed)
          for (const TendonJoint& tj : t->joints)
            if (scalar(tj.joint)) arms_[a.id].push_back({tj.joint, a.gear[0] * tj.coef});
      }
    }
  }

  void checkMass() {
    double minM = kInf, maxM = 0;
    Id minId = kNoId, maxId = kNoId;
    const DocIndex& ix = doc_.index();
    // MuJoCo's rule for a body with joints: it, or a body welded below it, needs a mass and three
    // principal inertias above mjMINVAL. A model that compiled has passed it.
    std::function<bool(uint32_t)> hasMass = [&](uint32_t bi) {
      const MassProps& mp = bodyMass_[bi];
      Vec3 ev;
      eigSym3(mp.inertia, &ev, nullptr);
      if (mp.mass >= 1e-15 && ev.x >= 1e-15 && ev.y >= 1e-15 && ev.z >= 1e-15) return true;
      for (uint32_t ci : ix.childBodies[bi])
        if (ix.bodyJoints[ci].empty() && hasMass(ci)) return true;
      return false;
    };
    for (size_t bi = 1; bi < doc_.bodies.size(); ++bi) {
      const Body& b = doc_.bodies[bi];
      if (doc_.roleOf(b.id) == Role::Model) rep_.modelMass += bodyMass_[bi].mass;
      if (!compiledOk() && !ix.bodyJoints[bi].empty() && !hasMass(uint32_t(bi))) {
        add(Severity::Error, IssueArea::Model, "massless", fmt("Body %s can move but has no mass", quote(b.name).c_str()),
            "MuJoCo needs a mass and inertia on every body with joints, or on a body attached to it without joints. "
            "Add a geom with volume, or an explicit inertial.", {b.id});
        continue;
      }
      if (weld_.weld[bi] != int(bi) || weld_.dof[bi] == 0) continue;  // only moving weld roots
      // Mass of this rigid group: the body and all bodies welded to it.
      double m = 0;
      for (Id sb : doc_.subtree(b.id)) {
        int si = doc_.bodyIndex(sb);
        if (weld_.weld[si] == int(bi)) m += bodyMass_[si].mass;
      }
      if (m > 0 && m < minM) { minM = m; minId = b.id; }
      if (m > maxM) { maxM = m; maxId = b.id; }
      if (m > 0 && m < 1e-3) {
        bool hasArm = false;
        for (uint32_t ji : ix.bodyJoints[bi]) hasArm |= doc_.joints[ji].armature > 0;
        if (!hasArm)
          add(Severity::Warning, IssueArea::Stability, "tiny-mass", fmt("Body %s is very light (%.2g g)", quote(b.name).c_str(), m * 1000),
              "Very light moving bodies make the dynamics stiff and can destabilize the simulation. Add some joint "
              "armature or increase the density.", {b.id});
      }
    }
    if (maxM > 0 && minM < kInf && maxM / minM > 1000) {
      add(Severity::Warning, IssueArea::Stability, "mass-ratio",
          fmt("Mass ratio %.0f : 1 between moving bodies", maxM / minM),
          fmt("%s weighs %.3g kg while %s weighs %.3g kg. Large ratios make the constraint solver converge slowly "
              "and can cause jitter. Keep ratios below ~100 if you can.",
              quote(doc_.nameOf(maxId)).c_str(), maxM, quote(doc_.nameOf(minId)).c_str(), minM),
          {maxId, minId});
    }
    if (doc_.compiler.settotalmass > 0) {
      Issue& i = add(Severity::Info, IssueArea::Model, "settotalmass",
                     fmt("The compiler scales the total mass to %.4g kg", doc_.compiler.settotalmass),
                     "MuJoCo 3.16 deprecates compiler settotalmass and warns about it. Scaling the geom densities, masses "
                     "and explicit inertials by the same factor gives the identical model without it.");
      i.fixLabel = "Bake it into the masses";
      i.fix = [](Document& d) { bakeTotalMass(d); };
    }
  }

  // Weight (mass times the share of gravity the joints carry) of a subtree and its first moment
  // about p.
  struct WeightSum {
    double w = 0;
    Vec3 moment;
  };
  WeightSum weightOf(Id root, const Vec3& p) const {
    WeightSum s;
    for (Id b : doc_.subtree(root)) {
      int bi = doc_.bodyIndex(b);
      s.w += bodyWeight_[bi];
      s.moment += (bodyMass_[bi].com - p) * bodyWeight_[bi];
    }
    return s;
  }

  // Mass properties of a subtree from the per-body values.
  MassAccumulator subtreeAcc(Id root) const {
    MassAccumulator acc;
    for (Id b : doc_.subtree(root)) {
      const MassProps& mp = bodyMass_[doc_.bodyIndex(b)];
      acc.add(mp.mass, mp.com, mp.inertia);
    }
    return acc;
  }

  Base base(Id root) const {
    auto it = base_.find(root);
    return it == base_.end() ? Base::Fixed : it->second;
  }

  // Moment arm of an actuator on a joint (0 when it doesn't drive it).
  double armOn(Id actuator, Id joint) const {
    auto it = arms_.find(actuator);
    if (it == arms_.end()) return 0;
    double s = 0;
    for (const Arm& a : it->second)
      if (a.joint == joint) s += a.arm;
    return s;
  }

  // Actuator stiffness and damping in actuator space: MuJoCo's compiled gains when available (they
  // include damping ratios turned into kv), else the document's.
  void servoGains(const Actuator& act, double jointInertia, double arm2, double* k, double* c) const {
    *k = *c = 0;
    if (act.type == ActuatorType::DcMotor) {
      // A voltage-driven motor brakes with K^2 / R (back-EMF); its controller compensates that and
      // acts with its own gains.
      double K, R;
      dcMotorConstants(act, &K, &R);
      if (act.dcinput == DcInput::Voltage) *c = R > 0 ? K * K / R : 0;
      else {
        *k = act.dcinput == DcInput::Pos ? act.controller[0] : 0;
        *c = act.controller[2];
      }
      return;
    }
    auto it = compiledActuator_.find(act.id);
    if (it != compiledActuator_.end()) {
      const mjModel* m = cr_->model.get();
      int a = it->second;
      if (m->actuator_biastype[a] == mjBIAS_AFFINE) {
        *k = std::max(0.0, -m->actuator_biasprm[mjNBIAS * a + 1]);
        *c = std::max(0.0, -m->actuator_biasprm[mjNBIAS * a + 2]);
      }
      return;
    }
    switch (act.type) {
      case ActuatorType::Position:
      case ActuatorType::IntVelocity:
        *k = act.kp;
        *c = act.dampratio > 0 ? act.dampratio * 2.0 * std::sqrt(act.kp * std::max(jointInertia, 1e-12) / arm2) : act.kv;
        break;
      case ActuatorType::Velocity: *c = act.kv; break;
      case ActuatorType::General:
        if (act.biastype == BiasType::Affine) {
          *k = std::max(0.0, -act.biasprm[1]);
          *c = std::max(0.0, -act.biasprm[2]);
        }
        break;
      default: break;
    }
  }

  // ---------------------------------------------------------------- joints
  void analyzeJoints() {
    for (const Joint& j : doc_.joints) {
      if (j.type != JointType::Hinge && j.type != JointType::Slide) continue;
      if (!doc_.body(j.body)) continue;
      JointReport r;
      r.joint = j.id;
      r.angular = j.type == JointType::Hinge;
      Vec3 p, a;
      jointWorldFrame(doc_, jointFrames_, j, &p, &a);
      MassAccumulator moved = subtreeAcc(j.body);
      WeightSum w = weightOf(j.body, p);
      double inertia = r.angular ? moved.inertiaAboutAxis(p, a) : moved.mass();
      double mass = moved.mass();
      // On a floating model the joint moves both of its sides, which push against each other; the
      // lighter side is the one that swings when the heavier one stands or lies on something.
      Id root = doc_.rootOf(j.body);
      if (root != j.body && base(root) != Base::Fixed) {
        const MassAccumulator& all = rootAcc_[root];
        inertia = floatingJointInertia(moved, all, p, a, r.angular);
        if (all.mass() - mass < mass) {
          WeightSum wAll = weightOf(root, p);
          w = {wAll.w - w.w, wAll.moment - w.moment};
          mass = all.mass() - mass;
        }
        // Swinging a limb from any angle is routine; tilting a big part of the body (a torso) all the
        // way over is not, so that lift is only advice.
        if (mass > 0.25 * all.mass()) bigPart_.insert(j.id);
      }
      r.movedMass = mass;
      r.inertia = inertia + j.armature;
      if (r.angular) {
        double h = length(cross(a, gdir_));
        r.axisTilt = h;
        r.liftLoad = gmag_ * length(w.moment - a * dot(a, w.moment)) * std::max(h, 0.25);
        r.holdLoad = gmag_ * std::fabs(dot(a, cross(w.moment, gdir_)));  // gravity's torque at the start pose
      } else {
        r.axisTilt = std::fabs(dot(a, gdir_));
        r.liftLoad = gmag_ * std::fabs(w.w) * std::max(r.axisTilt, 0.25);
        r.holdLoad = gmag_ * std::fabs(w.w) * r.axisTilt;
      }
      // Rotor inertia and damping that actuators add (MuJoCo scales them by the squared arm).
      double cPassive = 0;
      for (const Actuator& act : doc_.actuators) {
        double arm = armOn(act.id, j.id);
        r.inertia += act.armature * arm * arm;
        cPassive += act.damping * arm * arm;
      }
      double kServo = 0, cAct = 0;
      for (const Actuator& act : doc_.actuators) {
        double arm = armOn(act.id, j.id);
        if (arm == 0) continue;
        r.actuated = true;
        double f = actuatorMaxForce(doc_, act);
        if (std::isinf(f)) r.unbounded = true;
        r.capacity += std::fabs(arm) * f;
        double k, c;
        servoGains(act, r.inertia, arm * arm, &k, &c);
        kServo += k * arm * arm;
        cAct += c * arm * arm;
      }
      if (j.actuatorfrclimited)
        r.capacity = std::min(r.capacity, std::max(std::fabs(j.actuatorfrcrange.x), std::fabs(j.actuatorfrcrange.y)));
      r.stiffness = kServo + j.stiffness;
      r.damping = cAct + j.damping + cPassive;
      double dt = doc_.option.timestep;
      if (r.inertia > 0 && r.stiffness > 0) r.omegaDt = std::sqrt(r.stiffness / r.inertia) * dt;
      servoStiffness_[j.id] = kServo;
      actDamping_[j.id] = cAct;
      passiveDamping_[j.id] = j.damping + cPassive;
      rep_.joints.push_back(r);
    }
  }

  // The axle of a wheel: a hinge of the body about which the geoms it stands on are round (a sphere,
  // a cylinder on the same axis, a round mesh), so the load passes through the axle.
  Id wheelAxle(int bi, const std::vector<const Geom*>& ground) const {
    const DocIndex& ix = doc_.index();
    for (uint32_t ji : ix.bodyJoints[bi]) {
      const Joint& j = doc_.joints[ji];
      if (j.type != JointType::Hinge) continue;
      Vec3 p, a;
      jointWorldFrame(doc_, jointFrames_, j, &p, &a);
      for (const Geom* gg : ground) {
        const Geom& g = *gg;
        Pose gp = poses_[bi] * Pose{g.pos, g.quat};
        Vec3 d = gp.pos - p;
        if (g.type == GeomType::Sphere && length(d) < 0.2 * g.size.x) return j.id;
        if (g.type == GeomType::Cylinder && std::fabs(dot(rotate(gp.quat, {0, 0, 1}), a)) > 0.95 &&
            length(d - a * dot(d, a)) < 0.2 * g.size.x)
          return j.id;
        if (g.type == GeomType::Mesh) {
          // A mesh reaching equally far from the axle in every direction across it.
          const MeshAsset* mesh = doc_.mesh(g.mesh);
          if (!mesh) continue;
          Quat qi = conj(gp.quat);
          Vec3 al = rotate(qi, a), pl = rotate(qi, p - gp.pos);
          Vec3 u = normalize(cross(al, std::fabs(al.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0})), v = cross(al, u);
          double rmin = kInf, rmax = 0;
          for (int k = 0; k < 16; ++k) {
            Vec3 dir = u * std::cos(kPi * k / 8) + v * std::sin(kPi * k / 8);
            double ext = -kInf;
            for (const Vec3& q : meshSupportPoints(*mesh)) ext = std::max(ext, dot(q - pl, dir));
            rmin = std::min(rmin, ext);
            rmax = std::max(rmax, ext);
          }
          if (rmax > 0 && rmin > 0.95 * rmax) return j.id;
        }
      }
    }
    return kNoId;
  }

  int commonAncestor(int a, int b) const {
    const DocIndex& ix = doc_.index();
    while (a != b && a > 0 && b > 0) {
      if (ix.depth[a] >= ix.depth[b]) a = doc_.bodyIndex(doc_.bodies[a].parent);
      else b = doc_.bodyIndex(doc_.bodies[b].parent);
    }
    return a == b ? a : 0;
  }

  // Legs: chains from a floating model's root down to its feet, the bodies at the bottom.
  void analyzeLegs() {
    const DocIndex& ix = doc_.index();
    for (const Body& root : doc_.bodies) {
      if (root.id == kWorldId || root.parent != kWorldId || root.role != Role::Model) continue;
      if (base(root.id) == Base::Fixed) continue;  // fixed-base robots don't stand on legs
      int ri = doc_.bodyIndex(root.id);
      Aabb box = subtreeAabb(doc_, poses_, root.id, true);  // feet must be able to touch the ground
      if (!box.valid()) continue;
      double height = box.hi.z - box.lo.z;
      double tol = std::max(0.01, 0.06 * height);
      // Feet: bodies whose own geoms reach the bottom of the model, with the points they stand on.
      struct Foot {
        int body;
        std::vector<Vec3> points;
        double zmin;
        double mu;
        Id axle = kNoId;
      };
      std::vector<Foot> feet;
      std::vector<int> feetBelow(doc_.bodies.size(), 0), isFoot(doc_.bodies.size(), 0);
      for (Id b : doc_.subtree(root.id)) {
        if (b == root.id) continue;
        int bi = doc_.bodyIndex(b);
        std::vector<std::pair<const Geom*, std::vector<Vec3>>> low;
        double zmin = kInf;
        for (uint32_t gi : ix.bodyGeoms[bi]) {
          const Geom& g = doc_.geoms[gi];
          if (g.type == GeomType::Plane || g.type == GeomType::Hfield || !geomCollides(g)) continue;
          std::vector<Vec3> pts;
          geomLowPoints(doc_, g, poses_[bi] * Pose{g.pos, g.quat}, box.lo.z + tol, pts);
          for (const Vec3& q : pts) zmin = std::min(zmin, q.z);
          if (!pts.empty()) low.push_back({&g, std::move(pts)});
        }
        if (low.empty()) continue;
        // What it stands on: the geoms reaching within a centimeter of its lowest point.
        Foot f{bi, {}, zmin, 0};
        std::vector<const Geom*> ground;
        for (const auto& [g, pts] : low) {
          bool touches = false;
          for (const Vec3& q : pts)
            if (q.z <= zmin + 0.01) {
              f.points.push_back(q);
              touches = true;
            }
          if (!touches) continue;
          ground.push_back(g);
          f.mu = std::max(f.mu, g->friction.x);
        }
        f.axle = wheelAxle(bi, ground);
        feet.push_back(std::move(f));
        isFoot[bi] = 1;
        for (int x = bi; x > 0 && x != ri; x = doc_.bodyIndex(doc_.bodies[x].parent)) feetBelow[x]++;
      }
      if (feet.empty()) continue;
      // A leg runs from its feet up to where it meets the other legs: the root, or a body high in the
      // model (a pelvis). Feet that meet low down (toes and a heel) belong to the same leg.
      auto legTop = [&](int f) {
        int cur = f;
        while (true) {
          int p = doc_.bodyIndex(doc_.bodies[cur].parent);
          if (p <= 0 || p == ri) return cur;
          bool branches = feetBelow[p] - isFoot[p] > feetBelow[cur];  // other feet hang below p
          if (branches && poses_[p].pos.z - box.lo.z > 0.25 * height) return cur;
          cur = p;
        }
      };
      std::unordered_map<int, Id> wheelAxleOf;
      for (const Foot& f : feet) wheelAxleOf[f.body] = f.axle;
      std::map<int, std::vector<const Foot*>> byTop;
      for (const Foot& f : feet) byTop[legTop(f.body)].push_back(&f);
      // Each leg stands on its lowest feet (a shin that merely comes close does not carry it).
      struct Leg {
        int top, start = -1;
        std::vector<Vec3> points;
        Vec3 center;
        double mu = 0;
        bool wheel = true;
      };
      std::vector<Leg> legs;
      int nLegs = 0;
      for (const auto& [top, fs] : byTop) {
        double low = kInf;
        for (const Foot* f : fs) low = std::min(low, f->zmin);
        Leg leg;
        leg.top = top;
        for (const Foot* f : fs) {
          if (f->zmin > low + 0.01) continue;
          leg.wheel &= f->axle != kNoId;
          leg.mu = std::max(leg.mu, f->mu);
          leg.start = leg.start < 0 ? f->body : commonAncestor(leg.start, f->body);
          leg.points.insert(leg.points.end(), f->points.begin(), f->points.end());
        }
        for (const Vec3& q : leg.points) leg.center += q;
        leg.center = leg.center / double(leg.points.size());
        nLegs += !leg.wheel;
        legs.push_back(std::move(leg));
      }
      rep_.legCount += nLegs;
      legsOf_[root.id] = nLegs;
      double mass = rootAcc_[root.id].mass();
      double W = gmag_ * weightOf(root.id, {}).w;
      if (W <= 0) continue;
      double wStatic = W / double(legs.size());
      double wDyn = W / std::max(1.0, nLegs / 2.0);  // about half the feet are on the ground while moving
      Vec3 up = -gdir_;
      std::string ctx = fmt("%.3g kg on %d %s", mass, int(legs.size()),
                            nLegs ? (legs.size() > 1 ? "legs" : "leg") : (legs.size() > 1 ? "wheels" : "wheel"));
      // With friction the ground can push along a leg rather than straight up: the reaction then
      // points at the leg's top joint, which is how a splayed stance stands without hip torque. The
      // sideways pushes of all legs must cancel, and stay inside the friction cone.
      std::vector<Vec3> tilt(legs.size());
      Vec3 meanTilt;
      for (size_t k = 0; k < legs.size(); ++k) {
        const Leg& leg = legs[k];
        Vec3 hip = poses_[leg.top].pos;
        if (!ix.bodyJoints[leg.top].empty())
          jointWorldFrame(doc_, jointFrames_, doc_.joints[ix.bodyJoints[leg.top][0]], &hip, nullptr);
        Vec3 d = hip - leg.center;
        double vert = dot(d, up);
        tilt[k] = vert > 1e-9 ? (d - up * vert) / vert : Vec3{};
        meanTilt += tilt[k] / double(legs.size());
      }
      std::vector<int> footBodies;
      for (size_t k = 0; k < legs.size(); ++k) {
        const Leg& leg = legs[k];
        if (!leg.wheel) footBodies.push_back(leg.start);
        Vec3 t = tilt[k] - meanTilt;
        double mu = std::min(leg.mu, 1.0);
        if (length(t) > mu) t = t * (mu / length(t));
        Vec3 F = up + t;  // per unit of carried weight
        for (int bi = leg.start; bi > 0;) {
          for (uint32_t ji : ix.bodyJoints[bi]) {
            const Joint& j = doc_.joints[ji];
            JointReport* r = findReport(j.id);
            if (!r || (leg.wheel && j.id == wheelAxleOf[bi])) continue;
            Vec3 p, a;
            jointWorldFrame(doc_, jointFrames_, j, &p, &a);
            double lever, dynLever;
            if (r->angular) {
              Vec3 rr = leg.center - p;
              lever = std::fabs(dot(a, cross(rr, F)));
              // Striding moves the foot away from under the joint.
              dynLever = std::max(lever, 0.2 * length(rr - a * dot(a, rr)) * length(cross(a, up)));
              // The most the joint can use: the load at the edge of the foot.
              double reach = dynLever;
              for (const Vec3& q : leg.points) reach = std::max(reach, std::fabs(dot(a, cross(q - p, F))));
              footReach_[j.id] = std::max(footReach_[j.id], wDyn * reach);
            } else {
              lever = dynLever = std::fabs(dot(a, F));
            }
            r->holdLoad = std::max(r->holdLoad, wStatic * lever);
            if (!leg.wheel) r->supportLoad = std::max(r->supportLoad, wDyn * dynLever);
            supportCtx_[j.id] = ctx;
          }
          if (bi == leg.top) break;
          bi = doc_.bodyIndex(doc_.bodies[bi].parent);
        }
      }
      if (nLegs > 0 && nLegs <= 2) checkAnkleBalance(root, footBodies, box.lo.z + tol);
    }
    // Soft contacts let feet creep under a constant sideways load (splayed legs standing still).
    if (rep_.legCount > 0 && (doc_.option.cone == Cone::Pyramidal || doc_.option.impratio < 5)) {
      Issue& i = add(Severity::Tip, IssueArea::Physics, "friction-creep", "Feet may slowly creep while standing",
                     "MuJoCo's contacts are slightly soft, so a foot under a constant sideways load (splayed legs) "
                     "slides very slowly: standing still, a creature can drift and eventually do the splits. Elliptic "
                     "friction cones with impratio 10-100 make friction much stiffer; both are supported by MuJoCo Warp.",
                     {});
      i.fixLabel = "Use elliptic cones, impratio 10";
      i.fix = [](Document& d) {
        d.option.cone = Cone::Elliptic;
        d.option.impratio = std::max(d.option.impratio, 10.0);
      };
    }
  }

  // Servo actuators of a joint with the factor that brings its servo stiffness to `need`.
  std::vector<std::pair<Id, double>> servoScales(Id joint, double need) {
    std::vector<std::pair<Id, double>> out;
    double k = servoStiffness_[joint];
    if (k <= 0 || need <= k) return out;
    for (Id a : actuatorsOf(joint, false)) {
      const Actuator* act = doc_.actuator(a);
      if (act && (act->type == ActuatorType::Position || act->type == ActuatorType::IntVelocity || positionLikeGeneral(*act)))
        out.push_back({a, need / k});
    }
    return out;
  }

  // Armature that keeps a joint of total stiffness K accurate at the timestep (w*dt <= 0.5), or 0
  // when it already is. Fixes that stiffen servos add it, so they don't trade one issue for a
  // timestep one (the stiff-servo fix uses the same rule).
  double stableArmature(Id joint, double K) {
    const JointReport* r = findReport(joint);
    const Joint* j = doc_.joint(joint);
    if (!r || !j || !(r->inertia > 0) || !(K > 0)) return 0;
    double dt = doc_.option.timestep;
    double arm = K * (dt / 0.5) * (dt / 0.5) - (r->inertia - j->armature);
    return arm > j->armature ? niceCeil(arm) : 0;
  }

  // The stiffening fix for a set of joints: the servo factors, plus armature where needed.
  void stiffenFix(Issue& i, const std::vector<std::pair<Id, double>>& scales, const std::vector<std::pair<Id, double>>& jointStiffness) {
    std::vector<std::pair<Id, double>> armature;
    for (const auto& [jid, K] : jointStiffness)
      if (double a = stableArmature(jid, K); a > 0) armature.push_back({jid, a});
    i.fixLabel = stiffenLabel(doc_, scales) + (armature.empty() ? "" : " and add armature");
    i.fix = [scales, armature](Document& d) {
      for (const auto& [id, s] : scales) stiffenServo(d, id, s);
      for (const auto& [id, a] : armature)
        if (Joint* x = d.joint(id)) x->armature = std::max(x->armature, a);
    };
  }

  // Standing on one or two flat feet, the body above the ankles is an inverted pendulum: only the
  // ankle joints resist tipping, so their stiffness must exceed gravity's (m * g * h).
  void checkAnkleBalance(const Body& root, const std::vector<int>& footBodies, double floorLevel) {
    struct Group {
      Vec3 axis;
      double k = 0;
      std::vector<Id> joints;
      Vec3 anchors;
    };
    std::vector<Group> groups;
    MassAccumulator feet;
    for (int fb : footBodies) {
      bool flat = false;
      for (uint32_t gi : doc_.index().bodyGeoms[fb]) {
        const Geom& g = doc_.geoms[gi];
        if (g.type != GeomType::Box) continue;
        flat |= geomWorldAabb(doc_, g, poses_[fb] * Pose{g.pos, g.quat}).lo.z <= floorLevel;
      }
      if (!flat) return;  // point-like feet: covered by the stance check
      MassAccumulator fm = subtreeAcc(doc_.bodies[fb].id);
      feet.add(fm.mass(), fm.com(), fm.inertiaAboutCom());
      for (uint32_t ji : doc_.index().bodyJoints[fb]) {
        const Joint& j = doc_.joints[ji];
        const JointReport* r = findReport(j.id);
        if (j.type != JointType::Hinge || !r) continue;
        Vec3 p, a;
        jointWorldFrame(doc_, jointFrames_, j, &p, &a);
        if (length(cross(a, gdir_)) < 0.7) continue;  // near-vertical axis: gravity can't tip it
        Group* grp = nullptr;
        for (Group& x : groups)
          if (std::fabs(dot(x.axis, a)) > 0.9) grp = &x;
        if (!grp) {
          groups.push_back({a});
          grp = &groups.back();
        }
        grp->k += r->stiffness;
        grp->joints.push_back(j.id);
        grp->anchors += p;
      }
    }
    MassAccumulator all = subtreeAcc(root.id);
    double mAbove = all.mass() - feet.mass();
    if (mAbove <= 0) return;
    Vec3 comAbove = (all.com() * all.mass() - feet.com() * feet.mass()) * (1.0 / mAbove);
    for (const Group& g : groups) {
      Vec3 anchor = g.anchors * (1.0 / g.joints.size());
      double h = dot(comAbove - anchor, -gdir_);
      if (h <= 0) continue;
      double kGravity = mAbove * gmag_ * h;
      // Torque-controlled ankles (no servo or spring stiffness) are balanced by the policy.
      if (g.k <= 0 || g.k >= 1.5 * kGravity) continue;
      double kEach = niceCeil(2.0 * kGravity / g.joints.size());
      Issue& i = add(Severity::Info, IssueArea::Physics, "ankle-balance",
                     fmt("%s can't stand still: its ankles are softer than gravity's tipping pull", quote(root.name).c_str()),
                     fmt("On its feet, the %.3g kg above the ankles (center of mass %.2f m up) is an inverted pendulum: "
                         "gravity tips it with %.3g N*m/rad while the ankle joints resist with %.3g N*m/rad. Holding "
                         "a pose it slowly falls over, so a policy must balance it actively. For a stance that holds "
                         "by itself, raise each ankle's stiffness to about %.3g N*m/rad.",
                         mAbove, h, kGravity, g.k, kEach),
                     g.joints);
      std::vector<std::pair<Id, double>> scales, stiffness;
      for (Id jid : g.joints) {
        const Joint* jj = doc_.joint(jid);
        std::vector<std::pair<Id, double>> js = servoScales(jid, kEach - (jj ? jj->stiffness : 0));
        if (!js.empty()) stiffness.push_back({jid, kEach});
        scales.insert(scales.end(), js.begin(), js.end());
      }
      if (scales.empty()) continue;
      stiffenFix(i, scales, stiffness);
    }
  }

  JointReport* findReport(Id id) {
    for (JointReport& r : rep_.joints) if (r.joint == id) return &r;
    return nullptr;
  }

  void checkJointFindings() {
    const double dt = doc_.option.timestep;
    const std::unordered_set<Id> constrainedJoints = tendonOrEqualityJoints();
    const Integrator integ = doc_.option.integrator;
    bool explicitDamping = integ == Integrator::Euler || integ == Integrator::RK4;
    bool actuation = doc_.option.actuation;
    if (!actuation && !doc_.actuators.empty()) {
      Issue& i = add(Severity::Warning, IssueArea::Physics, "actuation-off",
                     fmt("Actuation is off: the %zu actuator(s) apply no force", doc_.actuators.size()),
                     "The actuation flag (Scene > Flags) disables every actuator, so the model can't move by itself and "
                     "the strength checks are skipped.");
      i.fixLabel = "Enable actuation";
      i.fix = [](Document& d) { d.option.actuation = true; };
    }
    for (JointReport& r : rep_.joints) {
      const Joint* j = doc_.joint(r.joint);
      if (r.stiffness > 0 && r.holdLoad > 0) r.sag = r.holdLoad / r.stiffness;
      const char* tu = r.angular ? "N*m" : "N";
      std::string jn = quote(j->name);
      bool leg = supportCtx_.count(r.joint) > 0;
      // Strength. Holding the start pose and lifting a limb are firm needs; carrying the body on
      // fewer feet while walking, or tilting a torso all the way over, are estimates, so falling short
      // of them only earns an Info.
      double need = r.required();
      double firm = std::max(bigPart_.count(r.joint) ? 0.0 : r.liftLoad, r.holdLoad);
      if (actuation && r.actuated && !r.unbounded && need > 1e-9 && r.capacity < need) {
        bool walking = r.capacity >= firm;
        double ratio = r.capacity / (walking ? need : firm);
        std::string why = r.holdLoad > r.liftLoad
                              ? fmt("to hold the start pose (%s)", supportCtx_.count(r.joint) ? supportCtx_[r.joint].c_str()
                                                                                              : "against gravity")
                              : fmt("to lift the %.3g kg it moves against gravity", r.movedMass);
        std::string motion = r.supportLoad >= r.liftLoad
                                 ? fmt("carrying the body on fewer feet in this posture takes about %.3g %s (%s). Fine if "
                                       "the policy walks more upright or the gait keeps more feet down.",
                                       r.supportLoad, tu, supportCtx_[r.joint].c_str())
                                 : fmt("tilting the %.3g kg it moves all the way over (getting up, say) takes about %.3g "
                                       "%s. Policies usually get help from the other joints and momentum.",
                                       r.movedMass, r.liftLoad, tu);
        Issue& i = walking
                       ? add(Severity::Info, IssueArea::Physics, "weak-for-motion",
                             fmt("Joint %s may be too weak for some motions: %.3g of ~%.3g %s", jn.c_str(), r.capacity,
                                 need, tu),
                             fmt("Its actuators can apply at most %.3g %s. That holds the start pose, but %s", r.capacity,
                                 tu, motion.c_str()),
                             actuatorsOf(r.joint, true))
                       : add(ratio < 0.25 ? Severity::Error : Severity::Warning, IssueArea::Physics, "weak-actuator",
                             fmt("Joint %s is too weak: %.3g of ~%.3g %s needed", jn.c_str(), r.capacity, need, tu),
                             fmt("Its actuators can apply at most %.3g %s but it needs about %.3g %s %s. That is %s of "
                                 "what is required, so the %s.",
                                 r.capacity, tu, firm, tu, why.c_str(),
                                 ratio < 0.001 ? "under 0.1%" : fmt("%.1f%%", ratio * 100).c_str(),
                                 ratio < 0.25 ? "creature will barely move" : "motion will be sluggish or it may collapse"),
                             actuatorsOf(r.joint, true));
        double target = niceCeil(1.5 * need);
        std::vector<std::pair<Id, double>> arms;
        for (const Actuator& act : doc_.actuators) {
          double arm = armOn(act.id, r.joint);
          if (arm != 0 && act.type != ActuatorType::Damper) arms.push_back({act.id, arm});
        }
        if (!arms.empty()) {
          Id jid = r.joint;
          i.fixLabel = fmt("Make actuators %.3g %s strong", target, tu);
          i.fix = [arms, target, jid](Document& d) { strengthenJoint(d, arms, target, jid); };
        }
      }
      // Servo sag under the weight the joint holds at the start pose.
      double sagLimit = r.angular ? 0.15 : 0.02;
      if (actuation && r.actuated && servoStiffness_[r.joint] > 0 && r.sag > sagLimit && (r.capacity >= need || r.unbounded)) {
        Issue& i = add(Severity::Warning, IssueArea::Physics, "servo-sag",
                       fmt("Servo on %s is soft: sags ~%.3g%s under the %s", jn.c_str(),
                           r.angular ? rad2deg(r.sag) : r.sag * 100, r.angular ? " deg" : " cm",
                           leg ? "body weight" : "weight it holds"),
                       fmt("Holding %s needs %.3g %s at this joint; with a stiffness of %.3g %s/%s the joint "
                           "deflects that much from its target. Raise kp for tighter tracking.",
                           leg ? "the body up" : "its load at the start pose", r.holdLoad, tu, r.stiffness, tu,
                           r.angular ? "rad" : "m"),
                       actuatorsOf(r.joint, true));
        double K = r.holdLoad / (r.angular ? 0.05 : 0.005);
        std::vector<std::pair<Id, double>> scales = servoScales(r.joint, K - j->stiffness);
        if (!scales.empty()) stiffenFix(i, scales, {{r.joint, K}});
      }
      // Very overpowered (only a tip).
      double useful = std::max(need, footReach_.count(r.joint) ? footReach_[r.joint] : 0.0);
      if (actuation && r.actuated && !r.unbounded && need > 1e-6 && r.axisTilt > 0.5 && r.capacity > 100 * useful) {
        add(Severity::Tip, IssueArea::Physics, "strong-actuator",
            fmt("Joint %s is ~%.0fx stronger than its load", jn.c_str(), r.capacity / useful),
            fmt("Up to %.3g %s for a load of ~%.3g %s. Fine for learning, but unrealistic forces can make policies "
                "rely on violent motions that don't transfer to hardware.",
                r.capacity, tu, useful, tu),
            actuatorsOf(r.joint, true));
      }
      // Timestep stability of stiff servos / springs. Damping the integrator treats implicitly lets
      // a stiff joint stay stable (and an overdamped one moves slowly anyway).
      if (r.inertia > 0 && r.stiffness > 0) {
        double w = std::sqrt(r.stiffness / r.inertia);
        double crit = 2.0 * std::sqrt(r.stiffness * r.inertia);
        double cImplicit = integ == Integrator::RK4 ? 0 : integ == Integrator::Euler ? passiveDamping_[r.joint] : r.damping;
        double zi = cImplicit / crit, z = r.damping / crit;
        double hDiverge = integ == Integrator::RK4 ? 2.8 : 2 * zi + 2 * std::sqrt(zi * zi + 1);
        double slow = z > 1 ? w * (z - std::sqrt(z * z - 1)) : w;
        bool explode = w * dt > hDiverge;
        if (explode || slow * dt > 0.6) {
          Issue& i = add(explode ? Severity::Error : Severity::Warning, IssueArea::Stability, "stiff-servo",
                         fmt("Joint %s is too stiff for the timestep (w*dt = %.2f)", jn.c_str(), r.omegaDt),
                         fmt("Stiffness %.3g over an effective inertia of %.3g gives a natural frequency of %.3g rad/s%s. "
                             "With dt = %.4g s that is w*dt = %.2f; %s Increase armature, reduce kp, or reduce the timestep.",
                             r.stiffness, r.inertia, w, z > 0.05 ? fmt(" (damping ratio %.2g)", z).c_str() : "", dt, r.omegaDt,
                             explode ? fmt("above %.2g this integrator diverges.", hDiverge).c_str()
                                     : "keep it below ~0.5 for accurate, stable motion."),
                         {r.joint});
          double base = r.inertia - j->armature;
          double arm = r.stiffness * (dt / 0.5) * (dt / 0.5) - base;
          if (arm > j->armature) {
            Id jid = r.joint;
            double armNice = niceCeil(arm);
            i.fixLabel = fmt("Set armature to %.3g", armNice);
            i.fix = [jid, armNice](Document& d) { if (Joint* x = d.joint(jid)) x->armature = armNice; };
          }
        }
      }
      if (explicitDamping && r.inertia > 0 && actDamping_[r.joint] * dt / r.inertia > 1.0) {
        Issue& i = add(Severity::Warning, IssueArea::Stability, "explicit-damping",
                       fmt("Actuator damping on %s is unstable with the %s integrator", jn.c_str(),
                           enumLabel(kIntegrators, doc_.option.integrator)),
                       "Actuator damping (kv / damping ratio) is integrated explicitly by Euler and RK4. The implicitfast "
                       "integrator treats it implicitly and is supported by MuJoCo Warp.",
                       {r.joint});
        i.fixLabel = "Use implicitfast";
        i.fix = [](Document& d) { d.option.integrator = Integrator::ImplicitFast; };
      }
      // The root joints of a floating model (planar walkers) move the whole body: not "passive".
      bool baseDof = doc_.body(j->body)->parent == kWorldId && base(j->body) != Base::Fixed;
      if (!r.actuated && j->damping == 0 && j->stiffness == 0 && j->frictionloss == 0 && r.movedMass > 0 &&
          !constrainedJoints.count(r.joint) && !baseDof) {
        add(Severity::Info, IssueArea::Physics, "passive-joint", fmt("Joint %s is passive and undamped", jn.c_str()),
            "It has no actuator, damping or friction, so it will swing freely. Add an actuator, or some damping if "
            "it's meant to be passive.",
            {r.joint});
      }
    }
  }

  std::vector<Id> actuatorsOf(Id joint, bool includeJoint) {
    std::vector<Id> out;
    if (includeJoint) out.push_back(joint);
    for (const Actuator& a : doc_.actuators)
      if (armOn(a.id, joint) != 0) out.push_back(a.id);
    return out;
  }

  // ---------------------------------------------------------------- start state
  void checkStartState() {
    // Floors: upward-facing planes of the world or of static bodies.
    double floorZ = -kInf;
    std::unordered_set<Id> floors;
    for (size_t bi = 0; bi < doc_.bodies.size(); ++bi) {
      if (weld_.weld[bi] != 0) continue;
      for (uint32_t gi : doc_.index().bodyGeoms[bi]) {
        const Geom& g = doc_.geoms[gi];
        if (g.type != GeomType::Plane) continue;
        Pose gp = poses_[bi] * Pose{g.pos, g.quat};
        if (rotate(gp.quat, {0, 0, 1}).z < 0.999) continue;  // walls and ceilings
        floors.insert(g.id);
        floorZ = std::max(floorZ, gp.pos.z);
      }
    }
    // Contacts MuJoCo finds at the start state, by geom pair (most penetrating distance).
    std::map<std::pair<Id, Id>, double> worst;
    if (compiledOk()) {
      for (const InitialContact& c : cr_->initialContacts) {
        if (c.dist > -1e-3) continue;
        if (c.geom1 < 0 || c.geom2 < 0 || size_t(c.geom1) >= cr_->map.geoms.size() || size_t(c.geom2) >= cr_->map.geoms.size()) continue;
        Id g1 = cr_->map.geoms[c.geom1], g2 = cr_->map.geoms[c.geom2];
        auto key = std::make_pair(std::min(g1, g2), std::max(g1, g2));
        worst[key] = std::min(worst.count(key) ? worst[key] : 0.0, c.dist);
      }
    }
    std::unordered_set<Id> floorChecked;
    for (const Body& root : doc_.bodies) {
      if (root.id == kWorldId || root.parent != kWorldId || root.role != Role::Model) continue;
      Base b = base(root.id);
      if (b == Base::Fixed || floorZ == -kInf) continue;
      double minZ = subtreeMinZ(doc_, poses_, root.id, true);
      if (!std::isfinite(minZ)) continue;
      floorChecked.insert(root.id);
      Id rid = root.id;
      double dz = minZ - floorZ;
      for (const auto& [pair, dist] : worst) {
        const Geom* a = doc_.geom(pair.first);
        const Geom* c = doc_.geom(pair.second);
        if (!a || !c) continue;
        if ((floors.count(a->id) && doc_.rootOf(c->body) == rid) || (floors.count(c->id) && doc_.rootOf(a->body) == rid))
          dz = std::min(dz, dist);
      }
      if (dz > 0.03) {
        Issue& i = add(Severity::Info, IssueArea::Start, "floating",
                       fmt("%s starts %.0f cm above the floor", quote(root.name).c_str(), dz * 100),
                       "It will fall at the beginning of every episode. Placing it on the floor makes resets cleaner.", {rid});
        i.fixLabel = "Place on floor";
        i.fix = [rid, dz](Document& d) { liftStartPose(d, rid, 0.002 - dz); };
      } else if (dz < -0.002) {
        Issue& i = add(Severity::Warning, IssueArea::Start, "below-floor",
                       fmt("%s starts %.1f cm inside the floor", quote(root.name).c_str(), -dz * 100),
                       "Penetrating contacts at the start pose create large repulsive forces (it gets launched).", {rid});
        i.fixLabel = "Place on floor";
        i.fix = [rid, dz](Document& d) { liftStartPose(d, rid, 0.002 - dz); };
      }
      // Balance: is the center of gravity above the support polygon of the points touching the floor?
      if (b == Base::Free && std::fabs(dz) < 0.01) {
        std::vector<Vec3> low;
        for (Id body : doc_.subtree(root.id)) {
          int bi = doc_.bodyIndex(body);
          for (uint32_t gi : doc_.index().bodyGeoms[bi]) {
            const Geom& g = doc_.geoms[gi];
            if (g.type != GeomType::Plane && geomCollides(g)) geomLowPoints(doc_, g, poses_[bi] * Pose{g.pos, g.quat}, floorZ + 0.01, low);
          }
        }
        std::vector<Vec2> pts;
        for (const Vec3& q : low) pts.push_back({q.x, q.y});
        WeightSum w = weightOf(root.id, {});
        std::vector<Vec2> hull = hull2d(pts);
        if (hull.size() >= 3 && w.w > 0) {
          Vec3 cog = w.moment / w.w;
          double d = polygonDistance(hull, {cog.x, cog.y});
          if (d > 0.005)
            add(Severity::Warning, IssueArea::Start, "tip-over",
                fmt("%s will tip over: its center of mass is %.1f cm outside the feet", quote(root.name).c_str(), d * 100),
                "The center of mass projected on the floor lies outside the polygon spanned by the contacts. Move the "
                "legs or the heavy parts so the COM sits above the feet (enable View > Centers of mass).",
                {rid});
        } else if (hull.size() < 3 && !pts.empty() && w.w > 0 && legsOf_[root.id] > 0) {
          add(Severity::Info, IssueArea::Start, "unstable-stance",
              fmt("%s touches the floor in fewer than 3 points", quote(root.name).c_str()),
              "With one or two contacts it cannot stand passively; it will fall over unless the policy balances it.", {rid});
        }
      }
    }
    // Other penetrations reported by MuJoCo at the start state.
    for (const auto& [pair, dist] : worst) {
      const Geom* a = doc_.geom(pair.first);
      const Geom* b = doc_.geom(pair.second);
      if (!a || !b || a == b) continue;  // a geom paired with itself is reported as such
      if ((floors.count(a->id) && floorChecked.count(doc_.rootOf(b->body))) ||
          (floors.count(b->id) && floorChecked.count(doc_.rootOf(a->body))))
        continue;  // reported by the floor check above
      Issue& i = add(Severity::Warning, IssueArea::Start, "penetration",
                     fmt("%s and %s overlap by %.1f mm at the start", quote(a->name).c_str(), quote(b->name).c_str(), -dist * 1000),
                     "Overlapping geoms of different bodies are pushed apart violently on the first step. Move them apart, "
                     "or exclude contacts between the two bodies if they are adjacent parts of the same creature.",
                     {a->id, b->id});
      // An exclusion does not stop an explicit contact pair: those geoms have to be moved apart.
      bool explicitPair = false;
      for (const ContactPair& c : doc_.pairs)
        explicitPair |= (c.geom1 == a->id && c.geom2 == b->id) || (c.geom1 == b->id && c.geom2 == a->id);
      if (a->body != kWorldId && b->body != kWorldId && a->body != b->body && !explicitPair) {
        Id b1 = a->body, b2 = b->body;
        i.fixLabel = "Exclude contacts between the bodies";
        i.fix = [b1, b2](Document& d) { addExclude(d, b1, b2); };
      }
    }
  }

  // ---------------------------------------------------------------- MJWarp
  void checkMjwarp() {
    const OptionSettings& opt = doc_.option;
    if (opt.solver == Solver::PGS) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-pgs", "PGS solver is not supported by MuJoCo Warp",
                     "MJWarp implements the CG and Newton solvers. Newton is MuJoCo's default and usually fastest.");
      i.fixLabel = "Use Newton";
      i.fix = [](Document& d) { d.option.solver = Solver::Newton; };
    }
    if (opt.noslip_iterations > 0) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-noslip", "The noslip solver is not supported by MuJoCo Warp",
                     "Set noslip_iterations to 0. If feet slip, increase friction or use elliptic cones / impratio instead.");
      i.fixLabel = "Disable noslip";
      i.fix = [](Document& d) { d.option.noslip_iterations = 0; };
    }
    auto flagErr = [&](bool bad, const char* code, const char* title, const char* fixLabel, std::function<void(Document&)> fix) {
      if (!bad) return;
      Issue& i = add(Severity::Error, IssueArea::MJWarp, code, title, "MuJoCo Warp raises NotImplementedError for this flag.");
      i.fixLabel = fixLabel;
      i.fix = std::move(fix);
    };
    flagErr(!opt.midphase, "mjw-midphase", "Disabling the midphase flag is not supported by MuJoCo Warp", "Enable midphase",
            [](Document& d) { d.option.midphase = true; });
    flagErr(!opt.autoreset, "mjw-autoreset", "Disabling the autoreset flag is not supported by MuJoCo Warp", "Enable autoreset",
            [](Document& d) { d.option.autoreset = true; });
    flagErr(opt.override_, "mjw-override", "The override flag is not supported by MuJoCo Warp", "Disable override",
            [](Document& d) { d.option.override_ = false; });
    flagErr(opt.fwdinv, "mjw-fwdinv", "The fwdinv flag is not supported by MuJoCo Warp", "Disable fwdinv",
            [](Document& d) { d.option.fwdinv = false; });
    int nv = 0;
    for (const Body& b : doc_.bodies) nv += doc_.bodyDofs(b.id);
    if (opt.jacobian == Jacobian::Dense && nv > 60) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-dense",
                     fmt("Dense Jacobian with %d DoF is not supported by MuJoCo Warp", nv),
                     "MJWarp supports dense Jacobians only up to 60 DoF. 'auto' picks sparse above 32 DoF.");
      i.fixLabel = "Use auto";
      i.fix = [](Document& d) { d.option.jacobian = Jacobian::Auto; };
    }
    // Measured with mujoco-warp 3.16: actuator armature is silently ignored (trajectories differ
    // from MuJoCo C). The joint / tendon armature it stands for gives the same dynamics.
    std::vector<Id> withArmature;
    // (Not on a tendon that wraps around a geom: MuJoCo refuses armature there, reported above.)
    auto wrapping = [&](Id tendon) {
      const Tendon* t = doc_.tendon(tendon);
      return t && std::any_of(t->path.begin(), t->path.end(), [](const TendonPoint& p) { return p.type == TendonPoint::Type::Geom; });
    };
    for (const Actuator& a : doc_.actuators)
      if (a.armature > 0 && a.type != ActuatorType::Adhesion &&
          (((a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) && doc_.joint(a.target)) ||
           (a.trn == TrnType::Tendon && doc_.tendon(a.target) && !wrapping(a.target))))
        withArmature.push_back(a.id);
    if (!withArmature.empty()) {
      Issue& i = add(Severity::Warning, IssueArea::MJWarp, "mjw-actuator-armature",
                     fmt("MuJoCo Warp ignores the armature of %zu actuator(s)", withArmature.size()),
                     "Actuator armature adds armature * gear^2 to the driven joint or tendon in MuJoCo, but MuJoCo Warp 3.16 "
                     "leaves it out, so trained policies see lighter, livelier joints than the preview. Putting the same "
                     "inertia on the joint (or tendon) works in both.",
                     withArmature);
      i.fixLabel = "Move it to the joints / tendons";
      i.fix = [withArmature](Document& d) {
        for (Id id : withArmature) {
          Actuator* a = d.actuator(id);
          if (!a) continue;
          double add = a->armature * a->gear[0] * a->gear[0];
          if (Joint* j = a->trn == TrnType::Tendon ? nullptr : d.joint(a->target)) j->armature += add;
          else if (Tendon* t = d.tendon(a->target)) t->armature += add;
          a->armature = 0;
        }
      };
    }
    if (opt.disableactuator != 0) {
      std::vector<Id> affected;
      for (const Actuator& a : doc_.actuators)
        if (a.group >= 0 && a.group < 31 && (opt.disableactuator & (1 << a.group))) affected.push_back(a.id);
      if (!affected.empty()) {
        Issue& i = add(Severity::Warning, IssueArea::MJWarp, "mjw-actuatorgroupdisable",
                       fmt("MuJoCo Warp ignores disabled actuator groups (%zu actuator(s) would still act)", affected.size()),
                       "MuJoCo turns these actuators off (option actuatorgroupdisable), MuJoCo Warp 3.16 does not read the "
                       "setting, so trained policies would see different dynamics. Remove the actuators instead, or zero "
                       "their controls in your environment.", affected);
        i.fixLabel = "Enable all actuator groups";
        i.fix = [](Document& d) { d.option.disableactuator = 0; };
      }
    }
    if (opt.sleep && opt.solver != Solver::Newton) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-sleep", "Sleeping requires the Newton solver in MuJoCo Warp", "");
      i.fixLabel = "Use Newton";
      i.fix = [](Document& d) { d.option.solver = Solver::Newton; };
    }
    // Measured with mujoco-warp 3.16: put_model refuses explicit tree sleep policies (never, allowed,
    // init), whether sleeping is enabled or not; only the automatic one is supported.
    std::vector<Id> policies;
    for (const Body& b : doc_.bodies)
      if (b.sleep != SleepPolicy::Auto && doc_.isTreeRoot(b.id)) policies.push_back(b.id);
    if (!policies.empty()) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-sleep-policy",
                     fmt("MuJoCo Warp does not support the sleep policy of %zu body(s)", policies.size()),
                     "MuJoCo Warp 3.16 refuses explicit sleep policies (never, allowed, start asleep) even with sleeping "
                     "off; it supports only Auto.",
                     policies);
      i.fixLabel = "Use Auto";
      i.fix = [policies](Document& d) {
        for (Id id : policies)
          if (Body* b = d.body(id)) b->sleep = SleepPolicy::Auto;
      };
    }
    // Box/mesh pairs with margins are rejected when (multi/native)CCD is on.
    if (opt.multiccd || opt.nativeccd) {
      std::vector<Id> offenders;
      auto explicitPair = [&](Id a, Id b) {
        for (const ContactPair& c : doc_.pairs)
          if ((c.geom1 == a && c.geom2 == b) || (c.geom1 == b && c.geom2 == a)) return true;
        return false;
      };
      const DocIndex& ix = doc_.index();
      ExclSet excl = excludeSet(doc_);
      for (size_t b1 = 0; b1 < doc_.bodies.size(); ++b1)
        for (uint32_t i1 : ix.bodyGeoms[b1]) {
          const Geom& g1 = doc_.geoms[i1];
          if (g1.type != GeomType::Box && g1.type != GeomType::Mesh) continue;
          for (size_t b2 = 0; b2 < doc_.bodies.size(); ++b2)
            for (uint32_t i2 : ix.bodyGeoms[b2]) {
              if (i2 <= i1) continue;
              const Geom& g2 = doc_.geoms[i2];
              if (g2.type != GeomType::Box && g2.type != GeomType::Mesh) continue;
              if (g1.margin == 0 && g2.margin == 0) continue;
              if (!opt.multiccd && !(g1.type == GeomType::Box && g2.type == GeomType::Box)) continue;
              if (!pairPasses(doc_, weld_, excl, g1, int(b1), g2, int(b2))) continue;
              if (explicitPair(g1.id, g2.id)) continue;  // the pair's own margin counts
              if (g1.margin != 0) offenders.push_back(g1.id);
              if (g2.margin != 0) offenders.push_back(g2.id);
            }
        }
      // Explicit pairs are checked with their own margin.
      for (const ContactPair& c : doc_.pairs) {
        const Geom* g1 = doc_.geom(c.geom1);
        const Geom* g2 = doc_.geom(c.geom2);
        if (!g1 || !g2 || c.margin == 0) continue;
        bool box1 = g1->type == GeomType::Box, box2 = g2->type == GeomType::Box;
        if ((!box1 && g1->type != GeomType::Mesh) || (!box2 && g2->type != GeomType::Mesh)) continue;
        if (opt.multiccd || (box1 && box2)) offenders.push_back(c.id);
      }
      std::sort(offenders.begin(), offenders.end());
      offenders.erase(std::unique(offenders.begin(), offenders.end()), offenders.end());
      if (!offenders.empty()) {
        Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-box-margin",
                       fmt("%zu box geom(s) or contact pair(s) with a margin can make box contacts", offenders.size()),
                       "With multiccd / nativeccd enabled (the default), MuJoCo Warp rejects box-box and box-mesh contacts "
                       "with a nonzero margin. Set the margin to 0, or disable multiccd and nativeccd in Scene > Flags.",
                       offenders);
        i.fixLabel = "Set those margins to 0";
        i.fix = [offenders](Document& d) {
          for (Id id : offenders) {
            if (Geom* g = d.geom(id)) g->margin = 0;
            if (ContactPair* c = d.pair(id)) c->margin = 0;
          }
        };
      }
    }
    // MuJoCo Warp warns that friction coefficients below mjMINMU in the directions a contact uses
    // (sliding from condim 3, torsional from 4, rolling at 6) may produce NaN.
    auto lowFriction = [](int condim, const std::array<double, 5>& f, std::initializer_list<std::pair<int, int>> checks) {
      for (auto [minCondim, idx] : checks)
        if (condim >= minCondim && f[size_t(idx)] < 1e-5) return true;
      return false;
    };
    std::vector<Id> slippery;
    for (const Geom& g : doc_.geoms)
      if ((g.contype || g.conaffinity) &&
          lowFriction(g.condim, {g.friction.x, g.friction.y, g.friction.z, 0, 0}, {{3, 0}, {4, 1}, {6, 2}}))
        slippery.push_back(g.id);
    for (const ContactPair& c : doc_.pairs) {
      const Vec5& f = c.friction;
      if (lowFriction(c.condim, {f[0], f[1], f[2], f[3], f[4]}, {{3, 0}, {4, 1}, {4, 2}, {6, 3}, {6, 4}})) slippery.push_back(c.id);
    }
    if (!slippery.empty())
      add(Severity::Warning, IssueArea::MJWarp, "mjw-friction",
          fmt("%zu geom(s) or contact pair(s) have a friction coefficient below 1e-5", slippery.size()),
          "MuJoCo Warp warns that a zero friction in a direction the contact uses (sliding with condim 3, torsional "
          "with 4, rolling with 6) may produce NaN. Lower the contact dimension instead, e.g. condim 1 for "
          "frictionless contacts.",
          slippery);
  }

  // ---------------------------------------------------------------- collisions
  void analyzeCollisions() {
    const DocIndex& ix = doc_.index();
    ExclSet excl = excludeSet(doc_);
    struct G {
      const Geom* g;
      int body;
      Pose pose;
      Aabb box;
      double rb;
      Id root;
    };
    std::vector<G> geoms;
    geoms.reserve(doc_.geoms.size());
    for (size_t bi = 0; bi < doc_.bodies.size(); ++bi)
      for (uint32_t gi : ix.bodyGeoms[bi]) {
        const Geom& g = doc_.geoms[gi];
        if (!g.contype && !g.conaffinity) continue;
        G e;
        e.g = &g;
        e.body = int(bi);
        e.pose = poses_[bi] * Pose{g.pos, g.quat};
        e.box = geomWorldAabb(doc_, g, e.pose);
        e.rb = rbound(doc_, g);
        e.root = doc_.rootOf(doc_.bodies[bi].id);
        geoms.push_back(e);
      }
    std::map<std::pair<int, int>, int> types;
    for (size_t i = 0; i < geoms.size(); ++i) {
      for (size_t k = i + 1; k < geoms.size(); ++k) {
        const G& a = geoms[i];
        const G& b = geoms[k];
        if (!pairPasses(doc_, weld_, excl, *a.g, a.body, *b.g, b.body)) continue;
        ++rep_.collisionPairs;
        if (a.root == b.root && a.root != kWorldId) ++rep_.selfCollisionPairs;
        int ta = int(a.g->type), tb = int(b.g->type);
        types[{std::min(ta, tb), std::max(ta, tb)}]++;
        // MJWarp broadphase at rest: plane distance or bounding sphere + box overlap.
        // 1 cm slack: count pairs that are touching or about to (e.g. feet resting on the floor).
        double margin = a.g->margin + a.g->gap + b.g->margin + b.g->gap + 0.01;
        bool cand;
        if (a.rb == 0 || b.rb == 0) {
          const G& pl = a.rb == 0 ? a : b;
          const G& ot = a.rb == 0 ? b : a;
          Vec3 n = rotate(pl.pose.quat, {0, 0, 1});
          cand = dot(ot.pose.pos - pl.pose.pos, n) <= ot.rb + margin;
        } else {
          double bound = a.rb + b.rb + margin;
          cand = length2(a.pose.pos - b.pose.pos) <= bound * bound;
          if (cand) {
            for (int ax = 0; ax < 3 && cand; ++ax)
              cand = a.box.lo[ax] - margin <= b.box.hi[ax] && b.box.lo[ax] - margin <= a.box.hi[ax];
          }
        }
        if (cand) ++rep_.broadphaseAtRest;
      }
    }
    std::vector<std::string> ccdNames;
    int ccdPairs = 0;
    for (const auto& [t, n] : types) {
      PairTypeStat s;
      s.a = GeomType(t.first);
      s.b = GeomType(t.second);
      s.count = n;
      // With nativeccd off, MJWarp collides boxes with its primitive box-box collider.
      s.ccd = isCcdPair(s.a, s.b) && !(s.a == GeomType::Box && s.b == GeomType::Box && !doc_.option.nativeccd);
      rep_.pairTypes.push_back(s);
      if (s.ccd) {
        ccdPairs += n;
        ccdNames.push_back(fmt("%s-%s (%d)", toXml(s.a), toXml(s.b), n));
      }
    }
    if (!ccdNames.empty()) {
      std::string list;
      for (size_t i = 0; i < ccdNames.size(); ++i) list += (i ? ", " : "") + ccdNames[i];
      add(Severity::Tip, IssueArea::Performance, "mjw-ccd",
          fmt("%d potential contact pair(s) use MJWarp's slower convex (CCD) collider", ccdPairs),
          "Pairs: " + list + ". Primitive colliders are fastest in MuJoCo Warp: plane against anything; sphere with "
          "sphere/capsule/cylinder/box; capsule with capsule/box. Ellipsoids, cylinders against non-spheres, box-box "
          "and meshes need CCD (more memory, longer kernel compilation). Use capsules/spheres for limbs and feet, or "
          "disable contacts you don't need (contype/conaffinity).");
    }
    if (rep_.selfCollisionPairs > 0) {
      add(Severity::Tip, IssueArea::Performance, "self-collision",
          fmt("%d self-collision pair(s) inside models", rep_.selfCollisionPairs),
          "Geoms of the same creature that are not parent/child can collide with each other. That is sometimes "
          "needed (legs hitting each other), but each pair costs broadphase/narrowphase work for every world. "
          "A common RL setup: contype=0/conaffinity=0 on limbs and enable contacts only for the feet, or add "
          "contact exclusions for pairs that never touch.");
    }
  }

  const Document& doc_;
  const CompileResult* cr_;
  bool crValid_;
  AnalysisReport rep_;
  std::vector<Pose> poses_;
  std::vector<JointFrame> jointFrames_;  // at the start pose, indexed like doc.joints
  Weld weld_;
  double gmag_ = 9.81;
  Vec3 gdir_{0, 0, -1};
  InertialTable inertials_;           // compiled body inertials, by body ID (empty without a compile)
  std::vector<MassProps> bodyMass_;    // per body index: mass, world COM, inertia
  std::vector<double> bodyWeight_;     // per body index: mass not carried by gravity compensation
  std::unordered_map<Id, Base> base_;  // per model root
  std::unordered_map<Id, MassAccumulator> rootAcc_;  // floating roots: the whole model
  std::unordered_map<Id, std::vector<Arm>> arms_;    // actuator -> hinge/slide joints it drives
  std::unordered_map<Id, int> compiledActuator_;     // actuator -> index in the compiled model
  std::unordered_map<Id, double> servoStiffness_, actDamping_, passiveDamping_;
  std::unordered_map<Id, std::string> supportCtx_;
  std::unordered_map<Id, int> legsOf_;  // per model root
  std::unordered_set<Id> bigPart_;      // joints that move a large part of a floating model
  std::unordered_map<Id, double> footReach_;  // leg joints: the load with the weight at the edge of the foot
};

}  // namespace

double actuatorMaxForce(const Document& doc, const Actuator& a) {
  auto absmax = [](const Vec2& r) { return std::max(std::fabs(r.x), std::fabs(r.y)); };
  bool limited = false;
  Vec2 ctrl = servoCtrlRange(doc, a, &limited);
  double umax = limited ? absmax(ctrl) : kInf;
  double f = kInf;
  switch (a.type) {
    case ActuatorType::Motor: f = umax; break;
    case ActuatorType::Position:
    case ActuatorType::IntVelocity: f = a.kp * servoSpan(doc, a); break;
    case ActuatorType::Velocity: f = a.kv * umax * 2.0; break;
    case ActuatorType::Damper: f = 0; break;  // cannot hold a load
    case ActuatorType::Cylinder: {
      double area = a.diameter > 0 ? kPi / 4 * a.diameter * a.diameter : a.area;  // diameter wins in MuJoCo
      f = area * umax + std::fabs(a.cylBias.x);
      break;
    }
    case ActuatorType::Muscle: f = a.muscleForce > 0 ? a.muscleForce : kInf; break;
    case ActuatorType::Adhesion: f = a.adhesionGain * (a.ctrllimited ? absmax(a.ctrlrange) : 1.0); break;
    case ActuatorType::General: {
      if (a.gaintype != GainType::Fixed) break;
      double g = std::fabs(a.gainprm[0]);
      f = (positionLikeGeneral(a) ? g * servoSpan(doc, a) : g * umax) + std::fabs(a.biasprm[0]);
      break;
    }
    case ActuatorType::DcMotor: {
      // Stall torque at the largest voltage: the control for a voltage-driven motor, the drive limit
      // of the on-board controller otherwise (none: it delivers whatever it is asked).
      double K, R;
      dcMotorConstants(a, &K, &R);
      if (!(K > 0 && R > 0)) return kInf;  // not specified yet (reported by dcmotor-constants): unknown, not weak
      double v = a.dcinput == DcInput::Voltage ? umax : a.controller[5] > 0 ? a.controller[5] : kInf;
      f = K * v / R;
      double cap = a.saturation.x > 0 ? a.saturation.x : a.saturation.y > 0 ? K * a.saturation.y : kInf;
      return std::min(f, cap);
    }
  }
  if (a.forcelimited) f = std::min(f, absmax(a.forcerange));
  return f;
}

std::string explainCompileError(const std::string& e) {
  struct M { const char* key; const char* text; };
  static const M table[] = {
      {"free joint can only be used on top level",
       "A free (6-DoF) joint must be on a body that is a direct child of the world. Move the body to the world, or "
       "use hinge/slide/ball joints."},
      {"plane only allowed in static bodies", "Planes are infinite and can't move. Use a thin box for moving parts."},
      {"mass and inertia of moving bodies must be larger than mjMINVAL",
       "A body that can move has no mass. Give it (or a body below it) a geom with volume, or an explicit inertial."},
      {"kv and dampratio cannot both be defined", "A position servo sets both kv and damping ratio: keep only one."},
      {"damper control range cannot be negative", "Damper actuators need a non-negative control range, e.g. [0, 1]."},
      {"must be positive in geom", "A geom has a zero size."},
      {"range[0] should be 0 in ball joint", "Ball joint ranges are [0, maximum angle]."},
      {"ball followed by rotation", "A body has a hinge or ball joint after a ball joint; put the ball joint last."},
      {"more than 6 dofs", "A body has more than 6 degrees of freedom."},
      {"is not a fixed child of world", "Mocap bodies must be children of the world without joints."},
      {"setting delay > 0 without a history buffer", "A delayed actuator or sensor needs a history buffer (nsample)."},
      {"samples in sensor history buffer", "A sensor keeps more history samples than MuJoCo allows (2^24)."},
      {"samples in history buffer", "An actuator keeps more history samples than MuJoCo allows (2^24)."},
      {"repeated name", "Two elements of the same type share a name."},
      {"inertia must satisfy A + B >= C",
       "An explicit inertia is impossible for a rigid body: each principal inertia must be at most the sum of the "
       "other two."},
      {"is repeated in equality constraint", "An equality constraint uses the same element on both sides."},
      {"1st body/geom must be different from 2nd", "A distance sensor measures a geom or body against itself."},
      {"requires joint or tendon transmission", "Actuator damping and armature need a joint or tendon transmission."},
      {"inheritrange", "An actuator inherits the range of a joint or tendon that has none, or of the wrong kind."},
      {"Lengthrange computation did not converge",
       "MuJoCo could not find a muscle's length range: its joint or tendon can move without limit. Limit it, or set "
       "the muscle's length range."},
      {"Invalid lengthrange", "A muscle's joint or tendon does not change length, so it has no length range."},
      {"fovy too large in camera", "A camera's fovy is 180 or more (for an orthographic camera, its view height in meters)."},
      {"geom wrapping not supported", "An actuator adds armature to a tendon that wraps around a geom, which MuJoCo does not support."},
      {"joint must be slide or hinge in sensor",
       "Joint position and velocity sensors need a hinge or slide joint; ball joints use ball sensors, free joints "
       "frame sensors."},
      {"unrecognized attribute", "The XML uses an attribute this MuJoCo version doesn't know (version mismatch?)."},
  };
  for (const M& m : table)
    if (e.find(m.key) != std::string::npos) return m.text;
  return "";
}

bool geomPairCollides(const Document& doc, const Geom& g1, const Geom& g2) {
  Weld w = computeWeld(doc);
  ExclSet excl = excludeSet(doc);
  int b1 = doc.bodyIndex(g1.body), b2 = doc.bodyIndex(g2.body);
  if (b1 < 0 || b2 < 0) return false;
  return pairPasses(doc, w, excl, g1, b1, g2, b2);
}

AnalysisReport analyzeModel(const Document& doc, const CompileResult* compiled, bool compiledValid) {
  Analyzer a(doc, compiled, compiledValid);
  return a.run();
}

}  // namespace mjf
