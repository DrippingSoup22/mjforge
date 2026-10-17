#include "analysis/Analysis.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <map>
#include <set>
#include <unordered_map>

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
    default: return 0;
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

// Moves a model's start pose up by dz: its free joint's value in the first keyframe if it has one,
// otherwise the body itself.
void liftStartPose(Document& d, Id root, double dz) {
  if (!d.keyframes.empty())
    for (const Joint& j : d.joints)
      if (j.body == root && j.type == JointType::Free)
        for (auto& [jid, q] : d.keyframes[0].qpos)
          if (jid == j.id && q.size() == 7) {
            q[2] += dz;
            return;
          }
  if (Body* b = d.body(root)) b->pos.z += dz;
}

class Analyzer {
 public:
  Analyzer(const Document& d, const CompileResult* cr, bool crValid) : doc_(d), cr_(cr), crValid_(crValid) {}

  AnalysisReport run() {
    // Everything is evaluated at the start pose: the first keyframe if there is one.
    poses_ = computeBodyPoses(doc_, doc_.keyframes.empty() ? nullptr : &doc_.keyframes[0]);
    weld_ = computeWeld(doc_);
    Vec3 g = doc_.option.gravity;
    gmag_ = doc_.option.gravityFlag ? length(g) : 0.0;
    gdir_ = normalize(g, {0, 0, -1});
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
        merged.fixLabel = "Fix all " + std::to_string(fixes.size());
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
    dupes(doc_.actuators, Kind::Actuator);
    dupes(doc_.sensors, Kind::Sensor);

    for (const Actuator& a : doc_.actuators) {
      bool ok = false;
      switch (a.trn) {
        case TrnType::Joint:
        case TrnType::JointInParent: ok = doc_.joint(a.target) != nullptr; break;
        case TrnType::Site: ok = doc_.site(a.target) != nullptr; break;
        case TrnType::Body: ok = doc_.body(a.target) != nullptr && a.target != kWorldId; break;
        case TrnType::Tendon: ok = false; break;
        case TrnType::SliderCrank: ok = doc_.site(a.target) && doc_.site(a.slidersite); break;
      }
      if (!ok) {
        add(Severity::Error, IssueArea::Model, "actuator-target", fmt("Actuator %s has no valid target", quote(a.name).c_str()),
            "Pick the joint (or site/body) this actuator drives in the inspector.", {a.id});
        continue;
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
        const Joint* j = doc_.joint(a.target);
        Vec2 r = (servo && j && j->limited) ? j->range : Vec2{-1, 1};
        i.fixLabel = servo && j && j->limited ? "Use the joint range" : "Limit to [-1, 1]";
        i.fix = [id, r](Document& d) { if (Actuator* x = d.actuator(id)) { x->ctrllimited = true; x->ctrlrange = r; } };
      }
      if (a.ctrllimited && a.ctrlrange.x > a.ctrlrange.y)
        add(Severity::Error, IssueArea::Model, "ctrl-range-order", fmt("Actuator %s: control range lower > upper", quote(a.name).c_str()), "", {a.id});
      if (a.forcelimited && a.forcerange.x > a.forcerange.y)
        add(Severity::Error, IssueArea::Model, "force-range-order", fmt("Actuator %s: force range lower > upper", quote(a.name).c_str()), "", {a.id});
      const Joint* tj = a.trn == TrnType::Joint ? doc_.joint(a.target) : nullptr;
      bool degrees = servo && a.ctrllimited && tj && tj->type == JointType::Hinge &&
                     std::max(std::fabs(a.ctrlrange.x), std::fabs(a.ctrlrange.y)) > 2 * kPi + 0.1;
      if (degrees) {
        // MuJoCo converts joint ranges from degrees (compiler angle="degree") but never ctrlrange.
        Issue& i = add(Severity::Warning, IssueArea::Physics, "ctrl-range-degrees",
                       fmt("Servo %s: control range [%.4g, %.4g] looks like degrees", quote(a.name).c_str(),
                           a.ctrlrange.x, a.ctrlrange.y),
                       "Servo targets are joint angles in radians. MuJoCo converts joint ranges written in degrees "
                       "(compiler angle=\"degree\") but never converts ctrlrange, a common mistake in imported files.",
                       {a.id});
        Id id = a.id;
        i.fixLabel = "Convert to radians";
        i.fix = [id](Document& d) {
          if (Actuator* x = d.actuator(id)) x->ctrlrange = {deg2rad(x->ctrlrange.x), deg2rad(x->ctrlrange.y)};
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
    for (const Sensor& s : doc_.sensors) {
      SensorObj o = sensorInfo(s.type).obj;
      bool ok = true;
      switch (o) {
        case SensorObj::Joint: ok = doc_.joint(s.obj) != nullptr; break;
        case SensorObj::Actuator: ok = doc_.actuator(s.obj) != nullptr; break;
        case SensorObj::Site: ok = doc_.site(s.obj) != nullptr; break;
        case SensorObj::Body: ok = doc_.body(s.obj) != nullptr; break;
        case SensorObj::Frame: ok = doc_.exists(s.obj); break;
        case SensorObj::Tendon: ok = false; break;
        case SensorObj::None: break;
      }
      if (!ok) add(Severity::Error, IssueArea::Model, "sensor-target", fmt("Sensor %s has no valid target", quote(s.name).c_str()),
                   "Choose what this sensor is attached to in the inspector.", {s.id});
    }
    for (const Exclude& e : doc_.excludes)
      if (!doc_.body(e.body1) || !doc_.body(e.body2))
        add(Severity::Error, IssueArea::Model, "exclude-target", "Contact exclusion refers to a missing body", "", {e.id});

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
      if (j.limited && (j.type == JointType::Hinge || j.type == JointType::Slide) && j.range.x >= j.range.y) {
        add(Severity::Error, IssueArea::Model, "joint-range", fmt("Joint %s: lower limit is not below the upper limit", quote(j.name).c_str()),
            "MuJoCo accepts this but the joint cannot move. Set range = [min, max].", {j.id});
      }
      if (j.type == JointType::Ball && j.limited && j.range.x != 0) {
        Issue& i = add(Severity::Error, IssueArea::Model, "ball-range", fmt("Ball joint %s: range must start at 0", quote(j.name).c_str()),
                       "Ball joint ranges are [0, maximum cone angle].", {j.id});
        Id id = j.id;
        i.fixLabel = "Set lower bound to 0";
        i.fix = [id](Document& d) { if (Joint* x = d.joint(id)) x->range.x = 0; };
      }
    }
    for (const Geom& g : doc_.geoms) {
      int n = g.type == GeomType::Sphere ? 1 : (g.type == GeomType::Capsule || g.type == GeomType::Cylinder) ? 2 : 3;
      if (g.type == GeomType::Plane || g.type == GeomType::Hfield) n = 0;
      bool bad = false;
      for (int i = 0; i < n; ++i) bad |= !(g.size[i] > 0);
      if (bad) add(Severity::Error, IssueArea::Model, "geom-size", fmt("Geom %s has a zero or negative size", quote(g.name).c_str()), "", {g.id});
      if (g.type == GeomType::Plane && g.body != kWorldId && weld_.weld[doc_.bodyIndex(g.body)] != 0) {
        add(Severity::Error, IssueArea::Model, "plane-moving", fmt("Plane %s is on a moving body", quote(g.name).c_str()),
            "Planes are infinite and static. Use a thin box for moving parts.", {g.id});
      }
      if (g.type == GeomType::Hfield && !doc_.hfield(g.hfield))
        add(Severity::Error, IssueArea::Model, "hfield-missing", fmt("Height field geom %s has no height field data", quote(g.name).c_str()), "", {g.id});
    }
  }

  // ---------------------------------------------------------------- mass
  void checkMass() {
    double minM = kInf, maxM = 0;
    Id minId = kNoId, maxId = kNoId;
    for (size_t bi = 1; bi < doc_.bodies.size(); ++bi) {
      const Body& b = doc_.bodies[bi];
      if (doc_.roleOf(b.id) == Role::Model) rep_.modelMass += bodyMassProps(doc_, poses_, b.id).mass;
      if (weld_.weld[bi] != int(bi) || weld_.dof[bi] == 0) continue;  // only moving weld roots
      // Mass of this rigid group: the body and all bodies welded to it.
      double m = 0;
      for (Id sb : doc_.subtree(b.id)) {
        int si = doc_.bodyIndex(sb);
        if (weld_.weld[si] == int(bi)) m += bodyMassProps(doc_, poses_, sb).mass;
      }
      double subtree = subtreeMassProps(doc_, poses_, b.id).mass;
      if (subtree <= 1e-12) {
        add(Severity::Error, IssueArea::Model, "massless", fmt("Body %s can move but has no mass", quote(b.name).c_str()),
            "It has joints but no geoms with volume below it. Add a geom, or an explicit inertial.", {b.id});
        continue;
      }
      if (m > 0 && m < minM) { minM = m; minId = b.id; }
      if (m > maxM) { maxM = m; maxId = b.id; }
      if (m > 0 && m < 1e-3) {
        bool hasArm = false;
        for (uint32_t ji : doc_.index().bodyJoints[bi]) hasArm |= doc_.joints[ji].armature > 0;
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
  }

  // ---------------------------------------------------------------- joints
  void analyzeJoints() {
    for (const Joint& j : doc_.joints) {
      if (j.type != JointType::Hinge && j.type != JointType::Slide) continue;
      if (!doc_.body(j.body)) continue;
      JointReport r;
      r.joint = j.id;
      r.angular = j.type == JointType::Hinge;
      MassAccumulator acc = subtreeAccumulator(doc_, poses_, j.body);
      Vec3 p, a;
      jointWorldFrame(doc_, poses_, j, &p, &a);
      r.movedMass = acc.mass();
      double base = r.angular ? acc.inertiaAboutAxis(p, a) : acc.mass();
      r.inertia = base + j.armature;
      if (r.angular) {
        Vec3 rr = acc.com() - p;
        Vec3 rperp = rr - a * dot(a, rr);
        double h = length(cross(a, gdir_));
        r.axisTilt = h;
        r.liftLoad = r.movedMass * gmag_ * length(rperp) * std::max(h, 0.25);
      } else {
        r.axisTilt = std::fabs(dot(a, gdir_));
        r.liftLoad = r.movedMass * gmag_ * std::max(r.axisTilt, 0.25);
      }
      double kServo = 0, cAct = 0;
      for (const Actuator& act : doc_.actuators) {
        if ((act.trn != TrnType::Joint && act.trn != TrnType::JointInParent) || act.target != j.id) continue;
        r.actuated = true;
        double gear = std::fabs(act.gear[0]);
        double f = actuatorMaxForce(doc_, act);
        if (std::isinf(f)) r.unbounded = true;
        r.capacity += gear * f;
        double g2 = gear * gear;
        if (act.type == ActuatorType::Position || act.type == ActuatorType::IntVelocity) {
          kServo += act.kp * g2;
          if (act.dampratio > 0) cAct += act.dampratio * 2.0 * std::sqrt(act.kp * g2 * std::max(r.inertia, 1e-12));
          else cAct += act.kv * g2;
        } else if (act.type == ActuatorType::Velocity) {
          cAct += act.kv * g2;
        } else if (act.type == ActuatorType::General && act.biastype == BiasType::Affine) {
          kServo += std::max(0.0, -act.biasprm[1]) * g2;
          cAct += std::max(0.0, -act.biasprm[2]) * g2;
        }
      }
      if (j.actuatorfrclimited)
        r.capacity = std::min(r.capacity, std::max(std::fabs(j.actuatorfrcrange.x), std::fabs(j.actuatorfrcrange.y)));
      r.stiffness = kServo + j.stiffness;
      r.damping = cAct + j.damping;
      double dt = doc_.option.timestep;
      if (r.inertia > 0 && r.stiffness > 0) r.omegaDt = std::sqrt(r.stiffness / r.inertia) * dt;
      servoStiffness_[j.id] = kServo;
      actDamping_[j.id] = cAct;
      rep_.joints.push_back(r);
    }
  }

  // Legs: chains from a model root down to "feet" (bodies at the bottom of the model).
  void analyzeLegs() {
    for (const Body& root : doc_.bodies) {
      if (root.id == kWorldId || root.parent != kWorldId || root.role != Role::Model) continue;
      int ri = doc_.bodyIndex(root.id);
      bool floating = false;
      for (uint32_t ji : doc_.index().bodyJoints[ri]) floating |= doc_.joints[ji].type == JointType::Free;
      if (!floating) continue;  // fixed-base robots don't stand on legs
      Aabb box = subtreeAabb(doc_, poses_, root.id, true);  // feet must be able to touch the ground
      if (!box.valid()) continue;
      double height = box.hi.z - box.lo.z;
      double tol = std::max(0.01, 0.06 * height);
      // Feet: bodies whose own geoms reach the bottom of the model.
      struct Foot { int body; Vec3 point; int leg; };
      std::vector<Foot> feet;
      for (Id b : doc_.subtree(root.id)) {
        if (b == root.id) continue;
        int bi = doc_.bodyIndex(b);
        Aabb own;
        for (uint32_t gi : doc_.index().bodyGeoms[bi]) {
          const Geom& g = doc_.geoms[gi];
          if (g.type == GeomType::Plane || !geomCollides(g)) continue;
          own.add(primitiveWorldAabb(g.type, g.size, poses_[bi] * Pose{g.pos, g.quat}));
        }
        if (!own.valid() || own.lo.z > box.lo.z + tol) continue;
        // The leg is identified by the root's child that leads to this foot.
        Id cur = b;
        while (doc_.body(cur)->parent != root.id) cur = doc_.body(cur)->parent;
        feet.push_back({bi, {own.center().x, own.center().y, own.lo.z}, doc_.bodyIndex(cur)});
      }
      if (feet.empty()) continue;
      std::set<int> legs;
      for (const Foot& f : feet) legs.insert(f.leg);
      int nLegs = int(legs.size());
      rep_.legCount += nLegs;
      double mass = subtreeMassProps(doc_, poses_, root.id).mass;
      double W = mass * gmag_;
      double wStatic = W / nLegs;
      double wDyn = W / std::max(1.0, nLegs / 2.0);  // about half the feet are on the ground while moving
      legInfo_.push_back({root.id, nLegs, mass});
      // One representative foot per leg: the lowest.
      std::map<int, Foot> perLeg;
      for (const Foot& f : feet) {
        auto it = perLeg.find(f.leg);
        if (it == perLeg.end() || f.point.z < it->second.point.z) perLeg[f.leg] = f;
      }
      for (const auto& [leg, foot] : perLeg) {
        // Joints on the chain from the foot up to the leg's first body.
        int bi = foot.body;
        while (true) {
          for (uint32_t ji : doc_.index().bodyJoints[bi]) {
            const Joint& j = doc_.joints[ji];
            JointReport* r = findReport(j.id);
            if (!r) continue;
            Vec3 p, a;
            jointWorldFrame(doc_, poses_, j, &p, &a);
            Vec3 up = -gdir_;
            if (r->angular) {
              Vec3 rr = foot.point - p;
              double leverRef = std::fabs(dot(a, cross(rr, up)));
              double lperp = length(rr - a * dot(a, rr));
              double h = length(cross(a, up));
              r->holdLoad = std::max(r->holdLoad, wStatic * leverRef);
              r->supportLoad = std::max(r->supportLoad, wDyn * std::max(leverRef, 0.35 * lperp * h));
            } else {
              double c = std::fabs(dot(a, up));
              r->holdLoad = std::max(r->holdLoad, wStatic * c);
              r->supportLoad = std::max(r->supportLoad, wDyn * c);
            }
            supportCtx_[j.id] = fmt("%.3g kg on %d leg%s", mass, nLegs, nLegs > 1 ? "s" : "");
          }
          if (bi == leg) break;
          bi = doc_.bodyIndex(doc_.bodies[bi].parent);
          if (bi <= 0) break;
        }
      }
      std::vector<int> footBodies;
      for (const auto& [leg, foot] : perLeg) footBodies.push_back(foot.body);
      if (nLegs <= 2) checkAnkleBalance(root, footBodies, box.lo.z + tol);
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
        flat |= primitiveWorldAabb(g.type, g.size, poses_[fb] * Pose{g.pos, g.quat}).lo.z <= floorLevel;
      }
      if (!flat) return;  // point-like feet: covered by the stance check
      MassProps fm = subtreeMassProps(doc_, poses_, doc_.bodies[fb].id);
      feet.add(fm.mass, fm.com, fm.inertia);
      for (uint32_t ji : doc_.index().bodyJoints[fb]) {
        const Joint& j = doc_.joints[ji];
        const JointReport* r = findReport(j.id);
        if (j.type != JointType::Hinge || !r) continue;
        Vec3 p, a;
        jointWorldFrame(doc_, poses_, j, &p, &a);
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
    MassProps all = subtreeMassProps(doc_, poses_, root.id);
    double mAbove = all.mass - feet.mass();
    if (mAbove <= 0) return;
    Vec3 comAbove = (all.com * all.mass - feet.com() * feet.mass()) * (1.0 / mAbove);
    for (const Group& g : groups) {
      Vec3 anchor = g.anchors * (1.0 / g.joints.size());
      double h = dot(comAbove - anchor, -gdir_);
      if (h <= 0) continue;
      double kGravity = mAbove * gmag_ * h;
      // Torque-controlled ankles (no servo or spring stiffness) are balanced by the policy.
      if (g.k <= 0 || g.k >= 1.5 * kGravity) continue;
      double kpEach = niceCeil(2.0 * kGravity / g.joints.size());
      Issue& i = add(Severity::Info, IssueArea::Physics, "ankle-balance",
                     fmt("%s can't stand still: its ankles are softer than gravity's tipping pull", quote(root.name).c_str()),
                     fmt("On its feet, the %.3g kg above the ankles (center of mass %.2f m up) is an inverted pendulum: "
                         "gravity tips it with %.3g N*m/rad while the ankle joints resist with %.3g N*m/rad. Holding "
                         "a pose it slowly falls over, so a policy must balance it actively. For a stance that holds "
                         "by itself, raise the ankle kp to about %.3g each.",
                         mAbove, h, kGravity, g.k, kpEach),
                     g.joints);
      std::vector<Id> acts;
      for (Id j : g.joints)
        for (Id a : actuatorsOf(j, false)) acts.push_back(a);
      if (acts.empty()) continue;
      i.fixLabel = fmt("Set ankle kp to %.3g", kpEach);
      i.fix = [acts, kpEach](Document& d) {
        for (Id id : acts)
          if (Actuator* a = d.actuator(id))
            if (a->type == ActuatorType::Position || a->type == ActuatorType::IntVelocity) a->kp = std::max(a->kp, kpEach);
      };
    }
  }

  JointReport* findReport(Id id) {
    for (JointReport& r : rep_.joints) if (r.joint == id) return &r;
    return nullptr;
  }

  void checkJointFindings() {
    const double dt = doc_.option.timestep;
    bool explicitDamping = doc_.option.integrator == Integrator::Euler || doc_.option.integrator == Integrator::RK4;
    for (JointReport& r : rep_.joints) {
      const Joint* j = doc_.joint(r.joint);
      if (r.stiffness > 0 && r.holdLoad > 0) r.sag = r.holdLoad / r.stiffness;
      const char* tu = r.angular ? "N*m" : "N";
      std::string jn = quote(j->name);
      // Strength.
      double need = r.required();
      if (r.actuated && !r.unbounded && need > 1e-9 && r.capacity < need) {
        double ratio = r.capacity / need;
        bool support = r.supportLoad >= r.liftLoad;
        std::string why = support ? fmt("to support the body while moving (%s, about half the feet on the ground)",
                                        supportCtx_[r.joint].c_str())
                                  : fmt("to lift the %.3g kg it moves against gravity", r.movedMass);
        Issue& i = add(ratio < 0.25 ? Severity::Error : Severity::Warning, IssueArea::Physics, "weak-actuator",
                       fmt("Joint %s is too weak: %.3g of ~%.3g %s needed", jn.c_str(), r.capacity, need, tu),
                       fmt("Its actuators can apply at most %.3g %s but it needs about %.3g %s %s. That is %s of "
                           "what is required, so the %s.",
                           r.capacity, tu, need, tu, why.c_str(),
                           ratio < 0.001 ? "under 0.1%" : fmt("%.1f%%", ratio * 100).c_str(),
                           ratio < 0.25 ? "creature will barely move" : "motion will be sluggish or it may collapse"),
                       actuatorsOf(r.joint, true));
        double target = niceCeil(1.5 * need);
        std::vector<Id> acts = actuatorsOf(r.joint, false);
        i.fixLabel = fmt("Make actuators %.3g %s strong", target, tu);
        i.fix = [acts, target](Document& d) {
          double share = target / std::max<size_t>(1, acts.size());
          for (Id id : acts) {
            Actuator* a = d.actuator(id);
            if (!a) continue;
            double gear = std::max(1e-9, std::fabs(a->gear[0]));
            double f = share / gear;
            switch (a->type) {
              case ActuatorType::Motor: {
                double umax = a->ctrllimited ? std::max(std::fabs(a->ctrlrange.x), std::fabs(a->ctrlrange.y)) : 1.0;
                a->gear[0] = niceCeil(share / std::max(umax, 1e-9));
                if (a->forcelimited) a->forcerange = {-share / a->gear[0], share / a->gear[0]};
                break;
              }
              case ActuatorType::Position:
              case ActuatorType::IntVelocity: {
                // Full force at ~0.25 rad (or 2.5 cm) tracking error.
                a->kp = std::max(a->kp, niceCeil(f / 0.25));
                if (a->forcelimited) a->forcerange = {-f, f};
                break;
              }
              default:
                if (a->forcelimited) a->forcerange = {-f, f};
                break;
            }
          }
        };
      }
      // Servo sag under the static body weight.
      double sagLimit = r.angular ? 0.15 : 0.02;
      if (r.actuated && servoStiffness_[r.joint] > 0 && r.sag > sagLimit && (r.capacity >= need || r.unbounded)) {
        Issue& i = add(Severity::Warning, IssueArea::Physics, "servo-sag",
                       fmt("Servo on %s is soft: sags ~%.3g%s under the body weight", jn.c_str(),
                           r.angular ? rad2deg(r.sag) : r.sag * 100, r.angular ? " deg" : " cm"),
                       fmt("Holding the body needs %.3g %s at this joint; with a stiffness of %.3g %s/%s the joint "
                           "deflects that much from its target. Raise kp for tighter tracking.",
                           r.holdLoad, tu, r.stiffness, tu, r.angular ? "rad" : "m"),
                       actuatorsOf(r.joint, true));
        std::vector<Id> acts = actuatorsOf(r.joint, false);
        double kpNew = niceCeil(r.holdLoad / (r.angular ? 0.05 : 0.005));
        i.fixLabel = fmt("Set kp to %.3g", kpNew);
        i.fix = [acts, kpNew](Document& d) {
          for (Id id : acts)
            if (Actuator* a = d.actuator(id))
              if (a->type == ActuatorType::Position || a->type == ActuatorType::IntVelocity) a->kp = std::max(a->kp, kpNew);
        };
      }
      // Very overpowered (only a tip).
      if (r.actuated && !r.unbounded && need > 1e-6 && r.axisTilt > 0.5 && r.capacity > 100 * need) {
        add(Severity::Tip, IssueArea::Physics, "strong-actuator",
            fmt("Joint %s is ~%.0fx stronger than its load", jn.c_str(), r.capacity / need),
            fmt("Up to %.3g %s for a load of ~%.3g %s. Fine for learning, but unrealistic forces can make policies "
                "rely on violent motions that don't transfer to hardware.",
                r.capacity, tu, need, tu),
            actuatorsOf(r.joint, true));
      }
      // Timestep stability of stiff servos / springs.
      if (r.omegaDt > 0.6) {
        bool explode = r.omegaDt > 2.0;
        Issue& i = add(explode ? Severity::Error : Severity::Warning, IssueArea::Stability, "stiff-servo",
                       fmt("Joint %s is too stiff for the timestep (w*dt = %.2f)", jn.c_str(), r.omegaDt),
                       fmt("Stiffness %.3g over an effective inertia of %.3g gives a natural frequency of %.3g rad/s. "
                           "With dt = %.4g s that is w*dt = %.2f; %s Increase armature, reduce kp, or reduce the timestep.",
                           r.stiffness, r.inertia, r.omegaDt / dt, dt, r.omegaDt,
                           explode ? "above 2 explicit integration diverges." : "keep it below ~0.5 for accurate, stable motion."),
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
      if (!r.actuated && j->damping == 0 && j->stiffness == 0 && j->frictionloss == 0 && r.movedMass > 0) {
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
      if ((a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) && a.target == joint) out.push_back(a.id);
    return out;
  }

  // ---------------------------------------------------------------- start state
  void checkStartState() {
    double floorZ = -kInf;
    for (const Geom& g : doc_.geoms)
      if (g.body == kWorldId && g.type == GeomType::Plane && std::fabs(rotate(g.quat, {0, 0, 1}).z) > 0.999)
        floorZ = std::max(floorZ, g.pos.z);
    for (const Body& root : doc_.bodies) {
      if (root.id == kWorldId || root.parent != kWorldId || root.role != Role::Model) continue;
      int ri = doc_.bodyIndex(root.id);
      bool floating = false;
      for (uint32_t ji : doc_.index().bodyJoints[ri]) floating |= doc_.joints[ji].type == JointType::Free;
      if (!floating || floorZ == -kInf) continue;
      double minZ = subtreeMinZ(doc_, poses_, root.id, true);
      if (!std::isfinite(minZ)) continue;
      Id rid = root.id;
      double dz = minZ - floorZ;
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
      // Balance: is the COM above the support polygon of the feet touching the floor?
      if (std::fabs(dz) < 0.01) {
        std::vector<Vec2> pts;
        for (Id b : doc_.subtree(root.id)) {
          int bi = doc_.bodyIndex(b);
          for (uint32_t gi : doc_.index().bodyGeoms[bi]) {
            const Geom& g = doc_.geoms[gi];
            if (g.type == GeomType::Plane || !geomCollides(g)) continue;
            Aabb ab = primitiveWorldAabb(g.type, g.size, poses_[bi] * Pose{g.pos, g.quat});
            if (ab.lo.z - floorZ < 0.01) {
              // Use the footprint corners for boxes, the center for round shapes.
              if (g.type == GeomType::Box) {
                pts.push_back({ab.lo.x, ab.lo.y});
                pts.push_back({ab.hi.x, ab.lo.y});
                pts.push_back({ab.hi.x, ab.hi.y});
                pts.push_back({ab.lo.x, ab.hi.y});
              } else {
                pts.push_back({ab.center().x, ab.center().y});
              }
            }
          }
        }
        MassProps mp = subtreeMassProps(doc_, poses_, root.id);
        std::vector<Vec2> hull = hull2d(pts);
        if (hull.size() >= 3 && mp.mass > 0) {
          double d = polygonDistance(hull, {mp.com.x, mp.com.y});
          if (d > 0.005)
            add(Severity::Warning, IssueArea::Start, "tip-over",
                fmt("%s will tip over: its center of mass is %.1f cm outside the feet", quote(root.name).c_str(), d * 100),
                "The center of mass projected on the floor lies outside the polygon spanned by the contacts. Move the "
                "legs or the heavy parts so the COM sits above the feet (enable View > Centers of mass).",
                {rid});
        } else if (hull.size() < 3 && pts.size() > 0 && mp.mass > 0 && rep_.legCount > 0) {
          add(Severity::Info, IssueArea::Start, "unstable-stance",
              fmt("%s touches the floor in fewer than 3 points", quote(root.name).c_str()),
              "With one or two contacts it cannot stand passively; it will fall over unless the policy balances it.", {rid});
        }
      }
    }
    if (!crValid_ || !cr_ || !cr_->ok) return;
    // Penetrations reported by MuJoCo at the reference pose.
    std::map<std::pair<Id, Id>, double> worst;
    for (const InitialContact& c : cr_->initialContacts) {
      if (c.dist > -1e-3) continue;
      if (c.geom1 < 0 || c.geom2 < 0 || size_t(c.geom1) >= cr_->map.geoms.size() || size_t(c.geom2) >= cr_->map.geoms.size()) continue;
      Id g1 = cr_->map.geoms[c.geom1], g2 = cr_->map.geoms[c.geom2];
      auto key = std::make_pair(std::min(g1, g2), std::max(g1, g2));
      worst[key] = std::min(worst.count(key) ? worst[key] : 0.0, c.dist);
    }
    for (const auto& [pair, dist] : worst) {
      const Geom* a = doc_.geom(pair.first);
      const Geom* b = doc_.geom(pair.second);
      if (!a || !b) continue;
      bool floorA = a->body == kWorldId && a->type == GeomType::Plane, floorB = b->body == kWorldId && b->type == GeomType::Plane;
      if (floorA || floorB) continue;  // handled by the floor check above
      Issue& i = add(Severity::Warning, IssueArea::Start, "penetration",
                     fmt("%s and %s overlap by %.1f mm at the start", quote(a->name).c_str(), quote(b->name).c_str(), -dist * 1000),
                     "Overlapping geoms of different bodies are pushed apart violently on the first step. Move them apart, "
                     "or exclude contacts between the two bodies if they are adjacent parts of the same creature.",
                     {a->id, b->id});
      if (a->body != kWorldId && b->body != kWorldId) {
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
    if (opt.sleep && opt.solver != Solver::Newton) {
      Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-sleep", "Sleeping requires the Newton solver in MuJoCo Warp", "");
      i.fixLabel = "Use Newton";
      i.fix = [](Document& d) { d.option.solver = Solver::Newton; };
    }
    // Box/mesh pairs with margins are rejected when (multi/native)CCD is on.
    if (opt.multiccd || opt.nativeccd) {
      std::vector<Id> offenders;
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
              if (g1.margin != 0) offenders.push_back(g1.id);
              if (g2.margin != 0) offenders.push_back(g2.id);
            }
        }
      std::sort(offenders.begin(), offenders.end());
      offenders.erase(std::unique(offenders.begin(), offenders.end()), offenders.end());
      if (!offenders.empty()) {
        Issue& i = add(Severity::Error, IssueArea::MJWarp, "mjw-box-margin",
                       fmt("%zu box geom(s) with a contact margin can collide with other boxes", offenders.size()),
                       "With multiccd / nativeccd enabled (the default), MuJoCo Warp rejects box-box and box-mesh pairs "
                       "with nonzero margin. Set the margin to 0, or disable multiccd and nativeccd in Scene > Flags.",
                       offenders);
        i.fixLabel = "Set those margins to 0";
        i.fix = [offenders](Document& d) { for (Id id : offenders) if (Geom* g = d.geom(id)) g->margin = 0; };
      }
    }
    for (const Geom& g : doc_.geoms) {
      if (g.condim >= 3 && g.friction.x < 1e-5 && (g.contype || g.conaffinity)) {
        add(Severity::Warning, IssueArea::MJWarp, "mjw-friction",
            fmt("Geom %s has sliding friction below 1e-5", quote(g.name).c_str()),
            "MuJoCo Warp warns that this may produce NaN with condim >= 3. Use condim 1 for frictionless contacts.", {g.id});
      }
    }
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
        e.box = primitiveWorldAabb(g.type, g.size, e.pose);
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
      s.ccd = isCcdPair(s.a, s.b);
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
  Weld weld_;
  double gmag_ = 9.81;
  Vec3 gdir_{0, 0, -1};
  std::unordered_map<Id, double> servoStiffness_, actDamping_;
  std::unordered_map<Id, std::string> supportCtx_;
  struct LegInfo { Id root; int legs; double mass; };
  std::vector<LegInfo> legInfo_;
};

}  // namespace

double actuatorMaxForce(const Document& doc, const Actuator& a) {
  auto absmax = [](const Vec2& r) { return std::max(std::fabs(r.x), std::fabs(r.y)); };
  double umax = a.ctrllimited ? absmax(a.ctrlrange) : kInf;
  double f = kInf;
  switch (a.type) {
    case ActuatorType::Motor: f = umax; break;
    case ActuatorType::Position:
    case ActuatorType::IntVelocity: {
      double span = kInf;
      const Joint* j = (a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) ? doc.joint(a.target) : nullptr;
      if (a.type == ActuatorType::IntVelocity && a.actlimited) span = a.actrange.y - a.actrange.x;
      else if (a.ctrllimited) span = a.ctrlrange.y - a.ctrlrange.x;
      if (j && j->limited) span = std::min(span, (j->range.y - j->range.x) * std::fabs(a.gear[0]));
      f = a.kp * span;
      break;
    }
    case ActuatorType::Velocity: f = a.kv * umax * 2.0; break;
    case ActuatorType::Damper: f = 0; break;  // cannot hold a load
    case ActuatorType::Cylinder: f = a.area * umax + std::fabs(a.cylBias.x); break;
    case ActuatorType::Muscle: f = a.muscleForce > 0 ? a.muscleForce : kInf; break;
    case ActuatorType::Adhesion: f = a.adhesionGain * (a.ctrllimited ? absmax(a.ctrlrange) : 1.0); break;
    case ActuatorType::General: {
      double g = std::fabs(a.gainprm[0]) * umax;
      double bias = std::fabs(a.biasprm[0]);
      f = (a.gaintype == GainType::Fixed) ? g + bias : kInf;
      break;
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
      {"repeated name", "Two elements of the same type share a name."},
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
