#include "core/MjcfWriter.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <list>
#include <unordered_map>
#include <unordered_set>

#include "core/ProjectIO.h"

namespace mjf {

std::string formatNumber(double v, int precision) {
  if (std::isnan(v)) return "0";
  if (std::isinf(v)) return v > 0 ? "1e+300" : "-1e+300";
  // Below MuJoCo's mjMINVAL (1e-15) values are rounding noise (cos 90 deg = 6e-17).
  if (std::fabs(v) < 1e-15) return "0";
  char buf[64];
  if (precision <= 0) {
    // The shortest of 15-17 significant digits that reads back as exactly v: hand-entered values
    // stay short ("0.1") and computed ones round-trip bit for bit.
    for (int p = 15; p <= 17; ++p) {
      std::snprintf(buf, sizeof buf, "%.*g", p, v);
      if (std::strtod(buf, nullptr) == v) break;
    }
    return buf;
  }
  std::snprintf(buf, sizeof buf, "%.*g", precision, v);
  return buf;
}

namespace {

struct XmlNode {
  std::string tag;
  std::vector<std::pair<std::string, std::string>> attrs;
  std::list<XmlNode> children;  // list: references stay valid while siblings are added (and the
                                // element type may be incomplete here, unlike deque)
  std::string comment;

  XmlNode() = default;
  explicit XmlNode(std::string t) : tag(std::move(t)) {}
  XmlNode& add(const std::string& t) {
    children.emplace_back(t);
    return children.back();
  }
  void set(const char* k, const std::string& v) { attrs.emplace_back(k, v); }
};

std::string escapeXml(const std::string& s) {
  std::string o;
  o.reserve(s.size());
  for (char c : s) {
    switch (c) {
      case '&': o += "&amp;"; break;
      case '<': o += "&lt;"; break;
      case '>': o += "&gt;"; break;
      case '"': o += "&quot;"; break;
      default:
        // Control characters other than tab / newline are not allowed in XML at all.
        if ((unsigned char)c >= 0x20 || c == '\t' || c == '\n' || c == '\r') o += c;
        break;
    }
  }
  return o;
}

void serialize(const XmlNode& n, std::string& out, int depth) {
  std::string ind(size_t(depth) * 2, ' ');
  if (!n.comment.empty()) out += ind + "<!-- " + n.comment + " -->\n";
  out += ind + "<" + n.tag;
  for (const auto& [k, v] : n.attrs) out += " " + k + "=\"" + escapeXml(v) + "\"";
  if (n.children.empty()) {
    out += "/>\n";
    return;
  }
  out += ">\n";
  for (const auto& c : n.children) serialize(c, out, depth + 1);
  out += ind + "</" + n.tag + ">\n";
}

bool nearly(double a, double b) { return std::fabs(a - b) <= 1e-12 * std::max(1.0, std::fabs(b)); }
bool nearly(const Vec2& a, const Vec2& b) { return nearly(a.x, b.x) && nearly(a.y, b.y); }
bool nearly(const Vec3& a, const Vec3& b) { return nearly(a.x, b.x) && nearly(a.y, b.y) && nearly(a.z, b.z); }
bool nearly(const Quat& a, const Quat& b) { return nearly(a.w, b.w) && nearly(a.x, b.x) && nearly(a.y, b.y) && nearly(a.z, b.z); }
template <int N> bool nearly(const VecN<N>& a, const VecN<N>& b) {
  for (int i = 0; i < N; ++i) if (!nearly(a[i], b[i])) return false;
  return true;
}

// Split export: the model file is written first and included by the scene file. The scene pass
// sees the model's bodies and skips what the model file already defines.
struct SplitState {
  std::unordered_set<Id> bodies;    // bodies in the model file
  std::unordered_set<Id> written;   // elements and assets the model file defines
  std::unordered_set<std::string> files;  // asset file names already used in the shared folder
  std::vector<const Joint*> joints;       // model joints, actuators and mocap bodies in compiled
  std::vector<Id> actuators;              // order (they come first in the scene: the include is
  std::vector<const Body*> mocap;         // at the top)
};

class Writer {
 public:
  Writer(const Document& d, const ExportOptions& o, ExportMap* m, std::vector<ExportFile>* f, SplitState* split = nullptr)
      : doc_(d), opt_(o), map_(m), files_(f), split_(split) {
    if (scenePass()) assetFiles_ = split_->files;
  }

  std::string run() {
    selectContent();
    computeNames();
    // Height of each height field's lowest sample: MuJoCo moves it to 0 (see writeAssets). Known
    // before any geom is written, also for geoms whose height field another split file defines.
    for (const Hfield& h : doc_.hfields)
      if (h.data && !h.data->empty() && h.data->size() == size_t(h.nrow) * size_t(h.ncol))
        hfieldOffset_[h.id] = *std::min_element(h.data->begin(), h.data->end()) * h.size.z;

    XmlNode root("mujoco");
    root.set("model", doc_.modelName.empty() ? "mjforge" : doc_.modelName);
    writeCompiler(root);
    if (!opt_.includeFile.empty()) root.add("include").set("file", opt_.includeFile);
    if (opt_.writeOption) writeOption(root);
    if (opt_.writeVisual) writeVisual(root);
    writeAssets(root);
    writeWorldbody(root);
    writeContact(root);
    writeEquality(root);
    writeTendons(root);
    writeActuators(root);
    writeSensors(root);
    if (!modelPass()) writeKeyframes(root);  // split export: the scene holds complete keyframes
    if (modelPass()) {
      split_->bodies = bodies_;
      for (const auto* set : {&tendons_, &equalities_, &pairs_, &excludes_, &actuators_, &sensors_, &materials_, &textures_,
                              &meshes_, &hfields_})
        split_->written.insert(set->begin(), set->end());
      split_->files = assetFiles_;
      split_->joints = writtenJoints_;
      split_->actuators = writtenActuators_;
      split_->mocap = writtenMocap_;
    }

    std::string out;
    if (opt_.header) {
      out += "<!-- Generated by MJForge " MJFORGE_VERSION
             " (MuJoCo / MuJoCo Warp model editor). Edit the .mjforge project, then re-export. -->\n";
    }
    serialize(root, out, 0);
    return out;
  }

 private:
  // ---------------------------------------------------------------- formatting helpers
  std::string num(double v) const { return formatNumber(v, opt_.precision); }
  std::string vec(const Vec2& v) const { return num(v.x) + " " + num(v.y); }
  std::string vec(const Vec3& v) const { return num(v.x) + " " + num(v.y) + " " + num(v.z); }
  std::string vec(const Vec4& v) const { return num(v.x) + " " + num(v.y) + " " + num(v.z) + " " + num(v.w); }
  template <int N> std::string vec(const VecN<N>& v, int count = N) const {
    std::string s;
    for (int i = 0; i < count; ++i) s += (i ? " " : "") + num(v[i]);
    return s;
  }
  // Colors are floats: 7 significant digits reproduce them exactly without float noise
  // ("0.85", not "0.850000024").
  std::string color(const Color& c) const {
    auto f = [](float v) { return formatNumber(v, 7); };
    return f(c.r) + " " + f(c.g) + " " + f(c.b) + " " + f(c.a);
  }
  std::string quat(const Quat& q_) const {
    Quat q = canonical(q_);
    return num(q.w) + " " + num(q.x) + " " + num(q.y) + " " + num(q.z);
  }
  static bool isIdentity(const Quat& q_) {
    Quat q = canonical(q_);
    return std::fabs(q.w - 1.0) < 1e-12 && std::fabs(q.x) < 1e-9 && std::fabs(q.y) < 1e-9 && std::fabs(q.z) < 1e-9;
  }
  double ang(double rad) const { return opt_.angle == AngleUnit::Radian ? rad : rad2deg(rad); }
  // Trims trailing zeros of a size-like vector to the number of meaningful components.
  std::string sizeStr(const Vec3& s, int count) const {
    std::string r = num(s.x);
    if (count > 1) r += " " + num(s.y);
    if (count > 2) r += " " + num(s.z);
    return r;
  }
  static int geomSizeCount(GeomType t) {
    switch (t) {
      case GeomType::Sphere: return 1;
      case GeomType::Capsule:
      case GeomType::Cylinder: return 2;
      case GeomType::Hfield:
      case GeomType::Mesh: return 0;
      default: return 3;
    }
  }

  void setPose(XmlNode& n, const Vec3& pos, const Quat& q) const {
    if (!nearly(pos, Vec3{})) n.set("pos", vec(pos));
    if (!isIdentity(q)) n.set("quat", quat(q));
  }
  void setName(XmlNode& n, Id id) const {
    auto it = names_.find(id);
    if (it != names_.end() && !it->second.empty()) n.set("name", it->second);
  }
  std::string nameOf(Id id) const {
    if (id == kWorldId) return "world";
    auto it = names_.find(id);
    return it == names_.end() ? std::string() : it->second;
  }

  // ---------------------------------------------------------------- content selection
  bool includeRoot(const Body& root) const {
    using S = ExportOptions::Scope;
    switch (opt_.scope) {
      case S::All: return true;
      case S::Models: return root.role == Role::Model && (opt_.modelRoot == kNoId || opt_.modelRoot == root.id);
      case S::Environment: return root.role == Role::Environment;
    }
    return true;
  }
  bool worldContent() const { return opt_.scope != ExportOptions::Scope::Models; }
  bool modelPass() const { return split_ && opt_.scope == ExportOptions::Scope::Models; }
  bool scenePass() const { return split_ && opt_.scope == ExportOptions::Scope::Environment; }
  bool bodyIncluded(Id b) const { return b == kWorldId || bodies_.count(b) > 0; }
  // Bodies this file can refer to: its own, plus the included model file's in a split scene.
  bool bodyVisible(Id b) const { return bodyIncluded(b) || (scenePass() && split_->bodies.count(b) > 0); }
  // Elements on a missing body count as world elements, as in the document index.
  Id bodyOf(Id body) const { return body != kWorldId && !doc_.body(body) ? kWorldId : body; }
  // Elements attached to the world are only exported with environment content.
  bool attachedIncluded(Id body) const {
    body = bodyOf(body);
    return body == kWorldId ? worldContent() : bodies_.count(body) > 0;
  }
  bool attachedVisible(Id body) const {
    body = bodyOf(body);
    return body == kWorldId ? worldContent() : bodyVisible(body);
  }
  bool alreadyWritten(Id id) const { return scenePass() && split_->written.count(id) > 0; }

  void selectContent() {
    // Top-level bodies as the index sees them (a body with a missing parent hangs from the world).
    for (uint32_t ci : doc_.index().childBodies[0]) {
      const Body& b = doc_.bodies[ci];
      if (!includeRoot(b)) continue;
      for (Id id : doc_.subtree(b.id)) bodies_.insert(id);
    }
    // Whether an element referenced by a tendon, actuator, sensor... exists in this file (or in the
    // model file a split scene includes).
    auto visible = [&](Id id) -> bool {
      switch (doc_.kindOf(id)) {
        case Kind::Body: return bodyVisible(id);
        case Kind::Joint: return attachedVisible(doc_.joint(id)->body);
        case Kind::Geom: return attachedVisible(doc_.geom(id)->body);
        case Kind::Site: return attachedVisible(doc_.site(id)->body);
        case Kind::Camera: return attachedVisible(doc_.camera(id)->body);
        case Kind::Tendon: return tendons_.count(id) > 0 || alreadyWritten(id);
        case Kind::Actuator: return actuators_.count(id) > 0 || alreadyWritten(id);
        default: return false;
      }
    };
    // Tendons whose joints / path elements all exist (before the actuators using them).
    for (const Tendon& t : doc_.tendons) {
      if (alreadyWritten(t.id)) continue;
      bool ok = t.type == TendonType::Fixed ? !t.joints.empty() : t.path.size() >= 2;
      for (const TendonJoint& j : t.joints) ok &= j.joint != kNoId && visible(j.joint);
      for (const TendonPoint& p : t.path) {
        if (p.type != TendonPoint::Type::Pulley) ok &= p.obj != kNoId && visible(p.obj);
        if (p.sidesite) ok &= visible(p.sidesite);
      }
      if (ok) tendons_.insert(t.id);
    }
    for (const Equality& e : doc_.equalities) {
      bool ok = e.obj1 != kNoId && visible(e.obj1) && (e.obj2 == kNoId || visible(e.obj2));
      if (ok && !alreadyWritten(e.id)) equalities_.insert(e.id);
    }
    for (const ContactPair& c : doc_.pairs)
      if (c.geom1 && c.geom2 && visible(c.geom1) && visible(c.geom2) && !alreadyWritten(c.id)) pairs_.insert(c.id);
    for (const Exclude& e : doc_.excludes) {
      if (e.body1 == kNoId || e.body2 == kNoId || (e.body1 == kWorldId && e.body2 == kWorldId)) continue;
      if (bodyVisible(e.body1) && bodyVisible(e.body2) && !alreadyWritten(e.id)) excludes_.insert(e.id);
    }
    for (const Actuator& a : doc_.actuators) {
      bool ok = a.target != kNoId && visible(a.target);
      if (ok && a.refsite != kNoId) ok = visible(a.refsite);
      if (ok && a.trn == TrnType::SliderCrank) ok = a.slidersite != kNoId && visible(a.slidersite);
      if (ok && !alreadyWritten(a.id)) actuators_.insert(a.id);
    }
    for (const Sensor& s : doc_.sensors) {
      const SensorInfo& info = sensorInfo(s.type);
      bool ok = true;
      if (info.obj == SensorObj::Contact) ok = s.obj == kNoId || visible(s.obj);  // no object: any contact
      else if (info.obj != SensorObj::None) ok = s.obj != kNoId && visible(s.obj);
      if (ok && s.ref != kNoId) ok = visible(s.ref);
      bool needsRef = info.obj == SensorObj::GeomPair || info.obj == SensorObj::Inside || info.obj == SensorObj::CamProj;
      if (needsRef && s.ref == kNoId) ok = false;
      if (ok && !alreadyWritten(s.id)) sensors_.insert(s.id);
    }
    // Assets referenced by exported geoms/sites/tendons (+ skyboxes with environment content); a
    // split scene does not redefine the model file's.
    for (const Geom& g : doc_.geoms) {
      if (!attachedIncluded(g.body)) continue;
      if (g.material != kNoId) materials_.insert(g.material);
      if (g.type == GeomType::Hfield && g.hfield != kNoId) hfields_.insert(g.hfield);
      if (g.type == GeomType::Mesh && g.mesh != kNoId && doc_.mesh(g.mesh)) meshes_.insert(g.mesh);
    }
    for (const Site& s : doc_.sites)
      if (attachedIncluded(s.body) && s.material != kNoId) materials_.insert(s.material);
    for (const Tendon& t : doc_.tendons)
      if (tendons_.count(t.id) && t.material != kNoId) materials_.insert(t.material);
    for (Id m : materials_) {
      const Material* mat = doc_.material(m);
      if (mat && mat->texture != kNoId) textures_.insert(mat->texture);
    }
    if (worldContent())
      for (const Texture& t : doc_.textures)
        if (t.type == TextureType::Skybox) textures_.insert(t.id);
    if (scenePass())
      for (auto* set : {&materials_, &textures_, &meshes_, &hfields_})
        for (auto it = set->begin(); it != set->end();) it = split_->written.count(*it) ? set->erase(it) : std::next(it);
  }

  void computeNames() {
    std::unordered_set<Id> referenced;
    for (const Actuator& a : doc_.actuators) {
      referenced.insert(a.target);
      referenced.insert(a.refsite);
      referenced.insert(a.slidersite);
    }
    for (const Sensor& s : doc_.sensors) {
      referenced.insert(s.obj);
      referenced.insert(s.ref);
    }
    for (const Exclude& e : doc_.excludes) {
      referenced.insert(e.body1);
      referenced.insert(e.body2);
    }
    for (const ContactPair& c : doc_.pairs) {
      referenced.insert(c.geom1);
      referenced.insert(c.geom2);
    }
    for (const Equality& e : doc_.equalities) {
      referenced.insert(e.obj1);
      referenced.insert(e.obj2);
    }
    for (const Tendon& t : doc_.tendons) {
      for (const TendonJoint& j : t.joints) referenced.insert(j.joint);
      for (const TendonPoint& p : t.path) {
        referenced.insert(p.obj);
        referenced.insert(p.sidesite);
      }
    }
    for (const Camera& c : doc_.cameras) referenced.insert(c.target);
    for (const Light& l : doc_.lights) referenced.insert(l.target);

    // Generated names ("joint12") must not collide with names the user chose for that kind.
    auto assign = [&](const auto& vec, Kind kind, bool alwaysName) {
      std::unordered_set<std::string> used;
      for (const auto& e : vec)
        if (!e.name.empty()) used.insert(e.name);
      for (const auto& e : vec) {
        std::string n = e.name;
        if (n.empty() && (alwaysName || referenced.count(e.id))) {
          std::string base = std::string(kindName(kind)) + std::to_string(e.id);
          n = base;
          for (int k = 2; used.count(n); ++k) n = base + "_" + std::to_string(k);
          used.insert(n);
        }
        names_[e.id] = n;
      }
    };
    assign(doc_.bodies, Kind::Body, false);
    assign(doc_.geoms, Kind::Geom, false);
    assign(doc_.joints, Kind::Joint, false);
    assign(doc_.sites, Kind::Site, false);
    assign(doc_.cameras, Kind::Camera, false);
    assign(doc_.lights, Kind::Light, false);
    assign(doc_.actuators, Kind::Actuator, false);
    assign(doc_.sensors, Kind::Sensor, false);
    assign(doc_.excludes, Kind::Exclude, false);
    assign(doc_.pairs, Kind::Pair, false);
    assign(doc_.equalities, Kind::Equality, false);
    assign(doc_.tendons, Kind::Tendon, false);
    assign(doc_.materials, Kind::Material, true);
    assign(doc_.textures, Kind::Texture, true);
    assign(doc_.hfields, Kind::Hfield, true);
    assign(doc_.meshes, Kind::Mesh, true);
  }

  // ---------------------------------------------------------------- sections
  void writeCompiler(XmlNode& root) {
    const CompilerSettings& c = doc_.compiler;
    XmlNode& n = root.add("compiler");
    n.set("angle", opt_.angle == AngleUnit::Radian ? "radian" : "degree");
    n.set("autolimits", c.autolimits ? "true" : "false");
    if (c.inertiafromgeom != InertiaFromGeom::Auto) n.set("inertiafromgeom", toXml(c.inertiafromgeom));
    if (c.boundmass > 0) n.set("boundmass", num(c.boundmass));
    if (c.boundinertia > 0) n.set("boundinertia", num(c.boundinertia));
    if (c.settotalmass > 0) n.set("settotalmass", num(c.settotalmass));
    if (c.balanceinertia) n.set("balanceinertia", "true");
  }

  void writeOption(XmlNode& root) {
    const OptionSettings& o = doc_.option;
    const OptionSettings d;
    XmlNode n("option");
    if (!nearly(o.timestep, d.timestep)) n.set("timestep", num(o.timestep));
    if (!nearly(o.gravity, d.gravity)) n.set("gravity", vec(o.gravity));
    if (!nearly(o.wind, d.wind)) n.set("wind", vec(o.wind));
    if (!nearly(o.magnetic, d.magnetic)) n.set("magnetic", vec(o.magnetic));
    if (!nearly(o.density, d.density)) n.set("density", num(o.density));
    if (!nearly(o.viscosity, d.viscosity)) n.set("viscosity", num(o.viscosity));
    if (!nearly(o.impratio, d.impratio)) n.set("impratio", num(o.impratio));
    if (o.integrator != d.integrator) n.set("integrator", toXml(o.integrator));
    if (o.cone != d.cone) n.set("cone", toXml(o.cone));
    if (o.jacobian != d.jacobian) n.set("jacobian", toXml(o.jacobian));
    if (o.solver != d.solver) n.set("solver", toXml(o.solver));
    if (o.iterations != d.iterations) n.set("iterations", std::to_string(o.iterations));
    if (!nearly(o.tolerance, d.tolerance)) n.set("tolerance", num(o.tolerance));
    if (o.ls_iterations != d.ls_iterations) n.set("ls_iterations", std::to_string(o.ls_iterations));
    if (!nearly(o.ls_tolerance, d.ls_tolerance)) n.set("ls_tolerance", num(o.ls_tolerance));
    if (o.noslip_iterations != d.noslip_iterations) n.set("noslip_iterations", std::to_string(o.noslip_iterations));
    if (o.ccd_iterations != d.ccd_iterations) n.set("ccd_iterations", std::to_string(o.ccd_iterations));
    if (!nearly(o.noslip_tolerance, d.noslip_tolerance)) n.set("noslip_tolerance", num(o.noslip_tolerance));
    if (!nearly(o.ccd_tolerance, d.ccd_tolerance)) n.set("ccd_tolerance", num(o.ccd_tolerance));
    if (!nearly(o.o_margin, d.o_margin)) n.set("o_margin", num(o.o_margin));
    if (!nearly(o.o_solref, d.o_solref)) n.set("o_solref", vec(o.o_solref));
    if (!nearly(o.o_solimp, d.o_solimp)) n.set("o_solimp", vec(o.o_solimp));
    if (!nearly(o.o_friction, d.o_friction)) n.set("o_friction", vec(o.o_friction));
    if (!nearly(o.sleep_tolerance, d.sleep_tolerance)) n.set("sleep_tolerance", num(o.sleep_tolerance));
    if (o.disableactuator != 0) {
      std::string groups;
      for (int g = 0; g < 31; ++g)
        if (o.disableactuator & (1 << g)) groups += (groups.empty() ? "" : " ") + std::to_string(g);
      n.set("actuatorgroupdisable", groups);
    }

    XmlNode flag("flag");
    auto f = [&](const char* name, bool v, bool def) {
      if (v != def) flag.set(name, v ? "enable" : "disable");
    };
    f("constraint", o.constraint, d.constraint);
    f("equality", o.equality, d.equality);
    f("frictionloss", o.frictionloss, d.frictionloss);
    f("limit", o.limit, d.limit);
    f("contact", o.contact, d.contact);
    f("spring", o.spring, d.spring);
    f("damper", o.damper, d.damper);
    f("gravity", o.gravityFlag, d.gravityFlag);
    f("clampctrl", o.clampctrl, d.clampctrl);
    f("warmstart", o.warmstart, d.warmstart);
    f("filterparent", o.filterparent, d.filterparent);
    f("actuation", o.actuation, d.actuation);
    f("refsafe", o.refsafe, d.refsafe);
    f("sensor", o.sensor, d.sensor);
    f("midphase", o.midphase, d.midphase);
    f("nativeccd", o.nativeccd, d.nativeccd);
    f("island", o.island, d.island);
    f("eulerdamp", o.eulerdamp, d.eulerdamp);
    f("autoreset", o.autoreset, d.autoreset);
    f("multiccd", o.multiccd, d.multiccd);
    f("energy", o.energy, d.energy);
    f("sleep", o.sleep, d.sleep);
    f("override", o.override_, d.override_);
    f("fwdinv", o.fwdinv, d.fwdinv);
    f("invdiscrete", o.invdiscrete, d.invdiscrete);
    if (!flag.attrs.empty()) n.children.push_back(flag);
    if (!n.attrs.empty() || !n.children.empty()) root.children.push_back(n);
    if (!o.memory.empty()) root.add("size").set("memory", o.memory);
  }

  void writeVisual(XmlNode& root) {
    if (!worldContent()) return;
    const VisualSettings& v = doc_.visual;
    const VisualSettings d;
    XmlNode n("visual");
    XmlNode hl("headlight");
    if (!nearly(v.headlightAmbient, d.headlightAmbient)) hl.set("ambient", vec(v.headlightAmbient));
    if (!nearly(v.headlightDiffuse, d.headlightDiffuse)) hl.set("diffuse", vec(v.headlightDiffuse));
    if (!nearly(v.headlightSpecular, d.headlightSpecular)) hl.set("specular", vec(v.headlightSpecular));
    if (!hl.attrs.empty()) n.children.push_back(hl);
    XmlNode gl("global");
    if (!nearly(v.azimuth, d.azimuth)) gl.set("azimuth", num(v.azimuth));
    if (!nearly(v.elevation, d.elevation)) gl.set("elevation", num(v.elevation));
    if (v.offwidth != d.offwidth) gl.set("offwidth", std::to_string(v.offwidth));
    if (v.offheight != d.offheight) gl.set("offheight", std::to_string(v.offheight));
    if (!gl.attrs.empty()) n.children.push_back(gl);
    if (v.shadowsize != d.shadowsize) n.add("quality").set("shadowsize", std::to_string(v.shadowsize));
    if (v.haze != d.haze) n.add("rgba").set("haze", color(v.haze));
    if (!n.children.empty()) root.children.push_back(n);
  }

  void writeAssets(XmlNode& root) {
    XmlNode n("asset");
    for (const Texture& t : doc_.textures) {
      if (!textures_.count(t.id)) continue;
      XmlNode& e = n.add("texture");
      setName(e, t.id);
      e.set("type", toXml(t.type));
      if (t.builtin != TextureBuiltin::None) {
        e.set("builtin", toXml(t.builtin));
        e.set("rgb1", vec(t.rgb1));
        e.set("rgb2", vec(t.rgb2));
        if (t.mark != TextureMark::None) {
          e.set("mark", toXml(t.mark));
          e.set("markrgb", vec(t.markrgb));
          if (t.mark == TextureMark::Random) e.set("random", num(t.random));
        }
        e.set("width", std::to_string(t.width));
        e.set("height", std::to_string(t.height));
      } else {
        bool cube = t.file.empty() && t.type != TextureType::TwoD;
        bool ok = !t.file.empty();
        if (cube)
          for (const std::string& f : t.cubefiles) ok |= !f.empty();
        std::error_code ec;
        if (ok && !cube) ok = std::filesystem::is_regular_file(std::filesystem::u8path(t.file), ec);
        if (ok && cube)
          for (const std::string& f : t.cubefiles) ok &= f.empty() || std::filesystem::is_regular_file(std::filesystem::u8path(f), ec);
        if (!ok) {
          // A missing image would stop MuJoCo from loading the model; a flat color keeps it usable
          // (the analysis reports the missing file).
          e.set("builtin", "flat");
          e.set("rgb1", vec(t.rgb1));
          e.set("width", "16");
          e.set("height", "16");
        } else if (!cube) {
          e.set("file", imageFile(t.file));
          if (t.gridsize[0] != 1 || t.gridsize[1] != 1) {
            e.set("gridsize", std::to_string(t.gridsize[0]) + " " + std::to_string(t.gridsize[1]));
            if (!t.gridlayout.empty()) e.set("gridlayout", t.gridlayout);
          }
        } else {
          for (int i = 0; i < 6; ++i)
            if (!t.cubefiles[i].empty()) e.set(kCubeFileAttrs[i], imageFile(t.cubefiles[i]));
        }
        if (ok && t.hflip) e.set("hflip", "true");
        if (ok && t.vflip) e.set("vflip", "true");
      }
    }
    for (const Material& m : doc_.materials) {
      if (!materials_.count(m.id)) continue;
      const Material d;
      XmlNode& e = n.add("material");
      setName(e, m.id);
      if (m.texture != kNoId && textures_.count(m.texture)) e.set("texture", nameOf(m.texture));
      if (m.rgba != d.rgba) e.set("rgba", color(m.rgba));
      if (!nearly(m.texrepeat, d.texrepeat)) e.set("texrepeat", vec(m.texrepeat));
      if (m.texuniform) e.set("texuniform", "true");
      if (!nearly(m.emission, d.emission)) e.set("emission", num(m.emission));
      if (!nearly(m.specular, d.specular)) e.set("specular", num(m.specular));
      if (!nearly(m.shininess, d.shininess)) e.set("shininess", num(m.shininess));
      if (!nearly(m.reflectance, d.reflectance)) e.set("reflectance", num(m.reflectance));
    }
    for (const MeshAsset& m : doc_.meshes) {
      if (!meshes_.count(m.id) || !m.vert || !m.face) continue;
      // Vertices are already in the geom's frame (centered like MuJoCo compiles them), so MuJoCo's
      // recentering is a no-op. Big meshes go to STL files, small ones inline (no files needed).
      XmlNode& e = n.add("mesh");
      setName(e, m.id);
      if (m.inertia != MeshInertia::Legacy) e.set("inertia", toXml(m.inertia));
      if (m.maxhullvert > 3) e.set("maxhullvert", std::to_string(m.maxhullvert));
      if (files_ && m.vertexCount() > opt_.externalMeshVertices) {
        std::string path = opt_.assetDir + "/" + assetFileName(nameOf(m.id), ".stl");
        files_->push_back({path, meshToStl(m)});
        e.set("file", path);
        continue;
      }
      std::string vs, fs;
      vs.reserve(m.vert->size() * 10);
      for (size_t i = 0; i < m.vert->size(); ++i) {
        if (i) vs += ' ';
        vs += formatNumber((*m.vert)[i], 7);
      }
      fs.reserve(m.face->size() * 5);
      for (size_t i = 0; i < m.face->size(); ++i) {
        if (i) fs += ' ';
        fs += std::to_string((*m.face)[i]);
      }
      e.set("vertex", vs);
      e.set("face", fs);
    }
    for (const Hfield& h : doc_.hfields) {
      if (!hfields_.count(h.id)) continue;
      XmlNode& e = n.add("hfield");
      setName(e, h.id);
      bool complete = h.data && h.data->size() == size_t(h.nrow) * size_t(h.ncol) && h.nrow > 0 && h.ncol > 0;
      // MuJoCo rescales elevation data to [0, 1]. Write it normalized, with the height scale and the
      // geoms' offset adjusted, so the terrain keeps exactly the heights the editor shows.
      Hfield out = h;
      if (complete) {
        auto [lo, hi] = std::minmax_element(h.data->begin(), h.data->end());
        double l = *lo, span = double(*hi) - double(*lo);
        auto data = std::make_shared<std::vector<float>>(h.data->size());
        for (size_t i = 0; i < data->size(); ++i) (*data)[i] = span > 1e-12 ? float(((*h.data)[i] - l) / span) : 0.0f;
        out.data = data;
        if (span > 1e-12) out.size.z = h.size.z * span;
      }
      if (files_ && complete && h.data->size() > opt_.externalHfieldSamples) {
        // Big terrains go to MuJoCo's binary height field format: exact float32 values and no
        // multi-megabyte attribute for the XML parser.
        std::string path = opt_.assetDir + "/" + assetFileName(nameOf(h.id), ".bin");
        files_->push_back({path, hfieldToBin(out)});
        e.set("file", path);
        e.set("size", vec(out.size));
        continue;
      }
      e.set("nrow", std::to_string(h.nrow));
      e.set("ncol", std::to_string(h.ncol));
      e.set("size", vec(out.size));
      if (complete) {
        std::string s;
        s.reserve(out.data->size() * 8);
        for (size_t i = 0; i < out.data->size(); ++i) {
          if (i) s += ' ';
          s += formatNumber((*out.data)[i], 7);
        }
        e.set("elevation", s);
      }
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  void writeWorldbody(XmlNode& root) {
    XmlNode& wb = root.add("worldbody");
    if (map_) map_->bodies.push_back(kWorldId);
    const DocIndex& ix = doc_.index();
    if (worldContent()) {
      for (uint32_t li : ix.bodyLights[0]) writeLight(wb, doc_.lights[li]);
      for (uint32_t ci : ix.bodyCameras[0]) writeCamera(wb, doc_.cameras[ci]);
      for (uint32_t gi : ix.bodyGeoms[0]) writeGeom(wb, doc_.geoms[gi]);
      for (uint32_t si : ix.bodySites[0]) writeSite(wb, doc_.sites[si]);
    }
    for (uint32_t ci : ix.childBodies[0]) {
      const Body& b = doc_.bodies[ci];
      if (bodies_.count(b.id)) writeBody(wb, b);
    }
  }

  void writeBody(XmlNode& parent, const Body& b) {
    const DocIndex& ix = doc_.index();
    int bi = doc_.bodyIndex(b.id);
    if (bi < 0) return;
    XmlNode& n = parent.add("body");
    setName(n, b.id);
    setPose(n, b.pos, b.quat);
    if (b.mocap) {
      n.set("mocap", "true");
      writtenMocap_.push_back(&b);
    }
    if (!nearly(b.gravcomp, 0.0)) n.set("gravcomp", num(b.gravcomp));
    if (b.sleep != SleepPolicy::Auto && doc_.isTreeRoot(b.id)) n.set("sleep", toXml(b.sleep));
    if (map_) map_->bodies.push_back(b.id);

    if (b.hasInertial) {
      XmlNode& in = n.add("inertial");
      in.set("pos", vec(b.ipos));
      if (!isIdentity(b.iquat)) in.set("quat", quat(b.iquat));
      in.set("mass", num(b.mass));
      in.set("diaginertia", vec(b.diaginertia));
    }
    for (uint32_t ji : ix.bodyJoints[bi]) writeJoint(n, doc_.joints[ji]);
    for (uint32_t gi : ix.bodyGeoms[bi]) writeGeom(n, doc_.geoms[gi]);
    for (uint32_t si : ix.bodySites[bi]) writeSite(n, doc_.sites[si]);
    for (uint32_t ci : ix.bodyCameras[bi]) writeCamera(n, doc_.cameras[ci]);
    for (uint32_t li : ix.bodyLights[bi]) writeLight(n, doc_.lights[li]);
    for (uint32_t ci : ix.childBodies[bi]) writeBody(n, doc_.bodies[ci]);
  }

  void writeJoint(XmlNode& parent, const Joint& j) {
    if (map_) map_->joints.push_back(j.id);
    writtenJoints_.push_back(&j);
    const Joint d;
    bool angular = j.type == JointType::Hinge || j.type == JointType::Ball;
    bool simpleFree = j.type == JointType::Free && nearly(j.damping, 0.0) && nearly(j.armature, 0.0) &&
                      nearly(j.stiffness, 0.0) && nearly(j.frictionloss, 0.0) && j.group == 0;
    if (simpleFree) {
      XmlNode& n = parent.add("freejoint");
      setName(n, j.id);
      return;
    }
    XmlNode& n = parent.add("joint");
    setName(n, j.id);
    if (j.type != JointType::Hinge) n.set("type", toXml(j.type));
    if (j.type != JointType::Free) {
      if (!nearly(j.pos, Vec3{})) n.set("pos", vec(j.pos));
      if ((j.type == JointType::Hinge || j.type == JointType::Slide) && !nearly(normalize(j.axis), Vec3{0, 0, 1}))
        n.set("axis", vec(normalize(j.axis)));
    }
    if (j.type != JointType::Free) {
      // MuJoCo converts a degree range to radians only when the joint is limited.
      Vec2 r = angular && j.limited ? Vec2{ang(j.range.x), ang(j.range.y)} : j.range;
      writeLimited(n, "limited", "range", j.limited, r);
    }
    if (!nearly(j.ref, 0.0)) n.set("ref", num(angular ? ang(j.ref) : j.ref));
    if (!nearly(j.springref, 0.0)) n.set("springref", num(angular ? ang(j.springref) : j.springref));
    if (!nearly(j.stiffness, 0.0)) n.set("stiffness", num(j.stiffness));
    if (!nearly(j.damping, 0.0)) n.set("damping", num(j.damping));
    if (!nearly(j.armature, 0.0)) n.set("armature", num(j.armature));
    if (!nearly(j.frictionloss, 0.0)) n.set("frictionloss", num(j.frictionloss));
    if (!nearly(j.margin, 0.0)) n.set("margin", num(j.margin));
    if (!nearly(j.solreflimit, d.solreflimit)) n.set("solreflimit", vec(j.solreflimit));
    if (!nearly(j.solimplimit, d.solimplimit)) n.set("solimplimit", vec(j.solimplimit));
    if (!nearly(j.solreffriction, d.solreffriction)) n.set("solreffriction", vec(j.solreffriction));
    if (!nearly(j.solimpfriction, d.solimpfriction)) n.set("solimpfriction", vec(j.solimpfriction));
    writeLimited(n, "actuatorfrclimited", "actuatorfrcrange", j.actuatorfrclimited, j.actuatorfrcrange);
    if (j.actuatorgravcomp) n.set("actuatorgravcomp", "true");
    if (j.group != 0) n.set("group", std::to_string(j.group));
  }

  void writeGeom(XmlNode& parent, const Geom& g) {
    if (map_) map_->geoms.push_back(g.id);
    const Geom d;
    XmlNode& n = parent.add("geom");
    setName(n, g.id);
    if (g.type != GeomType::Sphere) n.set("type", toXml(g.type));
    int sc = geomSizeCount(g.type);
    if (sc > 0) n.set("size", sizeStr(g.size, sc));
    if (g.type == GeomType::Hfield && g.hfield != kNoId) n.set("hfield", nameOf(g.hfield));
    if (g.type == GeomType::Mesh && g.mesh != kNoId) n.set("mesh", nameOf(g.mesh));
    Vec3 pos = g.pos;
    if (g.type == GeomType::Hfield) {  // the lowest sample moved to 0 by the normalization above
      auto it = hfieldOffset_.find(g.hfield);
      if (it != hfieldOffset_.end()) pos = pos + rotate(g.quat, Vec3{0, 0, it->second});
    }
    setPose(n, pos, g.quat);
    bool hasMat = g.material != kNoId && materials_.count(g.material);
    if (hasMat) n.set("material", nameOf(g.material));
    if (g.rgba != d.rgba) n.set("rgba", color(g.rgba));  // MuJoCo: any non-default rgba overrides the material
    if (g.contype != d.contype) n.set("contype", std::to_string(g.contype));
    if (g.conaffinity != d.conaffinity) n.set("conaffinity", std::to_string(g.conaffinity));
    if (g.condim != d.condim) n.set("condim", std::to_string(g.condim));
    if (g.priority != d.priority) n.set("priority", std::to_string(g.priority));
    if (g.group != d.group) n.set("group", std::to_string(g.group));
    if (!nearly(g.friction, d.friction)) n.set("friction", vec(g.friction));
    if (!nearly(g.solmix, d.solmix)) n.set("solmix", num(g.solmix));
    if (!nearly(g.solref, d.solref)) n.set("solref", vec(g.solref));
    if (!nearly(g.solimp, d.solimp)) n.set("solimp", vec(g.solimp));
    if (!nearly(g.margin, d.margin)) n.set("margin", num(g.margin));
    if (!nearly(g.gap, d.gap)) n.set("gap", num(g.gap));
    if (!nearly(g.adhesion, 0.0)) n.set("adhesion", num(g.adhesion));
    if (!nearly(g.surfacevel, Vec6{})) n.set("surfacevel", vec(g.surfacevel));
    if (g.fluidEllipsoid) {
      n.set("fluidshape", "ellipsoid");
      if (!nearly(g.fluidcoef, d.fluidcoef)) n.set("fluidcoef", vec(g.fluidcoef));
    }
    if (g.body != kWorldId) {  // mass of static geoms is irrelevant
      if (g.mass >= 0) n.set("mass", num(g.mass));
      else if (!nearly(g.density, d.density)) n.set("density", num(g.density));
      if (g.shellinertia) n.set("shellinertia", "true");
    }
  }

  void writeSite(XmlNode& parent, const Site& s) {
    if (map_) map_->sites.push_back(s.id);
    const Site d;
    XmlNode& n = parent.add("site");
    setName(n, s.id);
    if (s.type != GeomType::Sphere) n.set("type", toXml(s.type));
    int sc = geomSizeCount(s.type);
    if (!nearly(s.size, d.size)) n.set("size", sizeStr(s.size, sc));
    setPose(n, s.pos, s.quat);
    bool hasMat = s.material != kNoId && materials_.count(s.material);
    if (hasMat) n.set("material", nameOf(s.material));
    if (s.rgba != d.rgba) n.set("rgba", color(s.rgba));
    if (s.group != 0) n.set("group", std::to_string(s.group));
  }

  void writeCamera(XmlNode& parent, const Camera& c) {
    if (map_) map_->cameras.push_back(c.id);
    XmlNode& n = parent.add("camera");
    setName(n, c.id);
    // A target outside this file (another export scope) would not load: fall back to fixed.
    bool targeted = c.mode == CameraMode::TargetBody || c.mode == CameraMode::TargetBodyCom;
    bool targetOk = c.target != kNoId && bodyVisible(c.target);
    if (c.mode != CameraMode::Fixed && (!targeted || targetOk)) n.set("mode", toXml(c.mode));
    if (targeted && targetOk) n.set("target", nameOf(c.target));
    setPose(n, c.pos, c.quat);
    if (!nearly(c.fovy, 45.0)) n.set("fovy", num(c.fovy));
  }

  void writeLight(XmlNode& parent, const Light& l) {
    if (map_) map_->lights.push_back(l.id);
    const Light d;
    XmlNode& n = parent.add("light");
    setName(n, l.id);
    bool targeted = l.mode == CameraMode::TargetBody || l.mode == CameraMode::TargetBodyCom;
    bool targetOk = l.target != kNoId && bodyVisible(l.target);
    if (l.mode != CameraMode::Fixed && (!targeted || targetOk)) n.set("mode", toXml(l.mode));
    if (targeted && targetOk) n.set("target", nameOf(l.target));
    if (l.type != LightType::Spot) n.set("type", toXml(l.type));
    if (!nearly(l.pos, Vec3{})) n.set("pos", vec(l.pos));
    if (!nearly(normalize(l.dir), Vec3{0, 0, -1})) n.set("dir", vec(normalize(l.dir)));
    if (!l.castshadow) n.set("castshadow", "false");
    if (!l.active) n.set("active", "false");
    if (!nearly(l.diffuse, d.diffuse)) n.set("diffuse", vec(l.diffuse));
    if (!nearly(l.ambient, d.ambient)) n.set("ambient", vec(l.ambient));
    if (!nearly(l.specular, d.specular)) n.set("specular", vec(l.specular));
    if (!nearly(l.attenuation, d.attenuation)) n.set("attenuation", vec(l.attenuation));
    if (!nearly(l.cutoff, d.cutoff)) n.set("cutoff", num(l.cutoff));
    if (!nearly(l.exponent, d.exponent)) n.set("exponent", num(l.exponent));
  }

  void writeContact(XmlNode& root) {
    XmlNode n("contact");
    for (const Exclude& e : doc_.excludes) {
      if (!excludes_.count(e.id)) continue;
      XmlNode& x = n.add("exclude");
      setName(x, e.id);
      x.set("body1", nameOf(e.body1));
      x.set("body2", nameOf(e.body2));
    }
    // Explicit pairs: MuJoCo infers omitted values from the geoms, so everything is written.
    for (const ContactPair& c : doc_.pairs) {
      if (!pairs_.count(c.id)) continue;
      XmlNode& x = n.add("pair");
      setName(x, c.id);
      x.set("geom1", nameOf(c.geom1));
      x.set("geom2", nameOf(c.geom2));
      x.set("condim", std::to_string(c.condim));
      x.set("friction", vec(c.friction));
      x.set("solref", vec(c.solref));
      if (!nearly(c.solreffriction, Vec2{})) x.set("solreffriction", vec(c.solreffriction));
      x.set("solimp", vec(c.solimp));
      x.set("margin", num(c.margin));
      x.set("gap", num(c.gap));
      x.set("adhesion", num(c.adhesion));
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  void writeEquality(XmlNode& root) {
    XmlNode n("equality");
    for (const Equality& e : doc_.equalities) {
      if (!equalities_.count(e.id)) continue;
      XmlNode& x = n.add(enumXml(kEqualityTypes, e.type));
      setName(x, e.id);
      bool sites = doc_.kindOf(e.obj1) == Kind::Site;
      switch (e.type) {
        case EqualityType::Connect:
        case EqualityType::Weld:
          x.set(sites ? "site1" : "body1", nameOf(e.obj1));
          if (e.obj2 != kNoId) x.set(sites ? "site2" : "body2", nameOf(e.obj2));
          if (!sites && (e.type == EqualityType::Connect || !nearly(e.anchor, Vec3{}))) x.set("anchor", vec(e.anchor));
          if (!sites && e.type == EqualityType::Weld && e.relpose) x.set("relpose", vec(e.relpos) + " " + quat(e.relquat));
          if (e.type == EqualityType::Weld && !nearly(e.torquescale, 1.0)) x.set("torquescale", num(e.torquescale));
          break;
        case EqualityType::Joint:
        case EqualityType::Tendon: {
          bool joint = e.type == EqualityType::Joint;
          x.set(joint ? "joint1" : "tendon1", nameOf(e.obj1));
          if (e.obj2 != kNoId) x.set(joint ? "joint2" : "tendon2", nameOf(e.obj2));
          if (!nearly(e.polycoef, Vec5{{0, 1, 0, 0, 0}})) x.set("polycoef", vec(e.polycoef));
          break;
        }
      }
      if (!e.active) x.set("active", "false");
      if (!nearly(e.solref, kDefaultSolref)) x.set("solref", vec(e.solref));
      if (!nearly(e.solimp, kDefaultSolimp)) x.set("solimp", vec(e.solimp));
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  void writeTendons(XmlNode& root) {
    XmlNode n("tendon");
    const Tendon d;
    for (const Tendon& t : doc_.tendons) {
      if (!tendons_.count(t.id)) continue;
      if (map_) map_->tendons.push_back(t.id);
      bool spatial = t.type == TendonType::Spatial;
      XmlNode& x = n.add(spatial ? "spatial" : "fixed");
      setName(x, t.id);
      writeLimited(x, "limited", "range", t.limited, t.range);
      writeLimited(x, "actuatorfrclimited", "actuatorfrcrange", t.actfrclimited, t.actuatorfrcrange);
      if (!nearly(t.stiffness, 0.0)) x.set("stiffness", num(t.stiffness));
      if (!nearly(t.damping, 0.0)) x.set("damping", num(t.damping));
      if (!nearly(t.frictionloss, 0.0)) x.set("frictionloss", num(t.frictionloss));
      if (!nearly(t.armature, 0.0)) x.set("armature", num(t.armature));
      if (!nearly(t.springlength, d.springlength)) x.set("springlength", vec(t.springlength));
      if (!nearly(t.margin, 0.0)) x.set("margin", num(t.margin));
      if (!nearly(t.solreflimit, kDefaultSolref)) x.set("solreflimit", vec(t.solreflimit));
      if (!nearly(t.solimplimit, kDefaultSolimp)) x.set("solimplimit", vec(t.solimplimit));
      if (!nearly(t.solreffriction, kDefaultSolref)) x.set("solreffriction", vec(t.solreffriction));
      if (!nearly(t.solimpfriction, kDefaultSolimp)) x.set("solimpfriction", vec(t.solimpfriction));
      if (spatial && !nearly(t.width, d.width)) x.set("width", num(t.width));
      if (spatial && t.rgba != d.rgba) x.set("rgba", color(t.rgba));  // <fixed> has no appearance
      if (spatial && t.material != kNoId && materials_.count(t.material)) x.set("material", nameOf(t.material));
      if (t.group != 0) x.set("group", std::to_string(t.group));
      if (!spatial) {
        for (const TendonJoint& j : t.joints) {
          XmlNode& c = x.add("joint");
          c.set("joint", nameOf(j.joint));
          c.set("coef", num(j.coef));
        }
      } else {
        for (const TendonPoint& p : t.path) {
          switch (p.type) {
            case TendonPoint::Type::Site: x.add("site").set("site", nameOf(p.obj)); break;
            case TendonPoint::Type::Geom: {
              XmlNode& c = x.add("geom");
              c.set("geom", nameOf(p.obj));
              if (p.sidesite) c.set("sidesite", nameOf(p.sidesite));
              break;
            }
            case TendonPoint::Type::Pulley: x.add("pulley").set("divisor", num(p.divisor)); break;
          }
        }
      }
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  // With autolimits MuJoCo only infers "limited" from a range with lower < upper, so an empty range
  // (a ball joint locked by its limit, or a mistake the analysis reports) needs the attribute.
  void writeLimited(XmlNode& n, const char* lim, const char* rng, bool limited, const Vec2& range) {
    if (limited) {
      if (!doc_.compiler.autolimits || !(range.x < range.y)) n.set(lim, "true");
      n.set(rng, vec(range));
    } else if (!nearly(range, Vec2{})) {
      n.set(lim, "false");
      n.set(rng, vec(range));
    }
  }

  // MJCF shortcut elements accept only some transmissions and attributes (mjcf.schema): a motor on
  // a body or a geared adhesion actuator is written as the equivalent <general> element.
  static bool shortcutFits(const Actuator& a) {
    switch (a.type) {
      case ActuatorType::General: return true;
      case ActuatorType::Adhesion:
        return a.trn == TrnType::Body && nearly(a.gear[0], 1.0) && nearly(a.lengthrange, Vec2{}) && nearly(a.damping, 0.0) &&
               nearly(a.armature, 0.0);
      case ActuatorType::Muscle: return a.trn != TrnType::Body && a.trn != TrnType::Site;
      default: return a.trn != TrnType::Body;
    }
  }
  // The gain / bias / dynamics a shortcut stands for (MuJoCo's mjs_setTo* functions). A positive
  // biasprm[2] on a position-like actuator is MuJoCo's damping ratio, exactly as for <position>.
  static Actuator asGeneral(const Actuator& a) {
    Actuator g = a;
    g.type = ActuatorType::General;
    g.dyntype = DynType::None;
    g.gaintype = GainType::Fixed;
    g.biastype = BiasType::None;
    g.dynprm = g.gainprm = g.biasprm = Vec10{};
    g.dynprm[0] = 1;
    switch (a.type) {
      case ActuatorType::General: return a;
      case ActuatorType::Motor: g.gainprm[0] = 1; break;
      case ActuatorType::Position:
      case ActuatorType::IntVelocity:
        g.gainprm[0] = a.kp;
        g.biastype = BiasType::Affine;
        g.biasprm[1] = -a.kp;
        g.biasprm[2] = a.dampratio > 0 ? a.dampratio : -a.kv;
        if (a.type == ActuatorType::IntVelocity) g.dyntype = DynType::Integrator;
        else if (a.timeconst > 0) {
          g.dyntype = DynType::FilterExact;
          g.dynprm[0] = a.timeconst;
        }
        break;
      case ActuatorType::Velocity:
        g.gainprm[0] = a.kv;
        g.biastype = BiasType::Affine;
        g.biasprm[2] = -a.kv;
        break;
      case ActuatorType::Damper:
        g.gaintype = GainType::Affine;
        g.gainprm[2] = -a.kv;
        g.ctrllimited = true;
        break;
      case ActuatorType::Cylinder:
        g.dyntype = DynType::Filter;
        g.dynprm[0] = a.cylTimeconst;
        g.gainprm[0] = a.diameter > 0 ? 0.25 * kPi * a.diameter * a.diameter : a.area;
        g.biastype = BiasType::Affine;
        for (int i = 0; i < 3; ++i) g.biasprm[i] = a.cylBias[i];
        break;
      case ActuatorType::Muscle: {
        g.dyntype = DynType::Muscle;
        g.gaintype = GainType::Muscle;
        g.biastype = BiasType::Muscle;
        g.dynprm[0] = a.muscleTimeconst.x;
        g.dynprm[1] = a.muscleTimeconst.y;
        g.dynprm[2] = a.tausmooth;
        double prm[9] = {a.muscleRange.x, a.muscleRange.y, a.muscleForce, a.muscleScale, a.lmin, a.lmax, a.vmax, a.fpmax, a.fvmax};
        for (int i = 0; i < 9; ++i) g.gainprm[i] = g.biasprm[i] = prm[i];
        break;
      }
      case ActuatorType::Adhesion:
        g.gainprm[0] = a.adhesionGain;
        g.ctrllimited = true;
        break;
    }
    return g;
  }

  void writeActuators(XmlNode& root) {
    XmlNode n("actuator");
    for (const Actuator& orig : doc_.actuators) {
      if (!actuators_.count(orig.id)) continue;
      if (map_) map_->actuators.push_back(orig.id);
      writtenActuators_.push_back(orig.id);
      Actuator converted;
      if (!shortcutFits(orig)) converted = asGeneral(orig);
      const Actuator& a = shortcutFits(orig) ? orig : converted;
      XmlNode& e = n.add(toXml(a.type));
      setName(e, a.id);
      switch (a.trn) {
        case TrnType::Joint: e.set("joint", nameOf(a.target)); break;
        case TrnType::JointInParent: e.set("jointinparent", nameOf(a.target)); break;
        case TrnType::Tendon: e.set("tendon", nameOf(a.target)); break;
        case TrnType::Site:
          e.set("site", nameOf(a.target));
          if (a.refsite != kNoId) e.set("refsite", nameOf(a.refsite));
          break;
        case TrnType::Body: e.set("body", nameOf(a.target)); break;
        case TrnType::SliderCrank:
          e.set("cranksite", nameOf(a.target));
          e.set("slidersite", nameOf(a.slidersite));
          e.set("cranklength", num(a.cranklength));
          break;
      }
      if (a.group != 0) e.set("group", std::to_string(a.group));
      // gear: write only the meaningful prefix
      int glen = 0;
      for (int i = 0; i < 6; ++i) if (!nearly(a.gear[i], 0.0)) glen = i + 1;
      if (a.type != ActuatorType::Adhesion && !(glen == 1 && nearly(a.gear[0], 1.0))) e.set("gear", vec(a.gear, std::max(1, glen)));
      // inheritrange makes MuJoCo derive the range of a servo (its activation range for
      // intvelocity / integrator dynamics) from the joint: the range itself must then be absent.
      bool integrator = a.type == ActuatorType::IntVelocity || (a.type == ActuatorType::General && a.dyntype == DynType::Integrator);
      bool inherits = a.inheritrange > 0 && (a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity ||
                                             a.type == ActuatorType::General);
      if (a.type == ActuatorType::Damper || a.type == ActuatorType::Adhesion) {
        // These elements have no ctrllimited attribute: their control range is always enforced.
        if (!nearly(a.ctrlrange, Vec2{})) e.set("ctrlrange", vec(a.ctrlrange));
      } else if (!(inherits && !integrator)) {
        writeLimited(e, "ctrllimited", "ctrlrange", a.ctrllimited, a.ctrlrange);
      } else if (!doc_.compiler.autolimits) {
        e.set("ctrllimited", "true");  // the inherited range is a limit; without autolimits MuJoCo wants it said
      }
      if (a.type != ActuatorType::Adhesion || !nearly(a.forcerange, Vec2{}))
        writeLimited(e, "forcelimited", "forcerange", a.forcelimited, a.forcerange);
      if ((a.type == ActuatorType::IntVelocity || a.type == ActuatorType::General) && !(inherits && integrator))
        writeLimited(e, "actlimited", "actrange", a.actlimited, a.actrange);
      else if (inherits && integrator && !doc_.compiler.autolimits)
        e.set("actlimited", "true");
      if (a.type != ActuatorType::Adhesion && !nearly(a.lengthrange, Vec2{})) e.set("lengthrange", vec(a.lengthrange));
      if (a.delay > 0) e.set("delay", num(a.delay));
      if (a.nsample > 0) e.set("nsample", std::to_string(a.nsample));
      if (a.interp == 1 || a.interp == 2) e.set("interp", a.interp == 1 ? "linear" : "cubic");
      if (a.type != ActuatorType::Adhesion) {
        if (!nearly(a.damping, 0.0)) e.set("damping", num(a.damping));
        if (!nearly(a.armature, 0.0)) e.set("armature", num(a.armature));
      }

      const Actuator d;
      switch (a.type) {
        case ActuatorType::Motor: break;
        case ActuatorType::Position:
          if (!nearly(a.kp, 1.0)) e.set("kp", num(a.kp));
          // MuJoCo refuses both (the analysis reports it); the damping ratio wins.
          if (!nearly(a.kv, 0.0) && !(a.dampratio > 0)) e.set("kv", num(a.kv));
          if (!nearly(a.dampratio, 0.0)) e.set("dampratio", num(a.dampratio));
          if (!nearly(a.timeconst, 0.0)) e.set("timeconst", num(a.timeconst));
          if (!nearly(a.inheritrange, 0.0)) e.set("inheritrange", num(a.inheritrange));
          break;
        case ActuatorType::IntVelocity:
          if (!nearly(a.kp, 1.0)) e.set("kp", num(a.kp));
          if (!nearly(a.kv, 0.0) && !(a.dampratio > 0)) e.set("kv", num(a.kv));
          if (!nearly(a.dampratio, 0.0)) e.set("dampratio", num(a.dampratio));
          if (!nearly(a.inheritrange, 0.0)) e.set("inheritrange", num(a.inheritrange));
          break;
        case ActuatorType::Velocity:
        case ActuatorType::Damper:
          if (!nearly(a.kv, 1.0)) e.set("kv", num(a.kv));
          break;
        case ActuatorType::Cylinder:
          if (!nearly(a.cylTimeconst, d.cylTimeconst)) e.set("timeconst", num(a.cylTimeconst));
          if (!nearly(a.area, d.area)) e.set("area", num(a.area));
          if (!nearly(a.diameter, d.diameter)) e.set("diameter", num(a.diameter));
          if (!nearly(a.cylBias, d.cylBias)) e.set("bias", vec(a.cylBias));
          break;
        case ActuatorType::Muscle:
          if (!nearly(a.muscleTimeconst, d.muscleTimeconst)) e.set("timeconst", vec(a.muscleTimeconst));
          if (!nearly(a.tausmooth, d.tausmooth)) e.set("tausmooth", num(a.tausmooth));
          if (!nearly(a.muscleRange, d.muscleRange)) e.set("range", vec(a.muscleRange));
          if (!nearly(a.muscleForce, d.muscleForce)) e.set("force", num(a.muscleForce));
          if (!nearly(a.muscleScale, d.muscleScale)) e.set("scale", num(a.muscleScale));
          if (!nearly(a.lmin, d.lmin)) e.set("lmin", num(a.lmin));
          if (!nearly(a.lmax, d.lmax)) e.set("lmax", num(a.lmax));
          if (!nearly(a.vmax, d.vmax)) e.set("vmax", num(a.vmax));
          if (!nearly(a.fpmax, d.fpmax)) e.set("fpmax", num(a.fpmax));
          if (!nearly(a.fvmax, d.fvmax)) e.set("fvmax", num(a.fvmax));
          break;
        case ActuatorType::Adhesion:
          if (!nearly(a.adhesionGain, d.adhesionGain)) e.set("gain", num(a.adhesionGain));
          break;
        case ActuatorType::General:
          if (a.dyntype != DynType::None) e.set("dyntype", toXml(a.dyntype));
          if (a.gaintype != GainType::Fixed) e.set("gaintype", toXml(a.gaintype));
          if (a.biastype != BiasType::None) e.set("biastype", toXml(a.biastype));
          if (!nearly(a.dynprm, d.dynprm)) e.set("dynprm", trimmedVec(a.dynprm));
          if (!nearly(a.gainprm, d.gainprm)) e.set("gainprm", trimmedVec(a.gainprm));
          if (!nearly(a.biasprm, d.biasprm)) e.set("biasprm", trimmedVec(a.biasprm));
          if (!nearly(a.inheritrange, 0.0)) e.set("inheritrange", num(a.inheritrange));
          if (a.actearly) e.set("actearly", "true");
          break;
      }
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  // Keyframes list every exported joint in compiled order; joints without a value get their
  // reference position. qpos values are raw MuJoCo state (radians), never converted to degrees.
  void writeKeyframes(XmlNode& root) {
    if (doc_.keyframes.empty()) return;
    // A scene that includes a model file written separately (not through the split export) cannot
    // know the model's joints: leave keyframes to that file.
    if (opt_.scope == ExportOptions::Scope::Environment && !opt_.includeFile.empty() && !scenePass()) return;
    std::vector<const Joint*> joints = writtenJoints_;
    std::vector<Id> actuators = writtenActuators_;
    std::vector<const Body*> mocap = writtenMocap_;
    if (scenePass()) {  // the include comes first: the model's joints and actuators precede ours
      joints.insert(joints.begin(), split_->joints.begin(), split_->joints.end());
      actuators.insert(actuators.begin(), split_->actuators.begin(), split_->actuators.end());
      mocap.insert(mocap.begin(), split_->mocap.begin(), split_->mocap.end());
    }
    if (joints.empty()) return;
    XmlNode n("keyframe");
    for (const Keyframe& k : doc_.keyframes) {
      std::string qpos, qvel, ctrl;
      bool anyVel = false, anyCtrl = false;
      auto add = [&](std::string& out, double v) { out += (out.empty() ? "" : " ") + num(v); };
      for (const Joint* j : joints) {
        int nq = j->type == JointType::Free ? 7 : j->type == JointType::Ball ? 4 : 1;
        int nv = j->type == JointType::Free ? 6 : j->type == JointType::Ball ? 3 : 1;
        const std::vector<double>* q = k.jointQpos(j->id);
        if (q && int(q->size()) == nq) {
          for (double v : *q) add(qpos, v);
        } else if (j->type == JointType::Free) {
          const Body* b = doc_.body(j->body);
          Vec3 p = b ? b->pos : Vec3{};
          Quat r = b ? b->quat : Quat{};
          for (double v : {p.x, p.y, p.z, r.w, r.x, r.y, r.z}) add(qpos, v);
        } else if (j->type == JointType::Ball) {
          for (double v : {1.0, 0.0, 0.0, 0.0}) add(qpos, v);
        } else {
          add(qpos, j->ref);
        }
        const std::vector<double>* dq = nullptr;
        for (const auto& e : k.qvel) if (e.first == j->id) dq = &e.second;
        for (int i = 0; i < nv; ++i) {
          double v = dq && int(dq->size()) == nv ? (*dq)[i] : 0.0;
          anyVel |= v != 0;
          add(qvel, v);
        }
      }
      std::string act, mpos, mquat;
      bool anyAct = false, anyMocap = false;
      for (Id a : actuators) {
        double v = 0;
        for (const auto& e : k.ctrl) if (e.first == a) v = e.second;
        anyCtrl |= v != 0;
        add(ctrl, v);
        // Activations of filtered and integrating actuators, in the same order.
        const Actuator* x = doc_.actuator(a);
        if (!x || !actuatorHasActivation(*x)) continue;
        double s = 0;
        for (const auto& e : k.act) if (e.first == a) s = e.second;
        anyAct |= s != 0;
        add(act, s);
      }
      // Mocap bodies: world poses, by default where the body is.
      for (const Body* b : mocap) {
        Pose p{b->pos, b->quat};
        for (const auto& e : k.mocap)
          if (e.first == b->id) p = e.second;
        anyMocap |= !nearly(p.pos, b->pos) || !nearly(p.quat, b->quat);
        for (double v : {p.pos.x, p.pos.y, p.pos.z}) add(mpos, v);
        for (double v : {p.quat.w, p.quat.x, p.quat.y, p.quat.z}) add(mquat, v);
      }
      XmlNode& e = n.add("key");
      if (!k.name.empty()) e.set("name", k.name);
      if (k.time != 0) e.set("time", num(k.time));
      e.set("qpos", qpos);
      if (anyVel) e.set("qvel", qvel);
      if (anyAct) e.set("act", act);
      if (anyCtrl) e.set("ctrl", ctrl);
      if (anyMocap) {
        e.set("mpos", mpos);
        e.set("mquat", mquat);
      }
    }
    root.children.push_back(n);
  }

  std::string trimmedVec(const Vec10& v) const {
    int len = 1;
    for (int i = 0; i < 10; ++i) if (!nearly(v[i], 0.0)) len = i + 1;
    return vec(v, len);
  }

  void writeSensors(XmlNode& root) {
    XmlNode n("sensor");
    for (const Sensor& s : doc_.sensors) {
      if (!sensors_.count(s.id)) continue;
      if (map_) map_->sensors.push_back(s.id);
      const SensorInfo& info = sensorInfo(s.type);
      XmlNode& e = n.add(info.xml);
      setName(e, s.id);
      switch (info.obj) {
        case SensorObj::Joint: e.set("joint", nameOf(s.obj)); break;
        case SensorObj::Actuator: e.set("actuator", nameOf(s.obj)); break;
        case SensorObj::Site:
          e.set("site", nameOf(s.obj));
          if (s.type == SensorType::RangeFinder && s.raydata != 1 && s.raydata != 0) {
            std::string data;
            for (int i = 0; i < int(std::size(kRayData)); ++i)
              if (s.raydata & (1 << i)) data += (data.empty() ? "" : " ") + std::string(kRayData[i].xml);
            e.set("data", data);
          }
          break;
        case SensorObj::Body: e.set("body", nameOf(s.obj)); break;
        case SensorObj::Tendon: e.set("tendon", nameOf(s.obj)); break;
        case SensorObj::Frame:
          e.set("objtype", toXml(s.objtype));
          e.set("objname", nameOf(s.obj));
          if (s.ref != kNoId) {
            e.set("reftype", toXml(s.reftype));
            e.set("refname", nameOf(s.ref));
          }
          break;
        case SensorObj::GeomPair:
          e.set(s.objtype == FrameObj::Body ? "body1" : "geom1", nameOf(s.obj));
          e.set(s.reftype == FrameObj::Body ? "body2" : "geom2", nameOf(s.ref));
          break;
        case SensorObj::Contact: {
          if (s.obj != kNoId) {
            const char* a = s.objtype == FrameObj::Site ? "site" : s.objtype == FrameObj::Body ? "body1"
                            : s.objtype == FrameObj::XBody ? "subtree1" : "geom1";
            e.set(a, nameOf(s.obj));
          }
          if (s.ref != kNoId)
            e.set(s.reftype == FrameObj::Body ? "body2" : s.reftype == FrameObj::XBody ? "subtree2" : "geom2", nameOf(s.ref));
          if (s.connum != 1) e.set("num", std::to_string(std::max(1, s.connum)));
          if (s.condata != 1 && s.condata != 0) {
            std::string data;
            for (int i = 0; i < int(std::size(kContactData)); ++i)
              if (s.condata & (1 << i)) data += (data.empty() ? "" : " ") + std::string(kContactData[i].xml);
            e.set("data", data);
          }
          if (s.reduce > 0 && s.reduce < int(std::size(kContactReduce))) e.set("reduce", kContactReduce[s.reduce]);
          break;
        }
        case SensorObj::Inside:
          e.set("objtype", toXml(s.objtype));
          e.set("objname", nameOf(s.obj));
          e.set("site", nameOf(s.ref));
          if (s.enclosed) e.set("enclosed", "true");
          break;
        case SensorObj::CamProj:
          e.set("site", nameOf(s.obj));
          e.set("camera", nameOf(s.ref));
          break;
        case SensorObj::None: break;
      }
      if (!nearly(s.noise, 0.0)) e.set("noise", num(s.noise));
      if (!nearly(s.cutoff, 0.0)) e.set("cutoff", num(s.cutoff));
      if (s.nsample > 0) e.set("nsample", std::to_string(s.nsample));
      if (s.interp == 1 || s.interp == 2) e.set("interp", s.interp == 1 ? "linear" : "cubic");
      if (s.delay > 0) e.set("delay", num(s.delay));
      if (!nearly(s.interval, Vec2{})) e.set("interval", vec(s.interval));
    }
    if (!n.children.empty()) root.children.push_back(n);
  }

  // An image used by a texture: copied next to the model when exporting files, else referenced.
  std::string imageFile(const std::string& file) {
    std::string image;
    if (!files_ || !readTextFile(file, image)) return file;
    std::filesystem::path src = std::filesystem::u8path(file);
    std::string path = opt_.assetDir + "/" + assetFileName(src.stem().u8string(), src.extension().u8string());
    files_->push_back({path, std::move(image)});
    return path;
  }

  // File names are restricted to portable characters and kept unique after that.
  std::string assetFileName(const std::string& name, const std::string& ext) {
    std::string base;
    for (char c : name) base += std::isalnum((unsigned char)c) || c == '_' || c == '-' || c == '.' ? c : '_';
    if (base.empty()) base = "asset";
    std::string file = base + ext;
    for (int k = 2; !assetFiles_.insert(file).second; ++k) file = base + "_" + std::to_string(k) + ext;
    return file;
  }

  std::vector<const Joint*> writtenJoints_;
  std::vector<Id> writtenActuators_;
  std::vector<const Body*> writtenMocap_;
  const Document& doc_;
  const ExportOptions& opt_;
  ExportMap* map_;
  std::vector<ExportFile>* files_ = nullptr;
  SplitState* split_ = nullptr;
  std::unordered_set<std::string> assetFiles_;
  std::unordered_map<Id, double> hfieldOffset_;  // height of the lowest sample, per height field
  std::unordered_map<Id, std::string> names_;
  std::unordered_set<Id> bodies_, actuators_, sensors_, materials_, textures_, hfields_, meshes_, tendons_, equalities_,
      pairs_, excludes_;
};

}  // namespace

namespace {
bool writeExportFiles(const std::filesystem::path& dir, const std::string& assetDir, const std::vector<ExportFile>& files,
                      std::string* error, std::vector<std::string>* written) {
  namespace fs = std::filesystem;
  if (!files.empty()) {
    std::error_code ec;
    fs::create_directories(dir / assetDir, ec);
    if (ec) {
      if (error) *error = "Cannot create " + (dir / assetDir).u8string() + ": " + ec.message();
      return false;
    }
  }
  for (const ExportFile& f : files) {
    std::string fp = (dir / f.path).u8string();
    if (!writeTextFile(fp, f.content, error)) return false;
    if (written) written->push_back(fp);
  }
  return true;
}
}  // namespace

bool exportSplitMjcf(const Document& doc, const std::string& scenePath, const ExportOptions& optIn, std::string* error,
                     std::vector<std::string>* written) {
  namespace fs = std::filesystem;
  fs::path p(scenePath);
  std::string stem = p.stem().u8string();
  fs::path modelPath = p.parent_path() / (stem + "_model.xml");
  ExportOptions base = optIn;
  base.externalMeshVertices = 1000;
  base.externalHfieldSamples = 64 * 64;
  base.assetDir = stem + "_assets";  // one folder for both files
  ExportOptions mo = base, so = base;
  mo.scope = ExportOptions::Scope::Models;
  mo.modelRoot = kNoId;
  mo.includeFile.clear();
  so.scope = ExportOptions::Scope::Environment;
  so.includeFile = modelPath.filename().u8string();
  SplitState split;
  std::vector<ExportFile> files;
  std::string modelXml = Writer(doc, mo, nullptr, &files, &split).run();
  std::string sceneXml = Writer(doc, so, nullptr, &files, &split).run();
  if (!writeExportFiles(p.parent_path(), base.assetDir, files, error, written)) return false;
  if (!writeTextFile(modelPath.u8string(), modelXml, error)) return false;
  if (written) written->push_back(modelPath.u8string());
  if (!writeTextFile(scenePath, sceneXml, error)) return false;
  if (written) written->push_back(scenePath);
  return true;
}

std::string writeMjcf(const Document& doc, const ExportOptions& opt, ExportMap* map, std::vector<ExportFile>* files) {
  if (map) *map = {};
  Writer w(doc, opt, map, files);
  return w.run();
}

bool exportMjcfFile(const Document& doc, const std::string& path, const ExportOptions& optIn, std::string* error,
                    const std::string& assetStem, std::vector<std::string>* written) {
  namespace fs = std::filesystem;
  fs::path p(path);
  ExportOptions opt = optIn;
  opt.externalMeshVertices = 1000;
  opt.externalHfieldSamples = 64 * 64;
  opt.assetDir = (assetStem.empty() ? p.stem().u8string() : assetStem) + "_assets";
  std::vector<ExportFile> files;
  std::string xml = writeMjcf(doc, opt, nullptr, &files);
  if (!writeExportFiles(p.parent_path(), opt.assetDir, files, error, written)) return false;
  if (!writeTextFile(path, xml, error)) return false;
  if (written) written->push_back(path);
  return true;
}

std::string hfieldToBin(const Hfield& h) {
  size_t nrow = size_t(std::max(h.nrow, 0)), ncol = size_t(std::max(h.ncol, 0));
  std::string out(8 + nrow * ncol * 4, '\0');
  int32_t dims[2] = {int32_t(nrow), int32_t(ncol)};
  std::memcpy(&out[0], dims, 8);
  if (!h.data || h.data->size() != nrow * ncol) return out;
  // Documents keep MJCF's row order (first row = +Y edge); the binary format starts at -Y.
  for (size_t r = 0; r < nrow; ++r)
    std::memcpy(&out[8 + r * ncol * 4], h.data->data() + (nrow - 1 - r) * ncol, ncol * 4);
  return out;
}

std::string meshToStl(const MeshAsset& mesh) {
  // Binary STL (little endian): 80-byte header, triangle count, then per triangle the normal, three
  // vertices and a 16-bit attribute word.
  size_t ntri = mesh.faceCount();
  std::string out(84 + ntri * 50, '\0');
  const char header[] = "MJForge mesh";
  std::memcpy(&out[0], header, sizeof header - 1);
  uint32_t n32 = uint32_t(ntri);
  std::memcpy(&out[80], &n32, 4);
  const std::vector<float>& v = *mesh.vert;
  const std::vector<int>& f = *mesh.face;
  char* p = &out[84];
  for (size_t t = 0; t < ntri; ++t, p += 50) {
    float tri[12] = {};
    for (int k = 0; k < 3; ++k) {
      size_t vi = size_t(f[3 * t + k]);
      if (3 * vi + 2 >= v.size()) continue;  // damaged index: degenerate triangle instead of a bad read
      for (int c = 0; c < 3; ++c) tri[3 + 3 * k + c] = v[3 * vi + c];
    }
    Vec3 a{tri[3], tri[4], tri[5]}, b{tri[6], tri[7], tri[8]}, c{tri[9], tri[10], tri[11]};
    Vec3 nrm = normalize(cross(b - a, c - a), {0, 0, 1});
    tri[0] = float(nrm.x);
    tri[1] = float(nrm.y);
    tri[2] = float(nrm.z);
    std::memcpy(p, tri, sizeof tri);
  }
  return out;
}

}  // namespace mjf
