#include "core/ProjectIO.h"

#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>

#include <nlohmann/json.hpp>

namespace mjf {

using json = nlohmann::json;

namespace {

// A stored quaternion as it was when it is a rotation already (so saving and loading is exact),
// normalized when a hand-edited or damaged file gives anything else.
Quat unitQuat(const Quat& q) {
  double n2 = q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z;
  return std::fabs(n2 - 1) <= 1e-12 ? q : normalize(q);
}


inline constexpr EnumEntry<Role> kRoles[] = {{Role::Model, "model", "Model"}, {Role::Environment, "environment", "Environment"}};

template <class E> struct EnumTable;
#define MJF_ENUM_TABLE(E, T) \
  template <> struct EnumTable<E> { static constexpr const auto& table() { return T; } };
MJF_ENUM_TABLE(GeomType, kGeomTypes)
MJF_ENUM_TABLE(JointType, kJointTypes)
MJF_ENUM_TABLE(ActuatorType, kActuatorTypes)
MJF_ENUM_TABLE(DcInput, kDcInputs)
MJF_ENUM_TABLE(TrnType, kTrnTypes)
MJF_ENUM_TABLE(DynType, kDynTypes)
MJF_ENUM_TABLE(GainType, kGainTypes)
MJF_ENUM_TABLE(BiasType, kBiasTypes)
MJF_ENUM_TABLE(FrameObj, kFrameObjs)
MJF_ENUM_TABLE(Integrator, kIntegrators)
MJF_ENUM_TABLE(Cone, kCones)
MJF_ENUM_TABLE(Solver, kSolvers)
MJF_ENUM_TABLE(Jacobian, kJacobians)
MJF_ENUM_TABLE(LightType, kLightTypes)
MJF_ENUM_TABLE(CameraMode, kCameraModes)
MJF_ENUM_TABLE(TextureType, kTextureTypes)
MJF_ENUM_TABLE(TextureBuiltin, kTextureBuiltins)
MJF_ENUM_TABLE(TextureMark, kTextureMarks)
MJF_ENUM_TABLE(InertiaFromGeom, kInertiaFromGeom)
MJF_ENUM_TABLE(Role, kRoles)
MJF_ENUM_TABLE(EqualityType, kEqualityTypes)
MJF_ENUM_TABLE(TendonType, kTendonTypes)
MJF_ENUM_TABLE(SleepPolicy, kSleepPolicies)
MJF_ENUM_TABLE(MeshInertia, kMeshInertias)
#undef MJF_ENUM_TABLE

// ---------------------------------------------------------------- archives
// JSON has no infinity or NaN (nlohmann would write null and the file could not be reopened):
// infinite values are stored as +-1e300, which MuJoCo also reads as unbounded.
double finite(double v) { return std::isnan(v) ? 0.0 : std::isinf(v) ? (v > 0 ? 1e300 : -1e300) : v; }

struct Saver {
  json& j;
  void operator()(const char* k, double& v) { j[k] = finite(v); }
  void operator()(const char* k, int& v) { j[k] = v; }
  void operator()(const char* k, uint32_t& v) { j[k] = v; }
  void operator()(const char* k, bool& v) { j[k] = v; }
  void operator()(const char* k, std::string& v) { j[k] = v; }
  void operator()(const char* k, std::vector<double>& v) {
    json a = json::array();
    for (double x : v) a.push_back(finite(x));
    j[k] = a;
  }
  void operator()(const char* k, Vec2& v) { j[k] = {finite(v.x), finite(v.y)}; }
  void operator()(const char* k, Vec3& v) { j[k] = {finite(v.x), finite(v.y), finite(v.z)}; }
  void operator()(const char* k, Vec4& v) { j[k] = {finite(v.x), finite(v.y), finite(v.z), finite(v.w)}; }
  void operator()(const char* k, Quat& v) { j[k] = {finite(v.w), finite(v.x), finite(v.y), finite(v.z)}; }
  void operator()(const char* k, Color& v) { j[k] = {finite(v.r), finite(v.g), finite(v.b), finite(v.a)}; }
  template <int N> void operator()(const char* k, VecN<N>& v) {
    json a = json::array();
    for (int i = 0; i < N; ++i) a.push_back(finite(v[i]));
    j[k] = a;
  }
  void operator()(const char* k, SensorType& v) { j[k] = sensorInfo(v).xml; }
  template <class E, std::enable_if_t<std::is_enum_v<E>, int> = 0> void operator()(const char* k, E& v) {
    j[k] = enumXml(EnumTable<E>::table(), v);
  }
};

struct Loader {
  const json& j;
  template <class T> bool get(const char* k, T& out) {
    auto it = j.find(k);
    if (it == j.end() || it->is_null()) return false;
    out = it->template get<T>();
    return true;
  }
  void operator()(const char* k, double& v) { get(k, v); }
  void operator()(const char* k, int& v) { get(k, v); }
  void operator()(const char* k, uint32_t& v) { get(k, v); }
  void operator()(const char* k, bool& v) { get(k, v); }
  void operator()(const char* k, std::string& v) { get(k, v); }
  void operator()(const char* k, std::vector<double>& v) { get(k, v); }
  template <class T> void arr(const char* k, T* dst, int n) {
    auto it = j.find(k);
    if (it == j.end() || !it->is_array()) return;
    for (int i = 0; i < n && i < int(it->size()); ++i)
      if ((*it)[i].is_number()) dst[i] = (*it)[i].template get<T>();  // null (old files): keep the default
  }
  void operator()(const char* k, Vec2& v) { double a[2] = {v.x, v.y}; arr(k, a, 2); v = {a[0], a[1]}; }
  void operator()(const char* k, Vec3& v) { double a[3] = {v.x, v.y, v.z}; arr(k, a, 3); v = {a[0], a[1], a[2]}; }
  void operator()(const char* k, Vec4& v) { double a[4] = {v.x, v.y, v.z, v.w}; arr(k, a, 4); v = {a[0], a[1], a[2], a[3]}; }
  void operator()(const char* k, Quat& v) {
    double a[4] = {v.w, v.x, v.y, v.z};
    arr(k, a, 4);
    v = unitQuat(Quat{a[0], a[1], a[2], a[3]});
  }
  void operator()(const char* k, Color& v) { float a[4] = {v.r, v.g, v.b, v.a}; arr(k, a, 4); v = {a[0], a[1], a[2], a[3]}; }
  template <int N> void operator()(const char* k, VecN<N>& v) { arr(k, v.v, N); }
  void operator()(const char* k, SensorType& v) {
    std::string s;
    if (!get(k, s)) return;
    for (const auto& info : kSensorTypes) if (s == info.xml) v = info.value;
  }
  template <class E, std::enable_if_t<std::is_enum_v<E>, int> = 0> void operator()(const char* k, E& v) {
    std::string s;
    if (get(k, s)) enumParse(EnumTable<E>::table(), s.c_str(), v);
  }
};

// ---------------------------------------------------------------- binary blobs
// Large numeric arrays (mesh vertices) are stored as base64 of little-endian 32-bit values.
const char* kB64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

template <class T> std::string toBase64(const std::vector<T>& v) {
  static_assert(sizeof(T) == 4, "32-bit values");
  const auto* p = reinterpret_cast<const unsigned char*>(v.data());
  size_t n = v.size() * 4;
  std::string out;
  out.reserve((n + 2) / 3 * 4);
  for (size_t i = 0; i < n; i += 3) {
    uint32_t x = uint32_t(p[i]) << 16 | (i + 1 < n ? uint32_t(p[i + 1]) << 8 : 0) | (i + 2 < n ? p[i + 2] : 0);
    out += kB64[(x >> 18) & 63];
    out += kB64[(x >> 12) & 63];
    out += i + 1 < n ? kB64[(x >> 6) & 63] : '=';
    out += i + 2 < n ? kB64[x & 63] : '=';
  }
  return out;
}

template <class T> std::vector<T> fromBase64(const std::string& s) {
  int map[256];
  for (int& m : map) m = -1;
  for (int i = 0; i < 64; ++i) map[(unsigned char)kB64[i]] = i;
  std::vector<unsigned char> bytes;
  bytes.reserve(s.size() / 4 * 3);
  uint32_t acc = 0;
  int bits = 0;
  for (char ch : s) {
    int v = map[(unsigned char)ch];
    if (v < 0) continue;  // '=' padding and whitespace
    acc = acc << 6 | uint32_t(v);
    bits += 6;
    if (bits >= 8) {
      bits -= 8;
      bytes.push_back((unsigned char)((acc >> bits) & 0xFF));
    }
  }
  std::vector<T> out(bytes.size() / 4);
  std::memcpy(out.data(), bytes.data(), out.size() * 4);
  return out;
}

template <class T> std::vector<T> readNumbers(const json& j) {
  if (j.is_string()) return fromBase64<T>(j.get<std::string>());
  if (j.is_array()) return j.get<std::vector<T>>();
  return {};
}

// ---------------------------------------------------------------- field lists
template <class V> void visit(V& v, Body& b) {
  v("id", b.id); v("name", b.name); v("parent", b.parent); v("pos", b.pos); v("quat", b.quat);
  v("mocap", b.mocap); v("gravcomp", b.gravcomp); v("sleep", b.sleep); v("hasInertial", b.hasInertial); v("mass", b.mass);
  v("ipos", b.ipos); v("iquat", b.iquat); v("diaginertia", b.diaginertia); v("role", b.role);
  v("hidden", b.hidden); v("locked", b.locked); v("expanded", b.expanded);
  v("generator", b.generator); v("genParams", b.genParams);
}
template <class V> void visit(V& v, Geom& g) {
  v("id", g.id); v("name", g.name); v("body", g.body); v("type", g.type); v("size", g.size);
  v("pos", g.pos); v("quat", g.quat); v("contype", g.contype); v("conaffinity", g.conaffinity);
  v("condim", g.condim); v("priority", g.priority); v("friction", g.friction); v("solmix", g.solmix);
  v("solref", g.solref); v("solimp", g.solimp); v("margin", g.margin); v("gap", g.gap);
  v("adhesion", g.adhesion); v("surfacevel", g.surfacevel); v("fluidEllipsoid", g.fluidEllipsoid);
  v("fluidcoef", g.fluidcoef); v("density", g.density); v("mass", g.mass); v("shellinertia", g.shellinertia); v("rgba", g.rgba);
  v("material", g.material); v("group", g.group); v("hfield", g.hfield); v("mesh", g.mesh); v("hidden", g.hidden);
  v("locked", g.locked); v("genKey", g.genKey);
}
template <class V> void visit(V& v, Joint& j) {
  v("id", j.id); v("name", j.name); v("body", j.body); v("type", j.type); v("pos", j.pos); v("axis", j.axis);
  v("limited", j.limited); v("range", j.range); v("ref", j.ref); v("springref", j.springref);
  v("stiffness", j.stiffness); v("damping", j.damping); v("armature", j.armature);
  v("frictionloss", j.frictionloss); v("margin", j.margin); v("solreflimit", j.solreflimit);
  v("solimplimit", j.solimplimit); v("solreffriction", j.solreffriction); v("solimpfriction", j.solimpfriction);
  v("actuatorfrclimited", j.actuatorfrclimited); v("actuatorfrcrange", j.actuatorfrcrange);
  v("actuatorgravcomp", j.actuatorgravcomp); v("group", j.group); v("hidden", j.hidden);
}
template <class V> void visit(V& v, Site& s) {
  v("id", s.id); v("name", s.name); v("body", s.body); v("type", s.type); v("size", s.size); v("pos", s.pos);
  v("quat", s.quat); v("rgba", s.rgba); v("material", s.material); v("group", s.group); v("hidden", s.hidden);
}
template <class V> void visit(V& v, Camera& c) {
  v("id", c.id); v("name", c.name); v("body", c.body); v("pos", c.pos); v("quat", c.quat); v("fovy", c.fovy);
  v("mode", c.mode); v("target", c.target); v("hidden", c.hidden); v("orthographic", c.orthographic);
  v("resx", c.resolution[0]); v("resy", c.resolution[1]); v("output", c.output); v("sensorsize", c.sensorsize);
  v("focal", c.focal); v("principal", c.principal); v("ipd", c.ipd);
}
template <class V> void visit(V& v, Light& l) {
  v("id", l.id); v("name", l.name); v("body", l.body); v("type", l.type); v("pos", l.pos); v("dir", l.dir);
  v("castshadow", l.castshadow); v("active", l.active); v("diffuse", l.diffuse); v("ambient", l.ambient);
  v("specular", l.specular); v("attenuation", l.attenuation); v("cutoff", l.cutoff); v("exponent", l.exponent);
  v("intensity", l.intensity); v("range", l.range); v("bulbradius", l.bulbradius); v("softness", l.softness);
  v("mode", l.mode); v("target", l.target); v("hidden", l.hidden);
}
template <class V> void visit(V& v, Actuator& a) {
  v("id", a.id); v("name", a.name); v("type", a.type); v("trn", a.trn); v("target", a.target);
  v("refsite", a.refsite); v("slidersite", a.slidersite); v("cranklength", a.cranklength); v("gear", a.gear);
  v("ctrllimited", a.ctrllimited); v("ctrlrange", a.ctrlrange); v("forcelimited", a.forcelimited);
  v("forcerange", a.forcerange); v("actlimited", a.actlimited); v("actrange", a.actrange);
  v("lengthrange", a.lengthrange); v("group", a.group); v("kp", a.kp); v("kv", a.kv);
  v("dampratio", a.dampratio); v("timeconst", a.timeconst); v("inheritrange", a.inheritrange);
  v("cylTimeconst", a.cylTimeconst); v("area", a.area); v("diameter", a.diameter); v("cylBias", a.cylBias);
  v("muscleTimeconst", a.muscleTimeconst); v("tausmooth", a.tausmooth); v("muscleRange", a.muscleRange);
  v("muscleForce", a.muscleForce); v("muscleScale", a.muscleScale); v("lmin", a.lmin); v("lmax", a.lmax);
  v("vmax", a.vmax); v("fpmax", a.fpmax); v("fvmax", a.fvmax); v("adhesionGain", a.adhesionGain);
  v("dyntype", a.dyntype); v("gaintype", a.gaintype); v("biastype", a.biastype); v("dynprm", a.dynprm);
  v("gainprm", a.gainprm); v("biasprm", a.biasprm); v("actearly", a.actearly);
  v("delay", a.delay); v("nsample", a.nsample); v("interp", a.interp); v("damping", a.damping);
  v("armature", a.armature); v("resistance", a.resistance); v("motorconst", a.motorconst);
  v("nominal", a.nominal); v("saturation", a.saturation); v("inductance", a.inductance); v("cogging", a.cogging);
  v("controller", a.controller); v("thermal", a.thermal); v("lugre", a.lugre); v("dcinput", a.dcinput);
}
template <class V> void visit(V& v, Sensor& s) {
  v("id", s.id); v("name", s.name); v("type", s.type); v("obj", s.obj); v("objtype", s.objtype);
  v("ref", s.ref); v("reftype", s.reftype); v("noise", s.noise); v("cutoff", s.cutoff);
  v("condata", s.condata); v("raydata", s.raydata); v("connum", s.connum); v("reduce", s.reduce); v("enclosed", s.enclosed);
  v("delay", s.delay); v("nsample", s.nsample); v("interp", s.interp); v("interval", s.interval);
}
template <class V> void visit(V& v, Exclude& e) {
  v("id", e.id); v("name", e.name); v("body1", e.body1); v("body2", e.body2);
}
template <class V> void visit(V& v, ContactPair& p) {
  v("id", p.id); v("name", p.name); v("geom1", p.geom1); v("geom2", p.geom2); v("condim", p.condim);
  v("friction", p.friction); v("solref", p.solref); v("solreffriction", p.solreffriction); v("solimp", p.solimp);
  v("margin", p.margin); v("gap", p.gap); v("adhesion", p.adhesion);
}
template <class V> void visit(V& v, Equality& e) {
  v("id", e.id); v("name", e.name); v("type", e.type); v("obj1", e.obj1); v("obj2", e.obj2); v("anchor", e.anchor);
  v("relpose", e.relpose); v("relpos", e.relpos); v("relquat", e.relquat); v("torquescale", e.torquescale);
  v("polycoef", e.polycoef); v("active", e.active); v("solref", e.solref); v("solimp", e.solimp);
}
// Tendon joints and path are written separately (arrays of records).
template <class V> void visit(V& v, Tendon& t) {
  v("id", t.id); v("name", t.name); v("type", t.type); v("limited", t.limited); v("range", t.range);
  v("actfrclimited", t.actfrclimited); v("actuatorfrcrange", t.actuatorfrcrange); v("stiffness", t.stiffness);
  v("damping", t.damping); v("frictionloss", t.frictionloss); v("armature", t.armature);
  v("springlength", t.springlength); v("margin", t.margin); v("solreflimit", t.solreflimit);
  v("solimplimit", t.solimplimit); v("solreffriction", t.solreffriction); v("solimpfriction", t.solimpfriction);
  v("width", t.width); v("rgba", t.rgba); v("material", t.material); v("group", t.group);
}
const char* const kTendonPointNames[] = {"site", "geom", "pulley"};
template <class V> void visit(V& v, Texture& t) {
  v("id", t.id); v("name", t.name); v("type", t.type); v("builtin", t.builtin); v("rgb1", t.rgb1);
  v("rgb2", t.rgb2); v("mark", t.mark); v("markrgb", t.markrgb); v("random", t.random);
  v("width", t.width); v("height", t.height); v("file", t.file); v("gridrows", t.gridsize[0]);
  v("gridcols", t.gridsize[1]); v("gridlayout", t.gridlayout); v("hflip", t.hflip); v("vflip", t.vflip);
  for (int i = 0; i < 6; ++i) v(kCubeFileAttrs[i], t.cubefiles[i]);
}
template <class V> void visit(V& v, Material& m) {
  v("id", m.id); v("name", m.name); v("rgba", m.rgba); v("texture", m.texture); v("texrepeat", m.texrepeat);
  v("texuniform", m.texuniform); v("emission", m.emission); v("specular", m.specular);
  v("shininess", m.shininess); v("reflectance", m.reflectance);
}
template <class V> void visit(V& v, Hfield& h) {
  v("id", h.id); v("name", h.name); v("nrow", h.nrow); v("ncol", h.ncol); v("size", h.size);
}
template <class V> void visit(V& v, OptionSettings& o) {
  v("timestep", o.timestep); v("gravity", o.gravity); v("wind", o.wind); v("magnetic", o.magnetic);
  v("density", o.density); v("viscosity", o.viscosity); v("impratio", o.impratio);
  v("integrator", o.integrator); v("cone", o.cone); v("jacobian", o.jacobian); v("solver", o.solver);
  v("iterations", o.iterations); v("tolerance", o.tolerance); v("ls_iterations", o.ls_iterations);
  v("ls_tolerance", o.ls_tolerance); v("noslip_iterations", o.noslip_iterations);
  v("ccd_iterations", o.ccd_iterations); v("sleep_tolerance", o.sleep_tolerance);
  v("noslip_tolerance", o.noslip_tolerance); v("ccd_tolerance", o.ccd_tolerance); v("o_margin", o.o_margin);
  v("o_solref", o.o_solref); v("o_solimp", o.o_solimp); v("o_friction", o.o_friction);
  v("disableactuator", o.disableactuator); v("memory", o.memory);
  v("flag_constraint", o.constraint); v("flag_equality", o.equality); v("flag_frictionloss", o.frictionloss);
  v("flag_limit", o.limit); v("flag_contact", o.contact); v("flag_spring", o.spring); v("flag_damper", o.damper);
  v("flag_gravity", o.gravityFlag); v("flag_clampctrl", o.clampctrl); v("flag_warmstart", o.warmstart);
  v("flag_filterparent", o.filterparent); v("flag_actuation", o.actuation); v("flag_refsafe", o.refsafe);
  v("flag_sensor", o.sensor); v("flag_eulerdamp", o.eulerdamp); v("flag_midphase", o.midphase);
  v("flag_nativeccd", o.nativeccd); v("flag_island", o.island); v("flag_autoreset", o.autoreset);
  v("flag_multiccd", o.multiccd); v("flag_energy", o.energy); v("flag_sleep", o.sleep);
  v("flag_override", o.override_); v("flag_fwdinv", o.fwdinv); v("flag_invdiscrete", o.invdiscrete);
}
template <class V> void visit(V& v, CompilerSettings& c) {
  v("autolimits", c.autolimits); v("inertiafromgeom", c.inertiafromgeom); v("boundmass", c.boundmass);
  v("boundinertia", c.boundinertia); v("settotalmass", c.settotalmass); v("balanceinertia", c.balanceinertia);
}
template <class V> void visit(V& v, VisualSettings& s) {
  v("headlightAmbient", s.headlightAmbient); v("headlightDiffuse", s.headlightDiffuse);
  v("headlightSpecular", s.headlightSpecular); v("azimuth", s.azimuth); v("elevation", s.elevation);
  v("offwidth", s.offwidth); v("offheight", s.offheight); v("shadowsize", s.shadowsize); v("haze", s.haze);
}

template <class T> json saveObject(T& obj) {
  json j = json::object();
  Saver s{j};
  visit(s, obj);
  return j;
}
template <class T> json saveArray(std::vector<T>& vec) {
  json a = json::array();
  for (auto& e : vec) a.push_back(saveObject(e));
  return a;
}
template <class T> void loadArray(const json& root, const char* key, std::vector<T>& out) {
  auto it = root.find(key);
  if (it == root.end() || !it->is_array()) return;
  for (const auto& item : *it) {
    T obj;
    Loader l{item};
    visit(l, obj);
    out.push_back(std::move(obj));
  }
}

// Damaged or hand-edited files: elements on missing bodies go to the world, parent cycles are
// broken, and mesh / height field arrays are made consistent, so every later step can trust them.
// Returns how deep the bodies are nested.
int repairReferences(Document& doc) {
  std::unordered_map<Id, size_t> bodyAt;
  for (size_t i = 0; i < doc.bodies.size(); ++i) bodyAt[doc.bodies[i].id] = i;
  auto isBody = [&](Id id) { return bodyAt.count(id) > 0; };
  // Walk up from each body until a body whose depth is known; meeting a body of the current walk
  // closes a cycle, which is broken there. Every body is walked once.
  std::vector<int> depth(doc.bodies.size(), -1);  // -1 unknown, -2 on the current walk
  depth[0] = 0;
  int maxDepth = 0;
  std::vector<size_t> walk;
  for (size_t i = 1; i < doc.bodies.size(); ++i) {
    for (bool cycle = true; cycle;) {
      walk.clear();
      size_t k = i;
      cycle = false;
      while (depth[k] == -1) {
        depth[k] = -2;
        walk.push_back(k);
        Body& b = doc.bodies[k];
        if (!isBody(b.parent) || b.parent == b.id) b.parent = kWorldId;
        size_t p = bodyAt[b.parent];
        if (depth[p] == -2) {  // the body the cycle returns to goes to the world; walk again
          doc.bodies[p].parent = kWorldId;
          cycle = true;
          break;
        }
        k = p;
      }
      if (cycle) {
        for (size_t w : walk) depth[w] = -1;
        continue;
      }
      int d = depth[k];
      for (auto it = walk.rbegin(); it != walk.rend(); ++it) depth[*it] = ++d;
      maxDepth = std::max(maxDepth, d);
    }
  }
  auto onBody = [&](auto& vec) {
    for (auto& e : vec)
      if (!isBody(e.body)) e.body = kWorldId;
  };
  onBody(doc.geoms);
  // A joint cannot be in the world: one whose body is missing is dropped (what referred to it then
  // reports a missing target).
  doc.joints.erase(std::remove_if(doc.joints.begin(), doc.joints.end(),
                                  [&](const Joint& j) { return !isBody(j.body) || j.body == kWorldId; }),
                   doc.joints.end());
  onBody(doc.sites);
  onBody(doc.cameras);
  onBody(doc.lights);
  for (MeshAsset& m : doc.meshes) {
    if (!m.vert || !m.face) continue;
    size_t nv = m.vert->size() / 3;
    if (m.vert->size() % 3) m.vert = std::make_shared<const std::vector<float>>(m.vert->begin(), m.vert->begin() + long(nv * 3));
    bool bad = m.face->size() % 3 != 0;
    for (int f : *m.face) bad |= f < 0 || size_t(f) >= nv;
    if (!bad) continue;
    auto faces = std::make_shared<std::vector<int>>();
    for (size_t t = 0; t + 2 < m.face->size(); t += 3) {
      int a = (*m.face)[t], b = (*m.face)[t + 1], c = (*m.face)[t + 2];
      if (a >= 0 && b >= 0 && c >= 0 && size_t(a) < nv && size_t(b) < nv && size_t(c) < nv) faces->insert(faces->end(), {a, b, c});
    }
    m.face = std::move(faces);
  }
  for (Hfield& h : doc.hfields) {
    h.nrow = std::max(h.nrow, 0);
    h.ncol = std::max(h.ncol, 0);
    size_t n = size_t(h.nrow) * size_t(h.ncol);
    if (h.data && h.data->size() != n) {
      auto data = std::make_shared<std::vector<float>>(*h.data);
      data->resize(n, 0.0f);
      h.data = std::move(data);
    }
  }
  // Built-in textures are generated at their size by MuJoCo (the inspector's range).
  for (Texture& t : doc.textures) {
    t.width = std::clamp(t.width, 1, 8192);
    t.height = std::clamp(t.height, 1, 8192);
  }
  // Values the editor never produces and MuJoCo refuses: colors, material and light properties are
  // single precision in MuJoCo; time constants, damping and gains are never negative.
  auto finiteOr = [](float& v, float def) { if (!std::isfinite(v)) v = def; };
  auto color = [&](Color& c) {
    finiteOr(c.r, 0.5f);
    finiteOr(c.g, 0.5f);
    finiteOr(c.b, 0.5f);
    finiteOr(c.a, 1.0f);
  };
  auto single = [](double& v, double def) { if (!std::isfinite(v) || std::fabs(v) > 1e30) v = def; };
  auto nonNegative = [](double& v) { v = std::isfinite(v) ? std::clamp(v, 0.0, 1e30) : 0.0; };
  for (Geom& g : doc.geoms) color(g.rgba);
  for (Site& x : doc.sites) color(x.rgba);
  for (Tendon& t : doc.tendons) {
    color(t.rgba);
    nonNegative(t.armature);
  }
  for (Material& m : doc.materials) {
    color(m.rgba);
    single(m.specular, 0.5);
    single(m.shininess, 0.5);
    single(m.reflectance, 0);
    single(m.emission, 0);
  }
  color(doc.visual.haze);
  for (Vec3* v : {&doc.visual.headlightAmbient, &doc.visual.headlightDiffuse, &doc.visual.headlightSpecular})
    for (int i = 0; i < 3; ++i) single((*v)[i], 0.3);
  const Light defLight;
  for (Light& l : doc.lights) {
    for (Vec3* v : {&l.diffuse, &l.ambient, &l.specular, &l.attenuation})
      for (int i = 0; i < 3; ++i) single((*v)[i], 0.5);
    single(l.cutoff, defLight.cutoff);
    single(l.exponent, defLight.exponent);
    single(l.intensity, defLight.intensity);
    single(l.range, defLight.range);
    single(l.bulbradius, defLight.bulbradius);
    single(l.softness, defLight.softness);
  }
  // Masses beyond any real object overflow MuJoCo's inertia computation.
  const Geom defGeom;
  for (Geom& g : doc.geoms) {
    if (!std::isfinite(g.mass) || g.mass > 1e30) g.mass = defGeom.mass;
    if (!std::isfinite(g.density) || std::fabs(g.density) > 1e30) g.density = defGeom.density;
  }
  const Body defBody;
  for (Body& b : doc.bodies) {
    if (!std::isfinite(b.mass) || std::fabs(b.mass) > 1e30) b.mass = defBody.mass;
    for (int i = 0; i < 3; ++i)
      if (!std::isfinite(b.diaginertia[i]) || std::fabs(b.diaginertia[i]) > 1e30) b.diaginertia[i] = defBody.diaginertia[i];
  }
  for (Actuator& a : doc.actuators) {
    nonNegative(a.kv);
    nonNegative(a.dampratio);
    nonNegative(a.timeconst);
    nonNegative(a.adhesionGain);
    nonNegative(a.tausmooth);
    nonNegative(a.controller[3]);  // DC motor slew rate limit
  }
  // <size memory>: digits with an optional K M G T P E suffix.
  std::string& mem = doc.option.memory;
  size_t digits = 0;
  while (digits < mem.size() && mem[digits] >= '0' && mem[digits] <= '9') ++digits;
  bool memOk = digits > 0 && digits <= 12 &&
               (digits == mem.size() || (digits + 1 == mem.size() && std::string("KMGTPE").find(mem[digits]) != std::string::npos));
  if (!memOk) mem.clear();
  return maxDepth;
}

}  // namespace

std::string projectToJson(const Document& cdoc, const std::string& editorStateJson) {
  Document& doc = const_cast<Document&>(cdoc);  // the Saver only reads
  json root;
  root["format"] = "mjforge";
  root["version"] = kProjectFormatVersion;
  root["generator"] = "MJForge " MJFORGE_VERSION;
  root["modelName"] = doc.modelName;
  root["nextId"] = doc.nextId;
  root["option"] = saveObject(doc.option);
  root["compiler"] = saveObject(doc.compiler);
  root["visual"] = saveObject(doc.visual);
  root["bodies"] = saveArray(doc.bodies);
  root["geoms"] = saveArray(doc.geoms);
  root["joints"] = saveArray(doc.joints);
  root["sites"] = saveArray(doc.sites);
  root["cameras"] = saveArray(doc.cameras);
  root["lights"] = saveArray(doc.lights);
  root["actuators"] = saveArray(doc.actuators);
  root["sensors"] = saveArray(doc.sensors);
  root["excludes"] = saveArray(doc.excludes);
  root["pairs"] = saveArray(doc.pairs);
  root["equalities"] = saveArray(doc.equalities);
  json tendons = json::array();
  for (Tendon& t : doc.tendons) {
    json o = saveObject(t);
    json joints = json::array(), path = json::array();
    for (const TendonJoint& tj : t.joints) joints.push_back({tj.joint, tj.coef});
    for (const TendonPoint& tp : t.path)
      path.push_back({{"type", kTendonPointNames[int(tp.type)]}, {"obj", tp.obj}, {"sidesite", tp.sidesite}, {"divisor", tp.divisor}});
    o["joints"] = joints;
    o["path"] = path;
    tendons.push_back(o);
  }
  root["tendons"] = tendons;
  root["materials"] = saveArray(doc.materials);
  root["textures"] = saveArray(doc.textures);
  json hf = json::array();
  for (Hfield& h : doc.hfields) {
    json o = saveObject(h);
    o["data"] = h.data ? toBase64(*h.data) : std::string();  // float32, base64
    hf.push_back(o);
  }
  root["hfields"] = hf;
  json meshes = json::array();
  for (const MeshAsset& m : doc.meshes) {
    json o;
    o["id"] = m.id;
    o["name"] = m.name;
    o["source"] = m.source;
    o["inertia"] = toXml(m.inertia);
    o["maxhullvert"] = m.maxhullvert;
    o["vertex"] = m.vert ? toBase64(*m.vert) : std::string();  // float32, base64
    o["face"] = m.face ? toBase64(*m.face) : std::string();    // int32, base64
    meshes.push_back(o);
  }
  root["meshes"] = meshes;
  json keys = json::array();
  for (const Keyframe& k : doc.keyframes) {
    json o;
    o["name"] = k.name;
    o["time"] = k.time;
    json q = json::array(), v = json::array(), c = json::array(), a = json::array(), mo = json::array();
    for (const auto& [id, vals] : k.qpos) q.push_back({id, vals});
    for (const auto& [id, vals] : k.qvel) v.push_back({id, vals});
    for (const auto& [id, val] : k.ctrl) c.push_back({id, val});
    for (const auto& [id, val] : k.act) a.push_back({id, val});
    for (const auto& [id, p] : k.mocap)
      mo.push_back({id, std::vector<double>{p.pos.x, p.pos.y, p.pos.z, p.quat.w, p.quat.x, p.quat.y, p.quat.z}});
    o["qpos"] = q;
    o["qvel"] = v;
    o["ctrl"] = c;
    if (!a.empty()) o["act"] = a;
    if (!mo.empty()) o["mocap"] = mo;
    keys.push_back(o);
  }
  root["keyframes"] = keys;
  if (!editorStateJson.empty()) {
    json e = json::parse(editorStateJson, nullptr, false);
    if (!e.is_discarded()) root["editor"] = e;
  }
  // Invalid UTF-8 in names (from a Latin-1 file, say) must not make saving throw.
  return root.dump(1, ' ', false, json::error_handler_t::replace);
}

bool projectFromJson(Document& out, const std::string& text, std::string* error, std::string* editorStateJson) {
  json root = json::parse(text, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    if (error) *error = "Not a valid JSON document.";
    return false;
  }
  auto format = root.find("format");
  if (format == root.end() || !format->is_string() || format->get<std::string>() != "mjforge") {
    if (error) *error = "Not an MJForge project (missing \"format\": \"mjforge\").";
    return false;
  }
  auto ver = root.find("version");
  int version = ver != root.end() && ver->is_number_integer() ? ver->get<int>() : 0;
  if (version > kProjectFormatVersion) {
    if (error) *error = "Project was saved by a newer MJForge (format version " + std::to_string(version) + ").";
    return false;
  }
  try {
    Document doc;
    doc.bodies.clear();
    doc.modelName = root.value("modelName", std::string("untitled"));
    if (root.contains("option")) { Loader l{root["option"]}; visit(l, doc.option); }
    if (root.contains("compiler")) { Loader l{root["compiler"]}; visit(l, doc.compiler); }
    if (root.contains("visual")) { Loader l{root["visual"]}; visit(l, doc.visual); }
    loadArray(root, "bodies", doc.bodies);
    loadArray(root, "geoms", doc.geoms);
    loadArray(root, "joints", doc.joints);
    loadArray(root, "sites", doc.sites);
    loadArray(root, "cameras", doc.cameras);
    loadArray(root, "lights", doc.lights);
    loadArray(root, "actuators", doc.actuators);
    loadArray(root, "sensors", doc.sensors);
    loadArray(root, "excludes", doc.excludes);
    loadArray(root, "pairs", doc.pairs);
    loadArray(root, "equalities", doc.equalities);
    if (root.contains("tendons") && root["tendons"].is_array()) {
      for (const auto& item : root["tendons"]) {
        Tendon t;
        Loader l{item};
        visit(l, t);
        if (item.contains("joints") && item["joints"].is_array())
          for (const auto& e : item["joints"])
            if (e.is_array() && e.size() == 2) t.joints.push_back({e[0].get<Id>(), e[1].get<double>()});
        if (item.contains("path") && item["path"].is_array())
          for (const auto& e : item["path"]) {
            TendonPoint tp;
            std::string type = e.value("type", std::string("site"));
            for (int i = 0; i < 3; ++i)
              if (type == kTendonPointNames[i]) tp.type = TendonPoint::Type(i);
            tp.obj = e.value("obj", Id(0));
            tp.sidesite = e.value("sidesite", Id(0));
            tp.divisor = e.value("divisor", 1.0);
            t.path.push_back(tp);
          }
        doc.tendons.push_back(std::move(t));
      }
    }
    loadArray(root, "materials", doc.materials);
    loadArray(root, "textures", doc.textures);
    if (root.contains("hfields") && root["hfields"].is_array()) {
      for (const auto& item : root["hfields"]) {
        Hfield h;
        Loader l{item};
        visit(l, h);
        if (int64_t(std::max(h.nrow, 0)) * std::max(h.ncol, 0) > kMaxHfieldSamples)
          throw std::runtime_error("height field '" + h.name + "' is too large");
        if (item.contains("data")) h.data = std::make_shared<const std::vector<float>>(readNumbers<float>(item["data"]));
        doc.hfields.push_back(std::move(h));
      }
    }
    if (root.contains("meshes") && root["meshes"].is_array()) {
      for (const auto& item : root["meshes"]) {
        MeshAsset m;
        m.id = item.value("id", Id(0));
        m.name = item.value("name", std::string());
        m.source = item.value("source", std::string());
        m.inertia = enumParseOr(kMeshInertias, item.value("inertia", std::string()), MeshInertia::Legacy);
        m.maxhullvert = item.value("maxhullvert", -1);
        if (item.contains("vertex")) m.vert = std::make_shared<const std::vector<float>>(readNumbers<float>(item["vertex"]));
        if (item.contains("face")) m.face = std::make_shared<const std::vector<int>>(readNumbers<int>(item["face"]));
        doc.meshes.push_back(std::move(m));
      }
    }
    if (root.contains("keyframes") && root["keyframes"].is_array()) {
      for (const auto& item : root["keyframes"]) {
        Keyframe k;
        k.name = item.value("name", std::string());
        k.time = item.value("time", 0.0);
        auto pairs = [&](const char* key, auto& out) {
          if (!item.contains(key) || !item[key].is_array()) return;
          for (const auto& e : item[key])
            if (e.is_array() && e.size() == 2)
              out.push_back({e[0].get<Id>(), e[1].get<typename std::decay_t<decltype(out)>::value_type::second_type>()});
        };
        pairs("qpos", k.qpos);
        pairs("qvel", k.qvel);
        pairs("ctrl", k.ctrl);
        pairs("act", k.act);
        std::vector<std::pair<Id, std::vector<double>>> mocap;
        pairs("mocap", mocap);
        for (const auto& [id, v] : mocap)
          if (v.size() == 7) k.mocap.push_back({id, Pose{{v[0], v[1], v[2]}, unitQuat(Quat{v[3], v[4], v[5], v[6]})}});
        doc.keyframes.push_back(std::move(k));
      }
    }
    // The world body must be first and have the reserved ID.
    auto wit = std::find_if(doc.bodies.begin(), doc.bodies.end(), [](const Body& b) { return b.id == kWorldId; });
    Body world;
    if (wit != doc.bodies.end()) {
      world = *wit;
      doc.bodies.erase(wit);
    }
    world.id = kWorldId;
    world.name = "world";
    world.parent = kNoId;
    world.role = Role::Environment;
    doc.bodies.insert(doc.bodies.begin(), world);

    // Every element needs its own ID: references are by ID.
    Id maxId = kWorldId;
    std::unordered_set<Id> ids;
    std::string dup;
    auto scan = [&](const auto& vec) {
      for (const auto& e : vec) {
        maxId = std::max(maxId, e.id);
        if ((e.id == kNoId || !ids.insert(e.id).second) && dup.empty()) dup = std::to_string(e.id);
      }
    };
    scan(doc.bodies); scan(doc.geoms); scan(doc.joints); scan(doc.sites); scan(doc.cameras); scan(doc.lights);
    scan(doc.actuators); scan(doc.sensors); scan(doc.excludes); scan(doc.materials); scan(doc.textures);
    scan(doc.pairs); scan(doc.equalities); scan(doc.tendons);
    scan(doc.hfields);
    scan(doc.meshes);
    if (!dup.empty()) {
      if (error) *error = "Malformed project: element ID " + dup + " is missing or used twice.";
      return false;
    }
    if (maxId >= Id(1) << 30) {
      if (error) *error = "Malformed project: element IDs out of range.";
      return false;
    }
    auto next = root.find("nextId");
    Id stored = next != root.end() && next->is_number_unsigned() && next->get<uint64_t>() < (uint64_t(1) << 30)
                    ? Id(next->get<uint64_t>()) : 0;
    doc.nextId = std::max<Id>(stored, maxId + 1);
    // MuJoCo reads at most 496 nested bodies (the analysis says so); far deeper files are damaged
    // or hostile, and would exhaust the stack of recursive code.
    int depth = repairReferences(doc);
    if (depth > kMaxBodyDepth) {
      if (error) *error = "Malformed project: bodies are nested " + std::to_string(depth) + " deep.";
      return false;
    }
    if (version < 3)
      for (Geom& g : doc.geoms)
        if (g.material != kNoId && g.rgba == Color{1, 1, 1, 1}) g.rgba = Geom{}.rgba;
    // Files from before generated pieces had keys: every geom on a generated body is a piece
    // (regenerating replaced them all), known by the body's name at the start of its own; one
    // without it gets a key no generator makes, so it is replaced the next time as it was before.
    auto geomsJson = root.find("geoms");
    if (geomsJson != root.end() && geomsJson->is_array() &&
        std::none_of(geomsJson->begin(), geomsJson->end(), [](const json& g) { return g.is_object() && g.contains("genKey"); })) {
      std::unordered_map<Id, std::string> generated;
      for (const Body& b : doc.bodies)
        if (!b.generator.empty()) generated[b.id] = b.name;
      for (Geom& g : doc.geoms) {
        auto it = generated.find(g.body);
        if (it == generated.end()) continue;
        const std::string& base = it->second;
        bool named = g.name.size() > base.size() && g.name.compare(0, base.size(), base) == 0;
        g.genKey = named ? g.name.substr(base.size()) : "?" + std::to_string(g.id);
      }
    }
    doc.touch();
    if (editorStateJson)
      *editorStateJson = root.contains("editor") ? root["editor"].dump(-1, ' ', false, json::error_handler_t::replace) : std::string();
    out = std::move(doc);
    out.touch();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = std::string("Malformed project: ") + e.what();
    return false;
  }
}

bool readTextFile(const std::string& path, std::string& out, std::string* error) {
  std::ifstream f(std::filesystem::u8path(path), std::ios::binary);
  if (!f) {
    if (error) *error = "Cannot open " + path;
    return false;
  }
  std::ostringstream ss;
  ss << f.rdbuf();
  out = ss.str();
  return true;
}

bool writeTextFile(const std::string& path, const std::string& content, std::string* error) {
  // Write a temporary file next to the target and rename it over: a failed or interrupted write
  // never destroys the previous version.
  namespace fs = std::filesystem;
  fs::path target = fs::u8path(path), tmp = fs::u8path(path + ".mjforge-tmp");
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      if (error) *error = "Cannot write " + path;
      return false;
    }
    f << content;
    f.close();
    if (!f) {
      std::error_code ec;
      fs::remove(tmp, ec);
      if (error) *error = "Error while writing " + path;
      return false;
    }
  }
  std::error_code ec;
  fs::rename(tmp, target, ec);
  if (ec) {
    fs::remove(tmp, ec);
    if (error) *error = "Cannot replace " + path + ": " + ec.message();
    return false;
  }
  return true;
}

bool saveProject(const Document& doc, const std::string& path, std::string* error, const std::string& editorStateJson) {
  return writeTextFile(path, projectToJson(doc, editorStateJson), error);
}

bool loadProject(Document& doc, const std::string& path, std::string* error, std::string* editorStateJson) {
  std::string text;
  if (!readTextFile(path, text, error)) return false;
  return projectFromJson(doc, text, error, editorStateJson);
}

}  // namespace mjf
