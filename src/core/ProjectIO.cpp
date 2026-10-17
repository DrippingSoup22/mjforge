#include "core/ProjectIO.h"

#include <algorithm>
#include <fstream>
#include <sstream>
#include <type_traits>

#include <nlohmann/json.hpp>

namespace mjf {

using json = nlohmann::json;

namespace {

inline constexpr EnumEntry<Role> kRoles[] = {{Role::Model, "model", "Model"}, {Role::Environment, "environment", "Environment"}};

template <class E> struct EnumTable;
#define MJF_ENUM_TABLE(E, T) \
  template <> struct EnumTable<E> { static constexpr const auto& table() { return T; } };
MJF_ENUM_TABLE(GeomType, kGeomTypes)
MJF_ENUM_TABLE(JointType, kJointTypes)
MJF_ENUM_TABLE(ActuatorType, kActuatorTypes)
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
#undef MJF_ENUM_TABLE

// ---------------------------------------------------------------- archives
struct Saver {
  json& j;
  void operator()(const char* k, double& v) { j[k] = v; }
  void operator()(const char* k, int& v) { j[k] = v; }
  void operator()(const char* k, uint32_t& v) { j[k] = v; }
  void operator()(const char* k, bool& v) { j[k] = v; }
  void operator()(const char* k, std::string& v) { j[k] = v; }
  void operator()(const char* k, Vec2& v) { j[k] = {v.x, v.y}; }
  void operator()(const char* k, Vec3& v) { j[k] = {v.x, v.y, v.z}; }
  void operator()(const char* k, Vec4& v) { j[k] = {v.x, v.y, v.z, v.w}; }
  void operator()(const char* k, Quat& v) { j[k] = {v.w, v.x, v.y, v.z}; }
  void operator()(const char* k, Color& v) { j[k] = {v.r, v.g, v.b, v.a}; }
  template <int N> void operator()(const char* k, VecN<N>& v) {
    json a = json::array();
    for (int i = 0; i < N; ++i) a.push_back(v[i]);
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
  template <class T> void arr(const char* k, T* dst, int n) {
    auto it = j.find(k);
    if (it == j.end() || !it->is_array()) return;
    for (int i = 0; i < n && i < int(it->size()); ++i) dst[i] = (*it)[i].template get<T>();
  }
  void operator()(const char* k, Vec2& v) { double a[2] = {v.x, v.y}; arr(k, a, 2); v = {a[0], a[1]}; }
  void operator()(const char* k, Vec3& v) { double a[3] = {v.x, v.y, v.z}; arr(k, a, 3); v = {a[0], a[1], a[2]}; }
  void operator()(const char* k, Vec4& v) { double a[4] = {v.x, v.y, v.z, v.w}; arr(k, a, 4); v = {a[0], a[1], a[2], a[3]}; }
  void operator()(const char* k, Quat& v) {
    double a[4] = {v.w, v.x, v.y, v.z};
    arr(k, a, 4);
    v = normalize(Quat{a[0], a[1], a[2], a[3]});
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

// ---------------------------------------------------------------- field lists
template <class V> void visit(V& v, Body& b) {
  v("id", b.id); v("name", b.name); v("parent", b.parent); v("pos", b.pos); v("quat", b.quat);
  v("mocap", b.mocap); v("gravcomp", b.gravcomp); v("hasInertial", b.hasInertial); v("mass", b.mass);
  v("ipos", b.ipos); v("iquat", b.iquat); v("diaginertia", b.diaginertia); v("role", b.role);
  v("hidden", b.hidden); v("locked", b.locked); v("expanded", b.expanded);
}
template <class V> void visit(V& v, Geom& g) {
  v("id", g.id); v("name", g.name); v("body", g.body); v("type", g.type); v("size", g.size);
  v("pos", g.pos); v("quat", g.quat); v("contype", g.contype); v("conaffinity", g.conaffinity);
  v("condim", g.condim); v("priority", g.priority); v("friction", g.friction); v("solmix", g.solmix);
  v("solref", g.solref); v("solimp", g.solimp); v("margin", g.margin); v("gap", g.gap);
  v("density", g.density); v("mass", g.mass); v("shellinertia", g.shellinertia); v("rgba", g.rgba);
  v("material", g.material); v("group", g.group); v("hfield", g.hfield); v("hidden", g.hidden); v("locked", g.locked);
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
  v("mode", c.mode); v("target", c.target); v("hidden", c.hidden);
}
template <class V> void visit(V& v, Light& l) {
  v("id", l.id); v("name", l.name); v("body", l.body); v("type", l.type); v("pos", l.pos); v("dir", l.dir);
  v("castshadow", l.castshadow); v("active", l.active); v("diffuse", l.diffuse); v("ambient", l.ambient);
  v("specular", l.specular); v("attenuation", l.attenuation); v("cutoff", l.cutoff); v("exponent", l.exponent);
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
}
template <class V> void visit(V& v, Sensor& s) {
  v("id", s.id); v("name", s.name); v("type", s.type); v("obj", s.obj); v("objtype", s.objtype);
  v("ref", s.ref); v("reftype", s.reftype); v("noise", s.noise); v("cutoff", s.cutoff);
}
template <class V> void visit(V& v, Exclude& e) {
  v("id", e.id); v("name", e.name); v("body1", e.body1); v("body2", e.body2);
}
template <class V> void visit(V& v, Texture& t) {
  v("id", t.id); v("name", t.name); v("type", t.type); v("builtin", t.builtin); v("rgb1", t.rgb1);
  v("rgb2", t.rgb2); v("mark", t.mark); v("markrgb", t.markrgb); v("random", t.random);
  v("width", t.width); v("height", t.height); v("file", t.file);
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
  v("ccd_iterations", o.ccd_iterations);
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
  root["materials"] = saveArray(doc.materials);
  root["textures"] = saveArray(doc.textures);
  json hf = json::array();
  for (Hfield& h : doc.hfields) {
    json o = saveObject(h);
    o["data"] = h.data ? json(*h.data) : json::array();
    hf.push_back(o);
  }
  root["hfields"] = hf;
  if (!editorStateJson.empty()) {
    json e = json::parse(editorStateJson, nullptr, false);
    if (!e.is_discarded()) root["editor"] = e;
  }
  return root.dump(1);
}

bool projectFromJson(Document& out, const std::string& text, std::string* error, std::string* editorStateJson) {
  json root = json::parse(text, nullptr, false);
  if (root.is_discarded() || !root.is_object()) {
    if (error) *error = "Not a valid JSON document.";
    return false;
  }
  if (root.value("format", "") != "mjforge") {
    if (error) *error = "Not an MJForge project (missing \"format\": \"mjforge\").";
    return false;
  }
  int version = root.value("version", 0);
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
    loadArray(root, "materials", doc.materials);
    loadArray(root, "textures", doc.textures);
    if (root.contains("hfields") && root["hfields"].is_array()) {
      for (const auto& item : root["hfields"]) {
        Hfield h;
        Loader l{item};
        visit(l, h);
        if (item.contains("data") && item["data"].is_array())
          h.data = std::make_shared<const std::vector<float>>(item["data"].get<std::vector<float>>());
        doc.hfields.push_back(std::move(h));
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

    Id maxId = kWorldId;
    auto scan = [&](const auto& vec) { for (const auto& e : vec) maxId = std::max(maxId, e.id); };
    scan(doc.bodies); scan(doc.geoms); scan(doc.joints); scan(doc.sites); scan(doc.cameras); scan(doc.lights);
    scan(doc.actuators); scan(doc.sensors); scan(doc.excludes); scan(doc.materials); scan(doc.textures);
    scan(doc.hfields);
    doc.nextId = std::max<Id>(root.value("nextId", 0u), maxId + 1);
    doc.touch();
    if (editorStateJson) *editorStateJson = root.contains("editor") ? root["editor"].dump() : std::string();
    out = std::move(doc);
    out.touch();
    return true;
  } catch (const std::exception& e) {
    if (error) *error = std::string("Malformed project: ") + e.what();
    return false;
  }
}

bool readTextFile(const std::string& path, std::string& out, std::string* error) {
  std::ifstream f(path, std::ios::binary);
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
  std::ofstream f(path, std::ios::binary | std::ios::trunc);
  if (!f) {
    if (error) *error = "Cannot write " + path;
    return false;
  }
  f << content;
  f.close();
  if (!f) {
    if (error) *error = "Error while writing " + path;
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
