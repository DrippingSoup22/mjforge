// The editor's document: a MuJoCo model described as flat tables of elements with stable IDs.
//
// Every field defaults to MuJoCo's own default so that the MJCF writer can omit attributes that
// were never changed. References between elements use IDs (never names), so renaming an element
// never breaks an actuator, sensor or exclusion that points at it.
#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "core/Math.h"
#include "core/Types.h"

namespace mjf {

using Id = uint32_t;
constexpr Id kNoId = 0;
constexpr Id kWorldId = 1;  // the world body always exists and has this ID

enum class Kind : uint8_t {
  None, Body, Geom, Joint, Site, Camera, Light, Actuator, Sensor, Exclude, Material, Texture, Hfield, Mesh
};
const char* kindName(Kind k);  // "body", "geom", ...

struct Ref {
  Kind kind = Kind::None;
  Id id = kNoId;
  bool valid() const { return kind != Kind::None && id != kNoId; }
  bool operator==(const Ref& o) const { return kind == o.kind && id == o.id; }
  bool operator!=(const Ref& o) const { return !(*this == o); }
};

inline const Vec2 kDefaultSolref{0.02, 1.0};
inline const Vec5 kDefaultSolimp{{0.9, 0.95, 0.001, 0.5, 2.0}};

// Whether a top-level body (and its subtree) is part of a creature/robot or of the environment.
enum class Role : uint8_t { Model, Environment };

struct Body {
  Id id = kNoId;
  std::string name;
  Id parent = kNoId;  // kNoId only for the world body
  Vec3 pos;
  Quat quat;
  bool mocap = false;
  double gravcomp = 0;
  // Explicit <inertial>. When false, MuJoCo infers mass and inertia from the geoms.
  bool hasInertial = false;
  double mass = 1.0;
  Vec3 ipos;
  Quat iquat;
  Vec3 diaginertia{0.001, 0.001, 0.001};
  // Editor metadata (not exported).
  Role role = Role::Model;  // meaningful for top-level bodies
  bool hidden = false;
  bool locked = false;
  bool expanded = true;     // outliner state
  std::string generator;          // procedural environment piece (see Generators.h), "" if none
  std::vector<double> genParams;  // its parameters
};

struct Geom {
  Id id = kNoId;
  std::string name;
  Id body = kWorldId;
  GeomType type = GeomType::Sphere;
  Vec3 size{0.05, 0.05, 0.05};
  Vec3 pos;
  Quat quat;
  // Contact.
  int contype = 1;
  int conaffinity = 1;
  int condim = 3;
  int priority = 0;
  Vec3 friction{1.0, 0.005, 0.0001};  // sliding, torsional, rolling
  double solmix = 1.0;
  Vec2 solref = kDefaultSolref;
  Vec5 solimp = kDefaultSolimp;
  double margin = 0;
  double gap = 0;
  // Mass. If mass >= 0 it overrides density.
  double density = 1000.0;
  double mass = -1.0;
  bool shellinertia = false;
  // Visual.
  Color rgba{0.5f, 0.5f, 0.5f, 1.0f};
  Id material = kNoId;
  int group = 0;
  Id hfield = kNoId;
  Id mesh = kNoId;  // type mesh
  // Editor metadata.
  bool hidden = false;
  bool locked = false;
};

struct Joint {
  Id id = kNoId;
  std::string name;
  Id body = kNoId;
  JointType type = JointType::Hinge;
  Vec3 pos;
  Vec3 axis{0, 0, 1};
  bool limited = false;
  Vec2 range{0, 0};  // radians for hinge/ball, meters for slide
  double ref = 0;
  double springref = 0;
  double stiffness = 0;
  double damping = 0;
  double armature = 0;
  double frictionloss = 0;
  double margin = 0;
  Vec2 solreflimit = kDefaultSolref;
  Vec5 solimplimit = kDefaultSolimp;
  Vec2 solreffriction = kDefaultSolref;
  Vec5 solimpfriction = kDefaultSolimp;
  bool actuatorfrclimited = false;
  Vec2 actuatorfrcrange{0, 0};
  bool actuatorgravcomp = false;
  int group = 0;
  bool hidden = false;
};

struct Site {
  Id id = kNoId;
  std::string name;
  Id body = kWorldId;
  GeomType type = GeomType::Sphere;
  Vec3 size{0.005, 0.005, 0.005};
  Vec3 pos;
  Quat quat;
  Color rgba{0.5f, 0.5f, 0.5f, 1.0f};
  Id material = kNoId;
  int group = 0;
  bool hidden = false;
};

struct Camera {
  Id id = kNoId;
  std::string name;
  Id body = kWorldId;
  Vec3 pos;
  Quat quat;  // camera looks along its local -Z axis with +Y up
  double fovy = 45.0;  // degrees (always, regardless of compiler angle)
  CameraMode mode = CameraMode::Fixed;
  Id target = kNoId;  // body for targetbody/targetbodycom
  bool hidden = false;
};

struct Light {
  Id id = kNoId;
  std::string name;
  Id body = kWorldId;
  LightType type = LightType::Spot;
  Vec3 pos;
  Vec3 dir{0, 0, -1};
  bool castshadow = true;
  bool active = true;
  Vec3 diffuse{0.7, 0.7, 0.7};
  Vec3 ambient{0, 0, 0};
  Vec3 specular{0.3, 0.3, 0.3};
  Vec3 attenuation{1, 0, 0};
  double cutoff = 45.0;   // degrees
  double exponent = 10.0;
  CameraMode mode = CameraMode::Fixed;
  Id target = kNoId;
  bool hidden = false;
};

struct Actuator {
  Id id = kNoId;
  std::string name;
  ActuatorType type = ActuatorType::Position;
  TrnType trn = TrnType::Joint;
  Id target = kNoId;      // joint / tendon / site / body depending on trn
  Id refsite = kNoId;     // site transmission: reference frame (Cartesian control)
  Id slidersite = kNoId;  // slider-crank transmission
  double cranklength = 0;
  Vec6 gear{{1, 0, 0, 0, 0, 0}};
  bool ctrllimited = true;
  Vec2 ctrlrange{-1, 1};
  bool forcelimited = false;
  Vec2 forcerange{0, 0};
  bool actlimited = false;
  Vec2 actrange{0, 0};
  Vec2 lengthrange{0, 0};
  int group = 0;
  // position / intvelocity / velocity / damper
  double kp = 1.0;
  double kv = 0.0;
  double dampratio = 0;
  double timeconst = 0;   // position: optional first-order filter on the control
  double inheritrange = 0;
  // cylinder
  double cylTimeconst = 1.0;
  double area = 1.0;
  double diameter = 0;
  Vec3 cylBias{0, 0, 0};
  // muscle
  Vec2 muscleTimeconst{0.01, 0.04};
  double tausmooth = 0;
  Vec2 muscleRange{0.75, 1.05};
  double muscleForce = -1;
  double muscleScale = 200;
  double lmin = 0.5, lmax = 1.6, vmax = 1.5, fpmax = 1.3, fvmax = 1.2;
  // adhesion
  double adhesionGain = 1.0;
  // general
  DynType dyntype = DynType::None;
  GainType gaintype = GainType::Fixed;
  BiasType biastype = BiasType::None;
  Vec10 dynprm{{1, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Vec10 gainprm{{1, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  Vec10 biasprm{{0, 0, 0, 0, 0, 0, 0, 0, 0, 0}};
  bool actearly = false;
};
// MuJoCo's default kp/kv for each actuator shortcut.
double actuatorDefaultKp(ActuatorType t);
double actuatorDefaultKv(ActuatorType t);

struct Sensor {
  Id id = kNoId;
  std::string name;
  SensorType type = SensorType::JointPos;
  Id obj = kNoId;                   // joint / actuator / site / body / tendon
  FrameObj objtype = FrameObj::Site;  // frame sensors only
  Id ref = kNoId;                   // frame sensors: optional reference frame
  FrameObj reftype = FrameObj::Site;
  double noise = 0;
  double cutoff = 0;
};

struct Exclude {
  Id id = kNoId;
  std::string name;
  Id body1 = kNoId;
  Id body2 = kNoId;
};

struct Texture {
  Id id = kNoId;
  std::string name;
  TextureType type = TextureType::TwoD;
  TextureBuiltin builtin = TextureBuiltin::Checker;
  Vec3 rgb1{0.8, 0.8, 0.8};
  Vec3 rgb2{0.5, 0.5, 0.5};
  TextureMark mark = TextureMark::None;
  Vec3 markrgb{0, 0, 0};
  double random = 0.01;
  int width = 512;
  int height = 512;
  std::string file;  // used when builtin == None
};

struct Material {
  Id id = kNoId;
  std::string name;
  Color rgba{1, 1, 1, 1};
  Id texture = kNoId;
  Vec2 texrepeat{1, 1};
  bool texuniform = false;
  double emission = 0;
  double specular = 0.5;
  double shininess = 0.5;
  double reflectance = 0;
};

// Height field asset with inline elevation data (row-major, first row = +Y edge as in MJCF).
struct Hfield {
  Id id = kNoId;
  std::string name;
  int nrow = 0;
  int ncol = 0;
  Vec4 size{1, 1, 0.1, 0.05};  // radius_x radius_y elevation_z base_z
  std::shared_ptr<const std::vector<float>> data;  // normalized to [0, 1], copy-on-write
};

// A named state (MJCF <key>). Values are stored per joint and actuator, so keyframes survive
// structural edits: joints without an entry use their reference position. The first keyframe is
// the start state of simulations (MuJoCo resets to key 0 in MJForge and most training code).
struct Keyframe {
  std::string name;
  double time = 0;
  std::vector<std::pair<Id, std::vector<double>>> qpos;  // joint -> 1 (hinge/slide), 4 (ball) or 7 (free) values
  std::vector<std::pair<Id, std::vector<double>>> qvel;  // joint -> 1, 3 or 6 values
  std::vector<std::pair<Id, double>> ctrl;               // actuator -> control
  const std::vector<double>* jointQpos(Id joint) const {
    for (const auto& e : qpos) if (e.first == joint) return &e.second;
    return nullptr;
  }
};

// Triangle mesh asset. Vertices are in the frame the geom places them in (meshes imported or
// loaded through MuJoCo are already centered on their center of volume, as MuJoCo compiles them).
// Exported inline (<mesh vertex=... face=...>), or as STL files next to big exported models.
struct MeshAsset {
  Id id = kNoId;
  std::string name;
  std::shared_ptr<const std::vector<float>> vert;  // x y z per vertex, copy-on-write
  std::shared_ptr<const std::vector<int>> face;    // three vertex indices per triangle
  std::string source;                              // file it was loaded from (information only)
  size_t vertexCount() const { return vert ? vert->size() / 3 : 0; }
  size_t faceCount() const { return face ? face->size() / 3 : 0; }
  Aabb bounds() const;  // in the mesh frame
};

struct OptionSettings {
  double timestep = 0.002;
  Vec3 gravity{0, 0, -9.81};
  Vec3 wind{0, 0, 0};
  Vec3 magnetic{0, -0.5, 0};
  double density = 0;
  double viscosity = 0;
  double impratio = 1;
  Integrator integrator = Integrator::Euler;
  Cone cone = Cone::Pyramidal;
  Jacobian jacobian = Jacobian::Auto;
  Solver solver = Solver::Newton;
  int iterations = 100;
  double tolerance = 1e-8;
  int ls_iterations = 50;
  double ls_tolerance = 0.01;
  int noslip_iterations = 0;
  int ccd_iterations = 35;
  // <flag> settings: true means enabled.
  bool constraint = true, equality = true, frictionloss = true, limit = true, contact = true;
  bool spring = true, damper = true, gravityFlag = true, clampctrl = true, warmstart = true;
  bool filterparent = true, actuation = true, refsafe = true, sensor = true, eulerdamp = true;
  bool midphase = true, nativeccd = true, island = true, autoreset = true, multiccd = true;
  bool energy = false, sleep = false, override_ = false, fwdinv = false, invdiscrete = false;
};

struct CompilerSettings {
  bool autolimits = true;
  InertiaFromGeom inertiafromgeom = InertiaFromGeom::Auto;
  double boundmass = 0;
  double boundinertia = 0;
  double settotalmass = -1;
  bool balanceinertia = false;
};

struct VisualSettings {
  // <visual><headlight>
  Vec3 headlightAmbient{0.1, 0.1, 0.1};
  Vec3 headlightDiffuse{0.4, 0.4, 0.4};
  Vec3 headlightSpecular{0.5, 0.5, 0.5};
  // <visual><global>
  double azimuth = 90;
  double elevation = -45;
  int offwidth = 640;
  int offheight = 480;
  // <visual><quality>
  int shadowsize = 4096;
  // <visual><rgba haze>
  Color haze{0.15f, 0.25f, 0.35f, 1.0f};
};

// Lookup tables rebuilt whenever the document structure changes.
struct DocIndex {
  struct Loc { Kind kind; uint32_t index; };
  std::unordered_map<Id, Loc> loc;
  std::vector<std::vector<uint32_t>> childBodies;  // per body index
  std::vector<std::vector<uint32_t>> bodyGeoms, bodyJoints, bodySites, bodyCameras, bodyLights;
  std::vector<uint32_t> dfsOrder;                    // body indices, world first, pre-order
  std::vector<int> depth;                            // per body index
};

class Document {
 public:
  Document();

  std::string modelName = "untitled";
  OptionSettings option;
  CompilerSettings compiler;
  VisualSettings visual;

  std::vector<Body> bodies;  // bodies[0] is the world
  std::vector<Geom> geoms;
  std::vector<Joint> joints;
  std::vector<Site> sites;
  std::vector<Camera> cameras;
  std::vector<Light> lights;
  std::vector<Actuator> actuators;
  std::vector<Sensor> sensors;
  std::vector<Exclude> excludes;
  std::vector<Material> materials;
  std::vector<Texture> textures;
  std::vector<Hfield> hfields;
  std::vector<MeshAsset> meshes;
  std::vector<Keyframe> keyframes;  // keyframes[0] is the start state

  Id nextId = 2;
  Id newId() { return nextId++; }

  // Call after any structural change (add/remove/reparent/reorder). Value edits don't need it.
  void touch() { ++revision_; }
  uint64_t revision() const { return revision_; }

  const DocIndex& index() const;

  // Lookups by ID (nullptr if not found or of another kind).
  Kind kindOf(Id id) const;
  Body* body(Id id) { return find(bodies, Kind::Body, id); }
  Geom* geom(Id id) { return find(geoms, Kind::Geom, id); }
  Joint* joint(Id id) { return find(joints, Kind::Joint, id); }
  Site* site(Id id) { return find(sites, Kind::Site, id); }
  Camera* camera(Id id) { return find(cameras, Kind::Camera, id); }
  Light* light(Id id) { return find(lights, Kind::Light, id); }
  Actuator* actuator(Id id) { return find(actuators, Kind::Actuator, id); }
  Sensor* sensor(Id id) { return find(sensors, Kind::Sensor, id); }
  Exclude* exclude(Id id) { return find(excludes, Kind::Exclude, id); }
  Material* material(Id id) { return find(materials, Kind::Material, id); }
  Texture* texture(Id id) { return find(textures, Kind::Texture, id); }
  Hfield* hfield(Id id) { return find(hfields, Kind::Hfield, id); }
  MeshAsset* mesh(Id id) { return find(meshes, Kind::Mesh, id); }
  const Body* body(Id id) const { return const_cast<Document*>(this)->body(id); }
  const Geom* geom(Id id) const { return const_cast<Document*>(this)->geom(id); }
  const Joint* joint(Id id) const { return const_cast<Document*>(this)->joint(id); }
  const Site* site(Id id) const { return const_cast<Document*>(this)->site(id); }
  const Camera* camera(Id id) const { return const_cast<Document*>(this)->camera(id); }
  const Light* light(Id id) const { return const_cast<Document*>(this)->light(id); }
  const Actuator* actuator(Id id) const { return const_cast<Document*>(this)->actuator(id); }
  const Sensor* sensor(Id id) const { return const_cast<Document*>(this)->sensor(id); }
  const Exclude* exclude(Id id) const { return const_cast<Document*>(this)->exclude(id); }
  const Material* material(Id id) const { return const_cast<Document*>(this)->material(id); }
  const Texture* texture(Id id) const { return const_cast<Document*>(this)->texture(id); }
  const Hfield* hfield(Id id) const { return const_cast<Document*>(this)->hfield(id); }
  const MeshAsset* mesh(Id id) const { return const_cast<Document*>(this)->mesh(id); }

  int bodyIndex(Id id) const;  // -1 if not a body
  Ref ref(Id id) const { return {kindOf(id), id}; }
  bool exists(Id id) const { return kindOf(id) != Kind::None; }

  // Name of any element ("" if unnamed / unknown). World body is "world".
  std::string nameOf(Id id) const;
  // The name, or a description such as "box geom" or "motor on hip" for unnamed elements.
  std::string displayName(Id id) const;
  void setName(Id id, const std::string& name);
  // The body an element is attached to (bodies return their parent).
  Id ownerBody(Id id) const;
  // Top-level ancestor of a body (child of the world); kWorldId for the world itself.
  Id rootOf(Id body) const;
  bool isAncestor(Id ancestor, Id body) const;  // true if ancestor == body or is above it
  Role roleOf(Id body) const;  // role of the subtree's top-level body (world: Environment)
  // All bodies in the subtree rooted at `body` (inclusive), pre-order.
  std::vector<Id> subtree(Id body) const;
  // Number of degrees of freedom the joints of a body add.
  int bodyDofs(Id body) const;

  // Unique name for the given kind, derived from base ("leg" -> "leg", "leg_2", ...).
  std::string uniqueName(Kind kind, const std::string& base) const;
  bool nameTaken(Kind kind, const std::string& name, Id except = kNoId) const;

 private:
  template <class T> T* find(std::vector<T>& v, Kind k, Id id) {
    if (id == kNoId) return nullptr;
    const DocIndex& ix = index();
    auto it = ix.loc.find(id);
    if (it != ix.loc.end() && it->second.kind == k && it->second.index < v.size() && v[it->second.index].id == id)
      return &v[it->second.index];
    // Index stale (an edit forgot touch()): rebuild once and retry.
    touch();
    const DocIndex& ix2 = index();
    auto it2 = ix2.loc.find(id);
    if (it2 != ix2.loc.end() && it2->second.kind == k && it2->second.index < v.size()) return &v[it2->second.index];
    return nullptr;
  }

  uint64_t revision_ = 1;
  mutable uint64_t indexRevision_ = 0;
  mutable DocIndex index_;
};

// Sizes of a geom as human labels, e.g. {"Radius", "Half-length", ""} for capsules.
struct SizeLabels { const char* l[3]; int count; };
SizeLabels geomSizeLabels(GeomType t);

}  // namespace mjf
