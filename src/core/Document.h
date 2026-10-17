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
  None, Body, Geom, Joint, Site, Camera, Light, Actuator, Sensor, Exclude, Material, Texture, Hfield, Mesh,
  Pair, Equality, Tendon
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
  SleepPolicy sleep = SleepPolicy::Auto;  // top-level moving bodies only
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
  double adhesion = 0;  // contacts can pull with up to this force (sticky surfaces, magnets with gap)
  Vec6 surfacevel{};    // surface velocity seen by contacts: linear, angular (conveyors, turntables)
  // Geom-level fluid model (needs option density/viscosity): ellipsoid approximation.
  bool fluidEllipsoid = false;
  Vec5 fluidcoef{{0.5, 0.25, 1.5, 1.0, 1.0}};
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
  std::string genKey;  // piece of its body's generator ("_step3"); empty for geoms made by hand
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
  double fovy = 45.0;  // degrees (always, regardless of compiler angle); a length when orthographic
  CameraMode mode = CameraMode::Fixed;
  Id target = kNoId;  // body for targetbody/targetbodycom
  bool hidden = false;
  // Image settings for vision policies (batch renderers and camera sensors read them).
  bool orthographic = false;
  int resolution[2] = {1, 1};  // pixels: width, height
  int output = 1;              // bits: rgb 1, depth 2, distance 4, normal 8, segmentation 16
  Vec2 sensorsize{0, 0};       // physical sensor (m): when set, focal and principal replace fovy
  Vec2 focal{0, 0};            // focal length (m)
  Vec2 principal{0, 0};        // principal point offset from the image center (m)
  double ipd = 0.068;          // stereo eye distance
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
  // For physically-based renderers (MuJoCo's classic renderer uses only range).
  double intensity = 0;     // candela
  double range = 10;        // spotlights light nothing farther than this (m)
  double bulbradius = 0.02; // soft shadows
  double softness = 0.2;    // spotlight edge, fraction of the cutoff
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
  // Control latency: the actuator reads its control `delay` seconds late from a history buffer of
  // `nsample` samples (interp: 0 zero-order hold, 1 linear, 2 cubic).
  double delay = 0;
  int nsample = 0;
  int interp = 0;
  // Damping and rotor inertia added to the joint / tendon, scaled by gear^2 (not for adhesion).
  double damping = 0;
  double armature = 0;
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
  // dcmotor (MuJoCo's DC motor model): give the resistance and motor constants, or the nominal point
  // of a datasheet; the rest is optional. Its torque limit is saturation.x (no force range).
  double resistance = 0;      // Ohm
  Vec2 motorconst{0, 0};      // torque constant Kt (N*m/A), back-EMF constant Ke (V*s/rad)
  Vec3 nominal{0, 0, 0};      // voltage (V), stall torque (N*m), no-load speed (rad/s)
  Vec3 saturation{0, 0, 0};   // max torque (N*m), max current (A), max current rate (A/s)
  Vec2 inductance{0, 0};      // inductance (H), or electrical time constant (s)
  Vec3 cogging{0, 0, 0};      // amplitude (N*m), poles, phase (rad)
  Vec6 controller{{0, 0, 0, 0, 0, 0}};  // kp ki kd (torque space), slewmax, Imax, Vmax
  Vec6 thermal{{0, 0, 0, 0, 0, 0}};     // resistance (K/W), capacitance (J/K), timeconst (s),
                                        // tempcoef (1/K), reftemp, ambient (deg C)
  Vec5 lugre{{0, 0, 0, 0, 0}};          // stiffness, damping, coulomb, static, stribeck
  DcInput dcinput = DcInput::Voltage;
};
// Whether MuJoCo gives the actuator an activation state (a control filter or integrator, a muscle):
// keyframes then hold its activation too.
bool actuatorHasActivation(const Actuator& a);
// Number of activation variables MuJoCo gives the actuator (a DC motor can have several).
int actuatorActDim(const Actuator& a);
// A DC motor's effective motor constant K and resistance R, from its constants or nominal point (as
// MuJoCo derives them, without its viscous-damping refinement). Zero when not determined.
void dcMotorConstants(const Actuator& a, double* K, double* R);
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
  // Contact sensor: matched by obj (site / geom / body / subtree, kNoId = any) and ref (geom /
  // body / subtree); reports `connum` slots of the fields in `condata` (bits of kContactData).
  int condata = 1;
  int raydata = 1;  // rangefinder outputs: bits of kRayData (distance by default)
  int connum = 1;
  int reduce = 0;  // index into kContactReduce
  bool enclosed = false;  // insidesite: whole object inside instead of its frame origin
  // Latency and sample rate: a history of `nsample` values read `delay` seconds late; the value is
  // recomputed every `interval[0]` seconds (0: every step) with phase interval[1].
  double delay = 0;
  int nsample = 0;
  int interp = 0;  // 0 zero-order hold, 1 linear, 2 cubic
  Vec2 interval{0, 0};
};
// Number of values a sensor writes to sensordata.
int sensorDim(const Sensor& s);
// History samples a sensor needs for its delay: MuJoCo keeps one per step, or one per update period
// plus the one being held when there is a period (measured with MuJoCo 3.16).
int sensorHistoryNeeded(const Sensor& s, double timestep);

struct Exclude {
  Id id = kNoId;
  std::string name;
  Id body1 = kNoId;
  Id body2 = kNoId;
};

// Explicit contact pair (<contact><pair>): the two geoms always collide, with exactly these
// parameters, whatever their contype/conaffinity say.
struct ContactPair {
  Id id = kNoId;
  std::string name;
  Id geom1 = kNoId;
  Id geom2 = kNoId;
  int condim = 3;
  Vec5 friction{{1, 1, 0.005, 0.0001, 0.0001}};  // sliding (2 directions), torsional, rolling (2)
  Vec2 solref = kDefaultSolref;
  Vec2 solreffriction{0, 0};
  Vec5 solimp = kDefaultSolimp;
  double margin = 0;
  double gap = 0;
  double adhesion = 0;
};

// Equality constraint (<equality>): a ball-joint connection or a weld between two bodies (or two
// sites), or a polynomial coupling between two joints or two tendons.
struct Equality {
  Id id = kNoId;
  std::string name;
  EqualityType type = EqualityType::Connect;
  Id obj1 = kNoId;     // body / site (connect, weld), joint, tendon
  Id obj2 = kNoId;     // optional second object: the world for bodies, nothing for a coupling
  Vec3 anchor;         // body connect: point in body1's frame; body weld: point in body2's frame
  bool relpose = false;  // weld: explicit pose of body2 in body1's frame (else the reference pose)
  Vec3 relpos;
  Quat relquat;
  double torquescale = 1;           // weld
  Vec5 polycoef{{0, 1, 0, 0, 0}};   // joint / tendon: q1 = c0 + c1 q2 + c2 q2^2 + c3 q2^3 + c4 q2^4
  bool active = true;
  Vec2 solref = kDefaultSolref;
  Vec5 solimp = kDefaultSolimp;
};

// Spatial tendon path element: a site it passes through, a sphere / cylinder geom it wraps around
// (optionally on the side of `sidesite`), or a pulley that starts a new branch.
struct TendonPoint {
  enum class Type : uint8_t { Site, Geom, Pulley };
  Type type = Type::Site;
  Id obj = kNoId;
  Id sidesite = kNoId;
  double divisor = 1;
};

struct TendonJoint {
  Id joint = kNoId;
  double coef = 1;
};

// Tendon (<tendon>): a fixed tendon is a linear combination of joint positions, a spatial tendon
// the length of a path through sites. Actuators and sensors can act on either.
struct Tendon {
  Id id = kNoId;
  std::string name;
  TendonType type = TendonType::Fixed;
  std::vector<TendonJoint> joints;  // fixed
  std::vector<TendonPoint> path;    // spatial
  bool limited = false;
  Vec2 range{0, 0};
  bool actfrclimited = false;
  Vec2 actuatorfrcrange{0, 0};
  double stiffness = 0;
  double damping = 0;
  double frictionloss = 0;
  double armature = 0;
  Vec2 springlength{-1, -1};  // -1: the length at the reference pose
  double margin = 0;
  Vec2 solreflimit = kDefaultSolref;
  Vec5 solimplimit = kDefaultSolimp;
  Vec2 solreffriction = kDefaultSolref;
  Vec5 solimpfriction = kDefaultSolimp;
  double width = 0.003;
  Color rgba{0.5f, 0.5f, 0.5f, 1};
  Id material = kNoId;
  int group = 0;
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
  // Cube / skybox from one image: grid of faces (rows, columns) and which face is in each cell,
  // row-major with L R F B U D for the faces and '.' for unused cells (MJCF gridlayout).
  int gridsize[2] = {1, 1};
  std::string gridlayout;
  // Cube / skybox from six images (used when builtin == None and file is empty).
  std::string cubefiles[6];  // right left up down front back
  bool hflip = false, vflip = false;
};
// MJCF attribute names of Texture::cubefiles.
inline constexpr const char* kCubeFileAttrs[6] = {"fileright", "fileleft", "fileup", "filedown", "filefront", "fileback"};

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
  std::vector<std::pair<Id, double>> act;                // actuator with activation -> activation
  std::vector<std::pair<Id, Pose>> mocap;                // mocap body -> world pose
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
  MeshInertia inertia = MeshInertia::Legacy;       // how MuJoCo computes volume and inertia
  int maxhullvert = -1;                            // vertex limit of the collision hull (-1: none)
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
  double noslip_tolerance = 1e-6;
  int ccd_iterations = 35;
  double ccd_tolerance = 1e-6;
  double sleep_tolerance = 1e-3;
  // Contact parameters forced on every contact while the override flag is enabled.
  double o_margin = 0;
  Vec2 o_solref = kDefaultSolref;
  Vec5 o_solimp = kDefaultSolimp;
  Vec5 o_friction{{1, 1, 0.005, 0.0001, 0.0001}};
  int disableactuator = 0;  // bit g set: actuators of group g produce no force (actuatorgroupdisable)
  std::string memory;       // <size memory>: arena size such as "100M" ("" = MuJoCo's estimate)
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
  Color haze{1.0f, 1.0f, 1.0f, 1.0f};
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
  std::vector<ContactPair> pairs;
  std::vector<Equality> equalities;
  std::vector<Tendon> tendons;
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
  ContactPair* pair(Id id) { return find(pairs, Kind::Pair, id); }
  Equality* equality(Id id) { return find(equalities, Kind::Equality, id); }
  Tendon* tendon(Id id) { return find(tendons, Kind::Tendon, id); }
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
  const ContactPair* pair(Id id) const { return const_cast<Document*>(this)->pair(id); }
  const Equality* equality(Id id) const { return const_cast<Document*>(this)->equality(id); }
  const Tendon* tendon(Id id) const { return const_cast<Document*>(this)->tendon(id); }
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
  // First moving body of a kinematic tree: it has joints and every ancestor is static. Only these
  // bodies accept a sleep policy.
  bool isTreeRoot(Id body) const;

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
