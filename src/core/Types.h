// Enumerations of the MJCF vocabulary with their XML spellings and UI labels.
#pragma once

#include <cstdint>
#include <cstring>
#include <string>

namespace mjf {

template <class E> struct EnumEntry {
  E value;
  const char* xml;    // spelling in MJCF
  const char* label;  // human-readable label for the UI
};

// Values match mjtGeom.
enum class GeomType : uint8_t { Plane = 0, Hfield = 1, Sphere = 2, Capsule = 3, Ellipsoid = 4, Cylinder = 5, Box = 6, Mesh = 7 };
inline constexpr EnumEntry<GeomType> kGeomTypes[] = {
    {GeomType::Plane, "plane", "Plane"},       {GeomType::Hfield, "hfield", "Height field"},
    {GeomType::Sphere, "sphere", "Sphere"},    {GeomType::Capsule, "capsule", "Capsule"},
    {GeomType::Ellipsoid, "ellipsoid", "Ellipsoid"}, {GeomType::Cylinder, "cylinder", "Cylinder"},
    {GeomType::Box, "box", "Box"},             {GeomType::Mesh, "mesh", "Mesh"}};

// Values match mjtJoint.
enum class JointType : uint8_t { Free = 0, Ball = 1, Slide = 2, Hinge = 3 };
inline constexpr EnumEntry<JointType> kJointTypes[] = {
    {JointType::Free, "free", "Free (6 DoF)"}, {JointType::Ball, "ball", "Ball (3 DoF)"},
    {JointType::Slide, "slide", "Slide (linear)"}, {JointType::Hinge, "hinge", "Hinge (rotary)"}};

// MJCF actuator shortcut elements.
enum class ActuatorType : uint8_t { Motor, Position, Velocity, IntVelocity, Damper, Cylinder, Muscle, Adhesion, General };
inline constexpr EnumEntry<ActuatorType> kActuatorTypes[] = {
    {ActuatorType::Motor, "motor", "Motor (torque/force control)"},
    {ActuatorType::Position, "position", "Position servo (target angle)"},
    {ActuatorType::Velocity, "velocity", "Velocity servo (target speed)"},
    {ActuatorType::IntVelocity, "intvelocity", "Integrated velocity (speed -> position target)"},
    {ActuatorType::Damper, "damper", "Damper (controllable damping)"},
    {ActuatorType::Cylinder, "cylinder", "Cylinder (pneumatic/hydraulic)"},
    {ActuatorType::Muscle, "muscle", "Muscle"},
    {ActuatorType::Adhesion, "adhesion", "Adhesion (suction/gecko)"},
    {ActuatorType::General, "general", "General (custom gain/bias/dynamics)"}};

// Values match mjtTrn.
enum class TrnType : uint8_t { Joint = 0, JointInParent = 1, SliderCrank = 2, Tendon = 3, Site = 4, Body = 5 };
inline constexpr EnumEntry<TrnType> kTrnTypes[] = {
    {TrnType::Joint, "joint", "Joint"}, {TrnType::JointInParent, "jointinparent", "Joint in parent frame"},
    {TrnType::SliderCrank, "cranksite", "Slider-crank"}, {TrnType::Tendon, "tendon", "Tendon"},
    {TrnType::Site, "site", "Site"}, {TrnType::Body, "body", "Body"}};

enum class DynType : uint8_t { None = 0, Integrator = 1, Filter = 2, FilterExact = 3, Muscle = 4 };
inline constexpr EnumEntry<DynType> kDynTypes[] = {
    {DynType::None, "none", "None (stateless)"}, {DynType::Integrator, "integrator", "Integrator"},
    {DynType::Filter, "filter", "Filter (1st order)"}, {DynType::FilterExact, "filterexact", "Filter (exact)"},
    {DynType::Muscle, "muscle", "Muscle"}};

enum class GainType : uint8_t { Fixed = 0, Affine = 1, Muscle = 2 };
inline constexpr EnumEntry<GainType> kGainTypes[] = {
    {GainType::Fixed, "fixed", "Fixed"}, {GainType::Affine, "affine", "Affine"}, {GainType::Muscle, "muscle", "Muscle"}};

enum class BiasType : uint8_t { None = 0, Affine = 1, Muscle = 2 };
inline constexpr EnumEntry<BiasType> kBiasTypes[] = {
    {BiasType::None, "none", "None"}, {BiasType::Affine, "affine", "Affine"}, {BiasType::Muscle, "muscle", "Muscle"}};

// Equality constraint types (MJCF <equality> children).
enum class EqualityType : uint8_t { Connect = 0, Weld = 1, Joint = 2, Tendon = 3 };
inline constexpr EnumEntry<EqualityType> kEqualityTypes[] = {
    {EqualityType::Connect, "connect", "Connect (ball joint)"}, {EqualityType::Weld, "weld", "Weld (fixed)"},
    {EqualityType::Joint, "joint", "Joint coupling"}, {EqualityType::Tendon, "tendon", "Tendon coupling"}};

enum class TendonType : uint8_t { Fixed = 0, Spatial = 1 };
inline constexpr EnumEntry<TendonType> kTendonTypes[] = {
    {TendonType::Fixed, "fixed", "Fixed (joint combination)"}, {TendonType::Spatial, "spatial", "Spatial (path)"}};

// What kind of object a sensor is attached to. GeomPair: two geoms or bodies (collision sensors);
// Contact: optional first and second contact criteria; Inside: a frame object and a site volume;
// CamProj: a site and a camera.
enum class SensorObj : uint8_t { None, Joint, Actuator, Site, Body, Tendon, Frame, GeomPair, Contact, Inside, CamProj };

enum class SensorType : uint8_t {
  JointPos, JointVel, BallQuat, BallAngVel, JointLimitPos, JointLimitVel, JointLimitFrc, JointActuatorFrc,
  ActuatorPos, ActuatorVel, ActuatorFrc,
  Accelerometer, Velocimeter, Gyro, Force, Torque, Magnetometer, Touch, RangeFinder,
  FramePos, FrameQuat, FrameXAxis, FrameYAxis, FrameZAxis, FrameLinVel, FrameAngVel, FrameLinAcc, FrameAngAcc,
  SubtreeCom, SubtreeLinVel, SubtreeAngMom,
  TendonPos, TendonVel,
  Clock, EPotential, EKinetic,
  // Added later; project files store the XML names, so the order here is free.
  TendonActuatorFrc, TendonLimitPos, TendonLimitVel, TendonLimitFrc,
  Distance, Normal, FromTo, Contact, InsideSite, CamProjection
};
struct SensorInfo {
  SensorType value;
  const char* xml;
  const char* label;
  SensorObj obj;
  int dim;
};
inline constexpr SensorInfo kSensorTypes[] = {
    {SensorType::JointPos, "jointpos", "Joint position", SensorObj::Joint, 1},
    {SensorType::JointVel, "jointvel", "Joint velocity", SensorObj::Joint, 1},
    {SensorType::BallQuat, "ballquat", "Ball joint quaternion", SensorObj::Joint, 4},
    {SensorType::BallAngVel, "ballangvel", "Ball joint angular velocity", SensorObj::Joint, 3},
    {SensorType::JointLimitPos, "jointlimitpos", "Joint limit position", SensorObj::Joint, 1},
    {SensorType::JointLimitVel, "jointlimitvel", "Joint limit velocity", SensorObj::Joint, 1},
    {SensorType::JointLimitFrc, "jointlimitfrc", "Joint limit force", SensorObj::Joint, 1},
    {SensorType::JointActuatorFrc, "jointactuatorfrc", "Joint actuator force", SensorObj::Joint, 1},
    {SensorType::ActuatorPos, "actuatorpos", "Actuator length", SensorObj::Actuator, 1},
    {SensorType::ActuatorVel, "actuatorvel", "Actuator velocity", SensorObj::Actuator, 1},
    {SensorType::ActuatorFrc, "actuatorfrc", "Actuator force", SensorObj::Actuator, 1},
    {SensorType::Accelerometer, "accelerometer", "Accelerometer (IMU)", SensorObj::Site, 3},
    {SensorType::Velocimeter, "velocimeter", "Velocimeter", SensorObj::Site, 3},
    {SensorType::Gyro, "gyro", "Gyroscope (IMU)", SensorObj::Site, 3},
    {SensorType::Force, "force", "Force (3-axis)", SensorObj::Site, 3},
    {SensorType::Torque, "torque", "Torque (3-axis)", SensorObj::Site, 3},
    {SensorType::Magnetometer, "magnetometer", "Magnetometer", SensorObj::Site, 3},
    {SensorType::Touch, "touch", "Touch (contact normal force)", SensorObj::Site, 1},
    {SensorType::RangeFinder, "rangefinder", "Range finder (distance along site z)", SensorObj::Site, 1},
    {SensorType::Contact, "contact", "Contact (found, force, ... in fixed slots)", SensorObj::Contact, 1},
    {SensorType::Distance, "distance", "Distance between two geoms/bodies", SensorObj::GeomPair, 1},
    {SensorType::Normal, "normal", "Normal between two geoms/bodies", SensorObj::GeomPair, 3},
    {SensorType::FromTo, "fromto", "Closest points of two geoms/bodies", SensorObj::GeomPair, 6},
    {SensorType::InsideSite, "insidesite", "Inside a site volume (1/0)", SensorObj::Inside, 1},
    {SensorType::CamProjection, "camprojection", "Site position in a camera image", SensorObj::CamProj, 2},
    {SensorType::FramePos, "framepos", "Frame position", SensorObj::Frame, 3},
    {SensorType::FrameQuat, "framequat", "Frame orientation", SensorObj::Frame, 4},
    {SensorType::FrameXAxis, "framexaxis", "Frame x-axis", SensorObj::Frame, 3},
    {SensorType::FrameYAxis, "frameyaxis", "Frame y-axis", SensorObj::Frame, 3},
    {SensorType::FrameZAxis, "framezaxis", "Frame z-axis (up vector)", SensorObj::Frame, 3},
    {SensorType::FrameLinVel, "framelinvel", "Frame linear velocity", SensorObj::Frame, 3},
    {SensorType::FrameAngVel, "frameangvel", "Frame angular velocity", SensorObj::Frame, 3},
    {SensorType::FrameLinAcc, "framelinacc", "Frame linear acceleration", SensorObj::Frame, 3},
    {SensorType::FrameAngAcc, "frameangacc", "Frame angular acceleration", SensorObj::Frame, 3},
    {SensorType::SubtreeCom, "subtreecom", "Subtree center of mass", SensorObj::Body, 3},
    {SensorType::SubtreeLinVel, "subtreelinvel", "Subtree linear velocity", SensorObj::Body, 3},
    {SensorType::SubtreeAngMom, "subtreeangmom", "Subtree angular momentum", SensorObj::Body, 3},
    {SensorType::TendonPos, "tendonpos", "Tendon length", SensorObj::Tendon, 1},
    {SensorType::TendonVel, "tendonvel", "Tendon velocity", SensorObj::Tendon, 1},
    {SensorType::TendonActuatorFrc, "tendonactuatorfrc", "Tendon actuator force", SensorObj::Tendon, 1},
    {SensorType::TendonLimitPos, "tendonlimitpos", "Tendon limit position", SensorObj::Tendon, 1},
    {SensorType::TendonLimitVel, "tendonlimitvel", "Tendon limit velocity", SensorObj::Tendon, 1},
    {SensorType::TendonLimitFrc, "tendonlimitfrc", "Tendon limit force", SensorObj::Tendon, 1},
    {SensorType::Clock, "clock", "Simulation time", SensorObj::None, 1},
    {SensorType::EPotential, "e_potential", "Potential energy", SensorObj::None, 1},
    {SensorType::EKinetic, "e_kinetic", "Kinetic energy", SensorObj::None, 1}};
inline const SensorInfo& sensorInfo(SensorType t) {
  for (const auto& s : kSensorTypes) if (s.value == t) return s;
  return kSensorTypes[0];
}

// Contact sensor data fields (MJCF data="..."), in the order MuJoCo requires, with their sizes.
struct ContactDataInfo { const char* xml; const char* label; int dim; };
inline constexpr ContactDataInfo kContactData[] = {
    {"found", "Found (number of matches)", 1}, {"force", "Force (contact frame)", 3}, {"torque", "Torque (contact frame)", 3},
    {"dist", "Penetration distance", 1}, {"pos", "Position", 3}, {"normal", "Normal", 3}, {"tangent", "Tangent", 3},
    {"linvel", "Relative linear velocity", 3}, {"angvel", "Relative angular velocity", 3}};
inline constexpr const char* kContactReduce[] = {"none", "mindist", "maxforce", "netforce"};
// Range finder outputs (MJCF data="...").
inline constexpr ContactDataInfo kRayData[] = {
    {"dist", "Distance (-1: no hit)", 1}, {"dir", "Ray direction", 3}, {"origin", "Ray origin", 3},
    {"point", "Hit point", 3}, {"normal", "Surface normal at the hit", 3}, {"depth", "Depth along the view axis", 1}};

// Object types a frame sensor can observe.
enum class FrameObj : uint8_t { Body, XBody, Geom, Site, Camera };
inline constexpr EnumEntry<FrameObj> kFrameObjs[] = {
    {FrameObj::Body, "body", "Body (inertial frame)"}, {FrameObj::XBody, "xbody", "Body (body frame)"},
    {FrameObj::Geom, "geom", "Geom"}, {FrameObj::Site, "site", "Site"}, {FrameObj::Camera, "camera", "Camera"}};

enum class Integrator : uint8_t { Euler = 0, RK4 = 1, Implicit = 2, ImplicitFast = 3, Discrete = 4 };
inline constexpr EnumEntry<Integrator> kIntegrators[] = {
    {Integrator::Euler, "Euler", "Euler (semi-implicit)"}, {Integrator::RK4, "RK4", "Runge-Kutta 4"},
    {Integrator::Implicit, "implicit", "Implicit"}, {Integrator::ImplicitFast, "implicitfast", "Implicit fast"},
    {Integrator::Discrete, "discrete", "Discrete (implicit contacts)"}};

// Body sleep policy (root bodies of moving trees). Values are the MJCF order, not mjtSleepPolicy.
enum class SleepPolicy : uint8_t { Auto, Never, Allowed, Init };
inline constexpr EnumEntry<SleepPolicy> kSleepPolicies[] = {
    {SleepPolicy::Auto, "auto", "Auto"}, {SleepPolicy::Never, "never", "Never sleep"},
    {SleepPolicy::Allowed, "allowed", "Allowed to sleep"}, {SleepPolicy::Init, "init", "Start asleep"}};

// How MuJoCo computes a mesh's volume, center and inertia. Values match mjtMeshInertia.
enum class MeshInertia : uint8_t { Convex = 0, Exact = 1, Legacy = 2, Shell = 3 };
inline constexpr EnumEntry<MeshInertia> kMeshInertias[] = {
    {MeshInertia::Convex, "convex", "Convex hull"}, {MeshInertia::Exact, "exact", "Exact (closed mesh)"},
    {MeshInertia::Legacy, "legacy", "Legacy (MuJoCo default)"}, {MeshInertia::Shell, "shell", "Surface (thin shell)"}};

enum class Cone : uint8_t { Pyramidal = 0, Elliptic = 1 };
inline constexpr EnumEntry<Cone> kCones[] = {{Cone::Pyramidal, "pyramidal", "Pyramidal"}, {Cone::Elliptic, "elliptic", "Elliptic"}};

enum class Solver : uint8_t { PGS = 0, CG = 1, Newton = 2 };
inline constexpr EnumEntry<Solver> kSolvers[] = {
    {Solver::PGS, "PGS", "PGS (not in MJWarp)"}, {Solver::CG, "CG", "CG"}, {Solver::Newton, "Newton", "Newton"}};

enum class Jacobian : uint8_t { Dense = 0, Sparse = 1, Auto = 2 };
inline constexpr EnumEntry<Jacobian> kJacobians[] = {
    {Jacobian::Dense, "dense", "Dense"}, {Jacobian::Sparse, "sparse", "Sparse"}, {Jacobian::Auto, "auto", "Auto"}};

enum class LightType : uint8_t { Spot, Directional, Point };
inline constexpr EnumEntry<LightType> kLightTypes[] = {
    {LightType::Spot, "spot", "Spot"}, {LightType::Directional, "directional", "Directional"}, {LightType::Point, "point", "Point"}};

enum class CameraMode : uint8_t { Fixed, Track, TrackCom, TargetBody, TargetBodyCom };
inline constexpr EnumEntry<CameraMode> kCameraModes[] = {
    {CameraMode::Fixed, "fixed", "Fixed"}, {CameraMode::Track, "track", "Track (follow position)"},
    {CameraMode::TrackCom, "trackcom", "Track subtree COM"}, {CameraMode::TargetBody, "targetbody", "Look at body"},
    {CameraMode::TargetBodyCom, "targetbodycom", "Look at body COM"}};

enum class TextureType : uint8_t { TwoD, Cube, Skybox };
inline constexpr EnumEntry<TextureType> kTextureTypes[] = {
    {TextureType::TwoD, "2d", "2D"}, {TextureType::Cube, "cube", "Cube"}, {TextureType::Skybox, "skybox", "Skybox"}};

enum class TextureBuiltin : uint8_t { None, Gradient, Checker, Flat };
inline constexpr EnumEntry<TextureBuiltin> kTextureBuiltins[] = {
    {TextureBuiltin::None, "none", "None (file)"}, {TextureBuiltin::Gradient, "gradient", "Gradient"},
    {TextureBuiltin::Checker, "checker", "Checker"}, {TextureBuiltin::Flat, "flat", "Flat"}};

enum class TextureMark : uint8_t { None, Edge, Cross, Random };
inline constexpr EnumEntry<TextureMark> kTextureMarks[] = {
    {TextureMark::None, "none", "None"}, {TextureMark::Edge, "edge", "Edge"},
    {TextureMark::Cross, "cross", "Cross"}, {TextureMark::Random, "random", "Random"}};

enum class AngleUnit : uint8_t { Radian, Degree };
inline constexpr EnumEntry<AngleUnit> kAngleUnits[] = {{AngleUnit::Radian, "radian", "Radian"}, {AngleUnit::Degree, "degree", "Degree"}};

enum class InertiaFromGeom : uint8_t { False, True, Auto };
inline constexpr EnumEntry<InertiaFromGeom> kInertiaFromGeom[] = {
    {InertiaFromGeom::False, "false", "Never"}, {InertiaFromGeom::True, "true", "Always"}, {InertiaFromGeom::Auto, "auto", "Auto"}};

// Tri-state used by MuJoCo's `limited` style attributes.
enum class TriBool : uint8_t { False, True, Auto };
inline constexpr EnumEntry<TriBool> kTriBools[] = {{TriBool::False, "false", "Off"}, {TriBool::True, "true", "On"}, {TriBool::Auto, "auto", "Auto"}};

// ----- generic helpers -----
template <class E, size_t N> const char* enumXml(const EnumEntry<E> (&table)[N], E v) {
  for (const auto& e : table) if (e.value == v) return e.xml;
  return table[0].xml;
}
template <class E, size_t N> const char* enumLabel(const EnumEntry<E> (&table)[N], E v) {
  for (const auto& e : table) if (e.value == v) return e.label;
  return table[0].label;
}
template <class E, size_t N> bool enumParse(const EnumEntry<E> (&table)[N], const char* s, E& out) {
  for (const auto& e : table) {
    if (std::strcmp(e.xml, s) == 0) { out = e.value; return true; }
  }
  return false;
}
template <class E, size_t N> E enumParseOr(const EnumEntry<E> (&table)[N], const std::string& s, E fallback) {
  E v = fallback;
  enumParse(table, s.c_str(), v);
  return v;
}

inline const char* toXml(GeomType v) { return enumXml(kGeomTypes, v); }
inline const char* toXml(JointType v) { return enumXml(kJointTypes, v); }
inline const char* toXml(ActuatorType v) { return enumXml(kActuatorTypes, v); }
inline const char* toXml(DynType v) { return enumXml(kDynTypes, v); }
inline const char* toXml(GainType v) { return enumXml(kGainTypes, v); }
inline const char* toXml(BiasType v) { return enumXml(kBiasTypes, v); }
inline const char* toXml(Integrator v) { return enumXml(kIntegrators, v); }
inline const char* toXml(Cone v) { return enumXml(kCones, v); }
inline const char* toXml(Solver v) { return enumXml(kSolvers, v); }
inline const char* toXml(Jacobian v) { return enumXml(kJacobians, v); }
inline const char* toXml(LightType v) { return enumXml(kLightTypes, v); }
inline const char* toXml(CameraMode v) { return enumXml(kCameraModes, v); }
inline const char* toXml(TextureType v) { return enumXml(kTextureTypes, v); }
inline const char* toXml(TextureBuiltin v) { return enumXml(kTextureBuiltins, v); }
inline const char* toXml(TextureMark v) { return enumXml(kTextureMarks, v); }
inline const char* toXml(FrameObj v) { return enumXml(kFrameObjs, v); }
inline const char* toXml(SensorType v) { return sensorInfo(v).xml; }
inline const char* toXml(TriBool v) { return enumXml(kTriBools, v); }
inline const char* toXml(InertiaFromGeom v) { return enumXml(kInertiaFromGeom, v); }
inline const char* toXml(AngleUnit v) { return enumXml(kAngleUnits, v); }
inline const char* toXml(SleepPolicy v) { return enumXml(kSleepPolicies, v); }
inline const char* toXml(MeshInertia v) { return enumXml(kMeshInertias, v); }

}  // namespace mjf
