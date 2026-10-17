#include "core/Templates.h"

#include <cmath>
#include <tuple>

#include "core/Kinematics.h"
#include "core/Ops.h"

namespace mjf {

namespace {

Id limbGeom(Document& doc, Id body, const Vec3& a, const Vec3& b, double radius, const Color& c, const std::string& name) {
  Id g = addGeom(doc, body, GeomType::Capsule, {radius, 0.1, 0}, {}, name);
  setGeomFromTo(*doc.geom(g), a, b);
  doc.geom(g)->rgba = c;
  return g;
}

Id hinge(Document& doc, Id body, const Vec3& axis, double lo, double hi, const std::string& name, double damping = 0.5,
         double armature = 0.01) {
  Id j = addJoint(doc, body, JointType::Hinge, {}, axis, name);
  Joint* jj = doc.joint(j);
  jj->limited = true;
  jj->range = {lo, hi};
  jj->damping = damping;
  jj->armature = armature;
  return j;
}

Id servo(Document& doc, Id joint, double kp, double maxTorque) {
  Id a = addActuator(doc, ActuatorType::Position, joint);
  Actuator* act = doc.actuator(a);
  act->kp = kp;
  act->dampratio = 1.0;
  act->forcelimited = true;
  act->forcerange = {-maxTorque, maxTorque};
  return a;
}

const Color kShell{0.92f, 0.55f, 0.20f, 1};  // forge orange
const Color kLimb{0.30f, 0.33f, 0.38f, 1};
const Color kFoot{0.12f, 0.12f, 0.13f, 1};

}  // namespace

const std::vector<TemplateInfo>& templateList() {
  static const std::vector<TemplateInfo> list = {
      {"empty", "Empty scene", "Checker floor, sky and light. Start an environment or a creature from scratch."},
      {"quadruped", "Quadruped", "12-DoF four-legged robot with position servos, IMU and foot contact spheres."},
      {"hexapod", "Hexapod", "18-DoF six-legged walker built with a radial leg layout."},
      {"biped", "Biped", "Simple 10-DoF humanoid-style lower body with a torso."},
      {"hopper", "Hopper", "Classic planar one-legged hopper (as in RL benchmarks): torque motors, 3 planar root DoF."},
      {"arm", "Robot arm", "Fixed-base 4-DoF arm on a pedestal with a gripper-less end effector."},
  };
  return list;
}

void addDefaultEnvironment(Document& doc) {
  doc.option.integrator = Integrator::ImplicitFast;
  // Stiff friction: without it, feet under sideways load slowly creep (supported by MuJoCo Warp).
  doc.option.cone = Cone::Elliptic;
  doc.option.impratio = 10;
  doc.visual.headlightDiffuse = {0.6, 0.6, 0.6};
  doc.visual.headlightAmbient = {0.3, 0.3, 0.3};
  doc.visual.headlightSpecular = {0, 0, 0};
  doc.visual.azimuth = 120;
  doc.visual.elevation = -20;

  Texture sky;
  sky.name = "skybox";
  sky.type = TextureType::Skybox;
  sky.builtin = TextureBuiltin::Gradient;
  sky.rgb1 = {0.3, 0.5, 0.7};
  sky.rgb2 = {0, 0, 0};
  sky.width = 512;
  sky.height = 3072;
  addTexture(doc, sky);

  Texture grid;
  grid.name = "groundplane";
  grid.type = TextureType::TwoD;
  grid.builtin = TextureBuiltin::Checker;
  grid.rgb1 = {0.2, 0.3, 0.4};
  grid.rgb2 = {0.1, 0.2, 0.3};
  grid.mark = TextureMark::Edge;
  grid.markrgb = {0.8, 0.8, 0.8};
  grid.width = 300;
  grid.height = 300;
  Id gt = addTexture(doc, grid);

  Material mat;
  mat.name = "groundplane";
  mat.texture = gt;
  mat.texuniform = true;
  mat.texrepeat = {5, 5};
  mat.reflectance = 0.2;
  Id gm = addMaterial(doc, mat);

  Id floor = addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  doc.geom(floor)->material = gm;
  doc.geom(floor)->rgba = {1, 1, 1, 1};

  Id light = addLight(doc, kWorldId, {0, 0, 3.5}, {0, 0, -1}, "sun");
  doc.light(light)->type = LightType::Directional;
  doc.light(light)->diffuse = {0.6, 0.6, 0.6};
  doc.touch();
}

Id addQuadruped(Document& doc, const Vec3& pos, const std::string& name) {
  Id torso = addBody(doc, kWorldId, {pos, {}}, name);
  addJoint(doc, torso, JointType::Free, {}, {0, 0, 1}, name + "_root");
  Id tg = addGeom(doc, torso, GeomType::Ellipsoid, {0.22, 0.13, 0.07}, {}, name + "_shell");
  doc.geom(tg)->rgba = kShell;
  Id imu = addSite(doc, torso, {}, name + "_imu");
  addSensor(doc, SensorType::Accelerometer, imu);
  addSensor(doc, SensorType::Gyro, imu);
  addSensor(doc, SensorType::FrameZAxis, imu, name + "_upvector");

  struct LegDef { const char* tag; double x, y; };
  const LegDef legs[] = {{"FL", 0.17, 0.10}, {"FR", 0.17, -0.10}, {"RL", -0.17, 0.10}, {"RR", -0.17, -0.10}};
  for (const LegDef& L : legs) {
    std::string t = L.tag;
    Id thigh = addBody(doc, torso, {{L.x, L.y, 0}, axisAngle({0, 1, 0}, 0.4)}, t + "_thigh");
    Id roll = hinge(doc, thigh, {1, 0, 0}, -0.6, 0.6, t + "_hip_roll");
    Id pitch = hinge(doc, thigh, {0, 1, 0}, -1.2, 1.2, t + "_hip_pitch");
    limbGeom(doc, thigh, {0, 0, 0}, {0, 0, -0.2}, 0.025, kLimb, t + "_thigh_geom");
    Id calf = addBody(doc, thigh, {{0, 0, -0.2}, axisAngle({0, 1, 0}, -0.8)}, t + "_calf");
    Id knee = hinge(doc, calf, {0, 1, 0}, -1.4, 0.6, t + "_knee");
    limbGeom(doc, calf, {0, 0, 0}, {0, 0, -0.2}, 0.02, kLimb, t + "_calf_geom");
    Id foot = addGeom(doc, calf, GeomType::Sphere, {0.03, 0, 0}, {{0, 0, -0.2}, {}}, t + "_foot");
    doc.geom(foot)->rgba = kFoot;
    doc.geom(foot)->friction = {1.0, 0.02, 0.01};
    Id fs = addSite(doc, calf, {{0, 0, -0.2}, {}}, t + "_foot_site");
    doc.site(fs)->size = {0.035, 0.035, 0.035};
    doc.site(fs)->rgba = {0.2f, 0.9f, 0.5f, 0.3f};
    addSensor(doc, SensorType::Touch, fs, t + "_touch");
    servo(doc, roll, 30, 20);
    servo(doc, pitch, 30, 20);
    servo(doc, knee, 30, 20);
  }
  placeOnFloor(doc, torso);
  return torso;
}

Id addHexapod(Document& doc, const Vec3& pos, const std::string& name) {
  Id torso = addBody(doc, kWorldId, {pos, {}}, name);
  addJoint(doc, torso, JointType::Free, {}, {0, 0, 1}, name + "_root");
  Id tg = addGeom(doc, torso, GeomType::Ellipsoid, {0.20, 0.14, 0.05}, {}, name + "_shell");
  doc.geom(tg)->rgba = kShell;
  Id imu = addSite(doc, torso, {}, name + "_imu");
  addSensor(doc, SensorType::Accelerometer, imu);
  addSensor(doc, SensorType::Gyro, imu);
  const char* tags[] = {"L1", "L2", "L3", "R3", "R2", "R1"};
  const double angles[] = {50, 90, 130, 230, 270, 310};
  for (int i = 0; i < 6; ++i) {
    double a = deg2rad(angles[i]);
    Vec3 attach{0.19 * std::cos(a) * 1.0, 0.13 * std::sin(a), 0};
    std::string t = tags[i];
    Id coxa = addBody(doc, torso, {attach, axisAngle({0, 0, 1}, a)}, t + "_coxa");
    Id yaw = hinge(doc, coxa, {0, 0, 1}, -0.5, 0.5, t + "_hip_yaw", 0.3, 0.005);
    limbGeom(doc, coxa, {0, 0, 0}, {0.06, 0, 0}, 0.02, kLimb, t + "_coxa_geom");
    Id femur = addBody(doc, coxa, {{0.06, 0, 0}, axisAngle({1, 0, 0}, 0) * axisAngle({0, 1, 0}, -0.5)}, t + "_femur");
    Id lift = hinge(doc, femur, {0, -1, 0}, -0.8, 0.8, t + "_hip_lift", 0.3, 0.005);
    limbGeom(doc, femur, {0, 0, 0}, {0.12, 0, 0}, 0.018, kLimb, t + "_femur_geom");
    Id tibia = addBody(doc, femur, {{0.12, 0, 0}, axisAngle({0, 1, 0}, 1.6)}, t + "_tibia");
    Id knee = hinge(doc, tibia, {0, -1, 0}, -0.8, 0.8, t + "_knee", 0.3, 0.005);
    limbGeom(doc, tibia, {0, 0, 0}, {0.16, 0, 0}, 0.014, kLimb, t + "_tibia_geom");
    Id foot = addGeom(doc, tibia, GeomType::Sphere, {0.02, 0, 0}, {{0.16, 0, 0}, {}}, t + "_foot");
    doc.geom(foot)->rgba = kFoot;
    servo(doc, yaw, 15, 6);
    servo(doc, lift, 30, 8);
    servo(doc, knee, 30, 8);
  }
  placeOnFloor(doc, torso);
  return torso;
}

Id addBiped(Document& doc, const Vec3& pos, const std::string& name) {
  Id pelvis = addBody(doc, kWorldId, {pos, {}}, name);
  addJoint(doc, pelvis, JointType::Free, {}, {0, 0, 1}, name + "_root");
  Id pg = addGeom(doc, pelvis, GeomType::Box, {0.08, 0.13, 0.05}, {}, name + "_pelvis");
  doc.geom(pg)->rgba = kShell;
  Id torso = addBody(doc, pelvis, {{0, 0, 0.1}, {}}, name + "_torso");
  Id waist = hinge(doc, torso, {0, 0, 1}, -0.8, 0.8, name + "_waist_yaw", 1.0, 0.02);
  Id tgeom = limbGeom(doc, torso, {0, 0, 0.02}, {0, 0, 0.3}, 0.09, kShell, name + "_chest");
  (void)tgeom;
  Id head = addGeom(doc, torso, GeomType::Sphere, {0.08, 0, 0}, {{0, 0, 0.47}, {}}, name + "_head");
  doc.geom(head)->rgba = kLimb;
  Id imu = addSite(doc, torso, {{0, 0, 0.2}, {}}, name + "_imu");
  addSensor(doc, SensorType::Accelerometer, imu);
  addSensor(doc, SensorType::Gyro, imu);
  servo(doc, waist, 60, 40);
  // Left leg, then mirror it to get the right one.
  Id thigh = addBody(doc, pelvis, {{0, 0.09, -0.05}, {}}, "left_thigh");
  Id hy = hinge(doc, thigh, {0, 0, 1}, -0.6, 0.6, "left_hip_yaw", 1.0, 0.02);
  Id hr = hinge(doc, thigh, {1, 0, 0}, -0.5, 0.5, "left_hip_roll", 1.0, 0.02);
  Id hp = hinge(doc, thigh, {0, 1, 0}, -1.6, 0.6, "left_hip_pitch", 1.0, 0.02);
  limbGeom(doc, thigh, {0, 0, 0}, {0, 0, -0.32}, 0.05, kLimb, "left_thigh_geom");
  Id shin = addBody(doc, thigh, {{0, 0, -0.34}, {}}, "left_shin");
  Id kn = hinge(doc, shin, {0, 1, 0}, 0.0, 2.2, "left_knee", 1.0, 0.02);
  limbGeom(doc, shin, {0, 0, 0}, {0, 0, -0.3}, 0.04, kLimb, "left_shin_geom");
  Id foot = addBody(doc, shin, {{0, 0, -0.33}, {}}, "left_foot");
  Id an = hinge(doc, foot, {0, 1, 0}, -0.8, 0.8, "left_ankle", 1.0, 0.01);
  Id fg = addGeom(doc, foot, GeomType::Box, {0.10, 0.045, 0.02}, {{0.03, 0, -0.03}, {}}, "left_sole");
  doc.geom(fg)->rgba = kFoot;
  servo(doc, hy, 80, 60);
  servo(doc, hr, 120, 80);
  servo(doc, hp, 150, 100);
  servo(doc, kn, 150, 100);
  servo(doc, an, 200, 60);  // stiffer than gravity's tipping pull, so it can stand still
  mirrorSubtree(doc, thigh, 1);
  placeOnFloor(doc, pelvis);
  return pelvis;
}

Id addHopper(Document& doc, const Vec3& pos, const std::string& name) {
  // Planar: the torso slides in x and z and pitches about y (like the Gym/DM Control hopper).
  Id torso = addBody(doc, kWorldId, {pos + Vec3{0, 0, 1.25}, {}}, name + "_torso");
  for (auto [jn, type, axis] : {std::tuple{"_rootx", JointType::Slide, Vec3{1, 0, 0}},
                                std::tuple{"_rootz", JointType::Slide, Vec3{0, 0, 1}},
                                std::tuple{"_rooty", JointType::Hinge, Vec3{0, -1, 0}}}) {
    Id j = addJoint(doc, torso, type, {}, axis, name + jn);
    doc.joint(j)->armature = 0;
    doc.joint(j)->damping = 0;
  }
  limbGeom(doc, torso, {0, 0, 0.2}, {0, 0, -0.2}, 0.05, kShell, name + "_torso_geom");
  struct Seg { const char* n; double len, r, lo, hi; };
  const Seg segs[] = {{"thigh", 0.45, 0.05, -150, 0}, {"leg", 0.5, 0.04, -150, 0}};
  Id parent = torso;
  Vec3 at{0, 0, -0.2};
  for (const Seg& sg : segs) {
    Id b = addBody(doc, parent, {at, {}}, name + "_" + sg.n);
    Id j = hinge(doc, b, {0, -1, 0}, deg2rad(sg.lo), deg2rad(sg.hi), name + "_" + sg.n + "_joint", 1.0, 1.0);
    limbGeom(doc, b, {0, 0, 0}, {0, 0, -sg.len}, sg.r, kLimb, name + "_" + sg.n + "_geom");
    Id a = addActuator(doc, ActuatorType::Motor, j);
    doc.actuator(a)->gear[0] = 200;
    parent = b;
    at = {0, 0, -sg.len};
  }
  Id foot = addBody(doc, parent, {at, {}}, name + "_foot");
  Id fj = hinge(doc, foot, {0, -1, 0}, deg2rad(-45), deg2rad(45), name + "_foot_joint", 1.0, 1.0);
  Id fg = limbGeom(doc, foot, {-0.13, 0, 0}, {0.26, 0, 0}, 0.06, kFoot, name + "_foot_geom");
  doc.geom(fg)->friction = {2.0, 0.005, 0.0001};
  Id fa = addActuator(doc, ActuatorType::Motor, fj);
  doc.actuator(fa)->gear[0] = 200;
  placeOnFloor(doc, torso);
  return torso;
}

Id addArm(Document& doc, const Vec3& pos, const std::string& name) {
  Id base = addBody(doc, kWorldId, {pos, {}}, name + "_base");
  Id ped = addGeom(doc, base, GeomType::Cylinder, {0.08, 0.05, 0}, {{0, 0, 0.05}, {}}, name + "_pedestal");
  doc.geom(ped)->rgba = kLimb;
  Id shoulder = addBody(doc, base, {{0, 0, 0.12}, {}}, name + "_shoulder");
  Id yaw = hinge(doc, shoulder, {0, 0, 1}, -3.0, 3.0, name + "_yaw", 2.0, 0.05);
  Id sg = addGeom(doc, shoulder, GeomType::Cylinder, {0.06, 0.05, 0}, {{0, 0, 0.04}, {}}, name + "_shoulder_geom");
  doc.geom(sg)->rgba = kShell;
  Id upper = addBody(doc, shoulder, {{0, 0, 0.1}, {}}, name + "_upper");
  Id sp = hinge(doc, upper, {0, 1, 0}, -1.6, 1.6, name + "_shoulder_pitch", 2.0, 0.05);
  limbGeom(doc, upper, {0, 0, 0}, {0, 0, 0.35}, 0.04, kShell, name + "_upper_geom");
  Id fore = addBody(doc, upper, {{0, 0, 0.35}, {}}, name + "_forearm");
  Id el = hinge(doc, fore, {0, 1, 0}, -2.5, 2.5, name + "_elbow", 1.0, 0.02);
  limbGeom(doc, fore, {0, 0, 0}, {0, 0, 0.3}, 0.032, kLimb, name + "_forearm_geom");
  Id hand = addBody(doc, fore, {{0, 0, 0.3}, {}}, name + "_hand");
  Id wr = hinge(doc, hand, {0, 1, 0}, -2.0, 2.0, name + "_wrist", 0.5, 0.01);
  Id hg = addGeom(doc, hand, GeomType::Box, {0.03, 0.05, 0.04}, {{0, 0, 0.05}, {}}, name + "_hand_geom");
  doc.geom(hg)->rgba = kShell;
  Id tip = addSite(doc, hand, {{0, 0, 0.1}, {}}, name + "_tip");
  addSensor(doc, SensorType::FramePos, tip, name + "_tip_pos");
  servo(doc, yaw, 100, 80);
  servo(doc, sp, 200, 150);
  servo(doc, el, 120, 80);
  servo(doc, wr, 30, 20);
  return base;
}

bool makeTemplate(const std::string& id, Document& out) {
  Document doc;
  if (id == "empty") {
    doc.modelName = "scene";
    addDefaultEnvironment(doc);
  } else if (id == "quadruped") {
    doc.modelName = "quadruped";
    addDefaultEnvironment(doc);
    addQuadruped(doc, {0, 0, 0.5});
  } else if (id == "hexapod") {
    doc.modelName = "hexapod";
    addDefaultEnvironment(doc);
    addHexapod(doc, {0, 0, 0.3});
  } else if (id == "biped") {
    doc.modelName = "biped";
    addDefaultEnvironment(doc);
    addBiped(doc, {0, 0, 1.0});
  } else if (id == "hopper") {
    doc.modelName = "hopper";
    addDefaultEnvironment(doc);
    addHopper(doc, {0, 0, 0});
  } else if (id == "arm") {
    doc.modelName = "arm";
    addDefaultEnvironment(doc);
    addArm(doc, {0, 0, 0});
  } else {
    return false;
  }
  out = std::move(doc);
  out.touch();
  return true;
}

}  // namespace mjf
