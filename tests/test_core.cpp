#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <set>

#include "core/Document.h"
#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/ProjectIO.h"
#include "sim/MujocoBridge.h"

using namespace mjf;

int main(int argc, char** argv) {
  installMujocoErrorHandlers();
  doctest::Context ctx(argc, argv);
  return ctx.run();
}

namespace {

// A small creature: free-floating torso with one hinged leg, a position actuator and a sensor.
Document makeCreature() {
  Document doc;
  doc.modelName = "test_creature";
  addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  Id torso = addBody(doc, kWorldId, {{0, 0, 0.5}, {}}, "torso");
  addJoint(doc, torso, JointType::Free, {}, {0, 0, 1}, "root");
  addGeom(doc, torso, GeomType::Box, {0.2, 0.1, 0.05}, {}, "torso_geom");
  Id leg = addBody(doc, torso, {{0.2, 0, 0}, {}}, "leg");
  Id hip = addJoint(doc, leg, JointType::Hinge, {}, {0, 1, 0}, "hip");
  doc.joint(hip)->limited = true;
  doc.joint(hip)->range = {-1.0, 0.5};
  Id lg = addGeom(doc, leg, GeomType::Capsule, {0.03, 0.1, 0}, {}, "leg_geom");
  setGeomFromTo(*doc.geom(lg), {0, 0, 0}, {0, 0, -0.2});
  addActuator(doc, ActuatorType::Position, hip);
  addSensor(doc, SensorType::JointPos, hip);
  return doc;
}

int bodyIdByName(const mjModel* m, const char* name) { return mj_name2id(m, mjOBJ_BODY, name); }

}  // namespace

TEST_CASE("math: euler/quat round trip and fromTo") {
  Vec3 e{0.3, -0.7, 1.2};
  Vec3 e2 = toEulerXYZ(fromEulerXYZ(e));
  CHECK(e2.x == doctest::Approx(e.x));
  CHECK(e2.y == doctest::Approx(e.y));
  CHECK(e2.z == doctest::Approx(e.z));
  Quat q = fromTo({0, 0, 1}, {1, 0, 0});
  Vec3 r = rotate(q, {0, 0, 1});
  CHECK(r.x == doctest::Approx(1.0));
  CHECK(std::fabs(r.z) < 1e-9);
  Mat4 m = Mat4::fromPose({{1, 2, 3}, axisAngle({0, 0, 1}, 0.5)});
  Mat4 id = m * m.inverted();
  for (int i = 0; i < 4; ++i)
    for (int j = 0; j < 4; ++j) CHECK(id(i, j) == doctest::Approx(i == j ? 1.0 : 0.0).epsilon(1e-5));
}

TEST_CASE("geometry: ray intersections") {
  Ray ray{{-5, 0, 0}, {1, 0, 0}};
  CHECK(rayPrimitive(ray, GeomType::Sphere, {1, 0, 0}, {}) == doctest::Approx(4.0));
  CHECK(rayPrimitive(ray, GeomType::Box, {0.5, 0.5, 0.5}, {}) == doctest::Approx(4.5));
  CHECK(rayPrimitive(ray, GeomType::Capsule, {0.2, 1.0, 0}, {}) == doctest::Approx(4.8));
  // Capsule rotated so its axis is along x: hits the hemisphere cap at x = -(1 + 0.2).
  Pose p{{0, 0, 0}, fromTo({0, 0, 1}, {1, 0, 0})};
  CHECK(rayPrimitive(ray, GeomType::Capsule, {0.2, 1.0, 0}, p) == doctest::Approx(3.8));
  CHECK(rayPrimitive(ray, GeomType::Cylinder, {0.3, 1.0, 0}, {}) == doctest::Approx(4.7));
  CHECK(rayPrimitive(ray, GeomType::Ellipsoid, {2, 1, 1}, {}) == doctest::Approx(3.0));
  Ray down{{0.3, 0.2, 4}, {0, 0, -1}};
  CHECK(rayPrimitive(down, GeomType::Plane, {0, 0, 1}, {}) == doctest::Approx(4.0));
  CHECK(rayPrimitive({{-5, 3, 0}, {1, 0, 0}}, GeomType::Sphere, {1, 0, 0}, {}) < 0);
}

TEST_CASE("document: names, lookups, cascading delete, reparent") {
  Document doc = makeCreature();
  CHECK(doc.uniqueName(Kind::Body, "leg") == "leg_2");
  Id leg = doc.bodies[2].id;
  CHECK(doc.body(leg)->name == "leg");
  CHECK(doc.actuators.size() == 1);
  CHECK(doc.sensors.size() == 1);
  // Reparent leg to world keeping its world pose.
  Pose before = bodyWorldPose(doc, leg);
  CHECK(reparentBody(doc, leg, kWorldId));
  Pose after = bodyWorldPose(doc, leg);
  CHECK(after.pos.x == doctest::Approx(before.pos.x));
  CHECK(after.pos.z == doctest::Approx(before.pos.z));
  // Cannot reparent a body into its own subtree.
  Id torso = doc.bodies[1].id;
  Id child = addBody(doc, torso, {}, "child");
  CHECK_FALSE(canReparent(doc, torso, child));
  // Deleting the leg removes its joint, geom, actuator and sensor.
  deleteElements(doc, {leg});
  CHECK(doc.body(leg) == nullptr);
  CHECK(doc.actuators.empty());
  CHECK(doc.sensors.empty());
  for (const Joint& j : doc.joints) CHECK(j.name != "hip");
}

TEST_CASE("export: creature compiles in MuJoCo with expected structure") {
  Document doc = makeCreature();
  ExportMap map;
  std::string xml = writeMjcf(doc, {}, &map);
  CompileResult r = compileMjcf(xml);
  INFO(xml);
  INFO(r.error);
  REQUIRE(r.ok);
  const mjModel* m = r.model.get();
  CHECK(m->nbody == 3);
  CHECK(m->njnt == 2);
  CHECK(m->nu == 1);
  CHECK(m->nsensor == 1);
  CHECK(m->nq == 8);
  int hip = mj_name2id(m, mjOBJ_JOINT, "hip");
  REQUIRE(hip >= 0);
  CHECK(m->jnt_limited[hip] == 1);
  CHECK(m->jnt_range[2 * hip] == doctest::Approx(-1.0));
  CHECK(m->jnt_range[2 * hip + 1] == doctest::Approx(0.5));
  // Position actuator inherits the joint range as ctrlrange and has kp=1 by default.
  CHECK(m->actuator_ctrlrange[0] == doctest::Approx(-1.0));
  CHECK(m->actuator_ctrlrange[1] == doctest::Approx(0.5));
  // The export map matches MuJoCo's compiled ordering.
  REQUIRE(map.geoms.size() == size_t(m->ngeom));
  for (int i = 0; i < m->ngeom; ++i) CHECK(doc.nameOf(map.geoms[i]) == std::string(mj_id2name(m, mjOBJ_GEOM, i)));
  REQUIRE(map.bodies.size() == size_t(m->nbody));
  for (int i = 1; i < m->nbody; ++i) CHECK(doc.nameOf(map.bodies[i]) == std::string(mj_id2name(m, mjOBJ_BODY, i)));
}

TEST_CASE("export: degree output is equivalent") {
  Document doc = makeCreature();
  ExportOptions opt;
  opt.angle = AngleUnit::Degree;
  CompileResult r = compileMjcf(writeMjcf(doc, opt));
  REQUIRE(r.ok);
  int hip = mj_name2id(r.model.get(), mjOBJ_JOINT, "hip");
  CHECK(r.model->jnt_range[2 * hip] == doctest::Approx(-1.0));
}

TEST_CASE("mass properties match MuJoCo's compiler") {
  struct Case { GeomType t; Vec3 size; Quat q; };
  Case cases[] = {
      {GeomType::Sphere, {0.1, 0, 0}, {}},
      {GeomType::Box, {0.1, 0.2, 0.05}, axisAngle({1, 1, 0}, 0.4)},
      {GeomType::Capsule, {0.05, 0.2, 0}, axisAngle({0, 1, 0}, 0.9)},
      {GeomType::Cylinder, {0.07, 0.15, 0}, {}},
      {GeomType::Ellipsoid, {0.1, 0.05, 0.2}, axisAngle({0, 0, 1}, 0.3)},
  };
  for (const Case& c : cases) {
    Document doc;
    Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
    addJoint(doc, b, JointType::Free);
    Id g = addGeom(doc, b, c.t, c.size, {{0.1, -0.05, 0.02}, c.q}, "g");
    addGeom(doc, b, GeomType::Sphere, {0.03, 0, 0}, {{-0.2, 0, 0}, {}}, "g2");
    doc.geom(g)->density = 800;
    CompileResult r = compileMjcf(writeMjcf(doc));
    REQUIRE(r.ok);
    const mjModel* m = r.model.get();
    int bi = bodyIdByName(m, "b");
    auto poses = computeBodyPoses(doc);
    MassProps mp = bodyMassProps(doc, poses, b);
    CHECK(mp.mass == doctest::Approx(m->body_mass[bi]).epsilon(1e-6));
    // MuJoCo stores the principal inertia; compare eigenvalue sums and the COM.
    double trace = mp.inertia(0, 0) + mp.inertia(1, 1) + mp.inertia(2, 2);
    double mjTrace = m->body_inertia[3 * bi] + m->body_inertia[3 * bi + 1] + m->body_inertia[3 * bi + 2];
    CHECK(trace == doctest::Approx(mjTrace).epsilon(1e-6));
    Vec3 comLocal = rotate(conj(poses[doc.bodyIndex(b)].quat), mp.com - poses[doc.bodyIndex(b)].pos);
    CHECK(comLocal.x == doctest::Approx(m->body_ipos[3 * bi]).epsilon(1e-6));
    CHECK(comLocal.y == doctest::Approx(m->body_ipos[3 * bi + 1]).epsilon(1e-6));
  }
}

TEST_CASE("project: JSON round trip reproduces identical MJCF") {
  Document doc = makeCreature();
  Material mat;
  mat.name = "grid";
  Texture tex;
  tex.name = "grid";
  Id t = addTexture(doc, tex);
  mat.texture = t;
  Id mid = addMaterial(doc, mat);
  doc.geoms[0].material = mid;
  doc.option.integrator = Integrator::ImplicitFast;
  doc.option.timestep = 0.004;
  std::string json = projectToJson(doc, "{\"camera\":{\"distance\":3}}");
  Document loaded;
  std::string err, editor;
  REQUIRE(projectFromJson(loaded, json, &err, &editor));
  CHECK(editor.find("distance") != std::string::npos);
  CHECK(writeMjcf(loaded) == writeMjcf(doc));
  CHECK(loaded.nextId == doc.nextId);
}

TEST_CASE("duplicate: copies subtree with actuators and unique names") {
  Document doc = makeCreature();
  Id leg = doc.bodies[2].id;
  auto tops = duplicate(doc, {leg}, {0, 0.3, 0});
  REQUIRE(tops.size() == 1);
  CHECK(doc.body(tops[0])->name == "leg_2");
  CHECK(doc.actuators.size() == 2);
  CHECK(doc.sensors.size() == 2);
  CHECK(doc.actuator(doc.actuators[1].id)->target != doc.actuators[0].target);
  CompileResult r = compileMjcf(writeMjcf(doc));
  INFO(r.error);
  CHECK(r.ok);
  CHECK(r.model->nu == 2);
}

TEST_CASE("mirror: names, positions and symmetric hinge axes") {
  CHECK(mirrorName("left_hip") == "right_hip");
  CHECK(mirrorName("LeftKnee") == "RightKnee");
  CHECK(mirrorName("hip_L") == "hip_R");
  CHECK(mirrorName("FL_calf") == "FR_calf");
  CHECK(mirrorName("RR_thigh") == "RL_thigh");
  CHECK(mirrorName("upright_post") == "upright_post");
  bool changed = true;
  mirrorName("torso", &changed);
  CHECK_FALSE(changed);

  Document doc;
  Id torso = addBody(doc, kWorldId, {{0, 0, 0.5}, {}}, "torso");
  addGeom(doc, torso, GeomType::Sphere, {0.1, 0, 0}, {});
  Id leg = addBody(doc, torso, {{0.1, 0.15, 0}, axisAngle({1, 0, 0}, 0.3)}, "left_leg");
  Id roll = addJoint(doc, leg, JointType::Hinge, {}, {1, 0, 0}, "left_roll");
  addGeom(doc, leg, GeomType::Capsule, {0.02, 0.1, 0}, {{0, 0.05, -0.1}, {}});
  addActuator(doc, ActuatorType::Motor, roll);
  Id m = mirrorSubtree(doc, leg, 1);
  REQUIRE(m != kNoId);
  CHECK(doc.body(m)->name == "right_leg");
  CHECK(doc.body(m)->pos.y == doctest::Approx(-0.15));
  CHECK(doc.actuators.size() == 2);
  const Joint* mj = nullptr;
  for (const Joint& j : doc.joints) if (j.name == "right_roll") mj = &j;
  REQUIRE(mj);
  // Roll axis (x) is a pseudo-vector: mirrored across y it flips sign.
  auto poses = computeBodyPoses(doc);
  Vec3 aL, aR;
  jointWorldFrame(doc, poses, *doc.joint(roll), nullptr, &aL);
  jointWorldFrame(doc, poses, *mj, nullptr, &aR);
  CHECK(aR.x == doctest::Approx(-aL.x));
  CHECK(aR.y == doctest::Approx(aL.y));
  CHECK(aR.z == doctest::Approx(-aL.z));
  // Geometry is mirrored: compare world geom centers.
  Pose gL = elementWorldPose(doc, poses, doc.geoms[1].id);
  Pose gR = elementWorldPose(doc, poses, doc.geoms.back().id);
  CHECK(gR.pos.x == doctest::Approx(gL.pos.x));
  CHECK(gR.pos.y == doctest::Approx(-gL.pos.y));
  CHECK(gR.pos.z == doctest::Approx(gL.pos.z));
  CompileResult r = compileMjcf(writeMjcf(doc));
  CHECK(r.ok);
}

TEST_CASE("export scopes: model-only, environment-only and split include") {
  Document doc = makeCreature();
  Id box = addBody(doc, kWorldId, {{1, 0, 0.1}, {}}, "crate", Role::Environment);
  addJoint(doc, box, JointType::Free);
  addGeom(doc, box, GeomType::Box, {0.1, 0.1, 0.1}, {});

  ExportOptions mo;
  mo.scope = ExportOptions::Scope::Models;
  CompileResult rm = compileMjcf(writeMjcf(doc, mo));
  REQUIRE(rm.ok);
  CHECK(mj_name2id(rm.model.get(), mjOBJ_BODY, "torso") >= 0);
  CHECK(mj_name2id(rm.model.get(), mjOBJ_BODY, "crate") < 0);
  CHECK(mj_name2id(rm.model.get(), mjOBJ_GEOM, "floor") < 0);
  CHECK(rm.model->nu == 1);

  ExportOptions eo;
  eo.scope = ExportOptions::Scope::Environment;
  CompileResult re = compileMjcf(writeMjcf(doc, eo));
  REQUIRE(re.ok);
  CHECK(mj_name2id(re.model.get(), mjOBJ_BODY, "torso") < 0);
  CHECK(mj_name2id(re.model.get(), mjOBJ_BODY, "crate") >= 0);
  CHECK(re.model->nu == 0);

  // Split files on disk: scene.xml includes model.xml.
  namespace fs = std::filesystem;
  fs::path dir = fs::path(MJF_TEST_TMP) / "split";
  fs::create_directories(dir);
  REQUIRE(writeTextFile((dir / "creature.xml").string(), writeMjcf(doc, mo)));
  ExportOptions so = eo;
  so.includeFile = "creature.xml";
  REQUIRE(writeTextFile((dir / "scene.xml").string(), writeMjcf(doc, so)));
  char err[1000] = "";
  mjModel* m = mj_loadXML((dir / "scene.xml").string().c_str(), nullptr, err, sizeof err);
  INFO(err);
  REQUIRE(m != nullptr);
  CHECK(m->nbody == 4);
  CHECK(m->nu == 1);
  mj_deleteModel(m);
}

TEST_CASE("compile errors are reported, not fatal") {
  CompileResult r = compileMjcf("<mujoco><worldbody><geom type='box'/></worldbody></mujoco>");
  CHECK_FALSE(r.ok);
  CHECK_FALSE(r.error.empty());
  CompileResult r2 = compileMjcf("not xml at all");
  CHECK_FALSE(r2.ok);
}

TEST_CASE("limbs: attach a hinged leg, mirror it, analyze") {
  Document doc;
  addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  Id torso = addBody(doc, kWorldId, {{0, 0, 0.5}, {}}, "torso");
  addJoint(doc, torso, JointType::Free, {}, {0, 0, 1}, "root");
  addGeom(doc, torso, GeomType::Box, {0.2, 0.1, 0.05}, {}, "chest");
  LimbSpec spec;
  spec.name = "leg";
  spec.direction = {0, 0, -1};
  spec.axis = {0, 1, 0};
  Id leg = addLimb(doc, torso, {0.15, 0.1, 0.5}, spec);
  REQUIRE(leg);
  CHECK(doc.body(leg)->name == "leg_upper");
  CHECK(doc.joints.size() == 3);     // root + 2 hinges
  CHECK(doc.actuators.size() == 2);
  auto poses = computeBodyPoses(doc);
  Aabb box = subtreeAabb(doc, poses, leg);
  CHECK(box.lo.z == doctest::Approx(0.5 - 2 * 0.2 - spec.radius * spec.taper * spec.taper * 1.25).epsilon(1e-6));
  // The hinge axis is the world Y axis.
  Vec3 a, axis;
  jointWorldFrame(doc, poses, *doc.joint(doc.joints[1].id), &a, &axis);
  CHECK(std::fabs(axis.y) == doctest::Approx(1.0));
  // Mirror to the other side: symmetric in y.
  Id m = mirrorSubtree(doc, leg, 1);
  REQUIRE(m);
  poses = computeBodyPoses(doc);
  CHECK(poses[doc.bodyIndex(m)].pos.y == doctest::Approx(-0.1));
  CompileResult cr = compileMjcf(writeMjcf(doc));
  INFO(cr.error);
  CHECK(cr.ok);
  CHECK(cr.model->nu == 4);
}

TEST_CASE("arrays: radial legs and linear geoms") {
  Document doc;
  Id torso = addBody(doc, kWorldId, {{0, 0, 0.5}, {}}, "torso");
  addJoint(doc, torso, JointType::Free, {}, {0, 0, 1}, "root");
  addGeom(doc, torso, GeomType::Sphere, {0.1, 0, 0}, {}, "core");
  Id leg = addBody(doc, torso, {{0.15, 0, 0}, {}}, "leg");
  Id hip = addJoint(doc, leg, JointType::Hinge, {}, {0, 1, 0}, "hip");
  addGeom(doc, leg, GeomType::Capsule, {0.02, 0.05, 0}, {{0.05, 0, 0}, {}}, "leg_geom");
  addActuator(doc, ActuatorType::Position, hip);
  // Five copies around the torso's Z axis: six legs 60 degrees apart.
  std::vector<Id> copies = arrayDuplicate(doc, leg, 5, {{}, axisAngle({0, 0, 1}, kPi / 3)});
  REQUIRE(copies.size() == 5);
  CHECK(doc.actuators.size() == 6);
  auto poses = computeBodyPoses(doc);
  Vec3 p3 = poses[doc.bodyIndex(copies[2])].pos;  // 180 degrees
  CHECK(p3.x == doctest::Approx(-0.15));
  CHECK(std::fabs(p3.y) < 1e-9);
  std::set<std::string> names;
  for (const Body& b : doc.bodies) names.insert(b.name);
  CHECK(names.size() == doc.bodies.size());
  CompileResult cr = compileMjcf(writeMjcf(doc));
  CHECK(cr.ok);
  // Linear array of a world geom.
  Id post = addGeom(doc, kWorldId, GeomType::Cylinder, {0.05, 0.2, 0}, {{1, 0, 0.2}, {}}, "post");
  std::vector<Id> posts = arrayDuplicate(doc, post, 3, {{0.5, 0, 0}, {}});
  REQUIRE(posts.size() == 3);
  CHECK(doc.geom(posts[2])->pos.x == doctest::Approx(2.5));
}
