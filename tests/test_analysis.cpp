#include <doctest/doctest.h>

#include "analysis/Analysis.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/Templates.h"
#include "sim/MujocoBridge.h"

using namespace mjf;

namespace {

int countCode(const AnalysisReport& r, const std::string& code) {
  int n = 0;
  for (const Issue& i : r.issues) n += i.code == code;
  return n;
}

const Issue* findCode(const AnalysisReport& r, const std::string& code) {
  for (const Issue& i : r.issues) if (i.code == code) return &i;
  return nullptr;
}

AnalysisReport analyzeCompiled(const Document& doc) {
  CompileResult cr = compileMjcf(writeMjcf(doc));
  REQUIRE(cr.ok);
  return analyzeModel(doc, &cr, true);
}

}  // namespace

TEST_CASE("analysis: templates have no errors or warnings") {
  for (const char* t : {"quadruped", "hexapod", "biped", "hopper", "arm", "empty"}) {
    Document doc;
    REQUIRE(makeTemplate(t, doc));
    AnalysisReport r = analyzeCompiled(doc);
    INFO(std::string(t));
    for (const Issue& i : r.issues)
      if (i.severity <= Severity::Warning) MESSAGE(i.title);
    CHECK(r.counts[int(Severity::Error)] == 0);
    CHECK(r.counts[int(Severity::Warning)] == 0);
  }
}

TEST_CASE("analysis: weak actuators are detected and the fix resolves them") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  for (Actuator& a : doc.actuators) {
    a.forcelimited = true;
    a.forcerange = {-2e-6, 2e-6};
  }
  AnalysisReport r = analyzeCompiled(doc);
  CHECK(r.legCount == 4);
  const Issue* weak = findCode(r, "weak-actuator");
  REQUIRE(weak);
  CHECK(weak->severity == Severity::Error);
  REQUIRE(weak->fix);
  // Per-joint numbers: every leg joint needs real torque.
  for (const JointReport& j : r.joints) {
    CHECK(j.actuated);
    CHECK(j.capacity == doctest::Approx(2e-6));
    CHECK(j.required() > 1.0);
  }
  weak->fix(doc);
  AnalysisReport r2 = analyzeCompiled(doc);
  CHECK(countCode(r2, "weak-actuator") == 0);
  CHECK(r2.counts[int(Severity::Error)] == 0);
}

TEST_CASE("analysis: lifting load of a single hinged arm") {
  Document doc;
  Id base = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "base");
  Id arm = addBody(doc, base, {}, "arm");
  Id j = addJoint(doc, arm, JointType::Hinge, {}, {0, 1, 0}, "shoulder");
  // 1 kg point-like mass 0.5 m away along x.
  Id g = addGeom(doc, arm, GeomType::Sphere, {0.05, 0, 0}, {{0.5, 0, 0}, {}}, "mass");
  doc.geom(g)->mass = 1.0;
  Id act = addActuator(doc, ActuatorType::Motor, j);
  doc.actuator(act)->gear[0] = 2.0;  // 2 N*m max with ctrl in [-1, 1]
  AnalysisReport r = analyzeCompiled(doc);
  const JointReport* jr = r.joint(j);
  REQUIRE(jr);
  CHECK(jr->liftLoad == doctest::Approx(1.0 * 9.81 * 0.5).epsilon(1e-6));
  CHECK(jr->capacity == doctest::Approx(2.0));
  CHECK(countCode(r, "weak-actuator") == 1);
  doc.actuator(act)->gear[0] = 10.0;
  CHECK(countCode(analyzeCompiled(doc), "weak-actuator") == 0);
}

TEST_CASE("analysis: stiff servo vs timestep and armature fix") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j = addJoint(doc, b, JointType::Hinge, {}, {0, 0, 1}, "j");
  doc.joint(j)->limited = true;
  doc.joint(j)->range = {-1, 1};
  Id g = addGeom(doc, b, GeomType::Sphere, {0.01, 0, 0}, {{0.05, 0, 0}, {}}, "tiny");
  (void)g;
  Id a = addActuator(doc, ActuatorType::Position, j);
  doc.actuator(a)->kp = 5000;
  AnalysisReport r = analyzeCompiled(doc);
  const Issue* stiff = findCode(r, "stiff-servo");
  REQUIRE(stiff);
  REQUIRE(stiff->fix);
  stiff->fix(doc);
  const JointReport* jr = analyzeCompiled(doc).joint(j);
  REQUIRE(jr);
  CHECK(jr->omegaDt <= 0.6);
}

TEST_CASE("analysis: MJWarp incompatibilities are errors with fixes") {
  Document doc;
  REQUIRE(makeTemplate("empty", doc));
  doc.option.solver = Solver::PGS;
  doc.option.noslip_iterations = 3;
  Id b1 = addBody(doc, kWorldId, {{0, 0, 0.2}, {}}, "crate1", Role::Environment);
  addJoint(doc, b1, JointType::Free);
  Id g1 = addGeom(doc, b1, GeomType::Box, {0.1, 0.1, 0.1}, {});
  doc.geom(g1)->margin = 0.01;
  Id b2 = addBody(doc, kWorldId, {{0.5, 0, 0.2}, {}}, "crate2", Role::Environment);
  addJoint(doc, b2, JointType::Free);
  addGeom(doc, b2, GeomType::Box, {0.1, 0.1, 0.1}, {});
  AnalysisReport r = analyzeCompiled(doc);
  CHECK(countCode(r, "mjw-pgs") == 1);
  CHECK(countCode(r, "mjw-noslip") == 1);
  CHECK(countCode(r, "mjw-box-margin") == 1);
  for (const Issue& i : r.issues)
    if (i.area == IssueArea::MJWarp && i.severity == Severity::Error && i.fix) i.fix(doc);
  AnalysisReport r2 = analyzeCompiled(doc);
  CHECK(countCode(r2, "mjw-pgs") == 0);
  CHECK(countCode(r2, "mjw-noslip") == 0);
  CHECK(countCode(r2, "mjw-box-margin") == 0);
  CHECK(doc.geom(g1)->margin == 0);
}

TEST_CASE("analysis: MJWarp rules measured with mujoco-warp's put_model") {
  // Each case was refused (or warned about) by mujoco_warp 3.16 and missed by the analysis before.
  Document doc;
  REQUIRE(makeTemplate("empty", doc));
  Id b1 = addBody(doc, kWorldId, {{0, 0, 0.2}, {}}, "crate1", Role::Environment);
  addJoint(doc, b1, JointType::Free);
  Id g1 = addGeom(doc, b1, GeomType::Box, {0.1, 0.1, 0.1}, {});
  Id b2 = addBody(doc, kWorldId, {{0.5, 0, 0.2}, {}}, "crate2", Role::Environment);
  addJoint(doc, b2, JointType::Free);
  Id g2 = addGeom(doc, b2, GeomType::Box, {0.1, 0.1, 0.1}, {});
  CHECK(countCode(analyzeCompiled(doc), "mjw-box-margin") == 0);

  // Explicit sleep policies are refused even with sleeping off.
  doc.body(b1)->sleep = SleepPolicy::Init;
  CHECK(countCode(analyzeCompiled(doc), "mjw-sleep-policy") == 1);
  doc.body(b1)->sleep = SleepPolicy::Auto;

  // An explicit box pair is checked with its own margin: refused with multiccd, or nativeccd for boxes.
  Id p = addContactPair(doc, g1, g2);
  doc.pair(p)->margin = 0.01;
  AnalysisReport r = analyzeCompiled(doc);
  const Issue* margin = findCode(r, "mjw-box-margin");
  REQUIRE(margin);
  doc.option.multiccd = false;
  CHECK(countCode(analyzeCompiled(doc), "mjw-box-margin") == 1);
  doc.option.nativeccd = false;
  CHECK(countCode(analyzeCompiled(doc), "mjw-box-margin") == 0);
  doc.option.multiccd = doc.option.nativeccd = true;
  margin->fix(doc);
  CHECK(doc.pair(p)->margin == 0);
  CHECK(countCode(analyzeCompiled(doc), "mjw-box-margin") == 0);

  // Zero torsional friction with condim 4, zero rolling friction with condim 6.
  CHECK(countCode(analyzeCompiled(doc), "mjw-friction") == 0);
  doc.geom(g1)->condim = 4;
  doc.geom(g1)->friction = {1, 0, 0.0001};
  CHECK(countCode(analyzeCompiled(doc), "mjw-friction") == 1);
  doc.geom(g1)->condim = 3;
  CHECK(countCode(analyzeCompiled(doc), "mjw-friction") == 0);
  doc.pair(p)->condim = 6;
  doc.pair(p)->friction = Vec5{{1, 1, 0.005, 0, 0.0001}};
  CHECK(countCode(analyzeCompiled(doc), "mjw-friction") == 1);

  // A pair of a geom with itself: MuJoCo ignores it, mujoco_warp fails to load the model.
  doc.pair(p)->geom2 = g1;
  r = analyzeCompiled(doc);
  const Issue* same = findCode(r, "pair-same");
  REQUIRE(same);
  REQUIRE(same->fix);
  same->fix(doc);
  CHECK(doc.pairs.empty());
}

TEST_CASE("analysis: sensor update phase follows MuJoCo's rule") {
  Document doc;
  REQUIRE(makeTemplate("arm", doc));
  Id s = addSensor(doc, SensorType::JointPos, doc.joints[0].id);
  Sensor& sensor = *doc.sensor(s);
  sensor.nsample = 4;
  sensor.interval = {0.01, -0.004};  // first update at 6 ms
  CHECK(compileMjcf(writeMjcf(doc)).ok);
  CHECK(countCode(analyzeCompiled(doc), "sensor-interval") == 0);
  for (Vec2 bad : {Vec2{0.01, 0.004}, Vec2{0.01, -0.01}, Vec2{-0.01, 0}}) {
    sensor.interval = bad;
    CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
    AnalysisReport r = analyzeModel(doc, nullptr, false);
    const Issue* i = findCode(r, "sensor-interval");
    REQUIRE(i);
    i->fix(doc);
    CHECK(compileMjcf(writeMjcf(doc)).ok);
  }
  // Samples a delayed sensor needs: one per step, or one per period plus the one being held.
  sensor.interval = {0, 0};
  sensor.delay = 0.02;
  CHECK(sensorHistoryNeeded(sensor, 0.002) == 10);
  sensor.interval = {0.01, 0};
  CHECK(sensorHistoryNeeded(sensor, 0.002) == 3);
  sensor.delay = 0;
  CHECK(sensorHistoryNeeded(sensor, 0.002) == 1);
}

TEST_CASE("analysis: silent MuJoCo mistakes and start pose") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  // An empty limited range: with autolimits MuJoCo would silently drop such a limit, so the export
  // says limited="true" and MuJoCo refuses the model; the analysis names the joint and fixes it.
  doc.joints[1].limited = true;
  doc.joints[1].range = {0.5, -0.5};
  CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
  AnalysisReport r0 = analyzeModel(doc, nullptr, false);
  const Issue* jr = findCode(r0, "joint-range");
  REQUIRE(jr);
  REQUIRE(jr->fix);
  jr->fix(doc);
  // A model floating in the air.
  doc.bodies[1].pos.z += 0.3;
  AnalysisReport r = analyzeCompiled(doc);
  CHECK(countCode(r, "joint-range") == 0);
  const Issue* fl = findCode(r, "floating");
  REQUIRE(fl);
  REQUIRE(fl->fix);
  fl->fix(doc);
  CHECK(countCode(analyzeCompiled(doc), "floating") == 0);
}

TEST_CASE("analysis: collision pair filtering follows MuJoCo") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CompileResult cr = compileMjcf(writeMjcf(doc));
  REQUIRE(cr.ok);
  AnalysisReport r = analyzeModel(doc, &cr, true);
  // Compare with MuJoCo's own count of candidate geom pairs (mj_collision uses the same filter);
  // MuJoCo exposes nothing directly, so check basic expectations instead.
  CHECK(r.collisionPairs > 0);
  CHECK(r.selfCollisionPairs > 0);
  CHECK(r.broadphaseAtRest >= 4);  // four feet resting on the floor
  // Disable all collisions of the creature: only floor pairs remain impossible -> zero pairs.
  for (Geom& g : doc.geoms)
    if (g.body != kWorldId) g.contype = g.conaffinity = 0;
  AnalysisReport r2 = analyzeModel(doc, nullptr, false);
  CHECK(r2.collisionPairs == 0);
}

TEST_CASE("collision presets: feet only and no self-collision") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  Id root = doc.bodies[1].id;
  CHECK(lowestGeoms(doc, root).size() == 4);
  applyCollisionPreset(doc, root, CollisionPreset::FeetOnly);
  AnalysisReport r = analyzeModel(doc, nullptr, false);
  CHECK(r.selfCollisionPairs == 0);
  CHECK(r.collisionPairs == 4);  // four feet against the floor
  applyCollisionPreset(doc, root, CollisionPreset::NoSelfCollision);
  AnalysisReport r2 = analyzeModel(doc, nullptr, false);
  CHECK(r2.selfCollisionPairs == 0);
  CHECK(r2.collisionPairs == 13);  // every creature geom against the floor
  CompileResult cr = compileMjcf(writeMjcf(doc));
  CHECK(cr.ok);
}

TEST_CASE("analysis: friction creep tip for legged models with soft friction") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CHECK(countCode(analyzeCompiled(doc), "friction-creep") == 0);  // templates use elliptic cones
  doc.option.cone = Cone::Pyramidal;
  doc.option.impratio = 1;
  AnalysisReport r = analyzeCompiled(doc);
  const Issue* creep = findCode(r, "friction-creep");
  REQUIRE(creep);
  REQUIRE(creep->fix);
  creep->fix(doc);
  CHECK(doc.option.cone == Cone::Elliptic);
  CHECK(countCode(analyzeCompiled(doc), "friction-creep") == 0);
}

TEST_CASE("analysis: a biped with soft ankles can't stand still") {
  Document doc;
  REQUIRE(makeTemplate("biped", doc));
  CHECK(countCode(analyzeCompiled(doc), "ankle-balance") == 0);
  for (Actuator& a : doc.actuators)
    if (doc.nameOf(a.target).find("ankle") != std::string::npos) a.kp = 60;
  AnalysisReport r = analyzeCompiled(doc);
  const Issue* ankle = findCode(r, "ankle-balance");
  REQUIRE(ankle);
  CHECK(ankle->elements.size() == 2);  // both ankle joints
  REQUIRE(ankle->fix);
  ankle->fix(doc);
  CHECK(countCode(analyzeCompiled(doc), "ankle-balance") == 0);
}
