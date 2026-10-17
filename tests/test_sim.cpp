#include <doctest/doctest.h>

#include <cmath>

#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/Templates.h"
#include "sim/MujocoBridge.h"
#include "sim/Simulator.h"

using namespace mjf;

namespace {

CompileResult compileDoc(const Document& doc) {
  ExportMap map;
  std::string xml = writeMjcf(doc, {}, &map);
  CompileResult cr = compileMjcf(xml);
  cr.map = map;
  REQUIRE(cr.ok);
  return cr;
}

void run(Simulator& sim, double seconds) {
  int steps = int(std::lround(seconds / sim.model()->opt.timestep));
  for (int i = 0; i < steps && !sim.diverged(); ++i) sim.stepOnce();
}

double rootHeight(const Simulator& sim) { return sim.data()->qpos[2]; }

// One hinged arm sticking out sideways, driven by a torque motor.
Document makeArm(double gear) {
  Document doc;
  Id base = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "base");
  Id arm = addBody(doc, base, {}, "arm");
  Id j = addJoint(doc, arm, JointType::Hinge, {}, {0, 1, 0}, "shoulder");
  doc.joint(j)->armature = 0.01;
  Id g = addGeom(doc, arm, GeomType::Sphere, {0.05, 0, 0}, {{0.5, 0, 0}, {}}, "mass");
  doc.geom(g)->mass = 1.0;
  Id act = addActuator(doc, ActuatorType::Motor, j, "shoulder_motor");
  doc.actuator(act)->gear[0] = gear;  // ctrl in [-1, 1] -> max torque = gear
  return doc;
}

}  // namespace

TEST_CASE("simulator: templates stand still in Hold mode") {
  for (const char* t : {"quadruped", "hexapod", "biped"}) {
    INFO(std::string(t));
    Document doc;
    REQUIRE(makeTemplate(t, doc));
    CompileResult cr = compileDoc(doc);
    Simulator sim;
    REQUIRE(sim.start(cr.model, cr.map));
    double h0 = rootHeight(sim);
    run(sim, 2.0);
    CHECK_FALSE(sim.diverged());
    CHECK(sim.simTime() == doctest::Approx(2.0).epsilon(0.01));
    // Servos hold the body up: it may settle a little but must not collapse.
    CHECK(rootHeight(sim) > 0.8 * h0);
    CHECK(sim.sizing().peakContacts >= 2);
    CHECK(sim.sizing().peakBroadphase >= sim.sizing().peakContacts / 4);
    CHECK(sim.sizing().recommendedNconmax() >= sim.sizing().peakBroadphase);
    CHECK(sim.sizing().recommendedNjmax() >= sim.sizing().peakConstraints);
    CHECK(!sim.history().empty());
  }
}

TEST_CASE("simulator: limp mode collapses, sine test moves joints") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  double h0 = rootHeight(sim);
  sim.mode = CtrlMode::Limp;
  run(sim, 1.5);
  CHECK_FALSE(sim.diverged());
  CHECK(rootHeight(sim) < h0 - 0.02);

  sim.reset();
  sim.mode = CtrlMode::Sine;
  sim.sine.frequency = 1.0;
  run(sim, 0.75);
  CHECK_FALSE(sim.diverged());
  // Controls differ from the hold targets in the sine test.
  double maxDev = 0;
  for (int i = 0; i < sim.model()->nu; ++i)
    maxDev = std::max(maxDev, std::fabs(sim.data()->ctrl[i] - sim.drive(i).holdCtrl));
  CHECK(maxDev > 0.05);
}

TEST_CASE("simulator: live update keeps state, structural change restarts") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  run(sim, 0.5);
  double t = sim.simTime();
  // Parameter change only.
  for (Actuator& a : doc.actuators) a.kp *= 1.5;
  CompileResult cr2 = compileDoc(doc);
  CHECK(sim.update(cr2.model, cr2.map));
  CHECK(sim.simTime() == doctest::Approx(t));
  CHECK(sim.model()->actuator_gainprm[0] == doctest::Approx(cr2.model->actuator_gainprm[0]));
  // The simulator works on its own copy of the model.
  CHECK(sim.model() != cr2.model.get());
  run(sim, 0.2);
  CHECK_FALSE(sim.diverged());
  // Structural change: one more body -> restart from the reference pose.
  addBody(doc, doc.bodies[1].id, {{0, 0, 0.2}, {}}, "antenna");
  CompileResult cr3 = compileDoc(doc);
  CHECK_FALSE(sim.update(cr3.model, cr3.map));
  CHECK(sim.simTime() == 0.0);
  CHECK(sim.active());
}

TEST_CASE("simulator: torque motors hold the pose with the built-in PD when strong enough") {
  // Gravity torque at the shoulder: 1 kg * 9.81 * 0.5 m = 4.9 N*m.
  for (double gear : {20.0, 2.0}) {
    CompileResult cr = compileDoc(makeArm(gear));
    Simulator sim;
    REQUIRE(sim.start(cr.model, cr.map));
    REQUIRE(sim.drive(0).kind == ActuatorDrive::Motor);
    double maxQ = 0, maxCtrl = 0;
    for (int i = 0; i < 1000; ++i) {
      sim.stepOnce();
      maxQ = std::max(maxQ, std::fabs(sim.data()->qpos[0]));
      maxCtrl = std::max(maxCtrl, std::fabs(sim.data()->ctrl[0]));
    }
    CHECK_FALSE(sim.diverged());
    INFO("gear " << gear << " max |q| " << maxQ);
    if (gear > 10) {
      CHECK(maxQ < 0.25);  // holds roughly horizontal
      CHECK(maxCtrl < 1.0);
    } else {
      CHECK(maxQ > 0.8);  // too weak: the arm drops
      CHECK(maxCtrl == doctest::Approx(1.0));  // saturated control
    }
  }
}

TEST_CASE("simulator: perturbation lifts the body") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  int torso = sim.bodyIndex(doc.bodies[1].id);
  REQUIRE(torso > 0);
  CHECK(sim.bodyMoves(torso));
  CHECK_FALSE(sim.bodyMoves(0));
  double h0 = rootHeight(sim);
  Vec3 p{sim.data()->xpos[3 * torso], sim.data()->xpos[3 * torso + 1], sim.data()->xpos[3 * torso + 2]};
  sim.setPerturb(torso, {0, 0, 0}, p + Vec3{0, 0, 0.3});
  run(sim, 1.5);
  CHECK(rootHeight(sim) > h0 + 0.15);
  sim.clearPerturb();
  run(sim, 1.5);
  CHECK_FALSE(sim.diverged());
  CHECK(rootHeight(sim) < h0 + 0.1);
}

TEST_CASE("simulator: blow-ups are detected and attributed") {
  // A feather-light body on an extremely stiff servo with a large explicit timestep.
  Document doc;
  doc.option.timestep = 0.01;
  doc.option.integrator = Integrator::Euler;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j = addJoint(doc, b, JointType::Hinge, {}, {0, 0, 1}, "spin");
  Id g = addGeom(doc, b, GeomType::Sphere, {0.01, 0, 0}, {{0.05, 0, 0}, {}}, "tiny");
  doc.geom(g)->mass = 1e-3;
  Id a = addActuator(doc, ActuatorType::Position, j);
  doc.actuator(a)->kp = 1e6;
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  sim.mode = CtrlMode::Manual;
  sim.manual[0] = 0.5;
  run(sim, 2.0);
  REQUIRE(sim.diverged());
  CHECK(sim.paused());
  CHECK(sim.divergeJoint() == 0);
  sim.reset();
  CHECK_FALSE(sim.diverged());
}

TEST_CASE("simulator: contacts report the supported weight") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  run(sim, 1.5);
  std::vector<SimContact> cs;
  sim.contacts(cs);
  REQUIRE(!cs.empty());
  double fz = 0;
  for (const SimContact& c : cs) fz += c.normalForce * std::fabs(c.normal.z);
  double weight = sim.model()->body_subtreemass[0] * 9.81;
  CHECK(fz == doctest::Approx(weight).epsilon(0.1));
}

TEST_CASE("simulator: templates keep standing (no friction creep, ankles hold the biped)") {
  for (const char* t : {"quadruped", "biped"}) {
    INFO(std::string(t));
    Document doc;
    REQUIRE(makeTemplate(t, doc));
    CompileResult cr = compileDoc(doc);
    Simulator sim;
    REQUIRE(sim.start(cr.model, cr.map));
    run(sim, 2.0);
    double x = sim.data()->qpos[0], z = rootHeight(sim);
    run(sim, 20.0);
    CHECK_FALSE(sim.diverged());
    CHECK(std::fabs(sim.data()->qpos[0] - x) < 0.01);
    CHECK(rootHeight(sim) > z - 0.01);
  }
}

TEST_CASE("simulator: starts from the first keyframe") {
  Document doc;
  REQUIRE(makeTemplate("arm", doc));
  Keyframe k;
  k.name = "ready";
  for (const Joint& j : doc.joints)
    if (j.name == "arm_elbow") k.qpos.push_back({j.id, {1.0}});
  doc.keyframes.push_back(k);
  CompileResult cr = compileDoc(doc);
  Simulator sim;
  REQUIRE(sim.start(cr.model, cr.map));
  int elbow = mj_name2id(sim.model(), mjOBJ_JOINT, "arm_elbow");
  CHECK(sim.data()->qpos[sim.model()->jnt_qposadr[elbow]] == doctest::Approx(1.0));
  // Hold mode keeps the keyframe pose, not the reference pose.
  run(sim, 1.0);
  CHECK(sim.data()->qpos[sim.model()->jnt_qposadr[elbow]] == doctest::Approx(1.0).epsilon(0.05));
}
