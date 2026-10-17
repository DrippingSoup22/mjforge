// Physics analysis checked against MuJoCo: compiled masses, loads, legs, start pose and the fixes.
#include <doctest/doctest.h>

#include <cmath>

#include "analysis/Analysis.h"
#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/Templates.h"
#include "sim/MujocoBridge.h"

using namespace mjf;

namespace {

AnalysisReport analyzeWithCompile(const Document& doc, CompileResult* out = nullptr) {
  ExportMap map;
  CompileResult cr = compileMjcf(writeMjcf(doc, {}, &map));
  cr.map = map;
  INFO(cr.error);
  REQUIRE(cr.ok);
  AnalysisReport r = analyzeModel(doc, &cr, true);
  if (out) *out = std::move(cr);
  return r;
}

int countCode(const AnalysisReport& r, const std::string& code) {
  int n = 0;
  for (const Issue& i : r.issues) n += i.code == code;
  return n;
}

const Issue* findCode(const AnalysisReport& r, const std::string& code) {
  for (const Issue& i : r.issues)
    if (i.code == code) return &i;
  return nullptr;
}

// A static post 1 m up with an arm on a horizontal hinge, its weight `reach` out along x.
struct Arm {
  Document doc;
  Id arm = kNoId, joint = kNoId, geom = kNoId, act = kNoId;
};

Arm makeArm(double reach, double mass) {
  Arm a;
  Id base = addBody(a.doc, kWorldId, {{0, 0, 1}, {}}, "base");
  a.arm = addBody(a.doc, base, {}, "arm");
  a.joint = addJoint(a.doc, a.arm, JointType::Hinge, {}, {0, 1, 0}, "shoulder");
  a.geom = addGeom(a.doc, a.arm, GeomType::Sphere, {0.05, 0, 0}, {{reach, 0, 0}, {}}, "weight");
  a.doc.geom(a.geom)->mass = mass;
  a.act = addActuator(a.doc, ActuatorType::Motor, a.joint, "motor");
  return a;
}

}  // namespace

TEST_CASE("physics: masses come from the compiled model") {
  Arm a = makeArm(0.5, -1);
  Geom* g = a.doc.geom(a.geom);
  g->density = 1000;
  g->shellinertia = true;  // hollow: the mass comes from the surface area
  CompileResult cr;
  AnalysisReport r = analyzeWithCompile(a.doc, &cr);
  double mjMass = cr.model->body_mass[2];  // world, base, arm
  CHECK(mjMass == doctest::Approx(1000 * 4 * kPi * 0.05 * 0.05));
  const JointReport* jr = r.joint(a.joint);
  REQUIRE(jr);
  CHECK(jr->movedMass == doctest::Approx(mjMass));
  CHECK(jr->liftLoad == doctest::Approx(mjMass * 9.81 * 0.5));

  // settotalmass scales every body; the analysis sees the scaled masses and offers to bake them.
  a.doc.compiler.settotalmass = 20;
  AnalysisReport r2 = analyzeWithCompile(a.doc);
  CHECK(r2.joint(a.joint)->movedMass == doctest::Approx(20));
  const Issue* bake = findCode(r2, "settotalmass");
  REQUIRE(bake);
  REQUIRE(bake->fix);
  bake->fix(a.doc);
  CHECK(a.doc.compiler.settotalmass <= 0);
  CompileResult baked = compileMjcf(writeMjcf(a.doc));
  REQUIRE(baked.ok);
  CHECK(baked.model->body_mass[2] == doctest::Approx(20));
  CHECK(baked.model->body_inertia[6] == doctest::Approx(r2.joint(a.joint)->inertia - 20 * 0.25).epsilon(1e-6));
}

TEST_CASE("physics: the mass estimate follows the compiler options") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addJoint(doc, b, JointType::Hinge, {}, {0, 1, 0});
  Id g = addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {});
  doc.geom(g)->mass = 2;
  Body* body = doc.body(b);
  body->hasInertial = true;
  body->mass = 5;
  body->diaginertia = {0.1, 0.1, 0.1};
  auto agrees = [&](double expected) {
    auto poses = computeBodyPoses(doc);
    CHECK(bodyMassProps(doc, poses, b).mass == doctest::Approx(expected));
    CompileResult cr = compileMjcf(writeMjcf(doc));
    REQUIRE(cr.ok);
    CHECK(cr.model->body_mass[1] == doctest::Approx(expected));
  };
  agrees(5);  // the explicit inertial wins by default
  doc.compiler.inertiafromgeom = InertiaFromGeom::True;
  agrees(2);
  doc.compiler.inertiafromgeom = InertiaFromGeom::Auto;
  body->hasInertial = false;
  doc.compiler.boundmass = 3;
  agrees(3);
}

TEST_CASE("physics: gravity compensation takes the load off the joint") {
  Arm a = makeArm(0.5, 2.0);
  a.doc.actuator(a.act)->gear[0] = 2;  // 2 N*m against 9.8 N*m
  CHECK(countCode(analyzeWithCompile(a.doc), "weak-actuator") == 1);
  a.doc.body(a.arm)->gravcomp = 1;
  AnalysisReport r = analyzeWithCompile(a.doc);
  CHECK(countCode(r, "weak-actuator") == 0);
  CHECK(r.joint(a.joint)->liftLoad < 1e-9);
}

TEST_CASE("physics: actuators on tendons carry their force to the joints") {
  Arm a = makeArm(0.5, 2.0);
  Id t = addTendon(a.doc, TendonType::Fixed, "cable");
  a.doc.tendon(t)->joints.push_back({a.joint, 0.5});
  Actuator* m = a.doc.actuator(a.act);
  m->trn = TrnType::Tendon;
  m->target = t;
  m->gear[0] = 1e-3;
  AnalysisReport r = analyzeWithCompile(a.doc);
  const JointReport* jr = r.joint(a.joint);
  REQUIRE(jr);
  CHECK(jr->actuated);
  CHECK(jr->capacity == doctest::Approx(0.5e-3));
  CHECK(analyzeModel(a.doc, nullptr, false).joint(a.joint)->capacity == doctest::Approx(0.5e-3));
  const Issue* weak = findCode(r, "weak-actuator");
  REQUIRE(weak);
  REQUIRE(weak->fix);
  weak->fix(a.doc);
  CHECK(countCode(analyzeWithCompile(a.doc), "weak-actuator") == 0);
}

TEST_CASE("physics: a moving body may take its mass from a body welded below it") {
  Document doc;
  Id link = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "link");
  addJoint(doc, link, JointType::Hinge, {}, {0, 1, 0}, "j1");
  Id tip = addBody(doc, link, {{0.2, 0, 0}, {}}, "tip");  // no joint: welded to the link
  addGeom(doc, tip, GeomType::Sphere, {0.05, 0, 0}, {});
  CHECK(compileMjcf(writeMjcf(doc)).ok);
  CHECK(countCode(analyzeModel(doc, nullptr, false), "massless") == 0);
  addJoint(doc, tip, JointType::Hinge, {}, {0, 1, 0}, "j2");  // now it moves on its own
  CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
  AnalysisReport mReport = analyzeModel(doc, nullptr, false);
  const Issue* m = findCode(mReport, "massless");
  REQUIRE(m);
  CHECK(m->elements == std::vector<Id>{link});
}

TEST_CASE("physics: strength fixes reach their target without side effects") {
  SUBCASE("a reversed motor keeps its direction") {
    Arm a = makeArm(0.5, 1.0);
    a.doc.actuator(a.act)->gear[0] = -2;
    AnalysisReport wReport = analyzeWithCompile(a.doc);
    const Issue* w = findCode(wReport, "weak-actuator");
    REQUIRE(w);
    REQUIRE(w->fix);
    w->fix(a.doc);
    CHECK(a.doc.actuator(a.act)->gear[0] < -2);
    CHECK(countCode(analyzeWithCompile(a.doc), "weak-actuator") == 0);
  }
  SUBCASE("the joint's actuator force cap") {
    Arm a = makeArm(0.5, 1.0);
    a.doc.actuator(a.act)->gear[0] = 100;
    Joint* j = a.doc.joint(a.joint);
    j->actuatorfrclimited = true;
    j->actuatorfrcrange = {-0.5, 0.5};
    AnalysisReport wReport = analyzeWithCompile(a.doc);
    const Issue* w = findCode(wReport, "weak-actuator");
    REQUIRE(w);
    w->fix(a.doc);
    CHECK(countCode(analyzeWithCompile(a.doc), "weak-actuator") == 0);
  }
  SUBCASE("a general actuator") {
    Arm a = makeArm(0.5, 1.0);
    Actuator* m = a.doc.actuator(a.act);
    m->type = ActuatorType::General;
    m->gainprm[0] = 0.5;
    AnalysisReport wReport = analyzeWithCompile(a.doc);
    const Issue* w = findCode(wReport, "weak-actuator");
    REQUIRE(w);
    w->fix(a.doc);
    CHECK(countCode(analyzeWithCompile(a.doc), "weak-actuator") == 0);
  }
  SUBCASE("a position servo on a short slide") {
    Document doc;
    Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "lift");
    Id j = addJoint(doc, b, JointType::Slide, {}, {0, 0, 1}, "z");
    doc.joint(j)->limited = true;
    doc.joint(j)->range = {0, 0.05};
    doc.geom(addGeom(doc, b, GeomType::Box, {0.1, 0.1, 0.02}, {}))->mass = 5;
    Actuator* s = doc.actuator(addActuator(doc, ActuatorType::Position, j));
    s->kp = 10;
    s->ctrlrange = {0, 0.05};
    AnalysisReport wReport = analyzeWithCompile(doc);
    const Issue* w = findCode(wReport, "weak-actuator");
    REQUIRE(w);
    w->fix(doc);
    CHECK(countCode(analyzeWithCompile(doc), "weak-actuator") == 0);
  }
}

TEST_CASE("physics: servo sag is checked on every joint, and its fix accounts for the gear") {
  Arm a = makeArm(0.5, 2.0);
  Actuator* s = a.doc.actuator(a.act);
  s->type = ActuatorType::Position;
  s->kp = 40;
  s->gear[0] = 0.5;  // joint stiffness 40 * 0.5^2 = 10 N*m/rad against 9.8 N*m
  AnalysisReport r = analyzeWithCompile(a.doc);
  const Issue* sag = findCode(r, "servo-sag");
  REQUIRE(sag);
  REQUIRE(sag->fix);
  sag->fix(a.doc);
  AnalysisReport r2 = analyzeWithCompile(a.doc);
  CHECK(countCode(r2, "servo-sag") == 0);
  CHECK(r2.joint(a.joint)->sag < deg2rad(5));
}

TEST_CASE("physics: planar walkers get the floor and leg checks") {
  Document doc;
  REQUIRE(makeTemplate("hopper", doc));
  AnalysisReport r = analyzeWithCompile(doc);
  CHECK(r.legCount == 1);
  CHECK(countCode(r, "passive-joint") == 0);  // the root joints move the whole body
  CHECK(countCode(r, "below-floor") == 0);
  liftStartPose(doc, doc.bodies[1].id, -0.2);
  AnalysisReport belowReport = analyzeWithCompile(doc);
  const Issue* below = findCode(belowReport, "below-floor");
  REQUIRE(below);
  REQUIRE(below->fix);
  below->fix(doc);
  CHECK(countCode(analyzeWithCompile(doc), "below-floor") == 0);
}

TEST_CASE("physics: wheels are not legs") {
  Document doc;
  addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  Id car = addBody(doc, kWorldId, {{0, 0, 0.1}, {}}, "car");
  addJoint(doc, car, JointType::Free);
  addGeom(doc, car, GeomType::Box, {0.3, 0.15, 0.05}, {{0, 0, 0.05}, {}});
  for (double x : {-0.25, 0.25})
    for (double y : {-0.2, 0.2}) {
      Id w = addBody(doc, car, {{x, y, 0}, {}}, "wheel");
      Id j = addJoint(doc, w, JointType::Hinge, {}, {0, 1, 0});
      addGeom(doc, w, GeomType::Cylinder, {0.1, 0.02, 0}, {{}, fromTo({0, 0, 1}, {0, 1, 0})});
      doc.actuator(addActuator(doc, ActuatorType::Motor, j))->gear[0] = 0.01;
    }
  AnalysisReport r = analyzeWithCompile(doc);
  CHECK(r.legCount == 0);
  CHECK(countCode(r, "weak-actuator") == 0);
  CHECK(countCode(r, "friction-creep") == 0);
}

TEST_CASE("physics: only upward-facing planes are floors") {
  Document doc;
  addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {{0, 0, 3}, axisAngle({1, 0, 0}, kPi)}, "ceiling");
  Id ball = addBody(doc, kWorldId, {{0, 0, 0.1}, {}}, "ball");
  addJoint(doc, ball, JointType::Free);
  addGeom(doc, ball, GeomType::Sphere, {0.1, 0, 0}, {});
  AnalysisReport r = analyzeWithCompile(doc);
  CHECK(countCode(r, "below-floor") == 0);
  CHECK(countCode(r, "floating") == 0);
}

TEST_CASE("physics: a control delay longer than the history buffer") {
  Arm a = makeArm(0.5, 0.1);
  Actuator* m = a.doc.actuator(a.act);
  m->gear[0] = 10;
  m->delay = 0.01;  // five 2 ms steps
  m->nsample = 1;
  AnalysisReport dReport = analyzeWithCompile(a.doc);
  const Issue* d = findCode(dReport, "delay-history");
  REQUIRE(d);
  CHECK(d->severity == Severity::Warning);
  REQUIRE(d->fix);
  d->fix(a.doc);
  CHECK(a.doc.actuator(a.act)->nsample == 5);
  CHECK(countCode(analyzeWithCompile(a.doc), "delay-history") == 0);
}

TEST_CASE("physics: an intvelocity servo's control is a speed, its activation the target") {
  Arm a = makeArm(0.5, 0.2);
  Actuator* s = a.doc.actuator(a.act);
  s->type = ActuatorType::IntVelocity;
  s->kp = 50;
  s->ctrlrange = {-10, 10};
  s->actlimited = true;
  s->actrange = {-1, 1};
  CHECK(countCode(analyzeWithCompile(a.doc), "ctrl-range-degrees") == 0);
  s->actrange = {-90, 90};
  AnalysisReport degReport = analyzeWithCompile(a.doc);
  const Issue* deg = findCode(degReport, "ctrl-range-degrees");
  REQUIRE(deg);
  deg->fix(a.doc);
  CHECK(a.doc.actuator(a.act)->actrange.y == doctest::Approx(kPi / 2));
  CHECK(a.doc.actuator(a.act)->ctrlrange.y == doctest::Approx(10));
}

TEST_CASE("physics: disabled actuation and MuJoCo's per-body joint rules") {
  Arm a = makeArm(0.5, 0.2);
  a.doc.actuator(a.act)->gear[0] = 10;
  a.doc.option.actuation = false;
  AnalysisReport offReport = analyzeWithCompile(a.doc);
  const Issue* off = findCode(offReport, "actuation-off");
  REQUIRE(off);
  off->fix(a.doc);
  CHECK(a.doc.option.actuation);

  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {});
  addJoint(doc, b, JointType::Ball);
  addJoint(doc, b, JointType::Hinge);
  CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
  CHECK(countCode(analyzeModel(doc, nullptr, false), "ball-then-rotation") == 1);
  addJoint(doc, b, JointType::Ball);
  CHECK(countCode(analyzeModel(doc, nullptr, false), "body-dofs") == 1);  // 3 + 1 + 3
}

TEST_CASE("physics: rays from inside a capsule leave through its surface") {
  Pose pose;
  double t = rayPrimitive({{0, 0, 0}, {0, 0, 1}}, GeomType::Capsule, {0.1, 0.3, 0}, pose);
  CHECK(t == doctest::Approx(0.4));
  t = rayPrimitive({{0, 0, 0}, {1, 0, 0}}, GeomType::Capsule, {0.1, 0.3, 0}, pose);
  CHECK(t == doctest::Approx(0.1));
  t = rayPrimitive({{0, 0, -1}, {0, 0, 1}}, GeomType::Capsule, {0.1, 0.3, 0}, pose);
  CHECK(t == doctest::Approx(0.6));
}

TEST_CASE("physics: joint inertia on a floating model matches MuJoCo with the base free") {
  Document doc;
  REQUIRE(makeTemplate("biped", doc));
  CompileResult cr;
  AnalysisReport r = analyzeWithCompile(doc, &cr);
  const mjModel* m = cr.model.get();
  DataPtr d = makeData(m);
  if (m->nkey) mj_resetDataKeyframe(m, d.get(), 0);
  mj_forward(m, d.get());
  std::vector<mjtNum> M(size_t(m->nv) * m->nv);
  mj_fullM(m, d.get(), M.data());
  int checked = 0;
  for (int j = 0; j < m->njnt; ++j) {
    if (m->jnt_type[j] != mjJNT_HINGE && m->jnt_type[j] != mjJNT_SLIDE) continue;
    // 7x7 block: the free joint's 6 DoFs and this joint; its effective inertia is 1 / (inverse)_77.
    int idx[7] = {0, 1, 2, 3, 4, 5, m->jnt_dofadr[j]};
    double S[7][14] = {};
    for (int a = 0; a < 7; ++a) {
      for (int b = 0; b < 7; ++b) S[a][b] = M[size_t(idx[a]) * m->nv + idx[b]];
      S[a][7 + a] = 1;
    }
    for (int c = 0; c < 7; ++c) {
      int piv = c;
      for (int k = c + 1; k < 7; ++k)
        if (std::fabs(S[k][c]) > std::fabs(S[piv][c])) piv = k;
      for (int k = 0; k < 14; ++k) std::swap(S[c][k], S[piv][k]);
      for (int k = 0; k < 7; ++k) {
        if (k == c) continue;
        double f = S[k][c] / S[c][c];
        for (int q = 0; q < 14; ++q) S[k][q] -= f * S[c][q];
      }
    }
    double eff = 1.0 / (S[6][13] / S[6][6]);
    const JointReport* jr = r.joint(cr.map.joints[j]);
    REQUIRE(jr);
    INFO(doc.nameOf(jr->joint));
    CHECK(jr->inertia == doctest::Approx(eff).epsilon(1e-6));
    ++checked;
  }
  CHECK(checked >= 6);
}
