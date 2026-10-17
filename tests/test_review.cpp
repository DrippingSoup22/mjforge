// Regressions found by the adversarial review: each case failed before its fix.
#include <doctest/doctest.h>

#include <cmath>
#include <filesystem>
#include <limits>

#include <nlohmann/json.hpp>

#include "analysis/Analysis.h"
#include "core/Document.h"
#include "core/Generators.h"
#include "core/Kinematics.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/ProjectIO.h"
#include "core/Templates.h"
#include "sim/MjcfImport.h"
#include "sim/MujocoBridge.h"

using namespace mjf;
namespace fs = std::filesystem;

namespace {

fs::path tmpDir(const std::string& name) {
  fs::path d = fs::path(MJF_TEST_TMP) / name;
  std::error_code ec;
  fs::remove_all(d, ec);
  fs::create_directories(d);
  return d;
}

ModelPtr loadFile(const fs::path& p) {
  char err[1000] = "";
  mjModel* m = mj_loadXML(p.string().c_str(), nullptr, err, sizeof err);
  INFO(err);
  REQUIRE(m);
  return makeModelPtr(m);
}

ModelPtr compileDoc(const Document& doc, const ExportOptions& opt = {}) {
  CompileResult r = compileMjcf(writeMjcf(doc, opt));
  INFO(r.error);
  REQUIRE(r.ok);
  return r.model;
}

// A robot (Model role) standing in an environment, tied together by every cross-scope element.
Document crossScopeScene() {
  Document doc;
  Material mat;
  mat.name = "shared";
  mat.rgba = {0.2f, 0.6f, 0.9f, 1};
  Id shared = addMaterial(doc, mat);
  Id floor = addGeom(doc, kWorldId, GeomType::Plane, {0, 0, 0.05}, {}, "floor");
  doc.geom(floor)->material = shared;
  Id post = addBody(doc, kWorldId, {{1, 0, 0.5}, {}}, "post", Role::Environment);
  addGeom(doc, post, GeomType::Box, {0.05, 0.05, 0.5}, {}, "post_geom");
  Id postSite = addSite(doc, post, {{0, 0, 0.4}, {}}, "post_site");
  Id door = addBody(doc, kWorldId, {{2, 0, 0.5}, {}}, "door", Role::Environment);
  Id hinge = addJoint(doc, door, JointType::Hinge, {}, {0, 0, 1}, "door_hinge");
  addGeom(doc, door, GeomType::Box, {0.3, 0.02, 0.4}, {}, "door_geom");
  addActuator(doc, ActuatorType::Motor, hinge, "door_motor");

  Id robot = addBody(doc, kWorldId, {{0, 0, 0.3}, {}}, "robot");
  addJoint(doc, robot, JointType::Free, {}, {0, 0, 1}, "root");
  Id torso = addGeom(doc, robot, GeomType::Box, {0.1, 0.1, 0.1}, {}, "torso");
  doc.geom(torso)->material = shared;
  Id leg = addBody(doc, robot, {{0, 0, -0.15}, {}}, "leg");
  Id knee = addJoint(doc, leg, JointType::Hinge, {}, {0, 1, 0}, "knee");
  Id foot = addGeom(doc, leg, GeomType::Sphere, {0.04, 0, 0}, {}, "foot");
  Id robotSite = addSite(doc, robot, {}, "robot_site");
  addActuator(doc, ActuatorType::Position, knee, "knee_servo");

  addContactPair(doc, foot, floor);
  addExclude(doc, robot, post);
  addEquality(doc, EqualityType::Connect, robot, post);
  Id cable = addTendon(doc, TendonType::Spatial, "cable");
  doc.tendon(cable)->path = {{TendonPoint::Type::Site, robotSite}, {TendonPoint::Type::Site, postSite}};
  addSensor(doc, SensorType::TendonPos, cable, "cable_length");
  Id fp = addSensor(doc, SensorType::FramePos, robotSite, "robot_in_post");
  doc.sensor(fp)->ref = postSite;
  doc.sensor(fp)->reftype = FrameObj::Site;
  addSensor(doc, SensorType::Clock, kNoId, "time");

  Keyframe key;
  key.name = "home";
  setKeyJointQpos(key, knee, {0.4});
  setKeyJointQpos(key, hinge, {0.3});
  key.ctrl = {{doc.actuators[0].id, 0.1}, {doc.actuators[1].id, 0.4}};
  doc.keyframes = {key};
  doc.touch();
  return doc;
}

}  // namespace

TEST_CASE("review: split export equals the single-file export") {
  Document doc = crossScopeScene();
  fs::path dir = tmpDir("split");
  std::string err;
  REQUIRE(exportMjcfFile(doc, (dir / "single.xml").string(), {}, &err));
  REQUIRE(exportSplitMjcf(doc, (dir / "scene.xml").string(), {}, &err));
  CHECK(fs::exists(dir / "scene_model.xml"));
  ModelPtr a = loadFile(dir / "single.xml");
  ModelPtr b = loadFile(dir / "scene.xml");  // shared material defined once, cross elements kept
  CHECK(b->npair == a->npair);
  CHECK(b->nexclude == a->nexclude);
  CHECK(b->neq == a->neq);
  CHECK(b->ntendon == a->ntendon);
  CHECK(b->nsensor == a->nsensor);  // the clock sensor exactly once
  CHECK(b->nu == a->nu);
  CHECK(b->nmat == 1);
  REQUIRE(b->nkey == 1);
  // Keyframe values follow the joints and actuators of both files (environment door included).
  for (int j = 0; j < b->njnt; ++j) {
    int ja = mj_name2id(a.get(), mjOBJ_JOINT, mj_id2name(b.get(), mjOBJ_JOINT, j));
    REQUIRE(ja >= 0);
    int qa = a->jnt_qposadr[ja], qb = b->jnt_qposadr[j];
    int n = b->jnt_type[j] == mjJNT_FREE ? 7 : b->jnt_type[j] == mjJNT_BALL ? 4 : 1;
    for (int k = 0; k < n; ++k) CHECK(b->key_qpos[qb + k] == doctest::Approx(a->key_qpos[qa + k]));
  }
  for (int i = 0; i < b->nu; ++i) {
    int ia = mj_name2id(a.get(), mjOBJ_ACTUATOR, mj_id2name(b.get(), mjOBJ_ACTUATOR, i));
    CHECK(b->key_ctrl[i] == doctest::Approx(a->key_ctrl[ia]));
  }
}

TEST_CASE("review: split export keeps same-named asset files apart") {
  Document doc;
  auto addMesh = [&](const std::string& name, float r, Id body) {
    MeshAsset m;
    m.id = doc.newId();
    m.name = name;
    auto v = std::make_shared<std::vector<float>>();
    auto f = std::make_shared<std::vector<int>>();
    // A 40 x 30 UV sphere: > 1000 vertices, so it is written as an STL file.
    const int nu = 40, nv = 30;
    for (int i = 0; i <= nv; ++i)
      for (int j = 0; j < nu; ++j) {
        double th = kPi * i / nv, ph = 2 * kPi * j / nu;
        v->insert(v->end(), {float(r * std::sin(th) * std::cos(ph)), float(r * std::sin(th) * std::sin(ph)), float(r * std::cos(th))});
      }
    for (int i = 0; i < nv; ++i)
      for (int j = 0; j < nu; ++j) {
        int a = i * nu + j, b = i * nu + (j + 1) % nu, c = a + nu, d = b + nu;
        f->insert(f->end(), {a, c, b, b, c, d});
      }
    m.vert = v;
    m.face = f;
    doc.meshes.push_back(m);
    Id g = addGeom(doc, body, GeomType::Mesh, {}, {}, name + "_geom");
    doc.geom(g)->mesh = m.id;
  };
  Id robot = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "robot");
  addJoint(doc, robot, JointType::Free, {}, {0, 0, 1}, "root");
  addMesh("rock 1", 0.1f, robot);
  Id rock = addBody(doc, kWorldId, {{3, 0, 0.5}, {}}, "rock", Role::Environment);
  addMesh("rock_1", 0.5f, rock);
  doc.touch();
  fs::path dir = tmpDir("split_files");
  std::string err;
  REQUIRE(exportSplitMjcf(doc, (dir / "scene.xml").string(), {}, &err));
  ModelPtr m = loadFile(dir / "scene.xml");
  CHECK(m->body_mass[mj_name2id(m.get(), mjOBJ_BODY, "robot")] < 10);  // was 518 kg: the 0.5 m rock's file
}

TEST_CASE("review: actuator shortcuts MJCF cannot express become equivalent general actuators") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  Id slide = addJoint(doc, b, JointType::Slide, {}, {0, 0, 1}, "z");
  addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {}, "g");
  // Motor on a body (only <general> and <adhesion> accept body=).
  Id motor = addActuator(doc, ActuatorType::Motor, slide, "motor");
  doc.actuator(motor)->trn = TrnType::Body;
  doc.actuator(motor)->target = b;
  // Adhesion with a gear (adhesion has no gear attribute).
  Id adh = addActuator(doc, ActuatorType::Adhesion, kNoId, "suction");
  doc.actuator(adh)->trn = TrnType::Body;
  doc.actuator(adh)->target = b;
  doc.actuator(adh)->gear[0] = 2;
  doc.actuator(adh)->ctrlrange = {0, 1};
  // A position servo with a damping ratio, written as general: same gains after compilation.
  Id servo = addActuator(doc, ActuatorType::Position, slide, "servo");
  doc.actuator(servo)->kp = 50;
  doc.actuator(servo)->dampratio = 1;
  // Damper whose limit flag is off, with autolimits off (damper has no ctrllimited attribute).
  Id damper = addActuator(doc, ActuatorType::Damper, slide, "damper");
  doc.actuator(damper)->ctrllimited = false;
  doc.actuator(damper)->ctrlrange = {0, 1};
  doc.compiler.autolimits = false;
  // A servo inheriting its range from the joint must not also write a ctrlrange.
  doc.joint(slide)->limited = true;
  doc.joint(slide)->range = {-0.2, 0.3};
  Id inherit = addActuator(doc, ActuatorType::Position, slide, "inherit");
  doc.actuator(inherit)->inheritrange = 1;
  doc.touch();
  ModelPtr m = compileDoc(doc);
  CHECK(m->actuator_trntype[1] == mjTRN_BODY);
  CHECK(m->actuator_gainprm[1 * mjNGAIN] == doctest::Approx(1.0));
  int si = mj_name2id(m.get(), mjOBJ_ACTUATOR, "servo");
  CHECK(m->actuator_gainprm[si * mjNGAIN] == doctest::Approx(50));
  CHECK(m->actuator_biasprm[si * mjNBIAS + 2] < 0);  // the damping ratio became a damping
  int ii = mj_name2id(m.get(), mjOBJ_ACTUATOR, "inherit");
  CHECK(m->actuator_ctrlrange[2 * ii] == doctest::Approx(-0.2));
  CHECK(m->actuator_ctrlrange[2 * ii + 1] == doctest::Approx(0.3));
  // The general form of a shortcut behaves exactly like the shortcut.
  Document ref = doc;
  ref.actuator(motor)->trn = TrnType::Joint;
  ref.actuator(motor)->target = slide;
  ref.touch();
  ModelPtr r = compileDoc(ref);
  CHECK(m->actuator_biasprm[si * mjNBIAS + 2] == doctest::Approx(r->actuator_biasprm[si * mjNBIAS + 2]));
}

TEST_CASE("review: degree export leaves the range of unlimited joints alone") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j = addJoint(doc, b, JointType::Hinge, {}, {0, 1, 0}, "hinge");
  addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {}, "g");
  doc.joint(j)->range = {-1, 1};  // stored, but not limited
  doc.touch();
  ExportOptions deg;
  deg.angle = AngleUnit::Degree;
  ModelPtr m = compileDoc(doc, deg);
  CHECK(m->jnt_range[1] == doctest::Approx(1.0));  // MuJoCo converts only limited ranges
}

TEST_CASE("review: generated names never collide with user names") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j1 = addJoint(doc, b, JointType::Hinge, {}, {0, 1, 0}, "");
  Id j2 = addJoint(doc, b, JointType::Hinge, {}, {1, 0, 0}, "");
  doc.joint(j2)->name = "joint" + std::to_string(j1);  // the name the writer would generate for j1
  addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {}, "g");
  addActuator(doc, ActuatorType::Motor, j1, "m");
  doc.touch();
  compileDoc(doc);
}

TEST_CASE("review: a white geom color overrides the material, as in MuJoCo") {
  Document doc;
  Material mat;
  mat.name = "red";
  mat.rgba = {1, 0, 0, 1};
  Id red = addMaterial(doc, mat);
  Id g = addGeom(doc, kWorldId, GeomType::Box, {0.1, 0.1, 0.1}, {}, "white_override");
  doc.geom(g)->material = red;
  doc.geom(g)->rgba = {1, 1, 1, 1};
  doc.touch();
  ModelPtr m = compileDoc(doc);
  CHECK(m->geom_rgba[0] == doctest::Approx(1.0));
  CHECK(m->geom_rgba[1] == doctest::Approx(1.0));
  // Projects written before format 3 used white for "show the material".
  std::string json = projectToJson(doc);
  size_t at = json.find("\"version\": 3");
  REQUIRE(at != std::string::npos);
  json.replace(at, 12, "\"version\": 2");
  Document old;
  std::string err;
  REQUIRE(projectFromJson(old, json, &err));
  CHECK(old.geom(g)->rgba == Geom{}.rgba);
}

TEST_CASE("review: exported height fields keep the heights the editor shows") {
  Document doc;
  std::vector<double> p{6, 6, 0.08, 0.4, 3, 0.05, 0.0, 1};  // rough terrain without flat area
  addGenerated(doc, "rough", {0, 0, 0}, 0, &p);
  REQUIRE(doc.hfields.size() == 1);
  const Hfield& h = doc.hfields[0];
  const Geom* hg = nullptr;
  for (const Geom& g : doc.geoms)
    if (g.type == GeomType::Hfield) hg = &g;
  REQUIRE(hg);
  Pose base = bodyWorldPose(doc, hg->body) * Pose{hg->pos, hg->quat};
  ModelPtr m = compileDoc(doc);
  int gi = -1;
  for (int i = 0; i < m->ngeom; ++i)
    if (m->geom_type[i] == mjGEOM_HFIELD) gi = i;
  REQUIRE(gi >= 0);
  mjData* d = mj_makeData(m.get());
  mj_forward(m.get(), d);
  for (size_t i : {size_t(0), h.data->size() / 2, h.data->size() - 1}) {
    size_t r = i / size_t(h.ncol), c = i % size_t(h.ncol);
    double docZ = base.pos.z + (*h.data)[i] * h.size.z;
    float mjData = m->hfield_data[m->hfield_adr[0] + (size_t(h.nrow) - 1 - r) * size_t(h.ncol) + c];
    double mjZ = d->geom_xpos[3 * gi + 2] + mjData * m->hfield_size[2];
    CHECK(mjZ == doctest::Approx(docZ).epsilon(1e-5));
  }
  mj_deleteData(d);
}

TEST_CASE("review: damaged project files are refused or repaired, never crash") {
  std::string err;
  Document d;
  CHECK_FALSE(projectFromJson(d, R"({"format": 5})", &err));
  CHECK_FALSE(projectFromJson(d, R"({"format":"mjforge","bodies":[{"id":1},{"id":2,"parent":1},{"id":2,"parent":2}]})", &err));
  CHECK(err.find("used twice") != std::string::npos);
  // A parent cycle and a geom on a missing body.
  REQUIRE(projectFromJson(d, R"({"format":"mjforge","version":1,"nextId":-5,
      "bodies":[{"id":1},{"id":2,"name":"a","parent":3},{"id":3,"name":"b","parent":2}],
      "geoms":[{"id":4,"body":999,"type":"sphere","size":[0.1,0,0]}]})", &err));
  CHECK(d.body(2)->parent == kWorldId);
  CHECK(d.geom(4)->body == kWorldId);
  CHECK(d.subtree(2).size() >= 1);
  CHECK(d.nextId > 4);
  CHECK(compileMjcf(writeMjcf(d)).ok);
  // Infinite values survive a save / load cycle.
  Document inf;
  Id b = addBody(inf, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j = addJoint(inf, b, JointType::Slide, {}, {0, 0, 1}, "z");
  addGeom(inf, b, GeomType::Sphere, {0.1, 0, 0}, {}, "g");
  inf.joint(j)->limited = true;
  inf.joint(j)->range = {-std::numeric_limits<double>::infinity(), 0.5};
  inf.touch();
  Document back;
  REQUIRE(projectFromJson(back, projectToJson(inf), &err));
  CHECK(back.joint(j)->range.x < -1e299);
  // Mesh faces pointing past the vertex array are dropped.
  REQUIRE(projectFromJson(d, R"({"format":"mjforge","version":3,"bodies":[{"id":1}],
      "meshes":[{"id":2,"name":"m","vertex":[0,0,0, 1,0,0, 0,1,0, 0,0,1],"face":[0,1,2, 0,2,1000000, 0,1,3]}]})", &err));
  CHECK(d.mesh(2)->face->size() == 6);
  // Names with control characters still give loadable XML.
  Document named;
  addBody(named, kWorldId, {{0, 0, 1}, {}}, std::string("bad\x01name", 8));
  named.touch();
  CHECK(compileMjcf(writeMjcf(named)).ok);
}

TEST_CASE("review: numbers are written exactly and tiny values survive") {
  for (double v : {0.1, 1.0 / 3.0, 0.9076733711903687, -2.5e-7, 1e-13, 123456.789})
    CHECK(std::strtod(formatNumber(v, 0).c_str(), nullptr) == v);
  CHECK(formatNumber(0.1, 0) == "0.1");
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addJoint(doc, b, JointType::Slide, {}, {0, 0, 1}, "z");
  doc.body(b)->hasInertial = true;
  doc.body(b)->mass = 1e-6;
  doc.body(b)->diaginertia = {1e-13, 1e-13, 1e-13};
  doc.touch();
  compileDoc(doc);  // was "0 0 0": mass and inertia too small for MuJoCo
}

TEST_CASE("review: new physics settings round-trip through MJCF") {
  Document doc;
  doc.option.integrator = Integrator::Discrete;
  doc.option.sleep = true;
  doc.option.sleep_tolerance = 2e-3;
  doc.option.disableactuator = 1 << 2;
  doc.option.memory = "20M";
  doc.option.ccd_tolerance = 1e-5;
  doc.option.density = 1.2;
  Id floor = addGeom(doc, kWorldId, GeomType::Box, {2, 0.3, 0.05}, {}, "belt");
  doc.geom(floor)->surfacevel = Vec6{{0.5, 0, 0, 0, 0, 0}};
  doc.geom(floor)->adhesion = 2;
  Id box = addBody(doc, kWorldId, {{0, 0, 0.2}, {}}, "box");
  addJoint(doc, box, JointType::Free, {}, {0, 0, 1}, "root");
  doc.body(box)->sleep = SleepPolicy::Init;
  Id bg = addGeom(doc, box, GeomType::Box, {0.1, 0.1, 0.1}, {}, "box_geom");
  doc.geom(bg)->fluidEllipsoid = true;
  Id site = addSite(doc, box, {}, "box_site");
  Id contact = addSensor(doc, SensorType::Contact, site, "touching");
  doc.sensor(contact)->ref = floor;
  doc.sensor(contact)->reftype = FrameObj::Geom;
  doc.sensor(contact)->condata = 1 | 2;  // found, force
  doc.sensor(contact)->connum = 2;
  doc.sensor(contact)->nsample = 5;
  doc.sensor(contact)->delay = 0.004;
  Id dist = addSensor(doc, SensorType::Distance, bg, "gap");
  doc.sensor(dist)->ref = floor;
  doc.sensor(dist)->cutoff = 1;
  Id inside = addSensor(doc, SensorType::InsideSite, box, "inside");
  doc.sensor(inside)->objtype = FrameObj::Body;
  Id zone = addSite(doc, kWorldId, {{0, 0, 0.2}, {}}, "zone");
  doc.site(zone)->type = GeomType::Box;
  doc.site(zone)->size = {0.5, 0.5, 0.5};
  doc.sensor(inside)->ref = zone;
  doc.touch();
  std::string xml = writeMjcf(doc);
  CompileResult r = compileMjcf(xml);
  INFO(r.error);
  REQUIRE(r.ok);
  const mjModel* m = r.model.get();
  CHECK(m->opt.integrator == mjINT_DISCRETE);
  CHECK(m->opt.disableactuator == (1 << 2));
  CHECK(m->geom_surfacevel[0] == doctest::Approx(0.5));
  CHECK(m->geom_adhesion[0] == doctest::Approx(2));
  CHECK(m->tree_sleep_policy[0] == mjSLEEP_INIT);
  CHECK(m->sensor_dim[0] == 2 * (1 + 3));
  CHECK(m->sensor_delay[0] == doctest::Approx(0.004));
  // Import and export again: the same file.
  Document back;
  std::string err;
  REQUIRE(importMjcfString(xml, back, &err));
  CHECK(writeMjcf(back) == xml);
}

namespace {

bool hasIssue(const AnalysisReport& r, const char* code) {
  for (const Issue& i : r.issues)
    if (i.code == code) return true;
  return false;
}

}  // namespace

TEST_CASE("review: save and load give back every digit of the export") {
  // Quaternions a rounding error off unit length, as edits leave them, came back renormalized.
  Document doc;
  makeTemplate("quadruped", doc);
  Quat q{0.6350912284318131, -0.5881470238628586, 0.05390620477312463, 0.4978316291450259};
  REQUIRE(normalize(q).w != q.w);
  doc.bodies[2].quat = q;
  doc.geoms[1].quat = q;
  startKeyframe(doc);
  std::string xml = writeMjcf(doc);
  Document back;
  std::string err;
  REQUIRE(projectFromJson(back, projectToJson(doc), &err));
  CHECK(writeMjcf(back) == xml);
  // A quaternion that is really not unit length (a hand-edited file) is still normalized.
  std::string json = projectToJson(doc);
  nlohmann::json j = nlohmann::json::parse(json);
  for (auto& b : j["bodies"])
    if (b.contains("quat")) b["quat"] = {2, 0, 0, 0};
  REQUIRE(projectFromJson(back, j.dump(), &err));
  CHECK(back.bodies[2].quat.w == doctest::Approx(1));
}

TEST_CASE("review: sensors fit the joints and objects they read") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addGeom(doc, b, GeomType::Sphere, {0.1, 0, 0}, {}, "b_geom");
  Id free = addJoint(doc, b, JointType::Free, {}, {0, 0, 1}, "root");
  Id c = addBody(doc, b, {{0.2, 0, 0}, {}}, "c");
  addGeom(doc, c, GeomType::Sphere, {0.05, 0, 0}, {});
  Id ball = addJoint(doc, c, JointType::Ball, {}, {0, 0, 1}, "shoulder");
  Id site = addSite(doc, c, {}, "tip");
  // Joint position and velocity sensors on ball and free joints become the ones MuJoCo has for them.
  CHECK(doc.sensor(addSensor(doc, SensorType::JointPos, ball))->type == SensorType::BallQuat);
  CHECK(doc.sensor(addSensor(doc, SensorType::JointVel, ball))->type == SensorType::BallAngVel);
  Id fp = addSensor(doc, SensorType::JointPos, free);
  CHECK(doc.sensor(fp)->type == SensorType::FramePos);
  CHECK(doc.sensor(fp)->obj == b);
  CHECK(doc.sensor(fp)->objtype == FrameObj::Body);
  compileDoc(doc);

  // A joint sensor put on a ball joint is refused by MuJoCo, and the analysis says so first.
  Document bad = doc;
  bad.sensors[0].type = SensorType::JointPos;
  CHECK_FALSE(compileMjcf(writeMjcf(bad)).ok);
  AnalysisReport rep = analyzeModel(bad, nullptr, false);
  CHECK(hasIssue(rep, "sensor-kind"));

  // Changing a sensor's type keeps an object that fits the new type, with matching object types.
  Sensor& s = *doc.sensor(fp);
  s.ref = site;
  s.reftype = FrameObj::Site;
  setSensorType(doc, s, SensorType::FrameQuat);  // same kind: everything stays
  CHECK(s.obj == b);
  CHECK(s.ref == site);
  setSensorType(doc, s, SensorType::Contact);  // a body is a valid contact match, a site is not a second one
  CHECK(s.obj == b);
  CHECK(s.objtype == FrameObj::Body);
  CHECK(s.ref == kNoId);
  CHECK(s.reftype == FrameObj::Geom);
  setSensorType(doc, s, SensorType::Touch);  // touch reads a site
  CHECK(s.obj == kNoId);
  s.obj = site;
  setSensorType(doc, s, SensorType::Contact);  // contacts inside a site volume
  CHECK(s.obj == site);
  CHECK(s.objtype == FrameObj::Site);
  compileDoc(doc);
}

TEST_CASE("review: impossible inertias and self-referencing constraints are reported before MuJoCo refuses them") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addJoint(doc, b, JointType::Hinge, {}, {0, 1, 0});
  addGeom(doc, b, GeomType::Sphere, {0.05, 0, 0}, {});
  Body& body = *doc.body(b);
  body.hasInertial = true;
  body.mass = 1;
  body.diaginertia = {0.01, 0.01, 0.05};  // 0.01 + 0.01 < 0.05
  CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
  {
    AnalysisReport rep = analyzeModel(doc, nullptr, false);
    const Issue* tri = nullptr;
    for (const Issue& i : rep.issues)
      if (i.code == "inertia-triangle") tri = &i;
    REQUIRE(tri);
    REQUIRE(tri->fix);
    Document fixed = doc;
    tri->fix(fixed);
    compileDoc(fixed);
  }
  // MuJoCo averages them itself with balanceinertia, and ignores them when inertia comes from geoms.
  doc.compiler.balanceinertia = true;
  compileDoc(doc);
  CHECK_FALSE(hasIssue(analyzeModel(doc, nullptr, false), "inertia-triangle"));
  doc.compiler.balanceinertia = false;
  doc.compiler.inertiafromgeom = InertiaFromGeom::True;
  compileDoc(doc);
  CHECK_FALSE(hasIssue(analyzeModel(doc, nullptr, false), "inertia-triangle"));
  doc.compiler.inertiafromgeom = InertiaFromGeom::Auto;
  body.diaginertia = {0.01, 0.01, 0.01};

  // A weld from a site to itself.
  Id site = addSite(doc, b, {}, "s");
  addEquality(doc, EqualityType::Weld, site, site);
  CHECK_FALSE(compileMjcf(writeMjcf(doc)).ok);
  CHECK(hasIssue(analyzeModel(doc, nullptr, false), "equality-target"));
}

TEST_CASE("review: camera image settings reach MuJoCo and come back on import") {
  Document doc;
  Id b = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "b");
  addGeom(doc, b, GeomType::Sphere, {0.05, 0, 0}, {});
  Id c = addCamera(doc, kWorldId, {{1, 0, 1}, {}}, "eye");
  Camera& cam = *doc.camera(c);
  cam.resolution[0] = 640;
  cam.resolution[1] = 480;
  cam.output = 1 | 2;
  cam.sensorsize = {0.0036, 0.0027};
  cam.focal = {0.004, 0.004};
  cam.principal = {0.0001, -0.0002};
  Id o = addCamera(doc, kWorldId, {{0, 1, 1}, {}}, "top");
  doc.camera(o)->orthographic = true;
  doc.camera(o)->fovy = 2;
  std::string xml = writeMjcf(doc);
  ModelPtr m = compileDoc(doc);
  int id = mj_name2id(m.get(), mjOBJ_CAMERA, "eye");
  REQUIRE(id >= 0);
  CHECK(m->cam_resolution[2 * id] == 640);
  CHECK(m->cam_resolution[2 * id + 1] == 480);
  CHECK(m->cam_output[id] == 3);
  CHECK(m->cam_sensorsize[2 * id] == doctest::Approx(0.0036));
  CHECK(m->cam_intrinsic[4 * id] == doctest::Approx(0.004));
  CHECK(m->cam_intrinsic[4 * id + 3] == doctest::Approx(-0.0002));
  int top = mj_name2id(m.get(), mjOBJ_CAMERA, "top");
  CHECK(m->cam_projection[top] == mjPROJ_ORTHOGRAPHIC);
  CHECK(m->cam_fovy[top] == doctest::Approx(2));
  Document back;
  std::string err;
  REQUIRE(importMjcfString(xml, back, &err));
  CHECK(writeMjcf(back) == xml);
}
