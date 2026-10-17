// Importing MJCF the editor stores differently, and combining scenes: each case failed before.
#include <doctest/doctest.h>

#include <cmath>
#include <unordered_set>

#include "analysis/Analysis.h"
#include "core/Generators.h"
#include "core/Merge.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/Templates.h"
#include "sim/MjcfImport.h"
#include "sim/MujocoBridge.h"

using namespace mjf;

namespace {

Document importXml(const char* xml, ImportReport* rep = nullptr) {
  Document doc;
  std::string err;
  ImportReport local;
  INFO(err);
  REQUIRE(importMjcfString(xml, doc, &err, rep ? rep : &local));
  return doc;
}

ModelPtr compile(const Document& doc) {
  CompileResult r = compileMjcf(writeMjcf(doc));
  INFO(r.error);
  REQUIRE(r.ok);
  return r.model;
}

ModelPtr compileXml(const char* xml) {
  char err[1000] = "";
  mjSpec* spec = mj_parseXMLString(xml, nullptr, err, sizeof err);
  INFO(err);
  REQUIRE(spec);
  mjModel* m = mj_compile(spec, nullptr);
  mj_deleteSpec(spec);
  REQUIRE(m);
  return makeModelPtr(m);
}

double bodyMass(const mjModel* m, const char* name) { return m->body_mass[mj_name2id(m, mjOBJ_BODY, name)]; }

}  // namespace

TEST_CASE("import: keyframe activations and mocap poses survive the round trip") {
  const char* xml = R"(<mujoco>
    <worldbody>
      <body name="target" mocap="true" pos="0 0 1"><geom type="sphere" size="0.05" contype="0" conaffinity="0"/></body>
      <body name="arm" pos="0 0 0.5"><joint name="j" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/></body>
      <body name="arm2" pos="1 0 0.5"><joint name="j2" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.03"/></body>
    </worldbody>
    <actuator><position joint="j" kp="20" timeconst="0.1"/><intvelocity joint="j2" kp="20" actrange="-1 1"/></actuator>
    <keyframe><key qpos="0.2 -0.3" act="0.3 -0.4" ctrl="0.2 0" mpos="0.5 0.5 1.5" mquat="0.7071068 0 0 0.7071068"/></keyframe>
  </mujoco>)";
  ModelPtr src = compileXml(xml);
  Document doc = importXml(xml);
  REQUIRE(doc.keyframes.size() == 1);
  CHECK(doc.keyframes[0].act.size() == 2);
  CHECK(doc.keyframes[0].mocap.size() == 1);
  ModelPtr out = compile(doc);
  for (int i = 0; i < 2; ++i) CHECK(out->key_act[i] == doctest::Approx(src->key_act[i]));
  for (int i = 0; i < 3; ++i) CHECK(out->key_mpos[i] == doctest::Approx(src->key_mpos[i]));
  for (int i = 0; i < 4; ++i) CHECK(out->key_mquat[i] == doctest::Approx(src->key_mquat[i]).epsilon(1e-6));

  // Holding the start pose: a filtered servo starts settled, an intvelocity servo at its target.
  syncKeyframeControls(doc, doc.keyframes[0]);
  ModelPtr synced = compile(doc);
  CHECK(synced->key_act[0] == doctest::Approx(0.2));   // the position servo's filtered target
  CHECK(synced->key_act[1] == doctest::Approx(-0.3));  // the intvelocity servo's integrated target
  CHECK(synced->key_ctrl[1] == 0);
}

TEST_CASE("import: masses and gains scaled by a frame are MuJoCo's") {
  const char* xml = R"(<mujoco>
    <worldbody>
      <frame scale="2">
        <body name="b" pos="0 0 1">
          <joint name="j" axis="0 1 0"/>
          <geom type="sphere" size="0.05" mass="1"/>
        </body>
      </frame>
    </worldbody>
    <actuator><position joint="j" kp="10"/></actuator>
  </mujoco>)";
  ModelPtr src = compileXml(xml);
  ImportReport rep;
  Document doc = importXml(xml, &rep);
  ModelPtr out = compile(doc);
  CHECK(bodyMass(out.get(), "b") == doctest::Approx(bodyMass(src.get(), "b")));
  CHECK(out->actuator_gainprm[0] == doctest::Approx(src->actuator_gainprm[0]));
  CHECK(out->actuator_biasprm[1] == doctest::Approx(src->actuator_biasprm[1]));
  CHECK(!rep.notes.empty());
}

TEST_CASE("import: settotalmass with mixed inertials, inertiagrouprange, zero-mass inertials") {
  const char* xml = R"(<mujoco>
    <compiler settotalmass="10" inertiagrouprange="0 2"/>
    <worldbody>
      <body name="a" pos="0 0 1"><freejoint/><inertial pos="0 0 0" mass="3" diaginertia="0.1 0.1 0.1"/>
        <geom type="box" size="0.1 0.1 0.1"/></body>
      <body name="b" pos="1 0 1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/>
        <geom type="sphere" size="0.2" group="3"/></body>
      <body name="mount" pos="0 1 1"><inertial pos="0 0 0" mass="0" diaginertia="0 0 0"/>
        <geom type="box" size="0.05 0.05 0.05" contype="0" conaffinity="0"/></body>
    </worldbody>
  </mujoco>)";
  ModelPtr src = compileXml(xml);
  Document doc = importXml(xml);
  CHECK(doc.compiler.settotalmass <= 0);  // baked: never applied twice
  ModelPtr out = compile(doc);
  for (const char* name : {"a", "b", "mount"}) {
    INFO(name);
    CHECK(bodyMass(out.get(), name) == doctest::Approx(bodyMass(src.get(), name)));
  }
  CHECK(bodyMass(out.get(), "mount") == 0);
}

TEST_CASE("import: constraints anchored to the world as body1") {
  const char* xml = R"(<mujoco>
    <worldbody>
      <body name="box" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
      <body name="box2" pos="1 0 1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body>
    </worldbody>
    <equality><connect body1="world" body2="box" anchor="0 0 1"/><weld body1="world" body2="box2"/></equality>
  </mujoco>)";
  Document doc = importXml(xml);
  REQUIRE(doc.equalities.size() == 2);
  CHECK(doc.equalities[0].obj1 == kWorldId);
  ModelPtr out = compile(doc);
  CHECK(out->neq == 2);
}

TEST_CASE("merge: the start pose joins keyframe 0, other keyframes stay apart") {
  Document src = importXml(R"(<mujoco>
    <worldbody><body name="pend" pos="0 0 1"><joint name="h" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0 0 -0.5" size="0.02"/></body></worldbody>
    <keyframe><key name="balanced" qpos="0.3"/><key qpos="-0.7"/><key qpos="0.5"/></keyframe>
  </mujoco>)");
  Document dst;
  REQUIRE(makeTemplate("quadruped", dst));
  Keyframe home;
  home.name = "home";
  dst.keyframes.push_back(home);
  size_t before = dst.keyframes.size();
  MergeOptions mo;
  mo.environment = false;
  mergeDocument(dst, src, mo);
  CHECK(dst.keyframes.size() == before + 2);
  Id h = kNoId;
  for (const Joint& j : dst.joints) if (j.name == "h") h = j.id;
  const std::vector<double>* q = dst.keyframes[0].jointQpos(h);
  REQUIRE(q);
  CHECK((*q)[0] == doctest::Approx(0.3));
  ModelPtr out = compile(dst);
  CHECK(out->nkey == long(before + 2));
}

TEST_CASE("merge: walls are not floors, and pairs with a skipped floor use the scene's") {
  Document src = importXml(R"(<mujoco>
    <worldbody>
      <geom name="floor" type="plane" size="5 5 0.1"/>
      <geom name="wall" type="plane" size="5 5 0.1" pos="5 0 0" zaxis="-1 0 0"/>
      <body name="ball" pos="0 0 0.5"><freejoint/><geom name="foot" type="sphere" size="0.1"/></body>
    </worldbody>
    <contact><pair geom1="foot" geom2="floor" friction="2 2 0.01 0.0001 0.0001"/></contact>
  </mujoco>)");
  Document dst;
  REQUIRE(makeTemplate("empty", dst));
  MergeResult res = mergeDocument(dst, src, {});
  CHECK(res.skippedFloors == 1);
  int planes = 0;
  for (const Geom& g : dst.geoms) planes += g.body == kWorldId && g.type == GeomType::Plane;
  CHECK(planes == 2);  // the scene's floor and the wall
  REQUIRE(dst.pairs.size() == 1);
  compile(dst);
}

TEST_CASE("merge: imported bodies keep their masses, the scene keeps its settings") {
  Document src = importXml(R"(<mujoco>
    <compiler boundmass="0.5" boundinertia="0.01"/>
    <worldbody><body name="pebble" pos="0 0 1"><freejoint/><geom type="sphere" size="0.01"/></body></worldbody>
  </mujoco>)");
  ModelPtr alone = compile(src);
  Document dst;
  REQUIRE(makeTemplate("quadruped", dst));
  ModelPtr before = compile(dst);
  double dstMass = 0;
  for (int b = 1; b < before->nbody; ++b) dstMass += before->body_mass[b];
  MergeOptions mo;
  mo.settings = true;
  mergeDocument(dst, src, mo);
  ModelPtr after = compile(dst);
  CHECK(bodyMass(after.get(), "pebble") == doctest::Approx(bodyMass(alone.get(), "pebble")));
  double total = 0;
  for (int b = 1; b < after->nbody; ++b) total += after->body_mass[b];
  CHECK(total - bodyMass(after.get(), "pebble") == doctest::Approx(dstMass));
}

TEST_CASE("merge: a model welded to the world keeps its place when offset") {
  Document src = importXml(R"(<mujoco>
    <worldbody><body name="base" pos="0 0 1"><freejoint/><geom type="box" size="0.1 0.1 0.1"/></body></worldbody>
    <equality><weld body1="base" relpose="0 0 -1 1 0 0 0"/></equality>
  </mujoco>)");
  Document dst;
  REQUIRE(makeTemplate("empty", dst));
  MergeOptions mo;
  mo.environment = false;
  mo.modelOffset = {2, 0, 0};
  mergeDocument(dst, src, mo);
  ModelPtr m = compile(dst);
  DataPtr d = makeData(m.get());
  int b = mj_name2id(m.get(), mjOBJ_BODY, "base");
  for (int i = 0; i < 500; ++i) mj_step(m.get(), d.get());
  CHECK(d->xpos[3 * b] == doctest::Approx(2).epsilon(1e-3));
  CHECK(d->xpos[3 * b + 2] == doctest::Approx(1).epsilon(1e-3));
}

TEST_CASE("merge: replacing the environment, self merges, asset identity, lost camera targets") {
  const char* env = R"(<mujoco>
    <asset>
      <texture name="tex" type="2d" builtin="checker" rgb1="0 0 1" rgb2="1 1 1" width="64" height="64"/>
      <material name="mat" texture="tex"/>
    </asset>
    <worldbody>
      <camera name="overview" pos="3 0 2" xyaxes="0 1 0 -0.5 0 1"/>
      <site name="goal" pos="1 1 0"/>
      <camera name="follow" mode="targetbody" target="robot" pos="2 2 2"/>
      <geom name="blue_box" type="box" size="0.1 0.1 0.1" pos="0 2 0.1" material="mat"/>
      <body name="robot" pos="0 0 0.5"><freejoint/><geom type="sphere" size="0.1"/></body>
    </worldbody>
  </mujoco>)";
  Document src = importXml(env);
  Document dst;
  REQUIRE(makeTemplate("empty", dst));
  // A material of the same name whose texture differs must stay a separate material.
  Texture red;
  red.name = "tex";
  red.builtin = TextureBuiltin::Checker;
  red.rgb1 = {1, 0, 0};
  Material mat;
  mat.name = "mat";
  mat.texture = addTexture(dst, red);
  addMaterial(dst, mat);
  MergeOptions envOnly;
  envOnly.models = false;
  envOnly.replaceEnvironment = true;
  mergeDocument(dst, src, envOnly);
  mergeDocument(dst, src, envOnly);
  int cams = 0, sites = 0;
  for (const Camera& c : dst.cameras) {
    cams += c.body == kWorldId;
    if (c.name.rfind("follow", 0) == 0) CHECK(c.mode == CameraMode::Fixed);  // its target stayed behind
  }
  for (const Site& s : dst.sites) sites += s.body == kWorldId;
  CHECK(cams == 2);  // overview and follow, once
  CHECK(sites == 1);
  const Geom* box = nullptr;
  for (const Geom& g : dst.geoms) if (g.name == "blue_box") box = &g;
  REQUIRE(box);
  const Material* bm = dst.material(box->material);
  REQUIRE(bm);
  REQUIRE(dst.texture(bm->texture));
  CHECK(dst.texture(bm->texture)->rgb1.z == doctest::Approx(1));
  // Merging a document into itself copies everything once.
  Document self;
  REQUIRE(makeTemplate("biped", self));
  size_t bodies = self.bodies.size();
  mergeDocument(self, self, {});
  CHECK(self.bodies.size() == 2 * bodies - 1);
  compile(self);
}

TEST_CASE("DC motors: import, export, analysis and a fresh one") {
  const char* xml = R"(<mujoco>
    <worldbody>
      <body name="base" pos="0 0 1">
        <body name="arm"><joint name="shoulder" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.02" mass="1"/></body>
        <body name="arm2" pos="0 0.3 0"><joint name="elbow" axis="0 1 0"/><geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.02" mass="1"/></body>
      </body>
    </worldbody>
    <actuator>
      <dcmotor name="m1" joint="shoulder" nominal="24 8 12" ctrlrange="-24 24" saturation="6 0 0" inductance="0 0.001" thermal="2 0 30 0.004 25 25"/>
      <dcmotor name="m2" joint="elbow" motorconst="0.05 0.05" resistance="0.5" input="pos" controller="40 0 2 0 0 24" ctrlrange="-1.5 1.5"/>
    </actuator>
  </mujoco>)";
  ModelPtr src = compileXml(xml);
  Document doc = importXml(xml);
  REQUIRE(doc.actuators.size() == 2);
  CHECK(doc.actuators[0].type == ActuatorType::DcMotor);
  CHECK(doc.actuators[1].dcinput == DcInput::Pos);
  ModelPtr out = compile(doc);
  for (int k = 0; k < mjNGAIN; ++k) CHECK(out->actuator_gainprm[k] == doctest::Approx(src->actuator_gainprm[k]));
  for (int k = 0; k < mjNDYN; ++k) CHECK(out->actuator_dynprm[k] == doctest::Approx(src->actuator_dynprm[k]));
  CHECK(out->actuator_forcerange[1] == doctest::Approx(src->actuator_forcerange[1]));
  CHECK(out->actuator_ctrlspec[1] == src->actuator_ctrlspec[1]);
  CHECK(out->na == src->na);

  // Strength: the stall torque (K V / R = 8 N*m), capped by the saturation torque of 6 N*m.
  CHECK(actuatorMaxForce(doc, doc.actuators[0]) == doctest::Approx(6));
  doc.actuators[0].saturation = {0, 0, 0};
  CHECK(actuatorMaxForce(doc, doc.actuators[0]) == doctest::Approx(8));

  // A new DC motor on a joint compiles as it is.
  Document fresh;
  Id b = addBody(fresh, kWorldId, {{0, 0, 1}, {}}, "b");
  Id j = addJoint(fresh, b, JointType::Hinge, {}, {0, 1, 0}, "j");
  addGeom(fresh, b, GeomType::Sphere, {0.05, 0, 0}, {{0.2, 0, 0}, {}});
  Id a = addActuator(fresh, ActuatorType::Motor, j);
  setActuatorType(fresh, *fresh.actuator(a), ActuatorType::DcMotor);
  compile(fresh);
}

TEST_CASE("combine: generated terrain brought into a scene that has one regenerates on its own") {
  Document dst, src;
  REQUIRE(makeTemplate("empty", dst));
  REQUIRE(makeTemplate("empty", src));
  Id mine = addGenerated(dst, "rough", {0, 0, 0});
  Id theirs = addGenerated(src, "rough", {0, 0, 0});
  for (Geom& g : src.geoms)
    if (g.body == theirs) g.friction.x = 0.2;  // edited before the import
  MergeOptions opt;
  mergeDocument(dst, src, opt);
  Id imported = kNoId;
  for (const Body& b : dst.bodies)
    if (b.generator == "rough" && b.id != mine) imported = b.id;
  REQUIRE(imported);
  std::vector<float> before;
  for (const Geom& g : dst.geoms)
    if (g.body == mine) before = *dst.hfield(g.hfield)->data;
  dst.body(imported)->genParams[7] = 99;  // a new seed for the imported terrain only
  REQUIRE(regenerate(dst, imported));
  int pieces = 0;
  std::unordered_set<Id> ids;
  std::unordered_set<std::string> names;
  bool unique = true;
  for (const Geom& g : dst.geoms) {
    unique = ids.insert(g.id).second && names.insert(g.name).second && unique;
    if (g.body == imported) {
      ++pieces;
      CHECK(g.friction.x == doctest::Approx(0.2));
    }
    if (g.body == mine) CHECK(*dst.hfield(g.hfield)->data == before);
  }
  CHECK(unique);
  CHECK(pieces == 1);
  CompileResult cr = compileMjcf(writeMjcf(dst));
  INFO(cr.error);
  CHECK(cr.ok);
}
