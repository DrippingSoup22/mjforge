#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <filesystem>

#include "analysis/Analysis.h"
#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Merge.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
#include "core/ProjectIO.h"
#include "core/Templates.h"
#include "sim/MjcfImport.h"
#include "sim/MujocoBridge.h"
#include "sim/Simulator.h"

using namespace mjf;

namespace {

ModelPtr compileDoc(const Document& doc) {
  CompileResult cr = compileMjcf(writeMjcf(doc));
  INFO(cr.error);
  REQUIRE(cr.ok);
  return cr.model;
}

ModelPtr compileXml(const std::string& xml) {
  CompileResult cr = compileMjcf(xml);
  INFO(cr.error);
  REQUIRE(cr.ok);
  return cr.model;
}

// Relative tolerance: numbers are written with 9 significant digits.
void checkClose(const mjtNum* a, const mjtNum* b, int n, double tol, const std::string& what) {
  double worst = 0;
  for (int i = 0; i < n; ++i) worst = std::max(worst, std::fabs(a[i] - b[i]) / std::max(1.0, std::fabs(a[i])));
  INFO(what << " max relative diff " << worst);
  CHECK(worst <= tol);
}

// The physics-relevant parts of two compiled models agree.
void checkSameModel(const mjModel* a, const mjModel* b) {
  REQUIRE(a->nbody == b->nbody);
  REQUIRE(a->njnt == b->njnt);
  REQUIRE(a->ngeom == b->ngeom);
  REQUIRE(a->nu == b->nu);
  CHECK(a->nsite == b->nsite);
  CHECK(a->nsensor == b->nsensor);
  CHECK(a->nlight == b->nlight);
  CHECK(a->nmat == b->nmat);
  checkClose(a->body_pos, b->body_pos, 3 * a->nbody, 1e-9, "body_pos");
  checkClose(a->body_quat, b->body_quat, 4 * a->nbody, 1e-9, "body_quat");
  checkClose(a->body_mass, b->body_mass, a->nbody, 1e-9, "body_mass");
  checkClose(a->body_inertia, b->body_inertia, 3 * a->nbody, 1e-9, "body_inertia");
  checkClose(a->geom_size, b->geom_size, 3 * a->ngeom, 1e-9, "geom_size");
  checkClose(a->geom_pos, b->geom_pos, 3 * a->ngeom, 1e-9, "geom_pos");
  checkClose(a->geom_friction, b->geom_friction, 3 * a->ngeom, 1e-12, "geom_friction");
  checkClose(a->jnt_range, b->jnt_range, 2 * a->njnt, 1e-12, "jnt_range");
  checkClose(a->jnt_axis, b->jnt_axis, 3 * a->njnt, 1e-12, "jnt_axis");
  checkClose(a->dof_damping, b->dof_damping, a->nv, 1e-12, "dof_damping");
  checkClose(a->dof_armature, b->dof_armature, a->nv, 1e-12, "dof_armature");
  checkClose(a->actuator_gainprm, b->actuator_gainprm, mjNGAIN * a->nu, 1e-9, "actuator_gainprm");
  checkClose(a->actuator_biasprm, b->actuator_biasprm, mjNBIAS * a->nu, 1e-9, "actuator_biasprm");
  checkClose(a->actuator_dynprm, b->actuator_dynprm, mjNDYN * a->nu, 1e-9, "actuator_dynprm");
  checkClose(a->actuator_gear, b->actuator_gear, 6 * a->nu, 1e-12, "actuator_gear");
  checkClose(a->actuator_ctrlrange, b->actuator_ctrlrange, 2 * a->nu, 1e-12, "actuator_ctrlrange");
  checkClose(a->actuator_forcerange, b->actuator_forcerange, 2 * a->nu, 1e-12, "actuator_forcerange");
  for (int i = 0; i < a->nu; ++i) {
    CHECK(a->actuator_gaintype[i] == b->actuator_gaintype[i]);
    CHECK(a->actuator_biastype[i] == b->actuator_biastype[i]);
    CHECK(a->actuator_dyntype[i] == b->actuator_dyntype[i]);
    CHECK(a->actuator_trnid[2 * i] == b->actuator_trnid[2 * i]);
  }
  for (int i = 0; i < a->nsensor; ++i) {
    CHECK(a->sensor_type[i] == b->sensor_type[i]);
    CHECK(a->sensor_objid[i] == b->sensor_objid[i]);
  }
  CHECK(a->opt.timestep == b->opt.timestep);
  CHECK(a->opt.cone == b->opt.cone);
  CHECK(a->opt.impratio == b->opt.impratio);
  CHECK(a->opt.integrator == b->opt.integrator);
  REQUIRE(a->nhfielddata == b->nhfielddata);
  for (int i = 0; i < a->nhfielddata; ++i) CHECK(a->hfield_data[i] == doctest::Approx(b->hfield_data[i]));
}

}  // namespace

TEST_CASE("import: templates round-trip through MJCF") {
  for (const char* t : {"quadruped", "hexapod", "biped", "hopper", "arm", "empty"}) {
    INFO(std::string(t));
    Document doc;
    REQUIRE(makeTemplate(t, doc));
    std::string xml = writeMjcf(doc);
    Document back;
    std::string err;
    ImportReport rep;
    REQUIRE(importMjcfString(xml, back, &err, &rep));
    CHECK(rep.notes.empty());
    checkSameModel(compileXml(xml).get(), compileDoc(back).get());
    // Authoring intent survives: shortcut types, damping ratios, densities, roles.
    REQUIRE(back.actuators.size() == doc.actuators.size());
    for (size_t i = 0; i < doc.actuators.size(); ++i) {
      CHECK(back.actuators[i].type == doc.actuators[i].type);
      CHECK(back.actuators[i].dampratio == doctest::Approx(doc.actuators[i].dampratio));
    }
    for (size_t i = 0; i < doc.bodies.size(); ++i) {
      CHECK(back.bodies[i].hasInertial == doc.bodies[i].hasInertial);
      if (doc.bodies[i].parent == kWorldId) CHECK(back.bodies[i].role == doc.bodies[i].role);
    }
  }
}

TEST_CASE("import: MuJoCo features are resolved (defaults, degrees, fromto, meshes)") {
  const char* xml = R"(
<mujoco model="probe">
  <compiler angle="degree"/>
  <option timestep="0.004" integrator="implicitfast"/>
  <default>
    <joint damping="0.7" armature="0.02"/>
    <geom friction="0.8 0.01 0.001" rgba="0.2 0.4 0.6 1"/>
    <default class="leg">
      <joint range="-45 30"/>
      <geom density="400"/>
    </default>
  </default>
  <asset>
    <mesh name="tetra" vertex="0 0 0  0.2 0 0  0 0.2 0  0 0 0.2"/>
    <hfield name="hf" nrow="3" ncol="2" size="2 2 0.4 0.1" elevation="0 1 2 3 4 5"/>
  </asset>
  <worldbody>
    <geom name="floor" type="plane" size="0 0 0.05"/>
    <geom name="terrain" type="hfield" hfield="hf" pos="4 0 0"/>
    <body name="torso" pos="0 0 0.5" euler="0 0 90">
      <freejoint name="root"/>
      <geom name="chest" type="box" size="0.2 0.1 0.05"/>
      <geom name="hull" type="mesh" mesh="tetra"/>
      <body name="thigh" childclass="leg" pos="0.2 0 0">
        <joint name="hip" axis="0 1 0"/>
        <geom name="thigh_geom" type="capsule" fromto="0 0 0 0 0 -0.3" size="0.03"/>
        <site name="foot" pos="0 0 -0.3"/>
      </body>
    </body>
  </worldbody>
  <tendon>
    <fixed name="t"><joint joint="hip" coef="1"/></fixed>
  </tendon>
  <actuator>
    <position name="hip_pos" joint="hip" kp="60" dampratio="0.8" ctrlrange="-40 20"/>
    <velocity name="hip_vel" joint="hip" kv="3"/>
    <intvelocity name="hip_iv" joint="hip" kp="20" actrange="-0.5 0.5"/>
    <motor name="hip_motor" joint="hip" gear="25" ctrlrange="-1 1"/>
    <general name="hip_gen" joint="hip" gainprm="5" biasprm="0 -5 -0.5" biastype="affine"/>
    <motor name="tendon_motor" tendon="t"/>
  </actuator>
  <sensor>
    <jointpos joint="hip"/>
    <touch site="foot"/>
    <framequat objtype="body" objname="torso"/>
    <tendonpos tendon="t"/>
  </sensor>
  <keyframe><key name="home" qpos="0 0 0.5 1 0 0 0 0"/></keyframe>
</mujoco>)";
  Document doc;
  std::string err;
  ImportReport rep;
  REQUIRE(importMjcfString(xml, doc, &err, &rep));
  CHECK(doc.modelName == "probe");
  CHECK(doc.option.timestep == 0.004);
  CHECK(rep.approximatedGeoms == 0);
  CHECK(rep.notes.empty());
  REQUIRE(doc.meshes.size() == 1);
  CHECK(doc.meshes[0].name == "tetra");
  CHECK(doc.meshes[0].vertexCount() == 4);
  CHECK(doc.meshes[0].faceCount() == 4);  // MuJoCo built the convex hull
  REQUIRE(doc.keyframes.size() == 1);
  CHECK(doc.keyframes[0].name == "home");
  CHECK(doc.keyframes[0].qpos.size() == 2);  // free joint + hinge
  const Body* torso = nullptr;
  for (const Body& b : doc.bodies) if (b.name == "torso") torso = &b;
  REQUIRE(torso);
  CHECK(torso->role == Role::Model);
  CHECK(torso->hasInertial);  // its mesh was approximated: exact inertia kept
  const Joint* hip = nullptr;
  for (const Joint& j : doc.joints) if (j.name == "hip") hip = &j;
  REQUIRE(hip);
  CHECK(hip->range.x == doctest::Approx(deg2rad(-45)));
  CHECK(hip->damping == doctest::Approx(0.7));
  for (const Geom& g : doc.geoms)
    if (g.name == "thigh_geom") {
      CHECK(g.type == GeomType::Capsule);
      CHECK(g.density == 400);
      CHECK(g.size.y == doctest::Approx(0.15));  // half-length from fromto
    }
  REQUIRE(doc.actuators.size() == 6);
  REQUIRE(doc.tendons.size() == 1);
  CHECK(doc.tendons[0].type == TendonType::Fixed);
  CHECK(doc.tendons[0].joints.size() == 1);
  CHECK(doc.actuators[5].trn == TrnType::Tendon);
  CHECK(doc.actuators[5].target == doc.tendons[0].id);
  CHECK(doc.actuators[0].type == ActuatorType::Position);
  CHECK(doc.actuators[0].kp == 60);
  CHECK(doc.actuators[0].dampratio == doctest::Approx(0.8));
  CHECK(doc.actuators[0].ctrlrange.y == 20);  // MuJoCo never converts ctrlrange from degrees
  {
    // ... which the analysis points out, with a fix.
    AnalysisReport ar = analyzeModel(doc, nullptr, false);
    const Issue* deg = nullptr;
    for (const Issue& i : ar.issues) if (i.code == "ctrl-range-degrees") deg = &i;
    REQUIRE(deg);
    Document fixed = doc;
    deg->fix(fixed);
    CHECK(fixed.actuators[0].ctrlrange.y == doctest::Approx(deg2rad(20)));
  }
  CHECK(doc.actuators[1].type == ActuatorType::Velocity);
  CHECK(doc.actuators[2].type == ActuatorType::IntVelocity);
  CHECK(doc.actuators[3].type == ActuatorType::Motor);
  CHECK(doc.actuators[3].gear[0] == 25);
  CHECK(doc.actuators[4].type == ActuatorType::General);
  CHECK(doc.sensors.size() == 4);

  // Same physics.
  ModelPtr a = compileXml(xml);
  ModelPtr b = compileDoc(doc);
  CHECK(a->nbody == b->nbody);
  checkClose(a->body_mass, b->body_mass, a->nbody, 1e-9, "body_mass");
  checkClose(a->body_pos, b->body_pos, 3 * a->nbody, 1e-12, "body_pos");
  checkClose(a->body_quat, b->body_quat, 4 * a->nbody, 1e-12, "body_quat");
  CHECK(a->ntendon == b->ntendon);
  CHECK(a->nu == b->nu);
  CHECK(a->nsensor == b->nsensor);
  checkClose(a->actuator_gainprm, b->actuator_gainprm, mjNGAIN * a->nu, 1e-8, "gainprm");
  checkClose(a->actuator_biasprm, b->actuator_biasprm, mjNBIAS * a->nu, 1e-8, "biasprm");
  REQUIRE(a->nhfielddata == b->nhfielddata);
  for (int i = 0; i < a->nhfielddata; ++i) CHECK(a->hfield_data[i] == doctest::Approx(b->hfield_data[i]));
}

TEST_CASE("import: broken files report MuJoCo's error") {
  Document doc;
  std::string err;
  CHECK_FALSE(importMjcfString("<mujoco><worldbody><geom type='box'/></worldbody>", doc, &err));
  CHECK(!err.empty());
  err.clear();
  CHECK_FALSE(importMjcfString("<mujoco><actuator><motor joint='nope'/></actuator></mujoco>", doc, &err));
  CHECK(!err.empty());
}

TEST_CASE("merge: put a model into an environment and combine projects") {
  Document env;
  REQUIRE(makeTemplate("empty", env));
  Id ramp = addGeom(env, kWorldId, GeomType::Box, {1, 0.5, 0.05}, {{2, 0, 0.1}, axisAngle({0, 1, 0}, 0.2)}, "ramp");
  (void)ramp;
  Document creature;
  REQUIRE(makeTemplate("quadruped", creature));

  // Models only: no second floor, light or skybox.
  MergeOptions mo;
  mo.environment = false;
  mo.modelOffset = {0.5, 0, 0};
  MergeResult r = mergeDocument(env, creature, mo);
  CHECK(r.modelRoots.size() == 1);
  CHECK(r.renamed == 0);
  CHECK(env.actuators.size() == creature.actuators.size());
  CHECK(env.sensors.size() == creature.sensors.size());
  int floors = 0;
  for (const Geom& g : env.geoms) floors += g.body == kWorldId && g.type == GeomType::Plane;
  CHECK(floors == 1);
  CHECK(env.lights.size() == 1);
  CHECK(env.body(r.modelRoots[0])->pos.x == doctest::Approx(creature.bodies[1].pos.x + 0.5));
  ModelPtr m = compileDoc(env);
  CHECK(m->nu == int(creature.actuators.size()));

  // Importing the same creature again renames everything that clashes, and still compiles.
  MergeResult r2 = mergeDocument(env, creature, mo);
  CHECK(r2.modelRoots.size() == 1);
  CHECK(r2.renamed > 0);
  CHECK(env.nameOf(r2.modelRoots[0]) != env.nameOf(r.modelRoots[0]));
  ModelPtr m2 = compileDoc(env);
  CHECK(m2->nu == 2 * int(creature.actuators.size()));

  // Everything, including the environment: the duplicate floor is skipped, materials are shared.
  Document combined;
  REQUIRE(makeTemplate("quadruped", combined));
  size_t mats = combined.materials.size();
  MergeResult r3 = mergeDocument(combined, env, MergeOptions{});
  CHECK(r3.skippedFloors == 1);
  CHECK(combined.materials.size() == mats);
  compileDoc(combined);

  // Replacing the environment removes the old floor, light and props first.
  Document target;
  REQUIRE(makeTemplate("quadruped", target));
  MergeOptions re;
  re.models = false;
  re.replaceEnvironment = true;
  mergeDocument(target, env, re);
  int ramps = 0, planes = 0;
  for (const Geom& g : target.geoms) {
    ramps += g.name == "ramp";
    planes += g.body == kWorldId && g.type == GeomType::Plane;
  }
  CHECK(ramps == 1);
  CHECK(planes == 1);
  CHECK(target.lights.size() == 1);
  DocSummary s = summarize(target);
  CHECK(s.models.size() == 1);
  CHECK(s.hasFloor);
  compileDoc(target);
}

TEST_CASE("keyframes: export, simulate, edit, save") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  Id root = doc.bodies[1].id;
  Id rootJoint = kNoId;
  for (const Joint& j : doc.joints) if (j.body == root && j.type == JointType::Free) rootJoint = j.id;
  REQUIRE(rootJoint);
  // A crouched start pose: knees bent, body lower.
  Keyframe home;
  home.name = "home";
  home.qpos.push_back({rootJoint, {0.1, 0, 0.3, 1, 0, 0, 0}});
  for (const Joint& j : doc.joints)
    if (j.name.find("knee") != std::string::npos) home.qpos.push_back({j.id, {-0.5}});
  home.ctrl.push_back({doc.actuators[0].id, 0.25});
  doc.keyframes.push_back(home);

  CompileResult cr = compileMjcf(writeMjcf(doc));
  REQUIRE(cr.ok);
  const mjModel* m = cr.model.get();
  REQUIRE(m->nkey == 1);
  CHECK(m->key_qpos[0] == doctest::Approx(0.1));
  CHECK(m->key_qpos[2] == doctest::Approx(0.3));
  int knee = mj_name2id(m, mjOBJ_JOINT, "FL_knee");
  REQUIRE(knee >= 0);
  CHECK(m->key_qpos[m->jnt_qposadr[knee]] == doctest::Approx(-0.5));
  int hip = mj_name2id(m, mjOBJ_JOINT, "FL_hip_pitch");
  CHECK(m->key_qpos[m->jnt_qposadr[hip]] == doctest::Approx(0.0));  // no entry: reference
  CHECK(m->key_ctrl[0] == doctest::Approx(0.25));
  // Degree output doesn't touch keyframe values (MuJoCo never converts them).
  ExportOptions deg;
  deg.angle = AngleUnit::Degree;
  CompileResult crd = compileMjcf(writeMjcf(doc, deg));
  REQUIRE(crd.ok);
  CHECK(crd.model->key_qpos[m->jnt_qposadr[knee]] == doctest::Approx(-0.5));

  // Re-import keeps the keyframe.
  Document back;
  std::string err;
  REQUIRE(importMjcfString(writeMjcf(doc), back, &err));
  REQUIRE(back.keyframes.size() == 1);
  CompileResult crb = compileMjcf(writeMjcf(back));
  REQUIRE(crb.ok);
  for (int i = 0; i < m->nq; ++i) CHECK(crb.model->key_qpos[i] == doctest::Approx(m->key_qpos[i]));

  // Duplicating the creature copies its keyframe entries, shifted with the copy.
  std::vector<Id> tops = duplicate(doc, {root}, {1, 0, 0});
  REQUIRE(tops.size() == 1);
  Id copyJoint = kNoId;
  for (const Joint& j : doc.joints) if (j.body == tops[0] && j.type == JointType::Free) copyJoint = j.id;
  const std::vector<double>* q = doc.keyframes[0].jointQpos(copyJoint);
  REQUIRE(q);
  CHECK((*q)[0] == doctest::Approx(1.1));
  CHECK(doc.keyframes[0].qpos.size() == 2 * home.qpos.size());
  // Deleting it removes them again.
  deleteElements(doc, tops);
  CHECK(doc.keyframes[0].qpos.size() == home.qpos.size());

  // Project files keep keyframes.
  Document loaded;
  REQUIRE(projectFromJson(loaded, projectToJson(doc, ""), &err, nullptr));
  REQUIRE(loaded.keyframes.size() == 1);
  CHECK(loaded.keyframes[0].qpos.size() == home.qpos.size());
  CHECK(loaded.keyframes[0].ctrl.size() == 1);
}

TEST_CASE("keyframes: the analysis and the floor fix use the start pose") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  Id root = doc.bodies[1].id;
  Id rootJoint = kNoId;
  for (const Joint& j : doc.joints) if (j.body == root && j.type == JointType::Free) rootJoint = j.id;
  // The keyframe lifts the creature 20 cm: the start pose floats.
  Vec3 p = doc.body(root)->pos;
  Keyframe k;
  k.name = "home";
  k.qpos.push_back({rootJoint, {p.x, p.y, p.z + 0.2, 1, 0, 0, 0}});
  doc.keyframes.push_back(k);
  CompileResult cr = compileMjcf(writeMjcf(doc));
  REQUIRE(cr.ok);
  AnalysisReport r = analyzeModel(doc, &cr, true);
  const Issue* fl = nullptr;
  for (const Issue& i : r.issues) if (i.code == "floating") fl = &i;
  REQUIRE(fl);
  fl->fix(doc);
  CHECK(doc.body(root)->pos.z == p.z);  // the body stays, the keyframe moves
  CHECK((*doc.keyframes[0].jointQpos(rootJoint))[2] == doctest::Approx(p.z).epsilon(0.01));
  CompileResult cr2 = compileMjcf(writeMjcf(doc));
  AnalysisReport r2 = analyzeModel(doc, &cr2, true);
  for (const Issue& i : r2.issues) CHECK(i.code != "floating");
}

TEST_CASE("meshes: exact mass properties, import, export and project round trip") {
  // A 0.2 x 0.1 x 0.4 m box as a triangle mesh (12 triangles, outward winding).
  const char* xml = R"(
<mujoco model="meshes">
  <asset>
    <mesh name="brick"
      vertex="-0.1 -0.05 -0.2  0.1 -0.05 -0.2  0.1 0.05 -0.2  -0.1 0.05 -0.2
              -0.1 -0.05 0.2   0.1 -0.05 0.2   0.1 0.05 0.2   -0.1 0.05 0.2"
      face="0 2 1  0 3 2  4 5 6  4 6 7  0 1 5  0 5 4  1 2 6  1 6 5  2 3 7  2 7 6  3 0 4  3 4 7"/>
  </asset>
  <worldbody>
    <geom type="plane" size="0 0 1"/>
    <body name="block" pos="0 0 0.5">
      <freejoint/>
      <geom name="brick_geom" type="mesh" mesh="brick" pos="0.3 0 0" density="500"/>
    </body>
  </worldbody>
</mujoco>)";
  Document doc;
  std::string err;
  REQUIRE(importMjcfString(xml, doc, &err));
  REQUIRE(doc.meshes.size() == 1);
  const MeshAsset& m = doc.meshes[0];
  const MeshMass& mm = meshMassProps(m);
  CHECK(mm.volume == doctest::Approx(0.2 * 0.1 * 0.4));
  // Box inertia about its center for density 1: V/12 (b^2 + c^2) ... MuJoCo compiles mesh vertices
  // into the principal frame (axes may be reordered; the geom orientation compensates).
  double V = 0.008;
  std::vector<double> got{mm.inertia(0, 0), mm.inertia(1, 1), mm.inertia(2, 2)};
  std::vector<double> want{V / 12 * (0.1 * 0.1 + 0.4 * 0.4), V / 12 * (0.2 * 0.2 + 0.4 * 0.4), V / 12 * (0.2 * 0.2 + 0.1 * 0.1)};
  std::sort(got.begin(), got.end());
  std::sort(want.begin(), want.end());
  for (int i = 0; i < 3; ++i) CHECK(got[i] == doctest::Approx(want[i]));
  CHECK(std::fabs(mm.inertia(0, 1)) < 1e-12);
  // Picking hits the mesh surface.
  const Geom* g = nullptr;
  for (const Geom& x : doc.geoms) if (x.type == GeomType::Mesh) g = &x;
  REQUIRE(g);
  auto poses = computeBodyPoses(doc);
  Pose gp = poses[doc.bodyIndex(g->body)] * Pose{g->pos, g->quat};
  Vec3 n;
  double t = rayGeom(doc, {{0.3, 0, 2}, {0, 0, -1}}, *g, gp, 50, &n);
  CHECK(t == doctest::Approx(2 - 0.5 - 0.2).epsilon(1e-6));
  CHECK(n.z == doctest::Approx(1.0));
  Aabb box = geomWorldAabb(doc, *g, gp);
  CHECK(box.lo.z == doctest::Approx(0.3));
  // Same physics after export (inline mesh) and after a project round trip.
  ModelPtr a = compileXml(xml);
  ModelPtr b = compileDoc(doc);
  checkClose(a->body_mass, b->body_mass, a->nbody, 1e-6, "body_mass");
  checkClose(a->body_inertia, b->body_inertia, 3 * a->nbody, 1e-6, "body_inertia");
  Document loaded;
  REQUIRE(projectFromJson(loaded, projectToJson(doc, ""), &err, nullptr));
  CHECK(writeMjcf(loaded) == writeMjcf(doc));
  // Our own mass estimate agrees with MuJoCo when the body infers mass from the mesh.
  for (Body& body : doc.bodies) body.hasInertial = false;
  ModelPtr c = compileDoc(doc);
  MassProps mp = bodyMassProps(doc, computeBodyPoses(doc), doc.bodies[1].id);
  CHECK(mp.mass == doctest::Approx(c->body_mass[1]).epsilon(1e-6));
  CHECK(mp.mass == doctest::Approx(500 * V).epsilon(1e-6));
}

TEST_CASE("meshes: files for big meshes and textures, deduplication, merging") {
  namespace fs = std::filesystem;
  // A UV sphere written as a triangle soup (every triangle has its own three vertices, like STL).
  const int rings = 24, segs = 48;
  std::vector<Vec3> pts{{0, 0, 0.1}};
  for (int r = 1; r < rings; ++r)
    for (int s = 0; s < segs; ++s) {
      double th = kPi * r / rings, ph = 2 * kPi * s / segs;
      pts.push_back(Vec3{std::sin(th) * std::cos(ph), std::sin(th) * std::sin(ph), std::cos(th)} * 0.1);
    }
  pts.push_back({0, 0, -0.1});
  auto ring = [&](int r, int s) { return 1 + (r - 1) * segs + (s % segs); };
  std::vector<int> tris;
  for (int s = 0; s < segs; ++s) {
    tris.insert(tris.end(), {0, ring(1, s), ring(1, s + 1)});
    tris.insert(tris.end(), {int(pts.size()) - 1, ring(rings - 1, s + 1), ring(rings - 1, s)});
    for (int r = 1; r < rings - 1; ++r) {
      tris.insert(tris.end(), {ring(r, s), ring(r + 1, s), ring(r + 1, s + 1)});
      tris.insert(tris.end(), {ring(r, s), ring(r + 1, s + 1), ring(r, s + 1)});
    }
  }
  std::string vs, fs_;
  for (size_t i = 0; i < tris.size(); ++i) {
    const Vec3& p = pts[size_t(tris[i])];
    vs += formatNumber(p.x) + " " + formatNumber(p.y) + " " + formatNumber(p.z) + " ";
    fs_ += std::to_string(i) + " ";
  }
  fs::path dir = fs::temp_directory_path() / "mjforge_test_assets";
  fs::create_directories(dir);
  // A 2x2 PNG for a texture.
  const unsigned char png[] = {137, 80, 78, 71, 13, 10, 26, 10, 0, 0, 0, 13, 73, 72, 68, 82, 0, 0, 0, 2, 0, 0, 0, 2, 8, 2, 0, 0,
                               0, 253, 212, 154, 115, 0, 0, 0, 16, 73, 68, 65, 84, 120, 156, 99, 56, 97, 100, 4, 68, 12, 16, 10,
                               0, 35, 54, 4, 177, 70, 124, 22, 51, 0, 0, 0, 0, 73, 69, 78, 68, 174, 66, 96, 130};
  fs::path image = dir / "paint.png";
  REQUIRE(writeTextFile(image.string(), std::string(reinterpret_cast<const char*>(png), sizeof png)));
  std::string xml = R"(<mujoco model="ball">
  <asset>
    <texture name="paint" type="2d" file=")" + image.string() + R"("/>
    <material name="painted" texture="paint"/>
    <mesh name="ball" vertex=")" + vs + R"(" face=")" + fs_ + R"("/>
  </asset>
  <worldbody>
    <geom type="plane" size="0 0 1"/>
    <body name="ball" pos="0 0 0.3">
      <freejoint/>
      <geom type="mesh" mesh="ball" material="painted"/>
    </body>
  </worldbody>
</mujoco>)";
  Document doc;
  std::string err;
  REQUIRE(importMjcfString(xml, doc, &err));
  REQUIRE(doc.meshes.size() == 1);
  CHECK(doc.meshes[0].vertexCount() == pts.size());  // shared corners are stored once
  CHECK(doc.meshes[0].faceCount() == tris.size() / 3);
  ModelPtr inl = compileDoc(doc);

  // Written to disk: the mesh becomes an STL file and the image is copied, both next to the XML.
  fs::path out = dir / "export" / "ball.xml";
  fs::create_directories(out.parent_path());
  std::vector<std::string> written;
  REQUIRE(exportMjcfFile(doc, out.string(), {}, &err, "", &written));
  CHECK(written.size() == 3);
  CHECK(fs::is_regular_file(dir / "export" / "ball_assets" / "ball.stl"));
  CHECK(fs::is_regular_file(dir / "export" / "ball_assets" / "paint.png"));
  std::string text;
  REQUIRE(readTextFile(out.string(), text));
  CHECK(text.find("file=\"ball_assets/ball.stl\"") != std::string::npos);
  CHECK(text.find("vertex=") == std::string::npos);
  char lerr[1000] = "";
  mjModel* fromFile = mj_loadXML(out.string().c_str(), nullptr, lerr, sizeof lerr);
  INFO(lerr);
  REQUIRE(fromFile);
  CHECK(fromFile->mesh_vertnum[0] == int(pts.size()));
  checkClose(inl->body_mass, fromFile->body_mass, inl->nbody, 1e-6, "body_mass");
  checkClose(inl->body_inertia, fromFile->body_inertia, 3 * inl->nbody, 1e-6, "body_inertia");
  mj_deleteModel(fromFile);

  // A missing image keeps the model loadable (flat color) and is reported.
  fs::remove(image);
  CompileResult cr = compileMjcf(writeMjcf(doc));
  INFO(cr.error);
  REQUIRE(cr.ok);
  AnalysisReport r = analyzeModel(doc, &cr, true);
  bool reported = false;
  for (const Issue& i : r.issues) reported |= i.code == "texture-file";
  CHECK(reported);

  // Importing the model into another scene twice brings the mesh along, stored once.
  Document scene;
  REQUIRE(makeTemplate("empty", scene));
  MergeOptions mo;
  mo.environment = false;
  mergeDocument(scene, doc, mo);
  mo.modelOffset = {1, 0, 0};
  mergeDocument(scene, doc, mo);
  CHECK(scene.meshes.size() == 1);
  int meshGeoms = 0;
  for (const Geom& g : scene.geoms) meshGeoms += g.type == GeomType::Mesh && scene.mesh(g.mesh);
  CHECK(meshGeoms == 2);
  ModelPtr merged = compileDoc(scene);
  CHECK(merged->nmesh == 1);
  fs::remove_all(dir);
}

TEST_CASE("meshes: loading STL and OBJ files") {
  namespace fs = std::filesystem;
  fs::path dir = fs::temp_directory_path() / "mjforge_test_meshfiles";
  fs::create_directories(dir);
  // A 0.2 x 0.1 x 0.4 m box centered at (1.1, 0, 0) in file coordinates.
  MeshAsset box;
  box.vert = std::make_shared<const std::vector<float>>(std::vector<float>{
      1.0f, -0.05f, -0.2f, 1.2f, -0.05f, -0.2f, 1.2f, 0.05f, -0.2f, 1.0f, 0.05f, -0.2f,
      1.0f, -0.05f, 0.2f, 1.2f, -0.05f, 0.2f, 1.2f, 0.05f, 0.2f, 1.0f, 0.05f, 0.2f});
  box.face = std::make_shared<const std::vector<int>>(
      std::vector<int>{0, 2, 1, 0, 3, 2, 4, 5, 6, 4, 6, 7, 0, 1, 5, 0, 5, 4, 1, 2, 6, 1, 6, 5, 2, 3, 7, 2, 7, 6, 3, 0, 4, 3, 4, 7});
  std::string stl = meshToStl(box);
  CHECK(stl.size() == 84 + 12 * 50);
  REQUIRE(writeTextFile((dir / "brick.stl").string(), stl));
  std::string obj = "# box\n";
  for (size_t i = 0; i < 8; ++i)
    obj += "v " + formatNumber((*box.vert)[3 * i]) + " " + formatNumber((*box.vert)[3 * i + 1]) + " " + formatNumber((*box.vert)[3 * i + 2]) + "\n";
  for (size_t t = 0; t < 12; ++t)
    obj += "f " + std::to_string((*box.face)[3 * t] + 1) + " " + std::to_string((*box.face)[3 * t + 1] + 1) + " " +
           std::to_string((*box.face)[3 * t + 2] + 1) + "\n";
  REQUIRE(writeTextFile((dir / "brick.obj").string(), obj));

  for (const char* file : {"brick.stl", "brick.obj"}) {
    INFO(file);
    Document doc;
    REQUIRE(makeTemplate("empty", doc));
    Pose frame;
    std::string err;
    Id mesh = loadMeshFile(doc, (dir / file).string(), 2.0, &frame, &err);
    INFO(err);
    REQUIRE(mesh);
    const MeshAsset& m = *doc.mesh(mesh);
    CHECK(m.name == "brick");
    CHECK(m.vertexCount() == 8);  // the STL's 36 corners are merged
    CHECK(meshMassProps(m).volume == doctest::Approx(0.008 * 8).epsilon(1e-5));
    CHECK(frame.pos.x == doctest::Approx(2.2).epsilon(1e-5));  // the center, scaled
    CHECK(std::fabs(frame.pos.y) < 1e-6);
    // A geom placed at the frame shows the mesh where the file puts it.
    Id body = addBody(doc, kWorldId, {{0, 0, 1}, {}}, "holder", Role::Model);
    Id g = addGeom(doc, body, GeomType::Mesh, {}, frame);
    doc.geom(g)->mesh = mesh;
    Aabb bb = geomWorldAabb(doc, *doc.geom(g), Pose{{0, 0, 1}, {}} * frame);
    CHECK(bb.lo.x == doctest::Approx(2.0).epsilon(1e-5));
    CHECK(bb.hi.z == doctest::Approx(1.4).epsilon(1e-5));
    ModelPtr model = compileDoc(doc);
    CHECK(model->nmesh == 1);
    CHECK(model->body_mass[1] == doctest::Approx(1000 * 0.064).epsilon(1e-5));
  }
  Document doc;
  std::string err;
  CHECK_FALSE(loadMeshFile(doc, (dir / "missing.stl").string(), 1.0, nullptr, &err));
  CHECK(err.find("not found") != std::string::npos);
  fs::remove_all(dir);
}

TEST_CASE("tendons, equality constraints and contact pairs: export, import, edit, simulate") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  auto byName = [&](const auto& vec, const char* n) -> Id {
    for (const auto& e : vec) if (e.name == n) return e.id;
    FAIL("missing " << n);
    return kNoId;
  };
  Id flKnee = byName(doc.joints, "FL_knee"), frKnee = byName(doc.joints, "FR_knee");
  Id flHip = byName(doc.joints, "FL_hip_pitch"), frHip = byName(doc.joints, "FR_hip_pitch");
  Id torso = byName(doc.bodies, "quadruped"), flCalf = byName(doc.bodies, "FL_calf"), frCalf = byName(doc.bodies, "FR_calf");
  Id floor = byName(doc.geoms, "floor");
  Id flFoot = kNoId;
  for (const Geom& g : doc.geoms) if (g.body == flCalf && g.type == GeomType::Sphere) flFoot = g.id;
  REQUIRE(flFoot);

  // A fixed tendon driving both front knees with one motor, a spatial cable from the torso to a calf.
  Id fixed = addTendon(doc, TendonType::Fixed, "front_knees");
  doc.tendon(fixed)->joints = {{flKnee, 1.0}, {frKnee, 1.0}};
  doc.tendon(fixed)->limited = true;
  doc.tendon(fixed)->range = {-2, 1};
  Id motor = addActuator(doc, ActuatorType::Motor, kNoId, "knees_motor");
  doc.actuator(motor)->trn = TrnType::Tendon;
  doc.actuator(motor)->target = fixed;
  doc.actuator(motor)->gear[0] = 5;
  Id s1 = addSite(doc, torso, {{0.15, 0.1, 0.05}, {}}, "cable_top");
  Id s2 = addSite(doc, flCalf, {{0, 0, -0.1}, {}}, "cable_bottom");
  Id spatial = addTendon(doc, TendonType::Spatial, "cable");
  doc.tendon(spatial)->path = {{TendonPoint::Type::Site, s1}, {TendonPoint::Type::Site, s2}};
  doc.tendon(spatial)->stiffness = 50;
  doc.tendon(spatial)->width = 0.004;
  addSensor(doc, SensorType::TendonPos, spatial);
  // The two calves share a point; the hips are coupled; the left foot touches the floor through a pair.
  Id conn = addEquality(doc, EqualityType::Connect, flCalf, frCalf);
  Id coupling = addEquality(doc, EqualityType::Joint, frHip, flHip);
  Id weld = addEquality(doc, EqualityType::Weld, torso, kNoId);
  doc.equality(weld)->active = false;
  Id pair = addContactPair(doc, flFoot, floor);
  CHECK(doc.pair(pair)->friction[0] == doctest::Approx(std::max(doc.geom(flFoot)->friction.x, doc.geom(floor)->friction.x)));
  AnalysisReport ar = analyzeModel(doc, nullptr, false);
  for (const Issue& i : ar.issues) CHECK_MESSAGE(i.severity != Severity::Error, i.title);

  ExportMap map;
  std::string xml = writeMjcf(doc, {}, &map);
  CompileResult cr = compileMjcf(xml);
  INFO(cr.error);
  REQUIRE(cr.ok);
  const mjModel* m = cr.model.get();
  CHECK(m->ntendon == 2);
  CHECK(m->neq == 3);
  CHECK(m->npair == 1);
  CHECK(map.tendons.size() == 2);
  CHECK(m->wrap_type[m->tendon_adr[0]] == mjWRAP_JOINT);
  CHECK(m->tendon_num[0] == 2);
  CHECK(m->tendon_limited[0] == 1);
  CHECK(m->tendon_stiffness[1] == 50);
  int u = -1;
  for (int i = 0; i < m->nu; ++i) if (map.actuators[i] == motor) u = i;
  REQUIRE(u >= 0);
  CHECK(m->actuator_trntype[u] == mjTRN_TENDON);
  CHECK(m->eq_type[0] == mjEQ_CONNECT);
  CHECK(m->eq_type[1] == mjEQ_JOINT);
  CHECK(m->eq_type[2] == mjEQ_WELD);
  CHECK(m->eq_active0[2] == 0);
  CHECK(m->eq_obj2id[2] == 0);  // welded to the world
  // The connect anchor sits at the right calf's origin, as seen from the left calf.
  Pose fl = bodyWorldPose(doc, flCalf), fr = bodyWorldPose(doc, frCalf);
  Vec3 anchor = inverse(fl).apply(fr.pos);
  for (int k = 0; k < 3; ++k) CHECK(m->eq_data[k] == doctest::Approx(anchor[k]));

  // MJCF round trip through the importer.
  Document back;
  std::string err;
  ImportReport rep;
  REQUIRE(importMjcfString(xml, back, &err, &rep));
  CHECK(rep.notes.empty());
  CHECK(back.tendons.size() == 2);
  CHECK(back.equalities.size() == 3);
  CHECK(back.pairs.size() == 1);
  CompileResult cr2 = compileMjcf(writeMjcf(back));
  REQUIRE(cr2.ok);
  const mjModel* m2 = cr2.model.get();
  REQUIRE(m2->nwrap == m->nwrap);
  for (int i = 0; i < m->nwrap; ++i) {
    CHECK(m2->wrap_type[i] == m->wrap_type[i]);
    CHECK(m2->wrap_objid[i] == m->wrap_objid[i]);
    CHECK(m2->wrap_prm[i] == doctest::Approx(m->wrap_prm[i]));
  }
  checkClose(m->eq_data, m2->eq_data, mjNEQDATA * m->neq, 1e-8, "eq_data");
  checkClose(m->pair_friction, m2->pair_friction, 5 * m->npair, 1e-12, "pair_friction");
  checkClose(m->pair_solref, m2->pair_solref, mjNREF * m->npair, 1e-12, "pair_solref");
  checkClose(m->tendon_lengthspring, m2->tendon_lengthspring, 2 * m->ntendon, 1e-8, "tendon_lengthspring");
  CHECK(m2->actuator_trntype[u] == mjTRN_TENDON);

  // Project round trip.
  Document loaded;
  REQUIRE(projectFromJson(loaded, projectToJson(doc, ""), &err, nullptr));
  CHECK(writeMjcf(loaded) == writeMjcf(doc));

  // Duplicating the creature copies its tendons and constraints; the floor pair follows the copy's foot.
  std::vector<Id> copies = duplicate(doc, {torso}, {1, 0, 0});
  REQUIRE(copies.size() == 1);
  CHECK(doc.tendons.size() == 4);
  CHECK(doc.equalities.size() == 6);
  CHECK(doc.pairs.size() == 2);
  CompileResult cr3 = compileMjcf(writeMjcf(doc));
  INFO(cr3.error);
  REQUIRE(cr3.ok);
  CHECK(cr3.model->ntendon == 4);

  // Merging into an empty scene carries everything along.
  Document scene;
  REQUIRE(makeTemplate("empty", scene));
  MergeOptions mo;
  mo.environment = false;
  mergeDocument(scene, loaded, mo);
  CHECK(scene.tendons.size() == 2);
  CHECK(scene.equalities.size() == 3);
  // The scene's own floor stands in for the source's, so the foot's floor pair carries over.
  REQUIRE(scene.pairs.size() == 1);
  const Geom* partner = scene.geom(scene.pairs[0].geom2);
  if (partner && partner->body != kWorldId) partner = scene.geom(scene.pairs[0].geom1);
  REQUIRE(partner);
  CHECK(partner->body == kWorldId);
  CHECK(partner->type == GeomType::Plane);
  CompileResult cr4 = compileMjcf(writeMjcf(scene));
  INFO(cr4.error);
  REQUIRE(cr4.ok);

  // Deleting joints and sites trims the tendons; with nothing left they go, with their actuator.
  deleteElements(loaded, {loaded.joint(flKnee)->id});
  REQUIRE(loaded.tendon(fixed));
  CHECK(loaded.tendon(fixed)->joints.size() == 1);
  deleteElements(loaded, {frKnee});
  CHECK(loaded.tendon(fixed) == nullptr);
  CHECK(loaded.actuator(motor) == nullptr);
  deleteElements(loaded, {s2});
  CHECK(loaded.tendon(spatial) == nullptr);
  CHECK(loaded.equality(coupling) != nullptr);
  deleteElements(loaded, {flHip});
  CHECK(loaded.equality(coupling) == nullptr);
  CHECK(compileMjcf(writeMjcf(loaded)).ok);

  // The simulation holds the creature with the tendon motor among the actuators.
  Simulator sim;
  cr.map = map;
  REQUIRE(sim.start(cr.model, cr.map));
  for (int i = 0; i < 500 && !sim.diverged(); ++i) sim.stepOnce();
  CHECK_FALSE(sim.diverged());
  CHECK(sim.drive(u).kind == ActuatorDrive::Motor);
}

TEST_CASE("actuators: latency, reflected damping and armature; multi-control actuators on import") {
  const char* xml = R"(
<mujoco>
  <option timestep="0.002"/>
  <worldbody>
    <body name="arm" pos="0 0 1">
      <joint name="j" axis="0 1 0" damping="0.1"/>
      <geom type="capsule" fromto="0 0 0 0.4 0 0" size="0.03"/>
      <body name="arm2" pos="0.4 0 0">
        <joint name="j2" axis="0 1 0"/>
        <geom type="capsule" fromto="0 0 0 0.3 0 0" size="0.02"/>
      </body>
    </body>
  </worldbody>
  <actuator>
    <position name="servo" joint="j" kp="20" delay="0.01" nsample="5" interp="linear" damping="0.3" armature="0.002" gear="2"/>
    <dcmotor name="dc" joint="j2" motorconst="0.05" resistance="2" input="pos vel"/>
    <motor name="m2" joint="j2" ctrlrange="-1 1"/>
  </actuator>
  <keyframe><key name="home" ctrl="0.2 0 0 0.5"/></keyframe>
</mujoco>)";
  Document doc;
  std::string err;
  ImportReport rep;
  REQUIRE(importMjcfString(xml, doc, &err, &rep));
  REQUIRE(doc.actuators.size() == 2);  // the two-control DC motor is skipped and reported
  CHECK(rep.notes.size() == 1);
  const Actuator& servo = doc.actuators[0];
  CHECK(servo.delay == doctest::Approx(0.01));
  CHECK(servo.nsample == 5);
  CHECK(servo.interp == 1);
  CHECK(servo.damping == doctest::Approx(0.3));
  CHECK(servo.armature == doctest::Approx(0.002));
  // Keyframe controls follow the control addresses, not the actuator order.
  REQUIRE(doc.keyframes.size() == 1);
  double c0 = 0, c1 = 0;
  for (const auto& [id, c] : doc.keyframes[0].ctrl) {
    if (id == doc.actuators[0].id) c0 = c;
    if (id == doc.actuators[1].id) c1 = c;
  }
  CHECK(c0 == doctest::Approx(0.2));
  CHECK(c1 == doctest::Approx(0.5));

  // Export keeps everything.
  ExportMap map;
  CompileResult cr = compileMjcf(writeMjcf(doc, {}, &map));
  INFO(cr.error);
  REQUIRE(cr.ok);
  const mjModel* m = cr.model.get();
  CHECK(m->actuator_delay[0] == doctest::Approx(0.01));
  CHECK(m->actuator_history[0] == 5);
  CHECK(m->actuator_history[1] == 1);
  CHECK(m->actuator_damping[0] == doctest::Approx(0.3));
  CHECK(m->actuator_armature[0] == doctest::Approx(0.002));
  Document back;
  REQUIRE(projectFromJson(back, projectToJson(doc, ""), &err, nullptr));
  CHECK(writeMjcf(back) == writeMjcf(doc));

  // MuJoCo Warp ignores actuator armature: the fix moves armature * gear^2 to the joint, which
  // MuJoCo C simulates identically.
  AnalysisReport ar = analyzeModel(doc, &cr, true);
  const Issue* arm = nullptr;
  for (const Issue& i : ar.issues) if (i.code == "mjw-actuator-armature") arm = &i;
  REQUIRE(arm);
  Document fixed = doc;
  arm->fix(fixed);
  CHECK(fixed.actuators[0].armature == 0);
  for (const Joint& j : fixed.joints)
    if (j.name == "j") CHECK(j.armature == doctest::Approx(0.002 * 4));
  CompileResult cf = compileMjcf(writeMjcf(fixed));
  REQUIRE(cf.ok);
  auto roll = [](const mjModel* model) {
    DataPtr d = makeData(model);
    mj_resetDataKeyframe(model, d.get(), 0);
    for (int i = 0; i < 300; ++i) mj_step(model, d.get());
    return std::vector<double>(d->qpos, d->qpos + model->nq);
  };
  std::vector<double> qa = roll(cr.model.get()), qb = roll(cf.model.get());
  for (size_t i = 0; i < qa.size(); ++i) CHECK(qa[i] == doctest::Approx(qb[i]).epsilon(1e-6));

  // A delay without a history buffer is reported, with a fix.
  doc.actuators[0].nsample = 0;
  AnalysisReport ar2 = analyzeModel(doc, nullptr, false);
  const Issue* hist = nullptr;
  for (const Issue& i : ar2.issues) if (i.code == "delay-history") hist = &i;
  REQUIRE(hist);
  hist->fix(doc);
  CHECK(doc.actuators[0].nsample == 5);
}
