#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <type_traits>

#include <imgui.h>
#include <imgui_stdlib.h>

#include "core/Generators.h"
#include "core/Geometry.h"
#include "core/Kinematics.h"
#include "core/Ops.h"
#include "ui/Editor.h"
#include "ui/Icons.h"
#include "ui/MjcfDocs.h"
#include "ui/Theme.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {

template <class T> T* elemPtr(Document& d, Id id) {
  if constexpr (std::is_same_v<T, Body>) return d.body(id);
  else if constexpr (std::is_same_v<T, Geom>) return d.geom(id);
  else if constexpr (std::is_same_v<T, Joint>) return d.joint(id);
  else if constexpr (std::is_same_v<T, Site>) return d.site(id);
  else if constexpr (std::is_same_v<T, Camera>) return d.camera(id);
  else if constexpr (std::is_same_v<T, Light>) return d.light(id);
  else if constexpr (std::is_same_v<T, Actuator>) return d.actuator(id);
  else if constexpr (std::is_same_v<T, Sensor>) return d.sensor(id);
  else if constexpr (std::is_same_v<T, Material>) return d.material(id);
  else if constexpr (std::is_same_v<T, Texture>) return d.texture(id);
  else if constexpr (std::is_same_v<T, Exclude>) return d.exclude(id);
  else if constexpr (std::is_same_v<T, ContactPair>) return d.pair(id);
  else if constexpr (std::is_same_v<T, Equality>) return d.equality(id);
  else return d.tendon(id);
}

const char* D(const char* key) { return mjcfDoc(key); }

props::Opt O(const char* tip, const char* unit = nullptr, double speed = 0.01, double mn = -1e300, double mx = 1e300,
             const char* fmt = "%.4g") {
  props::Opt o;
  o.tip = tip;
  o.unit = unit;
  o.speed = speed;
  o.min = mn;
  o.max = mx;
  o.fmt = fmt;
  return o;
}

std::vector<std::pair<Id, std::string>> listOf(const std::vector<Joint>& v) {
  std::vector<std::pair<Id, std::string>> out;
  for (const auto& e : v) out.push_back({e.id, e.name});
  return out;
}
template <class T> std::vector<std::pair<Id, std::string>> listOfAny(const std::vector<T>& v) {
  std::vector<std::pair<Id, std::string>> out;
  for (const auto& e : v) out.push_back({e.id, e.name});
  return out;
}

const char* actuatorExplanation(ActuatorType t) {
  switch (t) {
    case ActuatorType::Motor:
      return "Direct torque/force control. Force = gear * ctrl. With ctrlrange [-1, 1] the gear is the maximum "
             "torque (N*m) or force (N) the policy can command. The most common choice for RL torque control.";
    case ActuatorType::Position:
      return "PD servo. ctrl is the TARGET joint position (radians for hinges, meters for slides). "
             "Force = kp*(ctrl - q) - kv*qdot, clamped to forcerange. Match ctrlrange to the joint range "
             "(or use inheritrange). Higher kp tracks better but needs a smaller timestep.";
    case ActuatorType::Velocity:
      return "Velocity servo. ctrl is the TARGET joint velocity (rad/s or m/s). Force = kv*(ctrl - qdot). "
             "It cannot hold a position against gravity by itself.";
    case ActuatorType::IntVelocity:
      return "Integrated-velocity servo: ctrl is a velocity that is integrated into an internal position target "
             "(the actuator activation) which is tracked with gain kp. actrange bounds the target.";
    case ActuatorType::Damper:
      return "Controllable damper: force = -kv * ctrl * velocity, with ctrl >= 0. Produces no force at rest.";
    case ActuatorType::Cylinder:
      return "Pneumatic / hydraulic cylinder. ctrl is the pressure command, filtered with timeconst; "
             "force = area * activation + bias.";
    case ActuatorType::Muscle:
      return "Hill-type muscle; ctrl in [0, 1] is the neural excitation. Usually attached to a tendon.";
    case ActuatorType::Adhesion:
      return "Adhesion (suction / gecko): pulls the target body's geoms towards surfaces they touch. ctrl in "
             "[0, 1] scales the force; gain is the maximum force.";
    case ActuatorType::General:
      return "General actuator: force = gain(ctrl) + bias(length, velocity), with optional activation dynamics. "
             "All the shortcut types are special cases of this one.";
  }
  return "";
}

}  // namespace

template <class T, class F> void Editor::applyToSelected(F&& f) {
  Id primary = sel_.primary();
  for (Id id : sel_.ids) {
    if (id == primary) continue;
    if (T* e = elemPtr<T>(doc_, id)) f(*e);
  }
}

// Header with icon, kind and editable name.
static bool nameHeader(Editor& ed, Id id, Icon icon, ImU32 col, const char* kind, std::string& name) {
  iconInline(icon, col);
  ImGui::SameLine();
  ImGui::AlignTextToFramePadding();
  ImGui::TextDisabled("%s", kind);
  ImGui::SameLine();
  ImGui::SetNextItemWidth(-1);
  std::string tmp = name;
  bool changed = false;
  if (ImGui::InputText("##name", &tmp, ImGuiInputTextFlags_EnterReturnsTrue) && tmp != name) {
    Kind k = ed.doc().kindOf(id);
    if (!tmp.empty() && ed.doc().nameTaken(k, tmp, id)) {
      ed.notify("Another " + std::string(kindName(k)) + " is already named '" + tmp + "'", pal::kWarning);
    } else {
      ed.checkpoint("Rename");
      name = tmp;
      changed = true;
    }
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Name (unique per element type). Press Enter to apply.");
  int n = int(ed.selection().ids.size());
  if (n > 1) ImGui::TextColored(pal::v4(pal::kInfo), "%d elements selected: edits apply to all of the same type.", n);
  ImGui::Separator();
  return changed;
}

// ------------------------------------------------------------------------------------ body

void Editor::inspectBody(Body& b) {
  nameHeader(*this, b.id, Icon::Body, pal::kBody, "Body", b.name);
  auto poses = computeBodyPoses(doc_);
  if (!props::begin("##body")) return;
  if (const GeneratorInfo* gi = findGenerator(b.generator)) {
    std::string title = std::string("Generator: ") + gi->label;
    if (props::section(title.c_str())) {
      bool regen = false;
      b.genParams.resize(gi->params.size());
      for (size_t i = 0; i < gi->params.size(); ++i) {
        const GenParam& p = gi->params[i];
        if (p.integer) {
          int v = int(std::lround(b.genParams[i]));
          if (props::integer(p.label, &v, int(p.def), O(p.tip, p.unit, p.speed, p.min, p.max))) {
            b.genParams[i] = v;
            regen = true;
          }
        } else if (props::real(p.label, &b.genParams[i], p.def, O(p.tip, p.unit, p.speed, p.min, p.max, "%.4g"))) {
          regen = true;
        }
      }
      if (regen) regenerate(doc_, b.id);
      props::end();
      for (size_t i = 0; i < gi->params.size(); ++i)
        if (std::string(gi->params[i].key) == "seed") {
          if (ImGui::Button("New random seed")) {
            checkpoint("New seed");
            b.genParams[i] = double((int(b.genParams[i]) * 7919 + 17) % 100000);
            regenerate(doc_, b.id);
          }
          ImGui::SameLine();
        }
      if (ImGui::Button("Bake")) {
        checkpoint("Bake generator");
        b.generator.clear();
        b.genParams.clear();
        notify("The generated geometry is now ordinary geoms you can edit one by one", pal::kTextDim, 3);
      }
      if (ImGui::IsItemHovered())
        ImGui::SetTooltip("Detach the generator: keep the geoms as they are and edit them individually.");
      if (!props::begin("##body2")) return;
    }
  }
  if (props::section("Transform (relative to parent)")) {
    if (props::vec3("Position", &b.pos, {}, O(D("body.pos"), "m", 0.005))) applyToSelected<Body>([&](Body& o) { o.pos = b.pos; });
    if (props::orientation("Orientation", &b.quat, "Euler angles XYZ (intrinsic, MuJoCo's default sequence), shown in degrees. Stored and exported as a quaternion."))
      applyToSelected<Body>([&](Body& o) { o.quat = b.quat; });
  }
  if (b.parent == kWorldId) {
    if (props::section("Role")) {
      const EnumEntry<Role> roles[] = {{Role::Model, "model", "Model (creature / robot)"},
                                       {Role::Environment, "environment", "Environment (prop / obstacle)"}};
      props::combo("Belongs to", &b.role, roles, Role::Model,
                   "Decides which export file this subtree goes to when exporting models and the environment "
                   "separately. Models get actuators and sensors; environment props are part of the world.");
    }
  }
  if (props::section("Mass & inertia")) {
    if (!movesInSimulation(b.id))
      props::info("Static", "%s", "welded to the world: its mass has no effect on the simulation");
    MassProps mp = bodyMassProps(doc_, poses, b.id);
    MassProps st = subtreeMassProps(doc_, poses, b.id);
    props::info("Body mass", "%.4g kg", mp.mass);
    props::info("Subtree mass", "%.4g kg  (this body and everything attached below it)", st.mass);
    if (compileValid()) {
      int ci = -1;
      for (size_t i = 0; i < compiled_.map.bodies.size(); ++i) if (compiled_.map.bodies[i] == b.id) ci = int(i);
      if (ci >= 0) {
        const mjModel* m = compiled_.model.get();
        props::info("MuJoCo inertia", "%.3g  %.3g  %.3g kg*m^2", m->body_inertia[3 * ci], m->body_inertia[3 * ci + 1],
                    m->body_inertia[3 * ci + 2]);
      }
    }
    if (props::checkbox("Explicit inertial", &b.hasInertial, false,
                        "Override the mass and inertia that MuJoCo would compute from the geoms (density/mass "
                        "of each geom). Usually you want the automatic values.")) {
      if (b.hasInertial) {
        // Initialize from the geometry so switching on is lossless.
        Pose bp = poses[doc_.bodyIndex(b.id)];
        if (mp.mass > 0) {
          b.mass = mp.mass;
          b.ipos = rotate(conj(bp.quat), mp.com - bp.pos);
          Vec3 ev;
          Mat3 evec;
          Mat3 Ilocal = toMat3(conj(bp.quat)) * mp.inertia * toMat3(bp.quat);
          eigSym3(Ilocal, &ev, &evec);
          b.diaginertia = ev;
          b.iquat = fromMat3(evec);
        }
      }
    }
    if (b.hasInertial) {
      if (props::real("Mass", &b.mass, 1.0, O(D("inertial.mass"), "kg", 0.01, 0, 1e9))) applyToSelected<Body>([&](Body& o) { o.mass = b.mass; });
      props::vec3("COM position", &b.ipos, {}, O(D("inertial.pos"), "m", 0.002));
      props::orientation("Inertia frame", &b.iquat, D("inertial.quat"));
      props::vec3("Diag. inertia", &b.diaginertia, {0.001, 0.001, 0.001}, O(D("inertial.diaginertia"), "kg*m^2", 1e-4, 0, 1e9, "%.4g"));
    }
  }
  if (props::section("Advanced", false)) {
    if (props::real("Gravity comp.", &b.gravcomp, 0, O(D("body.gravcomp"), nullptr, 0.01, 0, 1)))
      applyToSelected<Body>([&](Body& o) { o.gravcomp = b.gravcomp; });
    if (b.parent == kWorldId) props::checkbox("Mocap", &b.mocap, false, D("body.mocap"));
  }
  props::end();

  // Contents and quick actions.
  ImGui::Spacing();
  int bi = doc_.bodyIndex(b.id);
  const DocIndex& ix = doc_.index();
  ImGui::TextDisabled("%zu geoms, %zu joints (%d DoF), %zu sites, %zu child bodies", ix.bodyGeoms[bi].size(),
                      ix.bodyJoints[bi].size(), doc_.bodyDofs(b.id), ix.bodySites[bi].size(), ix.childBodies[bi].size());
  if (ix.bodyJoints[bi].empty() && b.parent != kWorldId)
    ImGui::TextColored(pal::v4(pal::kInfo), "No joints: this body is rigidly welded to its parent.");
  if (ix.bodyJoints[bi].empty() && b.parent == kWorldId && b.role == Role::Model)
    ImGui::TextColored(pal::v4(pal::kInfo), "No joints: fixed to the world. Add a free joint to let it move.");
  if (ImGui::Button("+ Child body")) addBodyTo(b.id);
  ImGui::SameLine();
  if (ImGui::Button("+ Geom")) ImGui::OpenPopup("addgeom");
  ImGui::SameLine();
  if (ImGui::Button("+ Joint")) ImGui::OpenPopup("addjoint");
  ImGui::SameLine();
  if (ImGui::Button("+ Site")) addSiteTo(b.id);
  if (ImGui::BeginPopup("addgeom")) {
    for (const auto& e : kGeomTypes) {
      if (e.value == GeomType::Hfield || e.value == GeomType::Mesh) continue;
      if (ImGui::MenuItem(e.label)) addGeomTo(b.id, e.value);
    }
    ImGui::Separator();
    if (ImGui::MenuItem("Mesh from file...")) openMeshDialog(b.id);
    ImGui::EndPopup();
  }
  if (ImGui::BeginPopup("addjoint")) {
    for (const auto& e : kJointTypes) if (ImGui::MenuItem(e.label)) addJointTo(b.id, e.value);
    ImGui::EndPopup();
  }
}

// ------------------------------------------------------------------------------------ geom

void Editor::inspectMesh(Geom& g) {
  std::vector<std::pair<Id, std::string>> items;
  for (const MeshAsset& m : doc_.meshes) items.emplace_back(m.id, m.name);
  if (props::element("Mesh", &g.mesh, items, false,
                     "The triangle mesh this geom shows. MuJoCo collides with its convex hull and computes mass "
                     "and inertia from the enclosed volume. Add meshes with Add > Geom > Mesh from file.")) {
    Id id = g.mesh;
    applyToSelected<Geom>([&](Geom& o) { if (o.type == GeomType::Mesh) o.mesh = id; });
  }
  MeshAsset* ma = doc_.mesh(g.mesh);
  if (!ma || !ma->vert) {
    props::info("", "No mesh: pick one, or the model will not compile.");
    return;
  }
  Vec3 ext = meshBounds(*ma).extent();
  props::info("Size", "%.3g x %.3g x %.3g m", ext.x, ext.y, ext.z);
  props::info("Triangles", "%zu  (%zu vertices)", ma->faceCount(), ma->vertexCount());
  if (!ma->source.empty()) {
    props::info("File", "%s", fs::path(ma->source).filename().string().c_str());
    if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", ma->source.c_str());
  }
  int users = 0;
  for (const Geom& o : doc_.geoms) users += o.type == GeomType::Mesh && o.mesh == ma->id;
  bool reset = false;
  props::beginRow("Rescale", false,
                  "Scales the mesh vertices around the mesh's center (e.g. 0.001 for a file in millimeters). "
                  "Every geom using this mesh changes.",
                  &reset);
  float w = ImGui::GetContentRegionAvail().x;
  ImGui::SetNextItemWidth(std::max(40.0f, w - 70 * scale_));
  ImGui::DragFloat("##factor", &meshScale_, 0.01f, 1e-4f, 1e4f, "x %.4g", ImGuiSliderFlags_Logarithmic);
  ImGui::SameLine();
  if (ImGui::Button("Apply", ImVec2(-1, 0)) && meshScale_ > 0 && meshScale_ != 1) {
    checkpoint("Scale mesh");
    auto v = std::make_shared<std::vector<float>>(*ma->vert);
    for (float& x : *v) x *= meshScale_;
    ma->vert = std::move(v);
    doc_.touch();
  }
  props::endRow();
  if (users > 1) props::info("", "Shared by %d geoms.", users);
}

void Editor::inspectGeom(Geom& g) {
  nameHeader(*this, g.id, Icon::Box, pal::kGeom, "Geom", g.name);
  if (!props::begin("##geom")) return;
  if (props::section("Shape")) {
    GeomType before = g.type;
    if (props::combo("Type", &g.type, kGeomTypes, GeomType::Sphere, D("geom.type"))) {
      // Keep a similar overall size when switching shapes.
      double r = before == GeomType::Box ? std::max({g.size.x, g.size.y, g.size.z}) : g.size.x;
      double h = (before == GeomType::Capsule || before == GeomType::Cylinder) ? g.size.y : r;
      switch (g.type) {
        case GeomType::Sphere: g.size = {r, 0, 0}; break;
        case GeomType::Capsule:
        case GeomType::Cylinder: g.size = {std::min(r, h), h, 0}; break;
        case GeomType::Box:
        case GeomType::Ellipsoid: g.size = {r, r, (before == GeomType::Capsule || before == GeomType::Cylinder) ? h : r}; break;
        case GeomType::Plane: g.size = {0, 0, 0.05}; break;
        case GeomType::Mesh:
          if (!doc_.mesh(g.mesh) && !doc_.meshes.empty()) g.mesh = doc_.meshes.front().id;
          break;
        default: break;
      }
      GeomType t = g.type;
      Vec3 s = g.size;
      applyToSelected<Geom>([&](Geom& o) { o.type = t; o.size = s; });
      doc_.touch();
    }
    SizeLabels sl = geomSizeLabels(g.type);
    for (int i = 0; i < sl.count; ++i) {
      double mn = g.type == GeomType::Plane ? 0 : 1e-4;
      const char* tip = g.type == GeomType::Plane
                            ? "Plane half-sizes (0 = infinite) and the spacing of the rendering grid lines."
                            : D("geom.size");
      if (props::real(sl.l[i], &g.size[i], g.type == GeomType::Plane && i < 2 ? 0.0 : -1.0, O(tip, "m", 0.002, mn, 1e6))) {
        double v = g.size[i];
        applyToSelected<Geom>([&](Geom& o) { if (o.type == g.type) o.size[i] = v; });
      }
    }
    if (g.type == GeomType::Mesh) inspectMesh(g);
    if (g.type == GeomType::Capsule || g.type == GeomType::Cylinder) {
      Vec3 a, b;
      geomEndpoints(g, &a, &b);
      bool ca = props::vec3("End A", &a, a, O("Axis endpoint (body frame). Editing the endpoints sets position, orientation and half-length, like MJCF's fromto.", "m", 0.002));
      bool cb = props::vec3("End B", &b, b, O("Axis endpoint (body frame).", "m", 0.002));
      if (ca || cb) setGeomFromTo(g, a, b);
    }
  }
  if (props::section("Transform (relative to body)")) {
    if (props::vec3("Position", &g.pos, {}, O(D("geom.pos"), "m", 0.002))) applyToSelected<Geom>([&](Geom& o) { o.pos = g.pos; });
    if (props::orientation("Orientation", &g.quat, "Euler angles XYZ in degrees (exported as a quaternion)."))
      applyToSelected<Geom>([&](Geom& o) { o.quat = g.quat; });
  }
  if (g.body != kWorldId && props::section("Mass")) {
    int mode = g.mass >= 0 ? 1 : 0;
    const MeshAsset* ma = g.type == GeomType::Mesh ? doc_.mesh(g.mesh) : nullptr;
    double volume = ma ? meshMassProps(*ma).volume : primitiveVolume(g.type, g.size);
    if (props::comboIndex("Specify by", &mode, {"Density", "Mass"}, 0,
                          "MuJoCo computes each geom's mass from its volume and density, unless an explicit mass is given.")) {
      if (mode == 1) g.mass = geomMass(doc_, g);
      else g.mass = -1;
      double m = g.mass;
      applyToSelected<Geom>([&](Geom& o) { o.mass = m >= 0 ? geomMass(doc_, o) : -1; });
    }
    if (g.mass >= 0) {
      if (props::real("Mass", &g.mass, -1, O(D("geom.mass"), "kg", 0.01, 0, 1e9))) applyToSelected<Geom>([&](Geom& o) { o.mass = g.mass; });
    } else {
      if (props::real("Density", &g.density, 1000, O(D("geom.density"), "kg/m^3", 5, 0, 1e7)))
        applyToSelected<Geom>([&](Geom& o) { o.density = g.density; });
    }
    props::info("Resulting mass", "%.4g kg   (volume %.4g L)", geomMass(doc_, g), volume * 1000);
    if (ma) props::info("", "Mesh volume assumes a closed surface (MuJoCo's default 'legacy' mesh inertia agrees for closed meshes).");
    else if (!geomHasMass(g)) props::info("", "This geom does not contribute mass (group outside 0..5 or plane).");
  }
  if (props::section("Contact & friction")) {
    if (props::real("Sliding friction", &g.friction.x, 1.0, O(D("geom.friction"), nullptr, 0.01, 0, 100)))
      applyToSelected<Geom>([&](Geom& o) { o.friction.x = g.friction.x; });
    if (props::real("Torsional friction", &g.friction.y, 0.005, O("Friction opposing rotation around the contact normal (needs condim >= 4).", nullptr, 0.0005, 0, 10)))
      applyToSelected<Geom>([&](Geom& o) { o.friction.y = g.friction.y; });
    if (props::real("Rolling friction", &g.friction.z, 0.0001, O("Friction opposing rolling (needs condim 6).", nullptr, 0.00005, 0, 10)))
      applyToSelected<Geom>([&](Geom& o) { o.friction.z = g.friction.z; });
    int ci = g.condim == 1 ? 0 : g.condim == 3 ? 1 : g.condim == 4 ? 2 : 3;
    if (props::comboIndex("Contact dims", &ci, {"1  frictionless", "3  sliding friction", "4  + torsional", "6  + rolling"}, 1, D("geom.condim"))) {
      const int vals[] = {1, 3, 4, 6};
      g.condim = vals[ci];
      applyToSelected<Geom>([&](Geom& o) { o.condim = g.condim; });
    }
    if (props::bits("Contype", &g.contype, 1, D("geom.contype"))) applyToSelected<Geom>([&](Geom& o) { o.contype = g.contype; });
    if (props::bits("Conaffinity", &g.conaffinity, 1, D("geom.conaffinity")))
      applyToSelected<Geom>([&](Geom& o) { o.conaffinity = g.conaffinity; });
    if (g.contype == 0 && g.conaffinity == 0) props::info("", "Collisions disabled for this geom (visual only).");
    if (props::integer("Priority", &g.priority, 0, O(D("geom.priority"), nullptr, 0.1, -100, 100)))
      applyToSelected<Geom>([&](Geom& o) { o.priority = g.priority; });
  }
  if (props::section("Solver (advanced)", false)) {
    if (props::vec2("solref", &g.solref, kDefaultSolref, O(D("geom.solref"), nullptr, 0.001)))
      applyToSelected<Geom>([&](Geom& o) { o.solref = g.solref; });
    if (props::vec5("solimp", &g.solimp, kDefaultSolimp, O(D("geom.solimp"), nullptr, 0.001)))
      applyToSelected<Geom>([&](Geom& o) { o.solimp = g.solimp; });
    if (props::real("solmix", &g.solmix, 1.0, O(D("geom.solmix"), nullptr, 0.01, 0, 1e6)))
      applyToSelected<Geom>([&](Geom& o) { o.solmix = g.solmix; });
    if (props::real("Margin", &g.margin, 0, O(D("geom.margin"), "m", 0.0005, 0, 10))) applyToSelected<Geom>([&](Geom& o) { o.margin = g.margin; });
    if (props::real("Gap", &g.gap, 0, O(D("geom.gap"), "m", 0.0005, 0, 10))) applyToSelected<Geom>([&](Geom& o) { o.gap = g.gap; });
  }
  if (props::section("Appearance")) {
    if (props::color("Color", &g.rgba, {0.5f, 0.5f, 0.5f, 1}, D("geom.rgba"))) applyToSelected<Geom>([&](Geom& o) { o.rgba = g.rgba; });
    auto mats = listOfAny(doc_.materials);
    if (props::element("Material", &g.material, mats, true, D("geom.material"))) applyToSelected<Geom>([&](Geom& o) { o.material = g.material; });
    if (props::integer("Group", &g.group, 0, O(D("geom.group"), nullptr, 0.1, 0, 30))) applyToSelected<Geom>([&](Geom& o) { o.group = g.group; });
  }
  props::end();
}

// ------------------------------------------------------------------------------------ joint

void Editor::inspectJoint(Joint& j) {
  nameHeader(*this, j.id, Icon::Hinge, pal::kJoint, "Joint", j.name);
  bool angular = j.type == JointType::Hinge || j.type == JointType::Ball;
  if (!props::begin("##joint")) return;
  if (props::section("Kinematics")) {
    if (props::combo("Type", &j.type, kJointTypes, JointType::Hinge, D("joint.type"))) {
      JointType t = j.type;
      applyToSelected<Joint>([&](Joint& o) { o.type = t; });
      doc_.touch();
    }
    if (j.type != JointType::Free) {
      if (props::vec3("Anchor", &j.pos, {}, O(D("joint.pos"), "m", 0.002))) applyToSelected<Joint>([&](Joint& o) { o.pos = j.pos; });
    }
    if (j.type == JointType::Hinge || j.type == JointType::Slide) {
      if (props::vec3("Axis", &j.axis, {0, 0, 1}, O(D("joint.axis"), nullptr, 0.01))) {
        applyToSelected<Joint>([&](Joint& o) { o.axis = j.axis; });
      }
      ImGui::TableNextRow();
      ImGui::TableSetColumnIndex(1);
      const char* names[] = {"X", "Y", "Z", "-X", "-Y", "-Z"};
      const Vec3 axes[] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {-1, 0, 0}, {0, -1, 0}, {0, 0, -1}};
      for (int i = 0; i < 6; ++i) {
        if (i) ImGui::SameLine(0, 3);
        if (ImGui::SmallButton(names[i])) {
          checkpoint("Joint axis");
          j.axis = axes[i];
          applyToSelected<Joint>([&](Joint& o) { o.axis = axes[i]; });
        }
      }
    }
  }
  if (j.type != JointType::Free && props::section("Range")) {
    if (props::checkbox("Limited", &j.limited, false, D("joint.limited"))) {
      if (j.limited && j.range.x == 0 && j.range.y == 0) j.range = angular ? Vec2{-deg2rad(90), deg2rad(90)} : Vec2{-0.1, 0.1};
      applyToSelected<Joint>([&](Joint& o) { o.limited = j.limited; if (o.limited && o.range == Vec2{}) o.range = j.range; });
    }
    if (j.limited) {
      bool c = angular ? props::angleRange("Range", &j.range, O(D("joint.range")))
                       : props::vec2("Range", &j.range, {}, O(D("joint.range"), "m", 0.002));
      if (c) applyToSelected<Joint>([&](Joint& o) { o.range = j.range; });
      if (j.type == JointType::Ball && j.range.x != 0) props::info("", "Ball joints: the first value must be 0 (the second is the cone angle).");
      if (j.range.x > j.range.y) props::info("", "Lower limit is above the upper limit.");
    }
    if (props::section("Limit softness (advanced)", false)) {
      if (props::vec2("solreflimit", &j.solreflimit, kDefaultSolref, O(D("joint.solreflimit"), nullptr, 0.001)))
        applyToSelected<Joint>([&](Joint& o) { o.solreflimit = j.solreflimit; });
      if (props::vec5("solimplimit", &j.solimplimit, kDefaultSolimp, O(D("joint.solimplimit"), nullptr, 0.001)))
        applyToSelected<Joint>([&](Joint& o) { o.solimplimit = j.solimplimit; });
      if (props::real("Margin", &j.margin, 0, O(D("joint.margin"), angular ? "rad" : "m", 0.001, 0, 10)))
        applyToSelected<Joint>([&](Joint& o) { o.margin = j.margin; });
    }
  }
  if (props::section("Dynamics")) {
    const char* tu = angular ? "N*m*s/rad" : "N*s/m";
    if (props::real("Damping", &j.damping, 0, O(D("joint.damping"), tu, 0.01, 0, 1e9))) applyToSelected<Joint>([&](Joint& o) { o.damping = j.damping; });
    if (props::real("Armature", &j.armature, 0, O(D("joint.armature"), angular ? "kg*m^2" : "kg", 0.001, 0, 1e9)))
      applyToSelected<Joint>([&](Joint& o) { o.armature = j.armature; });
    if (props::real("Friction loss", &j.frictionloss, 0, O(D("joint.frictionloss"), angular ? "N*m" : "N", 0.01, 0, 1e9)))
      applyToSelected<Joint>([&](Joint& o) { o.frictionloss = j.frictionloss; });
    if (props::real("Stiffness", &j.stiffness, 0, O(D("joint.stiffness"), angular ? "N*m/rad" : "N/m", 0.1, 0, 1e9)))
      applyToSelected<Joint>([&](Joint& o) { o.stiffness = j.stiffness; });
    if (j.stiffness > 0) {
      bool c = angular ? props::angle("Spring rest", &j.springref, 0, O(D("joint.springref")))
                       : props::real("Spring rest", &j.springref, 0, O(D("joint.springref"), "m", 0.002));
      if (c) applyToSelected<Joint>([&](Joint& o) { o.springref = j.springref; });
    }
  }
  if (j.type != JointType::Free && props::section("Actuator force at this joint")) {
    if (props::checkbox("Limit actuator force", &j.actuatorfrclimited, false, D("joint.actuatorfrclimited"))) {
      if (j.actuatorfrclimited && j.actuatorfrcrange == Vec2{}) j.actuatorfrcrange = {-10, 10};
      applyToSelected<Joint>([&](Joint& o) { o.actuatorfrclimited = j.actuatorfrclimited; o.actuatorfrcrange = j.actuatorfrcrange; });
    }
    if (j.actuatorfrclimited &&
        props::vec2("Force range", &j.actuatorfrcrange, {}, O(D("joint.actuatorfrcrange"), angular ? "N*m" : "N", 0.1)))
      applyToSelected<Joint>([&](Joint& o) { o.actuatorfrcrange = j.actuatorfrcrange; });
    if (props::checkbox("Actuator gravcomp", &j.actuatorgravcomp, false, D("joint.actuatorgravcomp")))
      applyToSelected<Joint>([&](Joint& o) { o.actuatorgravcomp = j.actuatorgravcomp; });
  }
  if (props::section("Reference (advanced)", false)) {
    bool c = angular ? props::angle("Reference", &j.ref, 0, O(D("joint.ref"))) : props::real("Reference", &j.ref, 0, O(D("joint.ref"), "m", 0.002));
    if (c) applyToSelected<Joint>([&](Joint& o) { o.ref = j.ref; });
    if (props::integer("Group", &j.group, 0, O(D("joint.group"), nullptr, 0.1, 0, 30))) applyToSelected<Joint>([&](Joint& o) { o.group = j.group; });
  }
  props::end();

  drawStrength(j.id);
  // Actuators driving this joint.
  ImGui::Spacing();
  ImGui::SeparatorText("Actuators driving this joint");
  int count = 0;
  for (const Actuator& a : doc_.actuators) {
    if ((a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) && a.target == j.id) {
      ImGui::PushID(int(a.id));
      iconInline(Icon::Actuator, pal::kActuator);
      ImGui::SameLine();
      if (ImGui::Selectable(a.name.c_str())) sel_.set(a.id);
      ImGui::SameLine();
      ImGui::TextDisabled("(%s)", toXml(a.type));
      ImGui::PopID();
      ++count;
    }
  }
  if (count == 0) ImGui::TextDisabled("None: this joint is passive.");
  if (j.type != JointType::Free) {
    if (ImGui::Button("+ Position servo")) addActuatorsFor({j.id}, ActuatorType::Position);
    ImGui::SameLine();
    if (ImGui::Button("+ Motor")) addActuatorsFor({j.id}, ActuatorType::Motor);
    ImGui::SameLine();
    if (ImGui::Button("+ Velocity")) addActuatorsFor({j.id}, ActuatorType::Velocity);
  }
}

// ------------------------------------------------------------------------------------ site, camera, light

void Editor::inspectSite(Site& s) {
  nameHeader(*this, s.id, Icon::Site, pal::kSite, "Site", s.name);
  if (!props::begin("##site")) return;
  if (props::section("Shape & placement")) {
    if (props::combo("Type", &s.type, kGeomTypes, GeomType::Sphere, D("site.type"))) {
      if (s.type == GeomType::Plane || s.type == GeomType::Hfield || s.type == GeomType::Mesh) s.type = GeomType::Sphere;
    }
    props::vec3("Size", &s.size, {0.005, 0.005, 0.005}, O(D("site.size"), "m", 0.001, 1e-4, 100));
    if (props::vec3("Position", &s.pos, {}, O(D("site.pos"), "m", 0.002))) applyToSelected<Site>([&](Site& o) { o.pos = s.pos; });
    props::orientation("Orientation", &s.quat, "Euler angles XYZ in degrees. Sensors such as accelerometers measure in this frame.");
    props::color("Color", &s.rgba, {0.5f, 0.5f, 0.5f, 1}, D("site.rgba"));
    props::integer("Group", &s.group, 0, O(D("site.group"), nullptr, 0.1, 0, 30));
  }
  props::end();
  ImGui::Spacing();
  ImGui::TextDisabled("Sites are massless frames used by sensors, tendons and Cartesian actuators.");
  if (ImGui::Button("+ IMU (accelerometer + gyro)")) {
    checkpoint("Add IMU sensors");
    addSensor(doc_, SensorType::Accelerometer, s.id);
    addSensor(doc_, SensorType::Gyro, s.id);
  }
  ImGui::SameLine();
  if (ImGui::Button("+ Touch")) {
    checkpoint("Add touch sensor");
    addSensor(doc_, SensorType::Touch, s.id);
  }
}

void Editor::inspectCamera(Camera& c) {
  nameHeader(*this, c.id, Icon::Camera, pal::kCamera, "Camera", c.name);
  if (!props::begin("##camera")) return;
  if (props::section("Camera")) {
    props::vec3("Position", &c.pos, {}, O(D("camera.pos"), "m", 0.005));
    props::orientation("Orientation", &c.quat, "The camera looks along its local -Z axis, with +Y up.");
    props::real("Field of view", &c.fovy, 45, O(D("camera.fovy"), "deg", 0.2, 1, 179));
    props::combo("Mode", &c.mode, kCameraModes, CameraMode::Fixed, D("camera.mode"));
    if (c.mode == CameraMode::TargetBody || c.mode == CameraMode::TargetBodyCom)
      props::element("Target body", &c.target, listOfAny(doc_.bodies), true, D("camera.target"));
  }
  props::end();
  if (ImGui::Button("Set from current view")) {
    checkpoint("Camera from view");
    Vec3 eye = vp_.cam.eye();
    Vec3 f = normalize(vp_.cam.target - eye);
    Vec3 r = normalize(cross(f, {0, 0, 1}), {1, 0, 0});
    Vec3 u = cross(r, f);
    Mat3 m;
    for (int i = 0; i < 3; ++i) { m(i, 0) = r[i]; m(i, 1) = u[i]; m(i, 2) = -f[i]; }
    Pose world{eye, fromMat3(m)};
    Pose parent = c.body == kWorldId ? Pose{} : bodyWorldPose(doc_, c.body);
    Pose local = inverse(parent) * world;
    c.pos = local.pos;
    c.quat = canonical(local.quat);
  }
}

void Editor::inspectLight(Light& l) {
  nameHeader(*this, l.id, Icon::Light, pal::kLight, "Light", l.name);
  if (!props::begin("##light")) return;
  if (props::section("Light")) {
    props::combo("Type", &l.type, kLightTypes, LightType::Spot, D("light.type"));
    props::vec3("Position", &l.pos, {}, O(D("light.pos"), "m", 0.01));
    if (props::vec3("Direction", &l.dir, {0, 0, -1}, O(D("light.dir"), nullptr, 0.01))) {}
    props::color3("Diffuse", &l.diffuse, {0.7, 0.7, 0.7}, D("light.diffuse"));
    props::color3("Ambient", &l.ambient, {0, 0, 0}, D("light.ambient"));
    props::color3("Specular", &l.specular, {0.3, 0.3, 0.3}, D("light.specular"));
    props::checkbox("Cast shadows", &l.castshadow, true, D("light.castshadow"));
    props::checkbox("Active", &l.active, true, D("light.active"));
    if (l.type == LightType::Spot) {
      props::real("Cutoff", &l.cutoff, 45, O(D("light.cutoff"), "deg", 0.2, 0, 180));
      props::real("Exponent", &l.exponent, 10, O(D("light.exponent"), nullptr, 0.1, 0, 200));
    }
    props::vec3("Attenuation", &l.attenuation, {1, 0, 0}, O(D("light.attenuation"), nullptr, 0.01, 0, 100));
  }
  props::end();
}

// ------------------------------------------------------------------------------------ actuator

void Editor::inspectActuator(Actuator& a) {
  nameHeader(*this, a.id, Icon::Actuator, pal::kActuator, "Actuator", a.name);
  ImGui::PushTextWrapPos(0);
  ImGui::TextColored(pal::v4(pal::kTextDim), "%s", actuatorExplanation(a.type));
  ImGui::PopTextWrapPos();
  const Joint* jt = (a.trn == TrnType::Joint || a.trn == TrnType::JointInParent) ? doc_.joint(a.target) : nullptr;
  bool angular = jt && (jt->type == JointType::Hinge || jt->type == JointType::Ball);
  const char* fu = jt ? (angular ? "N*m" : "N") : "N";
  if (!props::begin("##act")) return;
  if (props::section("Type & transmission")) {
    ActuatorType t = a.type;
    if (props::combo("Type", &t, kActuatorTypes, ActuatorType::Position, "The MJCF actuator shortcut element.")) {
      setActuatorType(doc_, a, t);
      applyToSelected<Actuator>([&](Actuator& o) { setActuatorType(doc_, o, t); });
    }
    const EnumEntry<TrnType> trns[] = {{TrnType::Joint, "joint", "Joint"},
                                       {TrnType::JointInParent, "jointinparent", "Joint (in parent frame)"},
                                       {TrnType::Tendon, "tendon", "Tendon"},
                                       {TrnType::Site, "site", "Site (Cartesian)"},
                                       {TrnType::Body, "body", "Body (adhesion)"}};
    TrnType tr = a.trn;
    if (props::combo("Transmission", &tr, trns, TrnType::Joint, "What the actuator acts on.")) {
      a.trn = tr;
      a.target = kNoId;
    }
    switch (a.trn) {
      case TrnType::Joint:
      case TrnType::JointInParent: props::element("Joint", &a.target, listOf(doc_.joints), false, D("general.joint")); break;
      case TrnType::Site:
        props::element("Site", &a.target, listOfAny(doc_.sites), false, D("general.site"));
        props::element("Reference site", &a.refsite, listOfAny(doc_.sites), true, D("general.refsite"));
        break;
      case TrnType::Body: props::element("Body", &a.target, listOfAny(doc_.bodies), false, D("general.body")); break;
      case TrnType::Tendon: props::element("Tendon", &a.target, listOfAny(doc_.tendons), false, D("general.tendon")); break;
      default: break;
    }
    if (a.trn == TrnType::Site) {
      props::vec3("Gear (force)", reinterpret_cast<Vec3*>(&a.gear.v[0]), {1, 0, 0}, O(D("general.gear"), nullptr, 0.01));
      props::vec3("Gear (torque)", reinterpret_cast<Vec3*>(&a.gear.v[3]), {0, 0, 0}, O(D("general.gear"), nullptr, 0.01));
    } else {
      if (props::real("Gear", &a.gear[0], 1.0, O(D("general.gear"), nullptr, 0.01))) applyToSelected<Actuator>([&](Actuator& o) { o.gear[0] = a.gear[0]; });
    }
  }
  if (props::section("Control range")) {
    if (props::checkbox("Limit control", &a.ctrllimited, true, D("general.ctrllimited")))
      applyToSelected<Actuator>([&](Actuator& o) { o.ctrllimited = a.ctrllimited; });
    if (a.ctrllimited) {
      bool c = (a.type == ActuatorType::Position && angular) ? props::angleRange("Control range", &a.ctrlrange, O(D("general.ctrlrange")))
                                                            : props::vec2("Control range", &a.ctrlrange, {0, 0}, O(D("general.ctrlrange"), nullptr, 0.01));
      if (c) applyToSelected<Actuator>([&](Actuator& o) { o.ctrlrange = a.ctrlrange; });
      if (a.type == ActuatorType::Position && jt && jt->limited) {
        ImGui::TableNextRow();
        ImGui::TableSetColumnIndex(1);
        if (ImGui::SmallButton("Match joint range")) {
          checkpoint("Match joint range");
          a.ctrlrange = jt->range;
        }
      }
    } else if (a.type == ActuatorType::Motor) {
      props::info("", "Unbounded controls: RL policies usually need a bounded action space.");
    }
    if (a.type == ActuatorType::Position || a.type == ActuatorType::IntVelocity) {
      if (props::real("Inherit range", &a.inheritrange, 0, O(D("position.inheritrange"), nullptr, 0.01, 0, 10)))
        applyToSelected<Actuator>([&](Actuator& o) { o.inheritrange = a.inheritrange; });
    }
  }
  if (props::section("Force limit")) {
    if (props::checkbox("Limit force", &a.forcelimited, false, D("general.forcelimited"))) {
      if (a.forcelimited && a.forcerange == Vec2{}) a.forcerange = {-10, 10};
      applyToSelected<Actuator>([&](Actuator& o) { o.forcelimited = a.forcelimited; o.forcerange = a.forcerange; });
    }
    if (a.forcelimited && props::vec2("Force range", &a.forcerange, {}, O(D("general.forcerange"), fu, 0.1)))
      applyToSelected<Actuator>([&](Actuator& o) { o.forcerange = a.forcerange; });
  }
  switch (a.type) {
    case ActuatorType::Position:
    case ActuatorType::IntVelocity:
      if (props::section("Servo gains")) {
        const char* kpu = jt ? (angular ? "N*m/rad" : "N/m") : nullptr;
        if (props::real("kp (stiffness)", &a.kp, 1.0, O(D("position.kp"), kpu, 0.1, 0, 1e9))) applyToSelected<Actuator>([&](Actuator& o) { o.kp = a.kp; });
        if (a.dampratio > 0 && a.kv > 0) props::info("", "kv and dampratio are exclusive: set one of them to 0.");
        if (props::real("Damping ratio", &a.dampratio, 0, O(D("position.dampratio"), nullptr, 0.01, 0, 100)))
          applyToSelected<Actuator>([&](Actuator& o) { o.dampratio = a.dampratio; });
        if (props::real("kv (damping)", &a.kv, 0, O(D("position.kv"), jt ? (angular ? "N*m*s/rad" : "N*s/m") : nullptr, 0.01, 0, 1e9)))
          applyToSelected<Actuator>([&](Actuator& o) { o.kv = a.kv; });
        if (a.type == ActuatorType::Position &&
            props::real("Time constant", &a.timeconst, 0, O(D("position.timeconst"), "s", 0.001, 0, 10)))
          applyToSelected<Actuator>([&](Actuator& o) { o.timeconst = a.timeconst; });
        if (a.type == ActuatorType::IntVelocity) {
          if (props::checkbox("Limit target (actrange)", &a.actlimited, false, D("general.actlimited"))) {}
          if (a.actlimited) props::vec2("Target range", &a.actrange, {}, O(D("general.actrange"), nullptr, 0.01));
        }
      }
      break;
    case ActuatorType::Velocity:
    case ActuatorType::Damper:
      if (props::section("Gain")) {
        if (props::real("kv", &a.kv, 1.0, O(D("velocity.kv"), nullptr, 0.01, 0, 1e9))) applyToSelected<Actuator>([&](Actuator& o) { o.kv = a.kv; });
      }
      break;
    case ActuatorType::Cylinder:
      if (props::section("Cylinder")) {
        props::real("Time constant", &a.cylTimeconst, 1.0, O(D("cylinder.timeconst"), "s", 0.01, 0, 100));
        props::real("Area", &a.area, 1.0, O(D("cylinder.area"), "m^2", 0.01, 0, 1e6));
        props::real("Diameter", &a.diameter, 0, O(D("cylinder.diameter"), "m", 0.001, 0, 100));
        props::vec3("Bias", &a.cylBias, {}, O(D("cylinder.bias"), nullptr, 0.01));
      }
      break;
    case ActuatorType::Muscle:
      if (props::section("Muscle")) {
        props::vec2("Time constants", &a.muscleTimeconst, {0.01, 0.04}, O(D("muscle.timeconst"), "s", 0.001, 0, 10));
        props::vec2("Operating range", &a.muscleRange, {0.75, 1.05}, O(D("muscle.range"), nullptr, 0.01, 0, 10));
        props::real("Force", &a.muscleForce, -1, O(D("muscle.force"), "N", 1, -1, 1e9));
        props::real("Scale", &a.muscleScale, 200, O(D("muscle.scale"), nullptr, 1, 0, 1e9));
        props::real("lmin", &a.lmin, 0.5, O(D("muscle.lmin"), nullptr, 0.01, 0, 10));
        props::real("lmax", &a.lmax, 1.6, O(D("muscle.lmax"), nullptr, 0.01, 0, 10));
        props::real("vmax", &a.vmax, 1.5, O(D("muscle.vmax"), nullptr, 0.01, 0, 100));
        props::real("fpmax", &a.fpmax, 1.3, O(D("muscle.fpmax"), nullptr, 0.01, 0, 10));
        props::real("fvmax", &a.fvmax, 1.2, O(D("muscle.fvmax"), nullptr, 0.01, 0, 10));
      }
      break;
    case ActuatorType::Adhesion:
      if (props::section("Adhesion")) props::real("Gain", &a.adhesionGain, 1.0, O(D("adhesion.gain"), "N", 0.1, 0, 1e9));
      break;
    case ActuatorType::General:
      if (props::section("General model")) {
        props::combo("Dynamics", &a.dyntype, kDynTypes, DynType::None, D("general.dyntype"));
        props::combo("Gain", &a.gaintype, kGainTypes, GainType::Fixed, D("general.gaintype"));
        props::combo("Bias", &a.biastype, kBiasTypes, BiasType::None, D("general.biastype"));
        props::vec5("dynprm[0:5]", reinterpret_cast<Vec5*>(&a.dynprm.v[0]), Vec5{{1, 0, 0, 0, 0}}, O(D("general.dynprm"), nullptr, 0.01));
        props::vec5("gainprm[0:5]", reinterpret_cast<Vec5*>(&a.gainprm.v[0]), Vec5{{1, 0, 0, 0, 0}}, O(D("general.gainprm"), nullptr, 0.01));
        props::vec5("biasprm[0:5]", reinterpret_cast<Vec5*>(&a.biasprm.v[0]), Vec5{}, O(D("general.biasprm"), nullptr, 0.01));
        if (props::checkbox("Limit activation", &a.actlimited, false, D("general.actlimited"))) {}
        if (a.actlimited) props::vec2("Activation range", &a.actrange, {}, O(D("general.actrange"), nullptr, 0.01));
        props::checkbox("Act early", &a.actearly, false, D("general.actearly"));
      }
      break;
    default: break;
  }
  if (props::section("Other", false)) {
    props::vec2("Length range", &a.lengthrange, {}, O(D("general.lengthrange"), nullptr, 0.01));
    if (props::integer("Group", &a.group, 0, O(D("general.group"), nullptr, 0.1, 0, 30))) applyToSelected<Actuator>([&](Actuator& o) { o.group = a.group; });
  }
  props::end();
  if (jt) drawStrength(jt->id);
}

// ------------------------------------------------------------------------------------ sensor

void Editor::inspectSensor(Sensor& s) {
  nameHeader(*this, s.id, Icon::Sensor, pal::kSensor, "Sensor", s.name);
  if (!props::begin("##sensor")) return;
  if (props::section("Sensor")) {
    std::vector<std::string> labels;
    int idx = 0;
    for (size_t i = 0; i < std::size(kSensorTypes); ++i) {
      labels.push_back(std::string(kSensorTypes[i].label) + "  (" + kSensorTypes[i].xml + ")");
      if (kSensorTypes[i].value == s.type) idx = int(i);
    }
    if (props::comboIndex("Type", &idx, labels, 0, "MJCF sensor element.")) {
      SensorObj before = sensorInfo(s.type).obj;
      s.type = kSensorTypes[idx].value;
      if (sensorInfo(s.type).obj != before) s.obj = kNoId;
    }
    const SensorInfo& info = sensorInfo(s.type);
    switch (info.obj) {
      case SensorObj::Joint: props::element("Joint", &s.obj, listOf(doc_.joints), false); break;
      case SensorObj::Actuator: props::element("Actuator", &s.obj, listOfAny(doc_.actuators), false); break;
      case SensorObj::Site: props::element("Site", &s.obj, listOfAny(doc_.sites), false); break;
      case SensorObj::Body: props::element("Body", &s.obj, listOfAny(doc_.bodies), false); break;
      case SensorObj::Tendon: props::element("Tendon", &s.obj, listOfAny(doc_.tendons), false); break;
      case SensorObj::Frame: {
        props::combo("Object type", &s.objtype, kFrameObjs, FrameObj::Site);
        std::vector<std::pair<Id, std::string>> items;
        switch (s.objtype) {
          case FrameObj::Body:
          case FrameObj::XBody: items = listOfAny(doc_.bodies); break;
          case FrameObj::Geom: items = listOfAny(doc_.geoms); break;
          case FrameObj::Site: items = listOfAny(doc_.sites); break;
          case FrameObj::Camera: items = listOfAny(doc_.cameras); break;
        }
        props::element("Object", &s.obj, items, false);
        props::combo("Reference type", &s.reftype, kFrameObjs, FrameObj::Site, "Optional frame in which the quantity is expressed.");
        std::vector<std::pair<Id, std::string>> refs;
        switch (s.reftype) {
          case FrameObj::Body:
          case FrameObj::XBody: refs = listOfAny(doc_.bodies); break;
          case FrameObj::Geom: refs = listOfAny(doc_.geoms); break;
          case FrameObj::Site: refs = listOfAny(doc_.sites); break;
          case FrameObj::Camera: refs = listOfAny(doc_.cameras); break;
        }
        props::element("Reference", &s.ref, refs, true);
        break;
      }
      default: break;
    }
    props::info("Output size", "%d value%s", info.dim, info.dim > 1 ? "s" : "");
    props::real("Noise (std)", &s.noise, 0, O(D("sensor.noise"), nullptr, 0.001, 0, 1e6));
    props::real("Cutoff", &s.cutoff, 0, O(D("sensor.cutoff"), nullptr, 0.01, 0, 1e9));
  }
  props::end();
}

// ------------------------------------------------------------------------------------ assets

void Editor::inspectMaterial(Material& m) {
  nameHeader(*this, m.id, Icon::Material, pal::kTextDim, "Material", m.name);
  if (!props::begin("##mat")) return;
  if (props::section("Material")) {
    props::color("Color", &m.rgba, {1, 1, 1, 1}, D("material.rgba"));
    props::element("Texture", &m.texture, listOfAny(doc_.textures), true, D("material.texture"));
    props::vec2("Texture repeat", &m.texrepeat, {1, 1}, O(D("material.texrepeat"), nullptr, 0.05));
    props::checkbox("Uniform texture", &m.texuniform, false, D("material.texuniform"));
    props::real("Specular", &m.specular, 0.5, O(D("material.specular"), nullptr, 0.01, 0, 1));
    props::real("Shininess", &m.shininess, 0.5, O(D("material.shininess"), nullptr, 0.01, 0, 1));
    props::real("Reflectance", &m.reflectance, 0, O(D("material.reflectance"), nullptr, 0.01, 0, 1));
    props::real("Emission", &m.emission, 0, O(D("material.emission"), nullptr, 0.01, 0, 10));
  }
  props::end();
}

void Editor::inspectTexture(Texture& t) {
  nameHeader(*this, t.id, Icon::Material, pal::kTextDim, "Texture", t.name);
  if (!props::begin("##tex")) return;
  if (props::section("Texture")) {
    props::combo("Type", &t.type, kTextureTypes, TextureType::TwoD, D("texture.type"));
    props::combo("Builtin", &t.builtin, kTextureBuiltins, TextureBuiltin::Checker, D("texture.builtin"));
    props::color3("Color 1", &t.rgb1, {0.8, 0.8, 0.8}, D("texture.rgb1"));
    props::color3("Color 2", &t.rgb2, {0.5, 0.5, 0.5}, D("texture.rgb2"));
    props::combo("Mark", &t.mark, kTextureMarks, TextureMark::None, D("texture.mark"));
    if (t.mark != TextureMark::None) props::color3("Mark color", &t.markrgb, {0, 0, 0}, D("texture.markrgb"));
    props::integer("Width", &t.width, 512, O(D("texture.width"), "px", 1, 1, 8192));
    props::integer("Height", &t.height, 512, O(D("texture.height"), "px", 1, 1, 8192));
  }
  props::end();
}

void Editor::inspectExclude(Exclude& e) {
  ImGui::TextDisabled("Contact exclusion");
  ImGui::Separator();
  if (!props::begin("##excl")) return;
  props::element("Body 1", &e.body1, listOfAny(doc_.bodies), false, D("exclude.body1"));
  props::element("Body 2", &e.body2, listOfAny(doc_.bodies), false, D("exclude.body2"));
  props::end();
  ImGui::TextWrapped("Geoms of these two bodies never collide with each other.");
}

// Elements listed by their display names (many geoms and sites are unnamed).
template <class T> std::vector<std::pair<Id, std::string>> listDisplay(const Document& doc, const std::vector<T>& v) {
  std::vector<std::pair<Id, std::string>> out;
  for (const auto& e : v) out.push_back({e.id, doc.displayName(e.id)});
  return out;
}

void Editor::inspectPair(ContactPair& c) {
  nameHeader(*this, c.id, Icon::Exclude, pal::kGeom, "Contact pair", c.name);
  ImGui::PushTextWrapPos(0);
  ImGui::TextColored(pal::v4(pal::kTextDim),
                     "These two geoms always collide, with exactly the parameters below: contype/conaffinity and the geoms' "
                     "own contact settings are ignored for this pair. Pairs are a cheap way to let specific geoms (feet) touch "
                     "specific others (the floor) while all other collisions are off.");
  ImGui::PopTextWrapPos();
  if (!props::begin("##pair")) return;
  if (props::section("Geoms")) {
    props::element("Geom 1", &c.geom1, listDisplay(doc_, doc_.geoms), false, D("pair.geom1"));
    props::element("Geom 2", &c.geom2, listDisplay(doc_, doc_.geoms), false, D("pair.geom2"));
  }
  if (props::section("Contact")) {
    int idx = c.condim == 1 ? 0 : c.condim == 4 ? 2 : c.condim == 6 ? 3 : 1;
    if (props::comboIndex("Contact dims", &idx, {"1  frictionless", "3  sliding friction", "4  + torsional", "6  + rolling"}, 1,
                          D("pair.condim")))
      c.condim = idx == 0 ? 1 : idx == 1 ? 3 : idx == 2 ? 4 : 6;
    props::vec5("Friction", &c.friction, Vec5{{1, 1, 0.005, 0.0001, 0.0001}},
                O("Sliding friction in two tangent directions, torsional friction, rolling friction in two directions.",
                  nullptr, 0.005, 0, 100));
    props::real("Margin", &c.margin, 0, O(D("pair.margin"), "m", 0.001, 0, 10));
    props::real("Gap", &c.gap, 0, O(D("pair.gap"), "m", 0.001, 0, 10));
  }
  if (props::section("Solver (advanced)", false)) {
    props::vec2("Solref", &c.solref, kDefaultSolref, O(D("pair.solref"), nullptr, 0.001));
    props::vec2("Solref friction", &c.solreffriction, Vec2{0, 0}, O(D("pair.solreffriction"), nullptr, 0.001));
    props::vec5("Solimp", &c.solimp, kDefaultSolimp, O(D("pair.solimp"), nullptr, 0.001));
  }
  props::end();
  if (ImGui::Button("Take values from the geoms")) {
    checkpoint("Pair from geoms");
    pairParamsFromGeoms(doc_, c);
  }
  if (ImGui::IsItemHovered()) ImGui::SetTooltip("Combine the two geoms' friction and solver settings the way MuJoCo would");
}

void Editor::inspectEquality(Equality& e) {
  nameHeader(*this, e.id, Icon::Ball, pal::kJoint, "Equality", e.name);
  const char* what = "";
  switch (e.type) {
    case EqualityType::Connect:
      what = "Connects two bodies at a point, like a ball joint: they can rotate but the point stays shared. Use it to "
             "close kinematic loops (four-bar linkages, parallel legs). Leave body 2 empty to pin body 1 to the world.";
      break;
    case EqualityType::Weld:
      what = "Holds two bodies at a fixed relative pose (by default the pose in the editor). Leave body 2 empty to fix "
             "body 1 in the world, e.g. to hang a creature while testing its legs.";
      break;
    case EqualityType::Joint:
      what = "Couples two joints: q1 = c0 + c1*q2 + c2*q2^2 + c3*q2^3 + c4*q2^4 (relative to their reference positions). "
             "Without a second joint, joint 1 is held at c0.";
      break;
    case EqualityType::Tendon:
      what = "Couples two tendon lengths with the same polynomial as joint couplings.";
      break;
  }
  ImGui::PushTextWrapPos(0);
  ImGui::TextColored(pal::v4(pal::kTextDim), "%s", what);
  ImGui::PopTextWrapPos();
  if (!props::begin("##eq")) return;
  if (props::section("Constraint")) {
    EqualityType t = e.type;
    if (props::combo("Type", &t, kEqualityTypes, EqualityType::Connect, "The MJCF equality element.")) {
      bool bodiesBefore = e.type == EqualityType::Connect || e.type == EqualityType::Weld;
      bool bodiesAfter = t == EqualityType::Connect || t == EqualityType::Weld;
      if (bodiesBefore != bodiesAfter || t == EqualityType::Joint || t == EqualityType::Tendon) e.obj1 = e.obj2 = kNoId;
      e.type = t;
    }
    switch (e.type) {
      case EqualityType::Connect:
      case EqualityType::Weld: {
        const char* key = e.type == EqualityType::Connect ? "connect." : "weld.";
        bool sites = doc_.kindOf(e.obj1) == Kind::Site || (e.obj1 == kNoId && doc_.kindOf(e.obj2) == Kind::Site);
        bool s = sites;
        if (props::checkbox("Between sites", &s, false, "Constrain two sites instead of two bodies (MuJoCo site1 / site2).")) {
          e.obj1 = e.obj2 = kNoId;
          sites = s;
        }
        std::string k1 = std::string(key) + (sites ? "site1" : "body1"), k2 = std::string(key) + (sites ? "site2" : "body2");
        if (sites) {
          props::element("Site 1", &e.obj1, listDisplay(doc_, doc_.sites), false, D(k1.c_str()));
          props::element("Site 2", &e.obj2, listDisplay(doc_, doc_.sites), false, D(k2.c_str()));
        } else {
          std::vector<std::pair<Id, std::string>> bodies;
          for (const Body& b : doc_.bodies) if (b.id != kWorldId) bodies.push_back({b.id, doc_.displayName(b.id)});
          props::element("Body 1", &e.obj1, bodies, false, D(k1.c_str()));
          props::element("Body 2", &e.obj2, bodies, true, "Leave empty (none) for the world.");
          std::string ka = std::string(key) + "anchor";
          props::vec3(e.type == EqualityType::Connect ? "Anchor (body 1)" : "Anchor (body 2)", &e.anchor, {},
                      O(D(ka.c_str()), "m", 0.002));
          if (e.type == EqualityType::Weld) {
            props::checkbox("Explicit relative pose", &e.relpose, false,
                            "Off: body 2 keeps the pose relative to body 1 that it has in the editor (MuJoCo computes it "
                            "from the reference configuration). On: give the pose of body 2 in body 1's frame.");
            if (e.relpose) {
              props::vec3("Relative position", &e.relpos, {}, O(D("weld.relpose"), "m", 0.002));
              props::orientation("Relative orientation", &e.relquat, D("weld.relpose"));
            }
          }
        }
        if (e.type == EqualityType::Weld)
          props::real("Torque scale", &e.torquescale, 1.0, O(D("weld.torquescale"), nullptr, 0.01, 0, 1e6));
        break;
      }
      case EqualityType::Joint: {
        std::vector<std::pair<Id, std::string>> joints;
        for (const Joint& j : doc_.joints)
          if (j.type == JointType::Hinge || j.type == JointType::Slide) joints.push_back({j.id, doc_.displayName(j.id)});
        props::element("Joint 1", &e.obj1, joints, false, D("eqjoint.joint1"));
        props::element("Joint 2", &e.obj2, joints, true, D("eqjoint.joint2"));
        props::vec5("Polynomial", &e.polycoef, Vec5{{0, 1, 0, 0, 0}}, O(D("eqjoint.polycoef"), nullptr, 0.01));
        break;
      }
      case EqualityType::Tendon:
        props::element("Tendon 1", &e.obj1, listDisplay(doc_, doc_.tendons), false, D("eqtendon.tendon1"));
        props::element("Tendon 2", &e.obj2, listDisplay(doc_, doc_.tendons), true, D("eqtendon.tendon2"));
        props::vec5("Polynomial", &e.polycoef, Vec5{{0, 1, 0, 0, 0}}, O(D("eqtendon.polycoef"), nullptr, 0.01));
        break;
    }
    props::checkbox("Active", &e.active, true, D("connect.active"));
  }
  if (props::section("Solver (advanced)", false)) {
    props::vec2("Solref", &e.solref, kDefaultSolref, O(D("connect.solref"), nullptr, 0.001));
    props::vec5("Solimp", &e.solimp, kDefaultSolimp, O(D("connect.solimp"), nullptr, 0.001));
  }
  props::end();
  if (e.type == EqualityType::Connect && doc_.body(e.obj1) && ImGui::Button("Anchor at body 2's origin")) {
    checkpoint("Connect anchor");
    Pose p1 = bodyWorldPose(doc_, e.obj1);
    Vec3 at = doc_.body(e.obj2) ? bodyWorldPose(doc_, e.obj2).pos : p1.pos;
    e.anchor = inverse(p1).apply(at);
  }
}

void Editor::inspectTendon(Tendon& t) {
  nameHeader(*this, t.id, Icon::Slide, pal::kActuator, "Tendon", t.name);
  ImGui::PushTextWrapPos(0);
  ImGui::TextColored(pal::v4(pal::kTextDim), "%s",
                     t.type == TendonType::Fixed
                         ? "A fixed tendon's length is a weighted sum of joint positions. An actuator on it drives all those "
                           "joints at once (one motor for a whole finger or a coupled leg); limits, springs and friction act "
                           "on the combination."
                         : "A spatial tendon is the length of a path through sites (wrapping around spheres or cylinders). "
                           "Actuators on it pull like a cable or muscle; it can only pull if the actuator force is limited "
                           "to negative values.");
  ImGui::PopTextWrapPos();
  if (!props::begin("##tendon")) return;
  const float s = scale_;
  // A row with a combo for an element, a number and a remove button.
  auto idCombo = [&](const char* id, Id* v, const std::vector<std::pair<Id, std::string>>& items, float w,
                     const char* empty = "(choose)") {
    std::string cur = empty;
    for (const auto& [i, n] : items) if (i == *v) cur = n;
    ImGui::SetNextItemWidth(w);
    bool changed = false;
    // "Add" pickers (nothing chosen yet) look like buttons.
    ImGuiComboFlags flags = ImGuiComboFlags_HeightLarge | (*v == kNoId && empty[0] == '+' ? ImGuiComboFlags_NoArrowButton : 0);
    if (ImGui::BeginCombo(id, cur.c_str(), flags)) {
      for (const auto& [i, n] : items) {
        ImGui::PushID(int(i));
        if (ImGui::Selectable(n.c_str(), i == *v) && i != *v) {
          props::notifyChange();
          *v = i;
          changed = true;
        }
        ImGui::PopID();
      }
      ImGui::EndCombo();
    }
    return changed;
  };
  if (props::section("Tendon")) {
    TendonType tt = t.type;
    if (props::combo("Type", &tt, kTendonTypes, TendonType::Fixed, "Fixed: joint combination. Spatial: path through sites."))
      t.type = tt;
    float btn = ImGui::GetFrameHeight();
    if (t.type == TendonType::Fixed) {
      std::vector<std::pair<Id, std::string>> joints;
      for (const Joint& j : doc_.joints)
        if (j.type == JointType::Hinge || j.type == JointType::Slide) joints.push_back({j.id, doc_.displayName(j.id)});
      int remove = -1;
      for (size_t i = 0; i < t.joints.size(); ++i) {
        ImGui::PushID(int(i));
        bool reset = false;
        std::string label = "Joint " + std::to_string(i + 1);
        props::beginRow(label.c_str(), false, D("fixedjoint.coef"), &reset);
        float w = ImGui::GetContentRegionAvail().x;
        idCombo("##j", &t.joints[i].joint, joints, w * 0.6f);
        ImGui::SameLine();
        double c = t.joints[i].coef;
        ImGui::SetNextItemWidth(std::max(30.0f, w * 0.4f - btn - 2 * ImGui::GetStyle().ItemSpacing.x));
        if (ImGui::DragScalar("##c", ImGuiDataType_Double, &c, 0.01f, nullptr, nullptr, "x %.3g")) {
          props::notifyChange();
          t.joints[i].coef = c;
        }
        ImGui::SameLine();
        if (ImGui::Button("x", ImVec2(btn, 0))) remove = int(i);
        props::endRow();
        ImGui::PopID();
      }
      if (remove >= 0) {
        props::notifyChange();
        t.joints.erase(t.joints.begin() + remove);
      }
      bool reset = false;
      props::beginRow("Add joint", false, "Adds a joint with coefficient 1.", &reset);
      Id add = kNoId;
      if (idCombo("##addj", &add, joints, ImGui::GetContentRegionAvail().x, "+ Joint")) t.joints.push_back({add, 1.0});
      props::endRow();
    } else {
      int remove = -1, up = -1;
      for (size_t i = 0; i < t.path.size(); ++i) {
        ImGui::PushID(int(i));
        TendonPoint& p = t.path[i];
        bool reset = false;
        const char* kind = p.type == TendonPoint::Type::Site ? "Site" : p.type == TendonPoint::Type::Geom ? "Wrap geom" : "Pulley";
        std::string label = std::to_string(i + 1) + ". " + kind;
        props::beginRow(label.c_str(), false,
                        p.type == TendonPoint::Type::Geom ? D("spatialgeom.geom")
                        : p.type == TendonPoint::Type::Pulley ? D("pulley.divisor")
                                                              : "The tendon passes through this site.",
                        &reset);
        float w = ImGui::GetContentRegionAvail().x - 2 * (btn + ImGui::GetStyle().ItemSpacing.x);
        if (p.type == TendonPoint::Type::Site) {
          idCombo("##o", &p.obj, listDisplay(doc_, doc_.sites), w);
        } else if (p.type == TendonPoint::Type::Geom) {
          std::vector<std::pair<Id, std::string>> wraps;
          for (const Geom& g : doc_.geoms)
            if (g.type == GeomType::Sphere || g.type == GeomType::Cylinder) wraps.push_back({g.id, doc_.displayName(g.id)});
          idCombo("##o", &p.obj, wraps, w, "(sphere or cylinder)");
        } else {
          double dv = p.divisor;
          ImGui::SetNextItemWidth(w);
          if (ImGui::DragScalar("##div", ImGuiDataType_Double, &dv, 0.05f, nullptr, nullptr, "divisor %.3g") && dv > 0) {
            props::notifyChange();
            p.divisor = dv;
          }
        }
        ImGui::SameLine();
        if (ImGui::ArrowButton("##up", ImGuiDir_Up) && i > 0) up = int(i);
        ImGui::SameLine();
        if (ImGui::Button("x", ImVec2(btn, 0))) remove = int(i);
        props::endRow();
        if (p.type == TendonPoint::Type::Geom) {
          // The side site gets its own row: which way around the geom the tendon goes.
          props::beginRow("    side site", false, D("spatialgeom.sidesite"), &reset);
          std::vector<std::pair<Id, std::string>> sides{{kNoId, "(shortest way)"}};
          for (const auto& x : listDisplay(doc_, doc_.sites)) sides.push_back(x);
          idCombo("##side", &p.sidesite, sides, w);
          props::endRow();
        }
        ImGui::PopID();
      }
      if (up > 0) {
        props::notifyChange();
        std::swap(t.path[size_t(up)], t.path[size_t(up - 1)]);
      }
      if (remove >= 0) {
        props::notifyChange();
        t.path.erase(t.path.begin() + remove);
      }
      bool reset = false;
      props::beginRow("Add to path", false, "Sites the tendon passes through, geoms it wraps around, pulleys that split it.",
                      &reset);
      float third = (ImGui::GetContentRegionAvail().x - 2 * ImGui::GetStyle().ItemSpacing.x) / 3;
      Id add = kNoId;
      if (idCombo("##adds", &add, listDisplay(doc_, doc_.sites), third, "+ Site")) t.path.push_back({TendonPoint::Type::Site, add});
      if (ImGui::IsItemHovered()) ImGui::SetTooltip("Add a site");
      ImGui::SameLine();
      if (ImGui::Button("+ Wrap", ImVec2(third, 0))) {
        props::notifyChange();
        t.path.push_back({TendonPoint::Type::Geom});
      }
      ImGui::SameLine();
      if (ImGui::Button("+ Pulley", ImVec2(third, 0))) {
        props::notifyChange();
        t.path.push_back({TendonPoint::Type::Pulley});
      }
      props::endRow();
    }
  }
  const char* k = t.type == TendonType::Spatial ? "spatial." : "spatial.";  // fixed tendons share the meaning
  auto Dk = [&](const char* attr) { return D((std::string(k) + attr).c_str()); };
  if (props::section("Limits, spring, friction")) {
    props::checkbox("Limited", &t.limited, false, Dk("limited"));
    if (t.limited) props::vec2("Length range", &t.range, Vec2{0, 0}, O(Dk("range"), t.type == TendonType::Spatial ? "m" : nullptr, 0.002));
    props::real("Stiffness", &t.stiffness, 0, O(Dk("stiffness"), nullptr, 1, 0, 1e9));
    props::vec2("Spring length", &t.springlength, Vec2{-1, -1},
                O("Rest length of the spring; -1 -1 uses the length at the reference pose. Two different values make a "
                  "dead band.",
                  nullptr, 0.002));
    props::real("Damping", &t.damping, 0, O(Dk("damping"), nullptr, 0.1, 0, 1e9));
    props::real("Friction loss", &t.frictionloss, 0, O(Dk("frictionloss"), nullptr, 0.01, 0, 1e9));
    props::real("Armature", &t.armature, 0, O(Dk("armature"), nullptr, 0.001, 0, 1e9));
    props::checkbox("Limit actuator force", &t.actfrclimited, false, Dk("actuatorfrclimited"));
    if (t.actfrclimited) props::vec2("Actuator force range", &t.actuatorfrcrange, Vec2{0, 0}, O(Dk("actuatorfrcrange"), "N", 0.5));
  }
  if (t.type == TendonType::Spatial && props::section("Appearance")) {
    props::real("Width", &t.width, 0.003, O(Dk("width"), "m", 0.0005, 0, 1));
    props::color("Color", &t.rgba, Color{0.5f, 0.5f, 0.5f, 1}, Dk("rgba"));
    props::element("Material", &t.material, listOfAny(doc_.materials), true, Dk("material"));
    props::integer("Group", &t.group, 0, O(Dk("group"), nullptr, 0.1, 0, 5));
  }
  if (props::section("Solver (advanced)", false)) {
    props::real("Margin", &t.margin, 0, O(Dk("margin"), nullptr, 0.001, 0, 10));
    props::vec2("Solref limit", &t.solreflimit, kDefaultSolref, O(Dk("solreflimit"), nullptr, 0.001));
    props::vec5("Solimp limit", &t.solimplimit, kDefaultSolimp, O(Dk("solimplimit"), nullptr, 0.001));
    props::vec2("Solref friction", &t.solreffriction, kDefaultSolref, O(Dk("solreffriction"), nullptr, 0.001));
    props::vec5("Solimp friction", &t.solimpfriction, kDefaultSolimp, O(Dk("solimpfriction"), nullptr, 0.001));
  }
  props::end();
  int users = 0;
  for (const Actuator& a : doc_.actuators) users += a.trn == TrnType::Tendon && a.target == t.id;
  ImGui::TextDisabled("%d actuator(s) on this tendon", users);
  if (ImGui::Button("+ Motor")) {
    checkpoint("Add tendon motor");
    Id a = addActuator(doc_, ActuatorType::Motor, kNoId, t.name.empty() ? "tendon_motor" : t.name + "_motor");
    if (Actuator* x = doc_.actuator(a)) {
      x->trn = TrnType::Tendon;
      x->target = t.id;
    }
    sel_.set(a);
  }
  ImGui::SameLine();
  if (ImGui::Button("+ Length sensor")) {
    checkpoint("Add tendon sensor");
    sel_.set(addSensor(doc_, SensorType::TendonPos, t.id));
  }
  (void)s;
}

void Editor::inspectWorld() {
  ImGui::TextDisabled("Nothing selected");
  ImGui::Separator();
  ImGui::TextWrapped("Click an element in the viewport or the outliner to edit it. Scene-wide physics settings "
                     "(timestep, integrator, solver, gravity...) are in the Scene panel.");
  ImGui::Spacing();
  ImGui::TextDisabled("Tips");
  ImGui::BulletText("Right-drag to orbit, middle-drag to pan, wheel to zoom.");
  ImGui::BulletText("W / E / R: move, rotate, resize. Hold Ctrl to toggle snapping.");
  ImGui::BulletText("Ctrl+D duplicates, Ctrl+M mirrors a limb to the other side.");
  ImGui::BulletText("Alt+click picks geoms instead of bodies (or vice versa).");
}

void Editor::drawInspector() {
  if (!ImGui::Begin("Inspector")) {
    ImGui::End();
    return;
  }
  sel_.prune(doc_);
  Id id = sel_.primary();
  ImGui::PushID(int(id));
  switch (doc_.kindOf(id)) {
    case Kind::Body:
      if (id == kWorldId) inspectWorld();
      else inspectBody(*doc_.body(id));
      break;
    case Kind::Geom: inspectGeom(*doc_.geom(id)); break;
    case Kind::Joint: inspectJoint(*doc_.joint(id)); break;
    case Kind::Site: inspectSite(*doc_.site(id)); break;
    case Kind::Camera: inspectCamera(*doc_.camera(id)); break;
    case Kind::Light: inspectLight(*doc_.light(id)); break;
    case Kind::Actuator: inspectActuator(*doc_.actuator(id)); break;
    case Kind::Sensor: inspectSensor(*doc_.sensor(id)); break;
    case Kind::Material: inspectMaterial(*doc_.material(id)); break;
    case Kind::Texture: inspectTexture(*doc_.texture(id)); break;
    case Kind::Exclude: inspectExclude(*doc_.exclude(id)); break;
    case Kind::Pair: inspectPair(*doc_.pair(id)); break;
    case Kind::Equality: inspectEquality(*doc_.equality(id)); break;
    case Kind::Tendon: inspectTendon(*doc_.tendon(id)); break;
    default: inspectWorld(); break;
  }
  ImGui::PopID();
  ImGui::End();
}

}  // namespace mjf
