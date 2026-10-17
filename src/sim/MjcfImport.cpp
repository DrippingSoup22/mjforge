#include "sim/MjcfImport.h"

#include <cmath>
#include <filesystem>
#include <functional>
#include <unordered_map>

#include "sim/MujocoBridge.h"

namespace fs = std::filesystem;

namespace mjf {

namespace {

struct SpecHolder {
  mjSpec* s = nullptr;
  ~SpecHolder() {
    if (s) mj_deleteSpec(s);
  }
};

Vec3 v3(const mjtNum* v) { return {v[0], v[1], v[2]}; }
Vec3 v3f(const float* v) { return {v[0], v[1], v[2]}; }
Quat q4(const mjtNum* q) { return {q[0], q[1], q[2], q[3]}; }
Color c4(const float* c) { return {c[0], c[1], c[2], c[3]}; }
Vec2 v2(const mjtNum* v) { return {v[0], v[1]}; }
Vec5 v5(const mjtNum* v) { return Vec5{{v[0], v[1], v[2], v[3], v[4]}}; }

std::string objName(const mjModel* m, mjtObj type, int id) {
  const char* n = mj_id2name(m, type, id);
  return n ? n : "";
}

std::string specString(const mjString* s) {
  const char* c = s ? mjs_getString(s) : nullptr;
  return c ? c : "";
}

// Visits every spec element of a type with its compiled index.
template <class F>
void forEachSpec(mjSpec* spec, mjtObj type, F&& f) {
  for (mjsElement* el = mjs_firstElement(spec, type); el; el = mjs_nextElement(spec, el)) {
    int id = mjs_getId(el);
    if (id >= 0) f(el, id);
  }
}

bool sensorTypeFrom(int t, SensorType* out) {
  switch (t) {
    case mjSENS_TOUCH: *out = SensorType::Touch; return true;
    case mjSENS_ACCELEROMETER: *out = SensorType::Accelerometer; return true;
    case mjSENS_VELOCIMETER: *out = SensorType::Velocimeter; return true;
    case mjSENS_GYRO: *out = SensorType::Gyro; return true;
    case mjSENS_FORCE: *out = SensorType::Force; return true;
    case mjSENS_TORQUE: *out = SensorType::Torque; return true;
    case mjSENS_MAGNETOMETER: *out = SensorType::Magnetometer; return true;
    case mjSENS_RANGEFINDER: *out = SensorType::RangeFinder; return true;
    case mjSENS_JOINTPOS: *out = SensorType::JointPos; return true;
    case mjSENS_JOINTVEL: *out = SensorType::JointVel; return true;
    case mjSENS_ACTUATORPOS: *out = SensorType::ActuatorPos; return true;
    case mjSENS_ACTUATORVEL: *out = SensorType::ActuatorVel; return true;
    case mjSENS_ACTUATORFRC: *out = SensorType::ActuatorFrc; return true;
    case mjSENS_JOINTACTFRC: *out = SensorType::JointActuatorFrc; return true;
    case mjSENS_BALLQUAT: *out = SensorType::BallQuat; return true;
    case mjSENS_BALLANGVEL: *out = SensorType::BallAngVel; return true;
    case mjSENS_JOINTLIMITPOS: *out = SensorType::JointLimitPos; return true;
    case mjSENS_JOINTLIMITVEL: *out = SensorType::JointLimitVel; return true;
    case mjSENS_JOINTLIMITFRC: *out = SensorType::JointLimitFrc; return true;
    case mjSENS_FRAMEPOS: *out = SensorType::FramePos; return true;
    case mjSENS_FRAMEQUAT: *out = SensorType::FrameQuat; return true;
    case mjSENS_FRAMEXAXIS: *out = SensorType::FrameXAxis; return true;
    case mjSENS_FRAMEYAXIS: *out = SensorType::FrameYAxis; return true;
    case mjSENS_FRAMEZAXIS: *out = SensorType::FrameZAxis; return true;
    case mjSENS_FRAMELINVEL: *out = SensorType::FrameLinVel; return true;
    case mjSENS_FRAMEANGVEL: *out = SensorType::FrameAngVel; return true;
    case mjSENS_FRAMELINACC: *out = SensorType::FrameLinAcc; return true;
    case mjSENS_FRAMEANGACC: *out = SensorType::FrameAngAcc; return true;
    case mjSENS_SUBTREECOM: *out = SensorType::SubtreeCom; return true;
    case mjSENS_SUBTREELINVEL: *out = SensorType::SubtreeLinVel; return true;
    case mjSENS_SUBTREEANGMOM: *out = SensorType::SubtreeAngMom; return true;
    case mjSENS_CLOCK: *out = SensorType::Clock; return true;
    case mjSENS_E_POTENTIAL: *out = SensorType::EPotential; return true;
    case mjSENS_E_KINETIC: *out = SensorType::EKinetic; return true;
    default: return false;
  }
}

bool frameObjFrom(int objtype, FrameObj* out) {
  switch (objtype) {
    case mjOBJ_BODY: *out = FrameObj::Body; return true;
    case mjOBJ_XBODY: *out = FrameObj::XBody; return true;
    case mjOBJ_GEOM: *out = FrameObj::Geom; return true;
    case mjOBJ_SITE: *out = FrameObj::Site; return true;
    case mjOBJ_CAMERA: *out = FrameObj::Camera; return true;
    default: return false;
  }
}

class Builder {
 public:
  Builder(mjSpec* spec, const mjModel* m, const std::string& baseDir, Document& doc, ImportReport& rep)
      : spec_(spec), m_(m), baseDir_(baseDir), doc_(doc), rep_(rep) {}

  void build() {
    doc_ = Document();
    std::string name = specString(spec_->modelname);
    doc_.modelName = name.empty() ? "imported" : name;
    options();
    assets();
    bodies();
    joints();
    geoms();
    sitesCamerasLights();
    actuators();
    sensors();
    excludes();
    keyframes();
    roles();
    unsupported();
    doc_.touch();
  }

 private:
  void note(const std::string& s) { rep_.notes.push_back(s); }

  void options() {
    const mjOption& o = m_->opt;
    OptionSettings& d = doc_.option;
    d.timestep = o.timestep;
    d.gravity = v3(o.gravity);
    d.wind = v3(o.wind);
    d.magnetic = v3(o.magnetic);
    d.density = o.density;
    d.viscosity = o.viscosity;
    d.impratio = o.impratio;
    d.integrator = Integrator(o.integrator);
    d.cone = Cone(o.cone);
    d.jacobian = Jacobian(o.jacobian);
    d.solver = Solver(o.solver);
    d.iterations = o.iterations;
    d.tolerance = o.tolerance;
    d.ls_iterations = o.ls_iterations;
    d.ls_tolerance = o.ls_tolerance;
    d.noslip_iterations = o.noslip_iterations;
    d.ccd_iterations = o.ccd_iterations;
    auto off = [&](int flag) { return (o.disableflags & flag) == 0; };
    auto on = [&](int flag) { return (o.enableflags & flag) != 0; };
    d.constraint = off(mjDSBL_CONSTRAINT);
    d.equality = off(mjDSBL_EQUALITY);
    d.frictionloss = off(mjDSBL_FRICTIONLOSS);
    d.limit = off(mjDSBL_LIMIT);
    d.contact = off(mjDSBL_CONTACT);
    d.spring = off(mjDSBL_SPRING);
    d.damper = off(mjDSBL_DAMPER);
    d.gravityFlag = off(mjDSBL_GRAVITY);
    d.clampctrl = off(mjDSBL_CLAMPCTRL);
    d.warmstart = off(mjDSBL_WARMSTART);
    d.filterparent = off(mjDSBL_FILTERPARENT);
    d.actuation = off(mjDSBL_ACTUATION);
    d.refsafe = off(mjDSBL_REFSAFE);
    d.sensor = off(mjDSBL_SENSOR);
    d.midphase = off(mjDSBL_MIDPHASE);
    d.eulerdamp = off(mjDSBL_EULERDAMP);
    d.autoreset = off(mjDSBL_AUTORESET);
    d.nativeccd = off(mjDSBL_NATIVECCD);
    d.island = off(mjDSBL_ISLAND);
    d.multiccd = off(mjDSBL_MULTICCD);
    d.override_ = on(mjENBL_OVERRIDE);
    d.energy = on(mjENBL_ENERGY);
    d.fwdinv = on(mjENBL_FWDINV);
    d.invdiscrete = on(mjENBL_INVDISCRETE);
    d.sleep = on(mjENBL_SLEEP);

    const mjsCompiler& c = spec_->compiler;
    doc_.compiler.autolimits = c.autolimits;
    doc_.compiler.inertiafromgeom = InertiaFromGeom(c.inertiafromgeom);
    doc_.compiler.boundmass = c.boundmass;
    doc_.compiler.boundinertia = c.boundinertia;
    doc_.compiler.settotalmass = c.settotalmass > 0 ? c.settotalmass : -1;
    doc_.compiler.balanceinertia = c.balanceinertia;

    const mjVisual& v = m_->vis;
    doc_.visual.headlightAmbient = v3f(v.headlight.ambient);
    doc_.visual.headlightDiffuse = v3f(v.headlight.diffuse);
    doc_.visual.headlightSpecular = v3f(v.headlight.specular);
    doc_.visual.azimuth = v.global.azimuth;
    doc_.visual.elevation = v.global.elevation;
    doc_.visual.offwidth = v.global.offwidth;
    doc_.visual.offheight = v.global.offheight;
    doc_.visual.shadowsize = v.quality.shadowsize;
    doc_.visual.haze = c4(v.rgba.haze);
  }

  void assets() {
    texIds_.assign(m_->ntex, kNoId);
    forEachSpec(spec_, mjOBJ_TEXTURE, [&](mjsElement* el, int id) {
      const mjsTexture* t = mjs_asTexture(el);
      Texture x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_TEXTURE, id);
      if (x.name.empty()) x.name = "texture_" + std::to_string(id);
      x.type = m_->tex_type[id] == mjTEXTURE_CUBE ? TextureType::Cube
               : m_->tex_type[id] == mjTEXTURE_SKYBOX ? TextureType::Skybox
                                                      : TextureType::TwoD;
      x.builtin = TextureBuiltin(std::min<int>(t->builtin, int(TextureBuiltin::Flat)));
      x.mark = TextureMark(std::min<int>(t->mark, int(TextureMark::Random)));
      x.rgb1 = v3(t->rgb1);
      x.rgb2 = v3(t->rgb2);
      x.markrgb = v3(t->markrgb);
      x.random = t->random;
      x.width = t->width > 0 ? t->width : m_->tex_width[id];
      x.height = t->height > 0 ? t->height : m_->tex_height[id];
      if (x.builtin == TextureBuiltin::None) {
        std::string file = specString(t->file);
        if (file.empty()) {
          note("Texture '" + x.name + "' uses per-face files and was replaced by a flat color");
          x.builtin = TextureBuiltin::Flat;
        } else {
          fs::path p = fs::path(file);
          if (p.is_relative()) p = fs::path(baseDir_) / specString(spec_->compiler.texturedir) / p;
          x.file = p.lexically_normal().string();
        }
      }
      texIds_[id] = x.id;
      doc_.textures.push_back(x);
    });
    matIds_.assign(m_->nmat, kNoId);
    for (int i = 0; i < m_->nmat; ++i) {
      Material x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_MATERIAL, i);
      if (x.name.empty()) x.name = "material_" + std::to_string(i);
      x.rgba = c4(m_->mat_rgba + 4 * i);
      int tex = m_->mat_texid[i * mjNTEXROLE + mjTEXROLE_RGB];
      if (tex < 0) tex = m_->mat_texid[i * mjNTEXROLE];  // older files: user role
      if (tex >= 0 && tex < m_->ntex) x.texture = texIds_[tex];
      x.texrepeat = {m_->mat_texrepeat[2 * i], m_->mat_texrepeat[2 * i + 1]};
      x.texuniform = m_->mat_texuniform[i];
      x.emission = m_->mat_emission[i];
      x.specular = m_->mat_specular[i];
      x.shininess = m_->mat_shininess[i];
      x.reflectance = m_->mat_reflectance[i];
      matIds_[i] = x.id;
      doc_.materials.push_back(x);
    }
    hfIds_.assign(m_->nhfield, kNoId);
    for (int i = 0; i < m_->nhfield; ++i) {
      Hfield h;
      h.id = doc_.newId();
      h.name = objName(m_, mjOBJ_HFIELD, i);
      if (h.name.empty()) h.name = "hfield_" + std::to_string(i);
      h.nrow = m_->hfield_nrow[i];
      h.ncol = m_->hfield_ncol[i];
      const mjtNum* hs = m_->hfield_size + 4 * i;
      h.size = Vec4(hs[0], hs[1], hs[2], hs[3]);
      // The compiled data starts at the -Y edge; documents use MJCF order (first row = +Y edge).
      auto data = std::make_shared<std::vector<float>>(size_t(h.nrow) * h.ncol);
      const float* src = m_->hfield_data + m_->hfield_adr[i];
      for (int r = 0; r < h.nrow; ++r)
        for (int c = 0; c < h.ncol; ++c) (*data)[size_t(r) * h.ncol + c] = src[size_t(h.nrow - 1 - r) * h.ncol + c];
      h.data = data;
      hfIds_[i] = h.id;
      doc_.hfields.push_back(h);
    }
  }

  void bodies() {
    bodyIds_.assign(m_->nbody, kNoId);
    bodyIds_[0] = kWorldId;
    std::vector<uint8_t> explicitInertial(m_->nbody, 0);
    forEachSpec(spec_, mjOBJ_BODY, [&](mjsElement* el, int id) {
      if (id < m_->nbody) explicitInertial[id] = mjs_asBody(el)->explicitinertial;
    });
    // Mesh / SDF geoms become boxes; keep the exact compiled inertia of their bodies.
    for (int g = 0; g < m_->ngeom; ++g)
      if (m_->geom_type[g] == mjGEOM_MESH || m_->geom_type[g] == mjGEOM_SDF) explicitInertial[m_->geom_bodyid[g]] = 1;
    for (int b = 1; b < m_->nbody; ++b) {
      Body x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_BODY, b);
      x.parent = bodyIds_[m_->body_parentid[b]];
      x.pos = v3(m_->body_pos + 3 * b);
      x.quat = q4(m_->body_quat + 4 * b);
      x.mocap = m_->body_mocapid[b] >= 0;
      x.gravcomp = m_->body_gravcomp[b];
      if (explicitInertial[b] && m_->body_mass[b] > 0) {
        x.hasInertial = true;
        x.mass = m_->body_mass[b];
        x.ipos = v3(m_->body_ipos + 3 * b);
        x.iquat = q4(m_->body_iquat + 4 * b);
        x.diaginertia = v3(m_->body_inertia + 3 * b);
      }
      bodyIds_[b] = x.id;
      doc_.bodies.push_back(x);
    }
  }

  void joints() {
    jointIds_.assign(m_->njnt, kNoId);
    for (int j = 0; j < m_->njnt; ++j) {
      Joint x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_JOINT, j);
      x.body = bodyIds_[m_->jnt_bodyid[j]];
      x.type = JointType(m_->jnt_type[j]);
      x.pos = v3(m_->jnt_pos + 3 * j);
      x.axis = v3(m_->jnt_axis + 3 * j);
      x.limited = m_->jnt_limited[j];
      x.range = v2(m_->jnt_range + 2 * j);
      int qa = m_->jnt_qposadr[j], da = m_->jnt_dofadr[j];
      if (x.type == JointType::Hinge || x.type == JointType::Slide) {
        x.ref = m_->qpos0[qa];
        x.springref = m_->qpos_spring[qa];
      }
      x.stiffness = m_->jnt_stiffness[j];
      x.damping = m_->dof_damping[da];
      x.armature = m_->dof_armature[da];
      x.frictionloss = m_->dof_frictionloss[da];
      x.margin = m_->jnt_margin[j];
      x.solreflimit = v2(m_->jnt_solref + mjNREF * j);
      x.solimplimit = v5(m_->jnt_solimp + mjNIMP * j);
      x.solreffriction = v2(m_->dof_solref + mjNREF * da);
      x.solimpfriction = v5(m_->dof_solimp + mjNIMP * da);
      x.actuatorfrclimited = m_->jnt_actfrclimited[j];
      x.actuatorfrcrange = v2(m_->jnt_actfrcrange + 2 * j);
      x.actuatorgravcomp = m_->jnt_actgravcomp[j];
      x.group = m_->jnt_group[j];
      jointIds_[j] = x.id;
      doc_.joints.push_back(x);
    }
  }

  void geoms() {
    std::vector<double> density(m_->ngeom, 1000.0), mass(m_->ngeom, -1.0);
    std::vector<uint8_t> shell(m_->ngeom, 0);
    forEachSpec(spec_, mjOBJ_GEOM, [&](mjsElement* el, int id) {
      if (id >= m_->ngeom) return;
      const mjsGeom* g = mjs_asGeom(el);
      density[id] = g->density;
      mass[id] = std::isnan(g->mass) ? -1.0 : g->mass;
      shell[id] = g->typeinertia == mjINERTIA_SHELL;
    });
    geomIds_.assign(m_->ngeom, kNoId);
    for (int g = 0; g < m_->ngeom; ++g) {
      Geom x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_GEOM, g);
      x.body = bodyIds_[m_->geom_bodyid[g]];
      int type = m_->geom_type[g];
      x.size = v3(m_->geom_size + 3 * g);
      x.pos = v3(m_->geom_pos + 3 * g);
      x.quat = q4(m_->geom_quat + 4 * g);
      if (type == mjGEOM_MESH || type == mjGEOM_SDF) {
        // Approximate by the geom's bounding box (the body keeps its exact inertia).
        const mjtNum* aabb = m_->geom_aabb + 6 * g;
        x.type = GeomType::Box;
        x.pos = x.pos + rotate(x.quat, v3(aabb));
        x.size = {std::max(1e-4, aabb[3]), std::max(1e-4, aabb[4]), std::max(1e-4, aabb[5])};
        ++rep_.approximatedGeoms;
      } else {
        x.type = GeomType(type);
      }
      x.contype = m_->geom_contype[g];
      x.conaffinity = m_->geom_conaffinity[g];
      x.condim = m_->geom_condim[g];
      x.priority = m_->geom_priority[g];
      x.friction = v3(m_->geom_friction + 3 * g);
      x.solmix = m_->geom_solmix[g];
      x.solref = v2(m_->geom_solref + mjNREF * g);
      x.solimp = v5(m_->geom_solimp + mjNIMP * g);
      x.margin = m_->geom_margin[g];
      x.gap = m_->geom_gap[g];
      x.density = density[g];
      x.mass = mass[g];
      x.shellinertia = shell[g];
      x.rgba = c4(m_->geom_rgba + 4 * g);
      int mat = m_->geom_matid[g];
      if (mat >= 0 && mat < m_->nmat) x.material = matIds_[mat];
      x.group = m_->geom_group[g];
      if (type == mjGEOM_HFIELD && m_->geom_dataid[g] >= 0) x.hfield = hfIds_[m_->geom_dataid[g]];
      geomIds_[g] = x.id;
      doc_.geoms.push_back(x);
    }
    if (rep_.approximatedGeoms > 0)
      note(std::to_string(rep_.approximatedGeoms) +
           " mesh geom(s) were replaced by their bounding boxes (bodies keep their exact mass and inertia)");
  }

  void sitesCamerasLights() {
    siteIds_.assign(m_->nsite, kNoId);
    for (int s = 0; s < m_->nsite; ++s) {
      Site x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_SITE, s);
      x.body = bodyIds_[m_->site_bodyid[s]];
      int type = m_->site_type[s];
      x.type = type == mjGEOM_MESH || type == mjGEOM_SDF || type == mjGEOM_HFIELD ? GeomType::Box : GeomType(type);
      x.size = v3(m_->site_size + 3 * s);
      x.pos = v3(m_->site_pos + 3 * s);
      x.quat = q4(m_->site_quat + 4 * s);
      x.rgba = c4(m_->site_rgba + 4 * s);
      int mat = m_->site_matid[s];
      if (mat >= 0 && mat < m_->nmat) x.material = matIds_[mat];
      x.group = m_->site_group[s];
      siteIds_[s] = x.id;
      doc_.sites.push_back(x);
    }
    camIds_.assign(m_->ncam, kNoId);
    for (int c = 0; c < m_->ncam; ++c) {
      Camera x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_CAMERA, c);
      x.body = bodyIds_[m_->cam_bodyid[c]];
      x.pos = v3(m_->cam_pos + 3 * c);
      x.quat = q4(m_->cam_quat + 4 * c);
      x.fovy = m_->cam_fovy[c];
      x.mode = CameraMode(std::min<int>(m_->cam_mode[c], int(CameraMode::TargetBodyCom)));
      if (m_->cam_targetbodyid[c] >= 0) x.target = bodyIds_[m_->cam_targetbodyid[c]];
      camIds_[c] = x.id;
      doc_.cameras.push_back(x);
    }
    for (int l = 0; l < m_->nlight; ++l) {
      Light x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_LIGHT, l);
      x.body = bodyIds_[m_->light_bodyid[l]];
      x.type = m_->light_type[l] == mjLIGHT_DIRECTIONAL ? LightType::Directional
               : m_->light_type[l] == mjLIGHT_POINT     ? LightType::Point
                                                        : LightType::Spot;
      x.pos = v3(m_->light_pos + 3 * l);
      x.dir = v3(m_->light_dir + 3 * l);
      x.castshadow = m_->light_castshadow[l];
      x.active = m_->light_active[l];
      x.diffuse = v3f(m_->light_diffuse + 3 * l);
      x.ambient = v3f(m_->light_ambient + 3 * l);
      x.specular = v3f(m_->light_specular + 3 * l);
      x.attenuation = v3f(m_->light_attenuation + 3 * l);
      x.cutoff = m_->light_cutoff[l];
      x.exponent = m_->light_exponent[l];
      x.mode = CameraMode(std::min<int>(m_->light_mode[l], int(CameraMode::TargetBodyCom)));
      if (m_->light_targetbodyid[l] >= 0) x.target = bodyIds_[m_->light_targetbodyid[l]];
      doc_.lights.push_back(x);
    }
  }

  void actuators() {
    std::vector<const mjsActuator*> specs(m_->nu, nullptr);
    forEachSpec(spec_, mjOBJ_ACTUATOR, [&](mjsElement* el, int id) {
      if (id < m_->nu) specs[id] = mjs_asActuator(el);
    });
    actIds_.assign(m_->nu, kNoId);
    int skipped = 0;
    for (int i = 0; i < m_->nu; ++i) {
      const mjsActuator* s = specs[i];
      Actuator x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_ACTUATOR, i);
      int trn = m_->actuator_trntype[i];
      int id0 = m_->actuator_trnid[2 * i], id1 = m_->actuator_trnid[2 * i + 1];
      bool ok = true;
      switch (trn) {
        case mjTRN_JOINT:
        case mjTRN_JOINTINPARENT:
          x.trn = TrnType(trn);
          x.target = id0 >= 0 ? jointIds_[id0] : kNoId;
          break;
        case mjTRN_SITE:
          x.trn = TrnType::Site;
          x.target = id0 >= 0 ? siteIds_[id0] : kNoId;
          if (id1 >= 0) x.refsite = siteIds_[id1];
          break;
        case mjTRN_BODY:
          x.trn = TrnType::Body;
          x.target = id0 >= 0 ? bodyIds_[id0] : kNoId;
          break;
        case mjTRN_SLIDERCRANK:
          x.trn = TrnType::SliderCrank;
          x.target = id0 >= 0 ? siteIds_[id0] : kNoId;
          if (id1 >= 0) x.slidersite = siteIds_[id1];
          x.cranklength = m_->actuator_cranklength[i];
          break;
        default: ok = false; break;  // tendons are not supported yet
      }
      if (!ok || !x.target || !s) {
        ++skipped;
        continue;
      }
      for (int k = 0; k < 6; ++k) x.gear[k] = m_->actuator_gear[6 * i + k];
      x.ctrllimited = m_->actuator_ctrllimited[i];
      x.ctrlrange = v2(m_->actuator_ctrlrange + 2 * i);
      x.forcelimited = m_->actuator_forcelimited[i];
      x.forcerange = v2(m_->actuator_forcerange + 2 * i);
      x.actlimited = m_->actuator_actlimited[i];
      x.actrange = v2(m_->actuator_actrange + 2 * i);
      x.lengthrange = v2(m_->actuator_lengthrange + 2 * i);
      x.group = m_->actuator_group[i];
      x.actearly = s->actearly;
      const double* g = s->gainprm;
      const double* b = s->biasprm;
      const double* d = s->dynprm;
      auto servoDamping = [&](Actuator& a) {
        // A positive biasprm[2] is a damping ratio (MuJoCo converts it at compile time).
        if (b[2] > 0) a.dampratio = b[2];
        else a.kv = -b[2];
      };
      switch (s->type) {
        case mjACTUATOR_MOTOR:
          x.type = ActuatorType::Motor;
          if (std::fabs(g[0] - 1.0) > 1e-12) generalFrom(x, s);
          break;
        case mjACTUATOR_POSITION:
          x.type = ActuatorType::Position;
          x.kp = g[0];
          servoDamping(x);
          if (s->dyntype == mjDYN_FILTEREXACT) x.timeconst = d[0];
          break;
        case mjACTUATOR_VELOCITY:
          x.type = ActuatorType::Velocity;
          x.kv = g[0];
          break;
        case mjACTUATOR_INTVELOCITY:
          x.type = ActuatorType::IntVelocity;
          x.kp = g[0];
          servoDamping(x);
          break;
        case mjACTUATOR_DAMPER:
          x.type = ActuatorType::Damper;
          x.kv = -g[2];
          break;
        case mjACTUATOR_CYLINDER:
          x.type = ActuatorType::Cylinder;
          x.cylTimeconst = d[0];
          x.area = g[0];
          x.cylBias = {b[0], b[1], b[2]};
          break;
        case mjACTUATOR_MUSCLE:
          x.type = ActuatorType::Muscle;
          x.muscleTimeconst = {d[0], d[1]};
          x.tausmooth = d[2];
          x.muscleRange = {g[0], g[1]};
          x.muscleForce = g[2];
          x.muscleScale = g[3];
          x.lmin = g[4];
          x.lmax = g[5];
          x.vmax = g[6];
          x.fpmax = g[7];
          x.fvmax = g[8];
          break;
        case mjACTUATOR_ADHESION:
          x.type = ActuatorType::Adhesion;
          x.adhesionGain = g[0];
          break;
        default:
          if (!generalFrom(x, s)) {
            ++skipped;
            continue;
          }
          break;
      }
      actIds_[i] = x.id;
      doc_.actuators.push_back(x);
    }
    if (skipped)
      note(std::to_string(skipped) + " actuator(s) with tendon transmissions or unsupported types were not imported");
  }

  static bool generalFrom(Actuator& x, const mjsActuator* s) {
    if (s->dyntype > mjDYN_MUSCLE || s->gaintype > mjGAIN_MUSCLE || s->biastype > mjBIAS_MUSCLE) return false;
    x.type = ActuatorType::General;
    x.dyntype = DynType(s->dyntype);
    x.gaintype = GainType(s->gaintype);
    x.biastype = BiasType(s->biastype);
    for (int k = 0; k < 10; ++k) {
      x.dynprm[k] = s->dynprm[k];
      x.gainprm[k] = s->gainprm[k];
      x.biasprm[k] = s->biasprm[k];
    }
    return true;
  }

  Id objectId(int objtype, int id) const {
    if (id < 0) return kNoId;
    switch (objtype) {
      case mjOBJ_BODY:
      case mjOBJ_XBODY: return id < int(bodyIds_.size()) ? bodyIds_[id] : kNoId;
      case mjOBJ_JOINT: return id < int(jointIds_.size()) ? jointIds_[id] : kNoId;
      case mjOBJ_GEOM: return id < int(geomIds_.size()) ? geomIds_[id] : kNoId;
      case mjOBJ_SITE: return id < int(siteIds_.size()) ? siteIds_[id] : kNoId;
      case mjOBJ_CAMERA: return id < int(camIds_.size()) ? camIds_[id] : kNoId;
      case mjOBJ_ACTUATOR: return id < int(actIds_.size()) ? actIds_[id] : kNoId;
      default: return kNoId;
    }
  }

  void sensors() {
    int skipped = 0;
    for (int i = 0; i < m_->nsensor; ++i) {
      Sensor x;
      if (!sensorTypeFrom(m_->sensor_type[i], &x.type)) {
        ++skipped;
        continue;
      }
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_SENSOR, i);
      int ot = m_->sensor_objtype[i], rt = m_->sensor_reftype[i];
      SensorObj kind = sensorInfo(x.type).obj;
      if (kind == SensorObj::Frame) {
        if (!frameObjFrom(ot, &x.objtype)) {
          ++skipped;
          continue;
        }
        if (m_->sensor_refid[i] >= 0 && frameObjFrom(rt, &x.reftype)) x.ref = objectId(rt, m_->sensor_refid[i]);
      }
      if (kind != SensorObj::None) {
        x.obj = objectId(ot, m_->sensor_objid[i]);
        if (!x.obj) {
          ++skipped;
          continue;
        }
      }
      x.noise = m_->sensor_noise[i];
      x.cutoff = m_->sensor_cutoff[i];
      doc_.sensors.push_back(x);
    }
    if (skipped) note(std::to_string(skipped) + " sensor(s) of unsupported types were not imported");
  }

  void excludes() {
    for (int i = 0; i < m_->nexclude; ++i) {
      int sig = m_->exclude_signature[i];
      int b1 = sig >> 16, b2 = sig & 0xFFFF;
      if (b1 >= m_->nbody || b2 >= m_->nbody) continue;
      Exclude x;
      x.id = doc_.newId();
      x.name = objName(m_, mjOBJ_EXCLUDE, i);
      x.body1 = bodyIds_[b1];
      x.body2 = bodyIds_[b2];
      doc_.excludes.push_back(x);
    }
  }

  void keyframes() {
    for (int k = 0; k < m_->nkey; ++k) {
      Keyframe x;
      x.name = objName(m_, mjOBJ_KEY, k);
      x.time = m_->key_time[k];
      const mjtNum* qpos = m_->key_qpos + k * m_->nq;
      const mjtNum* qvel = m_->key_qvel + k * m_->nv;
      for (int j = 0; j < m_->njnt; ++j) {
        int t = m_->jnt_type[j];
        int nq = t == mjJNT_FREE ? 7 : t == mjJNT_BALL ? 4 : 1;
        int nv = t == mjJNT_FREE ? 6 : t == mjJNT_BALL ? 3 : 1;
        const mjtNum* q = qpos + m_->jnt_qposadr[j];
        const mjtNum* v = qvel + m_->jnt_dofadr[j];
        x.qpos.push_back({jointIds_[j], std::vector<double>(q, q + nq)});
        bool moving = false;
        for (int i = 0; i < nv; ++i) moving |= v[i] != 0;
        if (moving) x.qvel.push_back({jointIds_[j], std::vector<double>(v, v + nv)});
      }
      for (int i = 0; i < m_->nu; ++i) {
        double c = m_->key_ctrl[k * m_->nu + i];
        if (c != 0 && actIds_[i]) x.ctrl.push_back({actIds_[i], c});
      }
      doc_.keyframes.push_back(std::move(x));
    }
  }

  // Top-level bodies with joints anywhere below them are models; the rest is environment.
  void roles() {
    std::vector<uint8_t> moving(m_->nbody, 0);
    for (int b = m_->nbody - 1; b > 0; --b) {
      if (m_->body_jntnum[b] > 0 || m_->body_mocapid[b] >= 0) moving[b] = 1;
      if (moving[b]) moving[m_->body_parentid[b]] = 1;
    }
    for (int b = 1; b < m_->nbody; ++b) {
      if (m_->body_parentid[b] != 0) continue;
      if (Body* x = doc_.body(bodyIds_[b])) x->role = moving[b] && m_->body_mocapid[b] < 0 ? Role::Model : Role::Environment;
    }
  }

  void unsupported() {
    auto count = [&](int n, const char* what) {
      if (n > 0) note(std::to_string(n) + " " + what + " not imported (not supported by the editor yet)");
    };
    count(m_->ntendon, "tendon(s)");
    count(m_->neq, "equality constraint(s)");
    count(m_->npair, "explicit contact pair(s)");
    count(m_->nflex, "flex object(s)");
    count(m_->nskin, "skin(s)");
    count(m_->nplugin, "plugin instance(s)");
  }

  mjSpec* spec_;
  const mjModel* m_;
  std::string baseDir_;
  Document& doc_;
  ImportReport& rep_;
  std::vector<Id> texIds_, matIds_, hfIds_, bodyIds_, jointIds_, geomIds_, siteIds_, camIds_, actIds_;
};

bool importSpec(mjSpec* spec, const std::string& baseDir, Document& out, std::string* error, ImportReport* report) {
  ImportReport local;
  ImportReport& rep = report ? *report : local;
  rep = {};
  ModelPtr model;
  try {
    model = makeModelPtr(mj_compile(spec, nullptr));
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
  if (!model) {
    if (error) *error = mjs_getError(spec);
    return false;
  }
  Builder(spec, model.get(), baseDir, out, rep).build();
  return true;
}

}  // namespace

bool importMjcfFile(const std::string& path, Document& out, std::string* error, ImportReport* report) {
  char err[1024] = "";
  SpecHolder spec;
  try {
    spec.s = mj_parseXML(path.c_str(), nullptr, err, sizeof err);
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
  if (!spec.s) {
    if (error) *error = err[0] ? err : "could not parse the file";
    return false;
  }
  return importSpec(spec.s, fs::path(path).parent_path().string(), out, error, report);
}

bool importMjcfString(const std::string& xml, Document& out, std::string* error, ImportReport* report) {
  char err[1024] = "";
  SpecHolder spec;
  try {
    spec.s = mj_parseXMLString(xml.c_str(), nullptr, err, sizeof err);
  } catch (const std::exception& e) {
    if (error) *error = e.what();
    return false;
  }
  if (!spec.s) {
    if (error) *error = err[0] ? err : "could not parse the XML";
    return false;
  }
  return importSpec(spec.s, "", out, error, report);
}

}  // namespace mjf
