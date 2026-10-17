#include "core/Document.h"

#include <algorithm>
#include <cctype>
#include <functional>
#include <iterator>

namespace mjf {

const char* kindName(Kind k) {
  switch (k) {
    case Kind::Body: return "body";
    case Kind::Geom: return "geom";
    case Kind::Joint: return "joint";
    case Kind::Site: return "site";
    case Kind::Camera: return "camera";
    case Kind::Light: return "light";
    case Kind::Actuator: return "actuator";
    case Kind::Sensor: return "sensor";
    case Kind::Exclude: return "exclude";
    case Kind::Material: return "material";
    case Kind::Texture: return "texture";
    case Kind::Hfield: return "hfield";
    case Kind::Mesh: return "mesh";
    case Kind::Pair: return "contact pair";
    case Kind::Equality: return "equality";
    case Kind::Tendon: return "tendon";
    default: return "none";
  }
}

double actuatorDefaultKp(ActuatorType t) {
  switch (t) {
    case ActuatorType::Position:
    case ActuatorType::IntVelocity: return 1.0;
    default: return 0.0;
  }
}

double actuatorDefaultKv(ActuatorType t) {
  switch (t) {
    case ActuatorType::Velocity:
    case ActuatorType::Damper: return 1.0;
    default: return 0.0;
  }
}

SizeLabels geomSizeLabels(GeomType t) {
  switch (t) {
    case GeomType::Plane: return {{"Half-width X", "Half-width Y", "Grid spacing"}, 3};
    case GeomType::Sphere: return {{"Radius", "", ""}, 1};
    case GeomType::Capsule: return {{"Radius", "Half-length", ""}, 2};
    case GeomType::Cylinder: return {{"Radius", "Half-height", ""}, 2};
    case GeomType::Ellipsoid: return {{"Radius X", "Radius Y", "Radius Z"}, 3};
    case GeomType::Box: return {{"Half-size X", "Half-size Y", "Half-size Z"}, 3};
    default: return {{"", "", ""}, 0};
  }
}

Document::Document() {
  Body world;
  world.id = kWorldId;
  world.name = "world";
  world.role = Role::Environment;
  bodies.push_back(world);
}

const DocIndex& Document::index() const {
  if (indexRevision_ == revision_) return index_;
  DocIndex& ix = index_;
  ix.loc.clear();
  const size_t nb = bodies.size();
  auto resetLists = [nb](std::vector<std::vector<uint32_t>>& v) {
    v.assign(nb, {});
  };
  resetLists(ix.childBodies);
  resetLists(ix.bodyGeoms);
  resetLists(ix.bodyJoints);
  resetLists(ix.bodySites);
  resetLists(ix.bodyCameras);
  resetLists(ix.bodyLights);

  auto addAll = [&](auto& vec, Kind k) {
    for (uint32_t i = 0; i < vec.size(); ++i) ix.loc[vec[i].id] = {k, i};
  };
  addAll(bodies, Kind::Body);
  addAll(geoms, Kind::Geom);
  addAll(joints, Kind::Joint);
  addAll(sites, Kind::Site);
  addAll(cameras, Kind::Camera);
  addAll(lights, Kind::Light);
  addAll(actuators, Kind::Actuator);
  addAll(sensors, Kind::Sensor);
  addAll(excludes, Kind::Exclude);
  addAll(materials, Kind::Material);
  addAll(textures, Kind::Texture);
  addAll(hfields, Kind::Hfield);
  addAll(meshes, Kind::Mesh);
  addAll(pairs, Kind::Pair);
  addAll(equalities, Kind::Equality);
  addAll(tendons, Kind::Tendon);

  auto bodyIdx = [&](Id id) -> int {
    auto it = ix.loc.find(id);
    if (it == ix.loc.end() || it->second.kind != Kind::Body) return -1;
    return int(it->second.index);
  };
  for (uint32_t i = 1; i < nb; ++i) {
    int p = bodyIdx(bodies[i].parent);
    if (p < 0) p = 0;  // orphan bodies are treated as children of the world
    ix.childBodies[p].push_back(i);
  }
  auto attach = [&](auto& vec, std::vector<std::vector<uint32_t>>& lists) {
    for (uint32_t i = 0; i < vec.size(); ++i) {
      int b = bodyIdx(vec[i].body);
      lists[b < 0 ? 0 : b].push_back(i);
    }
  };
  attach(geoms, ix.bodyGeoms);
  attach(joints, ix.bodyJoints);
  attach(sites, ix.bodySites);
  attach(cameras, ix.bodyCameras);
  attach(lights, ix.bodyLights);

  ix.dfsOrder.clear();
  ix.depth.assign(nb, 0);
  std::vector<uint8_t> visited(nb, 0);
  std::function<void(uint32_t, int)> dfs = [&](uint32_t b, int d) {
    if (visited[b]) return;  // guards against accidental cycles
    visited[b] = 1;
    ix.dfsOrder.push_back(b);
    ix.depth[b] = d;
    for (uint32_t c : ix.childBodies[b]) dfs(c, d + 1);
  };
  if (nb > 0) dfs(0, 0);
  indexRevision_ = revision_;
  return index_;
}

Kind Document::kindOf(Id id) const {
  if (id == kNoId) return Kind::None;
  const DocIndex& ix = index();
  auto it = ix.loc.find(id);
  return it == ix.loc.end() ? Kind::None : it->second.kind;
}

int Document::bodyIndex(Id id) const {
  const DocIndex& ix = index();
  auto it = ix.loc.find(id);
  if (it == ix.loc.end() || it->second.kind != Kind::Body) return -1;
  return int(it->second.index);
}

std::string Document::nameOf(Id id) const {
  switch (kindOf(id)) {
    case Kind::Body: return body(id)->name;
    case Kind::Geom: return geom(id)->name;
    case Kind::Joint: return joint(id)->name;
    case Kind::Site: return site(id)->name;
    case Kind::Camera: return camera(id)->name;
    case Kind::Light: return light(id)->name;
    case Kind::Actuator: return actuator(id)->name;
    case Kind::Sensor: return sensor(id)->name;
    case Kind::Exclude: return exclude(id)->name;
    case Kind::Material: return material(id)->name;
    case Kind::Texture: return texture(id)->name;
    case Kind::Hfield: return hfield(id)->name;
    case Kind::Mesh: return mesh(id)->name;
    case Kind::Pair: return pair(id)->name;
    case Kind::Equality: return equality(id)->name;
    case Kind::Tendon: return tendon(id)->name;
    default: return "";
  }
}

std::string Document::displayName(Id id) const {
  std::string n = nameOf(id);
  if (!n.empty()) return n;
  switch (kindOf(id)) {
    case Kind::Body: return "body";
    case Kind::Geom: return std::string(enumXml(kGeomTypes, geom(id)->type)) + " geom";
    case Kind::Joint: return std::string(enumXml(kJointTypes, joint(id)->type)) + " joint";
    case Kind::Site: return "site";
    case Kind::Camera: return "camera";
    case Kind::Light: return std::string(enumXml(kLightTypes, light(id)->type)) + " light";
    case Kind::Actuator: {
      const Actuator* a = actuator(id);
      std::string t = nameOf(a->target);
      return std::string(toXml(a->type)) + (t.empty() ? "" : " on " + t);
    }
    case Kind::Sensor: {
      const Sensor* s = sensor(id);
      std::string o = nameOf(s->obj);
      return std::string(sensorInfo(s->type).xml) + (o.empty() ? "" : " on " + o);
    }
    case Kind::Exclude: {
      const Exclude* e = exclude(id);
      return "exclude " + displayName(e->body1) + " / " + displayName(e->body2);
    }
    case Kind::Material: return "material";
    case Kind::Texture: return "texture";
    case Kind::Hfield: return "height field";
    case Kind::Mesh: return "mesh";
    case Kind::Pair: {
      const ContactPair* c = pair(id);
      return "pair " + displayName(c->geom1) + " / " + displayName(c->geom2);
    }
    case Kind::Equality: {
      const Equality* e = equality(id);
      std::string s = std::string(enumXml(kEqualityTypes, e->type)) + " " + displayName(e->obj1);
      if (e->obj2) s += " / " + displayName(e->obj2);
      return s;
    }
    case Kind::Tendon: return std::string(enumXml(kTendonTypes, tendon(id)->type)) + " tendon";
    default: return "";
  }
}

void Document::setName(Id id, const std::string& name) {
  switch (kindOf(id)) {
    case Kind::Body: if (id != kWorldId) body(id)->name = name; break;
    case Kind::Geom: geom(id)->name = name; break;
    case Kind::Joint: joint(id)->name = name; break;
    case Kind::Site: site(id)->name = name; break;
    case Kind::Camera: camera(id)->name = name; break;
    case Kind::Light: light(id)->name = name; break;
    case Kind::Actuator: actuator(id)->name = name; break;
    case Kind::Sensor: sensor(id)->name = name; break;
    case Kind::Exclude: exclude(id)->name = name; break;
    case Kind::Material: material(id)->name = name; break;
    case Kind::Texture: texture(id)->name = name; break;
    case Kind::Hfield: hfield(id)->name = name; break;
    case Kind::Mesh: mesh(id)->name = name; break;
    case Kind::Pair: pair(id)->name = name; break;
    case Kind::Equality: equality(id)->name = name; break;
    case Kind::Tendon: tendon(id)->name = name; break;
    default: break;
  }
}

Id Document::ownerBody(Id id) const {
  switch (kindOf(id)) {
    case Kind::Body: return body(id)->parent;
    case Kind::Geom: return geom(id)->body;
    case Kind::Joint: return joint(id)->body;
    case Kind::Site: return site(id)->body;
    case Kind::Camera: return camera(id)->body;
    case Kind::Light: return light(id)->body;
    default: return kNoId;
  }
}

Id Document::rootOf(Id b) const {
  if (b == kWorldId) return kWorldId;
  const Body* cur = body(b);
  int guard = 0;
  while (cur && cur->parent != kWorldId && cur->parent != kNoId && guard++ < 100000) cur = body(cur->parent);
  return cur ? cur->id : kNoId;
}

bool Document::isAncestor(Id ancestor, Id b) const {
  int guard = 0;
  while (b != kNoId && guard++ < 100000) {
    if (b == ancestor) return true;
    const Body* cur = body(b);
    if (!cur) return false;
    b = cur->parent;
  }
  return false;
}

Role Document::roleOf(Id b) const {
  Id root = rootOf(b);
  if (root == kWorldId || root == kNoId) return Role::Environment;
  return body(root)->role;
}

std::vector<Id> Document::subtree(Id b) const {
  std::vector<Id> out;
  int bi = bodyIndex(b);
  if (bi < 0) return out;
  const DocIndex& ix = index();
  std::vector<uint32_t> stack{uint32_t(bi)};
  std::vector<char> visited(bodies.size(), 0);  // a damaged parent cycle must not loop forever
  while (!stack.empty()) {
    uint32_t cur = stack.back();
    stack.pop_back();
    if (visited[cur]) continue;
    visited[cur] = 1;
    out.push_back(bodies[cur].id);
    const auto& ch = ix.childBodies[cur];
    for (auto it = ch.rbegin(); it != ch.rend(); ++it) stack.push_back(*it);
  }
  return out;
}

int sensorDim(const Sensor& s) {
  int per = 0;
  if (s.type == SensorType::RangeFinder) {
    for (int i = 0; i < int(std::size(kRayData)); ++i)
      if (s.raydata & (1 << i)) per += kRayData[i].dim;
    return std::max(1, per);
  }
  if (s.type != SensorType::Contact) return sensorInfo(s.type).dim;
  for (int i = 0; i < int(std::size(kContactData)); ++i)
    if (s.condata & (1 << i)) per += kContactData[i].dim;
  return std::max(1, s.connum) * std::max(1, per);
}

bool Document::isTreeRoot(Id b) const {
  if (b == kWorldId || bodyDofs(b) == 0) return false;
  for (const Body* x = body(b); x && x->parent != kWorldId && x->parent != kNoId;) {
    x = body(x->parent);
    if (x && bodyDofs(x->id) > 0) return false;
  }
  return true;
}

int Document::bodyDofs(Id b) const {
  int bi = bodyIndex(b);
  if (bi < 0) return 0;
  int n = 0;
  for (uint32_t ji : index().bodyJoints[bi]) {
    switch (joints[ji].type) {
      case JointType::Free: n += 6; break;
      case JointType::Ball: n += 3; break;
      default: n += 1; break;
    }
  }
  return n;
}

bool Document::nameTaken(Kind kind, const std::string& name, Id except) const {
  if (name.empty()) return false;
  auto check = [&](const auto& vec) {
    for (const auto& e : vec)
      if (e.id != except && e.name == name) return true;
    return false;
  };
  switch (kind) {
    case Kind::Body: return check(bodies);
    case Kind::Geom: return check(geoms);
    case Kind::Joint: return check(joints);
    case Kind::Site: return check(sites);
    case Kind::Camera: return check(cameras);
    case Kind::Light: return check(lights);
    case Kind::Actuator: return check(actuators);
    case Kind::Sensor: return check(sensors);
    case Kind::Exclude: return check(excludes);
    case Kind::Material: return check(materials);
    case Kind::Texture: return check(textures);
    case Kind::Hfield: return check(hfields);
    case Kind::Mesh: return check(meshes);
    case Kind::Pair: return check(pairs);
    case Kind::Equality: return check(equalities);
    case Kind::Tendon: return check(tendons);
    default: return false;
  }
}

std::string Document::uniqueName(Kind kind, const std::string& base_) const {
  std::string base = base_.empty() ? std::string(kindName(kind)) : base_;
  if (!nameTaken(kind, base)) return base;
  // Strip an existing numeric suffix ("leg_3" -> "leg").
  std::string stem = base;
  size_t us = stem.find_last_of('_');
  if (us != std::string::npos && us + 1 < stem.size() &&
      std::all_of(stem.begin() + us + 1, stem.end(), [](unsigned char c) { return std::isdigit(c); }))
    stem = stem.substr(0, us);
  for (int i = 2; i < 1000000; ++i) {
    std::string cand = stem + "_" + std::to_string(i);
    if (!nameTaken(kind, cand)) return cand;
  }
  return stem + "_x";
}

}  // namespace mjf

namespace mjf {

Aabb MeshAsset::bounds() const {
  Aabb box;
  if (!vert) return box;
  for (size_t i = 0; i + 2 < vert->size(); i += 3) box.add({(*vert)[i], (*vert)[i + 1], (*vert)[i + 2]});
  return box;
}

}  // namespace mjf
