#include "core/Kinematics.h"

#include <cmath>
#include <limits>

#include "core/Geometry.h"

namespace mjf {

std::vector<Pose> computeBodyPoses(const Document& doc) {
  const DocIndex& ix = doc.index();
  std::vector<Pose> poses(doc.bodies.size());
  for (uint32_t bi : ix.dfsOrder) {
    const Body& b = doc.bodies[bi];
    if (bi == 0) {
      poses[bi] = {};
      continue;
    }
    int pi = doc.bodyIndex(b.parent);
    Pose parent = pi >= 0 ? poses[pi] : Pose{};
    poses[bi] = parent * Pose{b.pos, b.quat};
  }
  return poses;
}

std::vector<Pose> computeBodyPoses(const Document& doc, const Keyframe* key) {
  if (!key) return computeBodyPoses(doc);
  const DocIndex& ix = doc.index();
  std::vector<Pose> poses(doc.bodies.size());
  for (uint32_t bi : ix.dfsOrder) {
    if (bi == 0) continue;
    const Body& b = doc.bodies[bi];
    int pi = doc.bodyIndex(b.parent);
    Pose p = (pi >= 0 ? poses[pi] : Pose{}) * Pose{b.pos, b.quat};
    // Joints move the body frame in order, rotating about their anchors (as mj_kinematics does).
    for (uint32_t ji : ix.bodyJoints[bi]) {
      const Joint& j = doc.joints[ji];
      const std::vector<double>* q = key->jointQpos(j.id);
      if (!q) continue;
      const std::vector<double>& v = *q;
      auto aboutAnchor = [&](const Quat& r) { return Pose{j.pos, {}} * Pose{{}, r} * Pose{-j.pos, {}}; };
      switch (j.type) {
        case JointType::Free:
          if (v.size() == 7) p = Pose{{v[0], v[1], v[2]}, normalize(Quat{v[3], v[4], v[5], v[6]})};
          break;
        case JointType::Ball:
          if (v.size() == 4) p = p * aboutAnchor(normalize(Quat{v[0], v[1], v[2], v[3]}));
          break;
        case JointType::Hinge:
          if (v.size() == 1) p = p * aboutAnchor(axisAngle(normalize(j.axis), v[0] - j.ref));
          break;
        case JointType::Slide:
          if (v.size() == 1) p = p * Pose{normalize(j.axis) * (v[0] - j.ref), {}};
          break;
      }
    }
    poses[bi] = p;
  }
  return poses;
}

Pose bodyWorldPose(const Document& doc, Id body) {
  Pose p;
  std::vector<const Body*> chain;
  const Body* b = doc.body(body);
  int guard = 0;
  while (b && b->id != kWorldId && guard++ < 100000) {
    chain.push_back(b);
    b = doc.body(b->parent);
  }
  for (auto it = chain.rbegin(); it != chain.rend(); ++it) p = p * Pose{(*it)->pos, (*it)->quat};
  return p;
}

static Pose bodyPoseFrom(const Document& doc, const std::vector<Pose>& poses, Id body) {
  int bi = doc.bodyIndex(body);
  if (bi < 0 || bi >= int(poses.size())) return {};
  return poses[bi];
}

Pose elementLocalPose(const Document& doc, Id id) {
  switch (doc.kindOf(id)) {
    case Kind::Body: { const Body* b = doc.body(id); return {b->pos, b->quat}; }
    case Kind::Geom: { const Geom* g = doc.geom(id); return {g->pos, g->quat}; }
    case Kind::Site: { const Site* s = doc.site(id); return {s->pos, s->quat}; }
    case Kind::Camera: { const Camera* c = doc.camera(id); return {c->pos, c->quat}; }
    case Kind::Light: {
      const Light* l = doc.light(id);
      // Lights point along their local -Z, like cameras.
      return {l->pos, fromTo({0, 0, -1}, l->dir)};
    }
    case Kind::Joint: { const Joint* j = doc.joint(id); return {j->pos, frameFromZ(j->axis)}; }
    default: return {};
  }
}

void setElementLocalPose(Document& doc, Id id, const Pose& p) {
  Quat q = canonical(p.quat);
  switch (doc.kindOf(id)) {
    case Kind::Body: { Body* b = doc.body(id); b->pos = p.pos; b->quat = q; break; }
    case Kind::Geom: { Geom* g = doc.geom(id); g->pos = p.pos; g->quat = q; break; }
    case Kind::Site: { Site* s = doc.site(id); s->pos = p.pos; s->quat = q; break; }
    case Kind::Camera: { Camera* c = doc.camera(id); c->pos = p.pos; c->quat = q; break; }
    case Kind::Light: { Light* l = doc.light(id); l->pos = p.pos; l->dir = normalize(rotate(q, {0, 0, -1})); break; }
    case Kind::Joint: { Joint* j = doc.joint(id); j->pos = p.pos; j->axis = normalize(rotate(q, {0, 0, 1})); break; }
    default: break;
  }
}

Pose elementWorldPose(const Document& doc, const std::vector<Pose>& poses, Id id) {
  Kind k = doc.kindOf(id);
  if (k == Kind::Body) return bodyPoseFrom(doc, poses, id);
  Id owner = doc.ownerBody(id);
  return bodyPoseFrom(doc, poses, owner) * elementLocalPose(doc, id);
}

void jointWorldFrame(const Document& doc, const std::vector<Pose>& poses, const Joint& j, Vec3* anchor, Vec3* axis) {
  Pose bp = bodyPoseFrom(doc, poses, j.body);
  if (anchor) *anchor = bp.apply(j.pos);
  if (axis) *axis = normalize(bp.applyDir(j.axis));
}

// ----- mass accumulation -----

static Mat3 pointInertia(double m, const Vec3& d) {
  // m * (|d|^2 E - d d^T)
  Mat3 r = Mat3::zero();
  double dd = dot(d, d);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) r(i, j) = m * ((i == j ? dd : 0.0) - d[i] * d[j]);
  return r;
}

void MassAccumulator::add(double m, const Vec3& c, const Mat3& Icom) {
  if (m <= 0) return;
  mass_ += m;
  first_ += c * m;
  originInertia_ = originInertia_ + Icom + pointInertia(m, c);
}

Mat3 MassAccumulator::inertiaAboutCom() const {
  if (mass_ <= 0) return Mat3::zero();
  Vec3 c = com();
  Mat3 shift = pointInertia(mass_, c);
  Mat3 r = originInertia_;
  for (int i = 0; i < 9; ++i) r.m[i] -= shift.m[i];
  return r;
}

double MassAccumulator::inertiaAboutAxis(const Vec3& p, const Vec3& a_) const {
  if (mass_ <= 0) return 0;
  Vec3 a = normalize(a_);
  Mat3 Ic = inertiaAboutCom();
  Vec3 d = com() - p;
  Vec3 dperp = d - a * dot(d, a);
  return dot(a, Ic * a) + mass_ * dot(dperp, dperp);
}

void accumulateBodyMass(const Document& doc, const std::vector<Pose>& poses, Id bodyId, MassAccumulator& acc) {
  const Body* b = doc.body(bodyId);
  if (!b || bodyId == kWorldId) return;
  Pose bp = bodyPoseFrom(doc, poses, bodyId);
  if (b->hasInertial) {
    Pose ip = bp * Pose{b->ipos, b->iquat};
    Mat3 R = toMat3(ip.quat);
    Mat3 I = R * Mat3::diag(b->diaginertia) * R.transposed();
    acc.add(b->mass, ip.pos, I);
    return;
  }
  int bi = doc.bodyIndex(bodyId);
  for (uint32_t gi : doc.index().bodyGeoms[bi]) {
    const Geom& g = doc.geoms[gi];
    if (g.type == GeomType::Mesh) {
      const MeshAsset* ma = doc.mesh(g.mesh);
      double m = geomMass(doc, g);
      if (!ma || m <= 0) continue;
      const MeshMass& mm = meshMassProps(*ma);
      if (mm.volume <= 0) continue;
      Pose gp = bp * Pose{g.pos, g.quat};
      Mat3 R = toMat3(gp.quat);
      acc.add(m, gp.apply(mm.center), R * (mm.inertia * (m / mm.volume)) * R.transposed());
      continue;
    }
    double m = geomMass(g);
    if (m <= 0) continue;
    Pose gp = bp * Pose{g.pos, g.quat};
    Mat3 R = toMat3(gp.quat);
    Mat3 I = R * Mat3::diag(primitiveInertia(g.type, g.size, m)) * R.transposed();
    acc.add(m, gp.pos, I);
  }
}

MassProps bodyMassProps(const Document& doc, const std::vector<Pose>& poses, Id body) {
  MassAccumulator acc;
  accumulateBodyMass(doc, poses, body, acc);
  return {acc.mass(), acc.com(), acc.inertiaAboutCom()};
}

MassAccumulator subtreeAccumulator(const Document& doc, const std::vector<Pose>& poses, Id body) {
  MassAccumulator acc;
  for (Id b : doc.subtree(body)) accumulateBodyMass(doc, poses, b, acc);
  return acc;
}

MassProps subtreeMassProps(const Document& doc, const std::vector<Pose>& poses, Id body) {
  MassAccumulator acc = subtreeAccumulator(doc, poses, body);
  return {acc.mass(), acc.com(), acc.inertiaAboutCom()};
}

Aabb subtreeAabb(const Document& doc, const std::vector<Pose>& poses, Id body, bool collidingOnly) {
  Aabb box;
  const DocIndex& ix = doc.index();
  for (Id b : doc.subtree(body)) {
    int bi = doc.bodyIndex(b);
    Pose bp = poses[bi];
    for (uint32_t gi : ix.bodyGeoms[bi]) {
      const Geom& g = doc.geoms[gi];
      if (g.type == GeomType::Plane || g.type == GeomType::Hfield) continue;
      if (collidingOnly && !geomCollides(g)) continue;
      box.add(geomWorldAabb(doc, g, bp * Pose{g.pos, g.quat}));
    }
  }
  return box;
}

double subtreeMinZ(const Document& doc, const std::vector<Pose>& poses, Id body, bool collidingOnly) {
  Aabb box = subtreeAabb(doc, poses, body, collidingOnly);
  return box.valid() ? box.lo.z : std::numeric_limits<double>::infinity();
}

}  // namespace mjf
