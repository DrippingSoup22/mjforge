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

std::vector<Pose> computeBodyPoses(const Document& doc, const Keyframe* key, std::vector<JointFrame>* frames) {
  if (!key && !frames) return computeBodyPoses(doc);
  const DocIndex& ix = doc.index();
  std::vector<Pose> poses(doc.bodies.size());
  if (frames) frames->assign(doc.joints.size(), JointFrame{});
  for (uint32_t bi : ix.dfsOrder) {
    if (bi == 0) continue;
    const Body& b = doc.bodies[bi];
    int pi = doc.bodyIndex(b.parent);
    Pose p = (pi >= 0 ? poses[pi] : Pose{}) * Pose{b.pos, b.quat};
    // Joints move the body frame in order, rotating about their anchors (as mj_kinematics does);
    // each joint's anchor and axis are taken before its own motion.
    for (uint32_t ji : ix.bodyJoints[bi]) {
      const Joint& j = doc.joints[ji];
      if (frames) (*frames)[ji] = {p.apply(j.pos), normalize(p.applyDir(j.axis))};
      const std::vector<double>* q = key ? key->jointQpos(j.id) : nullptr;
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

void jointWorldFrame(const Document& doc, const std::vector<JointFrame>& frames, const Joint& j, Vec3* anchor, Vec3* axis) {
  const DocIndex& ix = doc.index();
  auto it = ix.loc.find(j.id);
  if (it == ix.loc.end() || it->second.kind != Kind::Joint || it->second.index >= frames.size()) {
    if (anchor) *anchor = j.pos;
    if (axis) *axis = j.axis;
    return;
  }
  const JointFrame& f = frames[it->second.index];
  if (anchor) *anchor = f.anchor;
  if (axis) *axis = f.axis;
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

// Spatial inertia about point p, angular rows first: [[Ic + m [r]x[r]x^T, m [r]x], [m [r]x^T, m E]].
static void spatialInertia(const MassAccumulator& acc, const Vec3& p, double out[6][6]) {
  double m = acc.mass();
  Vec3 r = acc.com() - p;
  Mat3 Ic = acc.inertiaAboutCom();
  double rx[3][3] = {{0, -r.z, r.y}, {r.z, 0, -r.x}, {-r.y, r.x, 0}};
  double rr = dot(r, r);
  for (int i = 0; i < 3; ++i)
    for (int j = 0; j < 3; ++j) {
      out[i][j] = Ic(i, j) + m * ((i == j ? rr : 0.0) - r[i] * r[j]);
      out[i][j + 3] = m * rx[i][j];
      out[i + 3][j] = m * rx[j][i];
      out[i + 3][j + 3] = i == j ? m : 0.0;
    }
}

double floatingJointInertia(const MassAccumulator& moved, const MassAccumulator& whole, const Vec3& p, const Vec3& axis,
                            bool angular) {
  // With the rest free, momentum is conserved: I = s'As - (As)' T^-1 (As), T = A + rest.
  double A[6][6], T[6][7];
  double Tm[6][6];
  spatialInertia(moved, p, A);
  spatialInertia(whole, p, Tm);
  Vec3 a = normalize(axis);
  double sv[6] = {0, 0, 0, 0, 0, 0};
  for (int k = 0; k < 3; ++k) sv[angular ? k : k + 3] = a[k];
  double As[6], sAs = 0;
  for (int i = 0; i < 6; ++i) {
    As[i] = 0;
    for (int j = 0; j < 6; ++j) As[i] += A[i][j] * sv[j];
    sAs += sv[i] * As[i];
  }
  for (int i = 0; i < 6; ++i) {
    for (int j = 0; j < 6; ++j) T[i][j] = Tm[i][j];
    T[i][6] = As[i];
  }
  // Gaussian elimination with partial pivoting on [T | As].
  double scale = 0;
  for (int i = 0; i < 6; ++i) scale = std::max(scale, std::fabs(T[i][i]));
  for (int c = 0; c < 6; ++c) {
    int piv = c;
    for (int r = c + 1; r < 6; ++r)
      if (std::fabs(T[r][c]) > std::fabs(T[piv][c])) piv = r;
    if (std::fabs(T[piv][c]) <= 1e-12 * scale) return sAs;  // the whole has no inertia this way
    if (piv != c)
      for (int k = 0; k < 7; ++k) std::swap(T[c][k], T[piv][k]);
    for (int r = 0; r < 6; ++r) {
      if (r == c) continue;
      double f = T[r][c] / T[c][c];
      for (int k = c; k < 7; ++k) T[r][k] -= f * T[c][k];
    }
  }
  double coupling = 0;
  for (int i = 0; i < 6; ++i) coupling += As[i] * T[i][6] / T[i][i];
  return std::max(0.0, sAs - coupling);
}

void accumulateBodyMass(const Document& doc, const std::vector<Pose>& poses, Id bodyId, MassAccumulator& acc,
                        const InertialTable* table) {
  const Body* b = doc.body(bodyId);
  if (!b || bodyId == kWorldId) return;
  Pose bp = bodyPoseFrom(doc, poses, bodyId);
  auto addInertial = [&](MassAccumulator& to, double m, const Vec3& ipos, const Quat& iquat, const Vec3& diag) {
    Pose ip = bp * Pose{ipos, iquat};
    Mat3 R = toMat3(ip.quat);
    to.add(m, ip.pos, R * Mat3::diag(diag) * R.transposed());
  };
  if (table) {
    auto it = table->find(bodyId);
    if (it != table->end()) {
      addInertial(acc, it->second.mass, it->second.ipos, it->second.iquat, it->second.diaginertia);
      return;
    }
  }
  const CompilerSettings& cs = doc.compiler;
  MassAccumulator own;
  if (b->hasInertial && cs.inertiafromgeom != InertiaFromGeom::True) {
    addInertial(own, b->mass, b->ipos, b->iquat, b->diaginertia);
  } else if (cs.inertiafromgeom != InertiaFromGeom::False) {
    int bi = doc.bodyIndex(bodyId);
    for (uint32_t gi : doc.index().bodyGeoms[bi]) {
      const Geom& g = doc.geoms[gi];
      Pose gp = bp * Pose{g.pos, g.quat};
      Mat3 R = toMat3(gp.quat);
      if (g.type == GeomType::Mesh) {
        const MeshAsset* ma = doc.mesh(g.mesh);
        double m = geomMass(doc, g);
        if (!ma || m <= 0) continue;
        const MeshMass& mm = meshMassProps(*ma);
        if (mm.volume <= 0) continue;
        own.add(m, gp.apply(mm.center), R * (mm.inertia * (m / mm.volume)) * R.transposed());
        continue;
      }
      double m = geomMass(g);
      if (m <= 0) continue;
      own.add(m, gp.pos, R * Mat3::diag(primitiveInertia(g.type, g.size, m)) * R.transposed());
    }
  }
  if (cs.boundmass > 0 || cs.boundinertia > 0) {
    // The compiler raises the mass and each principal inertia to these minimums.
    Vec3 ev;
    Mat3 evec;
    eigSym3(own.inertiaAboutCom(), &ev, &evec);
    for (int k = 0; k < 3; ++k) ev[k] = std::max(ev[k], cs.boundinertia);
    acc.add(std::max(own.mass(), cs.boundmass), own.mass() > 0 ? own.com() : bp.pos,
            evec * Mat3::diag(ev) * evec.transposed());
    return;
  }
  acc.add(own.mass(), own.com(), own.inertiaAboutCom());
}

MassProps bodyMassProps(const Document& doc, const std::vector<Pose>& poses, Id body, const InertialTable* table) {
  MassAccumulator acc;
  accumulateBodyMass(doc, poses, body, acc, table);
  return {acc.mass(), acc.com(), acc.inertiaAboutCom()};
}

MassAccumulator subtreeAccumulator(const Document& doc, const std::vector<Pose>& poses, Id body, const InertialTable* table) {
  MassAccumulator acc;
  for (Id b : doc.subtree(body)) accumulateBodyMass(doc, poses, b, acc, table);
  return acc;
}

MassProps subtreeMassProps(const Document& doc, const std::vector<Pose>& poses, Id body, const InertialTable* table) {
  MassAccumulator acc = subtreeAccumulator(doc, poses, body, table);
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
