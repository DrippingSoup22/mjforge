#include "core/Geometry.h"

#include <cmath>

namespace mjf {

double primitiveVolume(GeomType type, const Vec3& s) {
  switch (type) {
    case GeomType::Sphere: return 4.0 / 3.0 * kPi * s.x * s.x * s.x;
    case GeomType::Capsule: return kPi * s.x * s.x * 2.0 * s.y + 4.0 / 3.0 * kPi * s.x * s.x * s.x;
    case GeomType::Cylinder: return kPi * s.x * s.x * 2.0 * s.y;
    case GeomType::Ellipsoid: return 4.0 / 3.0 * kPi * s.x * s.y * s.z;
    case GeomType::Box: return 8.0 * s.x * s.y * s.z;
    default: return 0.0;
  }
}

Vec3 primitiveInertia(GeomType type, const Vec3& s, double m) {
  switch (type) {
    case GeomType::Sphere: {
      double i = 0.4 * m * s.x * s.x;
      return {i, i, i};
    }
    case GeomType::Capsule: {
      // Cylinder of height 2*s.y plus two hemispheres, split by volume (as in MuJoCo).
      double r = s.x, h = 2.0 * s.y;
      double vcyl = kPi * r * r * h, vsph = 4.0 / 3.0 * kPi * r * r * r;
      double vtot = vcyl + vsph;
      if (vtot <= 0) return {};
      double mcyl = m * vcyl / vtot, msph = m * vsph / vtot;
      double ixx = mcyl * (3 * r * r + h * h) / 12.0 + msph * (2 * r * r / 5.0 + h * h / 4.0 + 3 * h * r / 8.0);
      double izz = mcyl * r * r / 2.0 + msph * 2 * r * r / 5.0;
      return {ixx, ixx, izz};
    }
    case GeomType::Cylinder: {
      double r = s.x, h = 2.0 * s.y;
      double ixx = m * (3 * r * r + h * h) / 12.0;
      return {ixx, ixx, m * r * r / 2.0};
    }
    case GeomType::Ellipsoid:
      return {m * (s.y * s.y + s.z * s.z) / 5.0, m * (s.x * s.x + s.z * s.z) / 5.0, m * (s.x * s.x + s.y * s.y) / 5.0};
    case GeomType::Box:
      return {m * (s.y * s.y + s.z * s.z) / 3.0, m * (s.x * s.x + s.z * s.z) / 3.0, m * (s.x * s.x + s.y * s.y) / 3.0};
    default: return {};
  }
}

bool geomHasMass(const Geom& g) {
  if (g.type == GeomType::Plane || g.type == GeomType::Hfield || g.type == GeomType::Mesh) return false;
  return g.group >= 0 && g.group <= 5;
}

double geomMass(const Geom& g) {
  if (!geomHasMass(g)) return 0.0;
  if (g.mass >= 0) return g.mass;
  return g.density * primitiveVolume(g.type, g.size);
}

Vec3 primitiveWorldHalfExtents(GeomType type, const Vec3& s, const Mat3& R) {
  Vec3 e;
  for (int k = 0; k < 3; ++k) {
    double a0 = R(k, 0), a1 = R(k, 1), a2 = R(k, 2);  // world axis k expressed in local frame
    switch (type) {
      case GeomType::Sphere: e[k] = s.x; break;
      case GeomType::Capsule: e[k] = std::fabs(s.y * a2) + s.x; break;
      case GeomType::Cylinder: e[k] = std::fabs(s.y * a2) + s.x * std::sqrt(std::max(0.0, 1.0 - a2 * a2)); break;
      case GeomType::Ellipsoid:
        e[k] = std::sqrt((s.x * a0) * (s.x * a0) + (s.y * a1) * (s.y * a1) + (s.z * a2) * (s.z * a2));
        break;
      case GeomType::Box: e[k] = std::fabs(s.x * a0) + std::fabs(s.y * a1) + std::fabs(s.z * a2); break;
      default: e[k] = 0; break;
    }
  }
  return e;
}

Aabb primitiveWorldAabb(GeomType type, const Vec3& size, const Pose& pose) {
  Aabb b;
  if (type == GeomType::Plane || type == GeomType::Hfield || type == GeomType::Mesh) {
    b.add(pose.pos);
    return b;
  }
  Vec3 e = primitiveWorldHalfExtents(type, size, toMat3(pose.quat));
  b.add(pose.pos - e);
  b.add(pose.pos + e);
  return b;
}

namespace {

// Ray vs unit sphere at origin (local coordinates); returns smallest t >= 0 or -1.
double rayUnitSphere(const Vec3& o, const Vec3& d) {
  double a = dot(d, d), b = dot(o, d), c = dot(o, o) - 1.0;
  double disc = b * b - a * c;
  if (disc < 0) return -1;
  double sq = std::sqrt(disc);
  double t = (-b - sq) / a;
  if (t >= 0) return t;
  t = (-b + sq) / a;
  return t >= 0 ? t : -1;
}

double raySphereLocal(const Vec3& o, const Vec3& d, const Vec3& c, double r) {
  return rayUnitSphere((o - c) / r, d / r);
}

// Infinite cylinder along z with radius r, clipped to |z| <= h. Local coordinates.
double rayCylinderSide(const Vec3& o, const Vec3& d, double r, double h) {
  double a = d.x * d.x + d.y * d.y;
  if (a < 1e-18) return -1;
  double b = o.x * d.x + o.y * d.y;
  double c = o.x * o.x + o.y * o.y - r * r;
  double disc = b * b - a * c;
  if (disc < 0) return -1;
  double sq = std::sqrt(disc);
  for (double t : {(-b - sq) / a, (-b + sq) / a}) {
    if (t < 0) continue;
    double z = o.z + t * d.z;
    if (std::fabs(z) <= h) return t;
  }
  return -1;
}

double rayDisk(const Vec3& o, const Vec3& d, double z, double r) {
  if (std::fabs(d.z) < 1e-18) return -1;
  double t = (z - o.z) / d.z;
  if (t < 0) return -1;
  double x = o.x + t * d.x, y = o.y + t * d.y;
  return (x * x + y * y <= r * r) ? t : -1;
}

double minPositive(double a, double b) {
  if (a < 0) return b;
  if (b < 0) return a;
  return std::min(a, b);
}

}  // namespace

double rayPrimitive(const Ray& ray, GeomType type, const Vec3& s, const Pose& pose, double infiniteExtent,
                    Vec3* normal) {
  // Transform the ray into the primitive's local frame (rotation preserves t).
  Quat qi = conj(pose.quat);
  Vec3 o = rotate(qi, ray.origin - pose.pos);
  Vec3 d = rotate(qi, ray.dir);
  double t = -1;
  Vec3 nLocal{0, 0, 1};
  switch (type) {
    case GeomType::Sphere: {
      t = raySphereLocal(o, d, {}, s.x);
      if (t >= 0) nLocal = normalize(o + d * t);
      break;
    }
    case GeomType::Ellipsoid: {
      Vec3 inv{1.0 / s.x, 1.0 / s.y, 1.0 / s.z};
      t = rayUnitSphere(cwiseMul(o, inv), cwiseMul(d, inv));
      if (t >= 0) {
        Vec3 p = o + d * t;
        nLocal = normalize(cwiseMul(p, cwiseMul(inv, inv)));
      }
      break;
    }
    case GeomType::Capsule: {
      double tc = rayCylinderSide(o, d, s.x, s.y);
      double t1 = raySphereLocal(o, d, {0, 0, s.y}, s.x);
      double t2 = raySphereLocal(o, d, {0, 0, -s.y}, s.x);
      t = minPositive(tc, minPositive(t1, t2));
      if (t >= 0) {
        Vec3 p = o + d * t;
        Vec3 c{0, 0, clampv(p.z, -s.y, s.y)};
        nLocal = normalize(p - c);
      }
      break;
    }
    case GeomType::Cylinder: {
      double tc = rayCylinderSide(o, d, s.x, s.y);
      double t1 = rayDisk(o, d, s.y, s.x);
      double t2 = rayDisk(o, d, -s.y, s.x);
      t = minPositive(tc, minPositive(t1, t2));
      if (t >= 0) {
        if (t == t1) nLocal = {0, 0, 1};
        else if (t == t2) nLocal = {0, 0, -1};
        else { Vec3 p = o + d * t; nLocal = normalize(Vec3{p.x, p.y, 0}); }
      }
      break;
    }
    case GeomType::Box: {
      double tmin = -1e300, tmax = 1e300;
      int axis = -1;
      double sign = 1;
      for (int i = 0; i < 3; ++i) {
        if (std::fabs(d[i]) < 1e-18) {
          if (std::fabs(o[i]) > s[i]) return -1;
          continue;
        }
        double ta = (-s[i] - o[i]) / d[i], tb = (s[i] - o[i]) / d[i];
        double sg = -1;
        if (ta > tb) { std::swap(ta, tb); sg = 1; }
        if (ta > tmin) { tmin = ta; axis = i; sign = sg; }
        tmax = std::min(tmax, tb);
        if (tmin > tmax) return -1;
      }
      if (tmax < 0) return -1;
      t = tmin >= 0 ? tmin : tmax;
      if (axis >= 0) { nLocal = {}; nLocal[axis] = (d[axis] > 0 ? -1.0 : 1.0); (void)sign; }
      break;
    }
    case GeomType::Plane:
    case GeomType::Hfield: {
      if (std::fabs(d.z) < 1e-18) return -1;
      t = -o.z / d.z;
      if (t < 0) return -1;
      double ex = s.x > 0 ? s.x : infiniteExtent, ey = s.y > 0 ? s.y : infiniteExtent;
      Vec3 p = o + d * t;
      if (std::fabs(p.x) > ex || std::fabs(p.y) > ey) return -1;
      nLocal = {0, 0, 1};
      break;
    }
    default: return -1;
  }
  if (t >= 0 && normal) *normal = rotate(pose.quat, nLocal);
  return t;
}

double raySegmentDistance(const Ray& ray, const Vec3& a, const Vec3& b, double* tRay, double* tSeg) {
  // Minimize |w + s*u - t*v|^2 over s >= 0 (ray) and t in [0, 1] (segment).
  Vec3 u = ray.dir, v = b - a, w = ray.origin - a;
  double A = dot(u, u), B = dot(u, v), C = dot(v, v), D = dot(u, w), E = dot(v, w);
  double den = A * C - B * B;
  double tc = den > 1e-12 ? (A * E - B * D) / den : (C > 1e-12 ? E / C : 0.0);
  tc = clampv(tc, 0.0, 1.0);
  double sc = (tc * B - D) / A;
  if (sc < 0) {
    sc = 0;
    tc = C > 1e-12 ? clampv(E / C, 0.0, 1.0) : 0.0;
  }
  Vec3 pr = ray.origin + u * sc, ps = a + v * tc;
  if (tRay) *tRay = sc;
  if (tSeg) *tSeg = tc;
  return length(pr - ps);
}

}  // namespace mjf
