// Small linear-algebra toolkit used by the document model (double precision, MuJoCo
// conventions: quaternions are (w, x, y, z), matrices are row-major) and by the renderer
// (single precision, column-major 4x4 matrices for OpenGL).
#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace mjf {

constexpr double kPi = 3.14159265358979323846;
inline double deg2rad(double d) { return d * (kPi / 180.0); }
inline double rad2deg(double r) { return r * (180.0 / kPi); }
template <class T> inline T clampv(T v, T lo, T hi) { return v < lo ? lo : (v > hi ? hi : v); }

struct Vec2 {
  double x = 0, y = 0;
  constexpr Vec2() = default;
  constexpr Vec2(double x_, double y_) : x(x_), y(y_) {}
  double& operator[](int i) { return i == 0 ? x : y; }
  double operator[](int i) const { return i == 0 ? x : y; }
  bool operator==(const Vec2& o) const { return x == o.x && y == o.y; }
  bool operator!=(const Vec2& o) const { return !(*this == o); }
};

struct Vec3 {
  double x = 0, y = 0, z = 0;
  constexpr Vec3() = default;
  constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
  double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : z); }
  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : z); }
  Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
  Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
  Vec3 operator-() const { return {-x, -y, -z}; }
  Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
  Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
  Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
  Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
  Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
  bool operator==(const Vec3& o) const { return x == o.x && y == o.y && z == o.z; }
  bool operator!=(const Vec3& o) const { return !(*this == o); }
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double length(const Vec3& v) { return std::sqrt(dot(v, v)); }
inline double length2(const Vec3& v) { return dot(v, v); }
inline Vec3 normalize(const Vec3& v, const Vec3& fallback = {0, 0, 1}) {
  double l = length(v);
  if (l > 1e-12 && std::isfinite(l)) return v / l;
  // Components so large that the length overflows: scale them down first. A non-finite or
  // vanishing vector has no direction.
  double m = std::max({std::fabs(v.x), std::fabs(v.y), std::fabs(v.z)});
  if (!std::isfinite(v.x) || !std::isfinite(v.y) || !std::isfinite(v.z) || !(m > 1e-12)) return fallback;
  Vec3 s = v / m;
  return s / length(s);
}
inline Vec3 cwiseMul(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
inline Vec3 vmin(const Vec3& a, const Vec3& b) { return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)}; }
inline Vec3 vmax(const Vec3& a, const Vec3& b) { return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)}; }
inline Vec3 lerp(const Vec3& a, const Vec3& b, double t) { return a + (b - a) * t; }
// Any unit vector perpendicular to v.
inline Vec3 perpendicular(const Vec3& v) {
  Vec3 a = std::fabs(v.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
  return normalize(cross(v, a));
}

struct Vec4 {
  double x = 0, y = 0, z = 0, w = 0;
  constexpr Vec4() = default;
  constexpr Vec4(double x_, double y_, double z_, double w_) : x(x_), y(y_), z(z_), w(w_) {}
  double& operator[](int i) { return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w)); }
  double operator[](int i) const { return i == 0 ? x : (i == 1 ? y : (i == 2 ? z : w)); }
  bool operator==(const Vec4& o) const { return x == o.x && y == o.y && z == o.z && w == o.w; }
  bool operator!=(const Vec4& o) const { return !(*this == o); }
};

// Solver impedance (5 numbers) and other fixed-size arrays.
template <int N> struct VecN {
  double v[N] = {};
  double& operator[](int i) { return v[i]; }
  double operator[](int i) const { return v[i]; }
  bool operator==(const VecN& o) const {
    for (int i = 0; i < N; ++i) if (v[i] != o.v[i]) return false;
    return true;
  }
  bool operator!=(const VecN& o) const { return !(*this == o); }
};
using Vec5 = VecN<5>;
using Vec6 = VecN<6>;
using Vec10 = VecN<10>;

struct Color {
  float r = 0.5f, g = 0.5f, b = 0.5f, a = 1.0f;
  constexpr Color() = default;
  constexpr Color(float r_, float g_, float b_, float a_ = 1.0f) : r(r_), g(g_), b(b_), a(a_) {}
  bool operator==(const Color& o) const { return r == o.r && g == o.g && b == o.b && a == o.a; }
  bool operator!=(const Color& o) const { return !(*this == o); }
};

// Row-major 3x3 matrix.
struct Mat3 {
  double m[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  static Mat3 zero() { Mat3 r; for (double& v : r.m) v = 0; return r; }
  static Mat3 diag(const Vec3& d) { Mat3 r = zero(); r.m[0] = d.x; r.m[4] = d.y; r.m[8] = d.z; return r; }
  double& operator()(int r, int c) { return m[r * 3 + c]; }
  double operator()(int r, int c) const { return m[r * 3 + c]; }
  Vec3 operator*(const Vec3& v) const {
    return {m[0] * v.x + m[1] * v.y + m[2] * v.z, m[3] * v.x + m[4] * v.y + m[5] * v.z,
            m[6] * v.x + m[7] * v.y + m[8] * v.z};
  }
  Mat3 operator*(const Mat3& o) const {
    Mat3 r = zero();
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k) r(i, j) += (*this)(i, k) * o(k, j);
    return r;
  }
  Mat3 operator+(const Mat3& o) const { Mat3 r; for (int i = 0; i < 9; ++i) r.m[i] = m[i] + o.m[i]; return r; }
  Mat3 operator*(double s) const { Mat3 r; for (int i = 0; i < 9; ++i) r.m[i] = m[i] * s; return r; }
  Mat3 transposed() const {
    Mat3 r;
    for (int i = 0; i < 3; ++i)
      for (int j = 0; j < 3; ++j) r(i, j) = (*this)(j, i);
    return r;
  }
  Vec3 col(int c) const { return {m[c], m[3 + c], m[6 + c]}; }
};

// Unit quaternion (w, x, y, z), MuJoCo convention.
struct Quat {
  double w = 1, x = 0, y = 0, z = 0;
  constexpr Quat() = default;
  constexpr Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}
  double& operator[](int i) { return i == 0 ? w : (i == 1 ? x : (i == 2 ? y : z)); }
  double operator[](int i) const { return i == 0 ? w : (i == 1 ? x : (i == 2 ? y : z)); }
  bool operator==(const Quat& o) const { return w == o.w && x == o.x && y == o.y && z == o.z; }
  bool operator!=(const Quat& o) const { return !(*this == o); }
};

inline Quat operator*(const Quat& a, const Quat& b) {
  return {a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
          a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
          a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
          a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w};
}
inline Quat conj(const Quat& q) { return {q.w, -q.x, -q.y, -q.z}; }
inline Quat normalize(const Quat& q) {
  double n = std::sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n >= 1e-12 && std::isfinite(n)) return {q.w / n, q.x / n, q.y / n, q.z / n};
  // As for vectors: huge components are scaled down first; non-finite or vanishing ones give the
  // identity.
  double m = std::max({std::fabs(q.w), std::fabs(q.x), std::fabs(q.y), std::fabs(q.z)});
  if (!std::isfinite(q.w) || !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !(m >= 1e-12)) return {};
  Quat s{q.w / m, q.x / m, q.y / m, q.z / m};
  double ns = std::sqrt(s.w * s.w + s.x * s.x + s.y * s.y + s.z * s.z);
  return {s.w / ns, s.x / ns, s.y / ns, s.z / ns};
}
// Canonical form: w >= 0 (q and -q represent the same rotation).
inline Quat canonical(const Quat& q) {
  Quat r = normalize(q);
  if (r.w < 0) r = {-r.w, -r.x, -r.y, -r.z};
  return r;
}
inline Vec3 rotate(const Quat& q, const Vec3& v) {
  // v' = v + 2w(u x v) + 2 u x (u x v), u = (x, y, z)
  Vec3 u{q.x, q.y, q.z};
  Vec3 t = cross(u, v) * 2.0;
  return v + t * q.w + cross(u, t);
}
inline Quat axisAngle(const Vec3& axis, double angle) {
  Vec3 a = normalize(axis);
  double s = std::sin(angle * 0.5);
  return {std::cos(angle * 0.5), a.x * s, a.y * s, a.z * s};
}
// Shortest rotation taking unit vector a to unit vector b.
inline Quat fromTo(const Vec3& a_, const Vec3& b_) {
  Vec3 a = normalize(a_), b = normalize(b_);
  double d = dot(a, b);
  if (d > 1.0 - 1e-12) return {};
  if (d < -1.0 + 1e-12) return axisAngle(perpendicular(a), kPi);
  Vec3 c = cross(a, b);
  return normalize(Quat{1.0 + d, c.x, c.y, c.z});
}
inline Mat3 toMat3(const Quat& q_) {
  Quat q = normalize(q_);
  double w = q.w, x = q.x, y = q.y, z = q.z;
  Mat3 r;
  r.m[0] = 1 - 2 * (y * y + z * z); r.m[1] = 2 * (x * y - w * z);     r.m[2] = 2 * (x * z + w * y);
  r.m[3] = 2 * (x * y + w * z);     r.m[4] = 1 - 2 * (x * x + z * z); r.m[5] = 2 * (y * z - w * x);
  r.m[6] = 2 * (x * z - w * y);     r.m[7] = 2 * (y * z + w * x);     r.m[8] = 1 - 2 * (x * x + y * y);
  return r;
}
inline Quat fromMat3(const Mat3& m) {
  double tr = m(0, 0) + m(1, 1) + m(2, 2);
  Quat q;
  if (tr > 0) {
    double s = std::sqrt(tr + 1.0) * 2;
    q = {0.25 * s, (m(2, 1) - m(1, 2)) / s, (m(0, 2) - m(2, 0)) / s, (m(1, 0) - m(0, 1)) / s};
  } else if (m(0, 0) > m(1, 1) && m(0, 0) > m(2, 2)) {
    double s = std::sqrt(1.0 + m(0, 0) - m(1, 1) - m(2, 2)) * 2;
    q = {(m(2, 1) - m(1, 2)) / s, 0.25 * s, (m(0, 1) + m(1, 0)) / s, (m(0, 2) + m(2, 0)) / s};
  } else if (m(1, 1) > m(2, 2)) {
    double s = std::sqrt(1.0 + m(1, 1) - m(0, 0) - m(2, 2)) * 2;
    q = {(m(0, 2) - m(2, 0)) / s, (m(0, 1) + m(1, 0)) / s, 0.25 * s, (m(1, 2) + m(2, 1)) / s};
  } else {
    double s = std::sqrt(1.0 + m(2, 2) - m(0, 0) - m(1, 1)) * 2;
    q = {(m(1, 0) - m(0, 1)) / s, (m(0, 2) + m(2, 0)) / s, (m(1, 2) + m(2, 1)) / s, 0.25 * s};
  }
  return canonical(q);
}
// Frame whose z axis points along `dir` (used for capsules/cylinders defined by two points).
inline Quat frameFromZ(const Vec3& dir) { return fromTo({0, 0, 1}, dir); }

// Euler angles with MuJoCo's default sequence "xyz" (intrinsic): R = Rx(a) * Ry(b) * Rz(c).
inline Quat fromEulerXYZ(const Vec3& e) {
  return normalize(axisAngle({1, 0, 0}, e.x) * axisAngle({0, 1, 0}, e.y) * axisAngle({0, 0, 1}, e.z));
}
inline Vec3 toEulerXYZ(const Quat& q) {
  Mat3 r = toMat3(q);
  double sb = clampv(r(0, 2), -1.0, 1.0);
  Vec3 e;
  e.y = std::asin(sb);
  if (std::fabs(sb) < 0.999999) {
    e.x = std::atan2(-r(1, 2), r(2, 2));
    e.z = std::atan2(-r(0, 1), r(0, 0));
    // Every rotation has two XYZ solutions; prefer the one with the smallest angles.
    auto wrap = [](double a) { return a > kPi ? a - 2 * kPi : (a < -kPi ? a + 2 * kPi : a); };
    Vec3 alt{wrap(e.x + kPi), wrap(kPi - e.y), wrap(e.z + kPi)};
    if (std::fabs(alt.x) + std::fabs(alt.y) + std::fabs(alt.z) < std::fabs(e.x) + std::fabs(e.y) + std::fabs(e.z) - 1e-9)
      e = alt;
  } else {  // gimbal lock: put everything into x
    e.x = std::atan2(r(2, 1), r(1, 1));
    e.z = 0;
  }
  return e;
}

// Eigen-decomposition of a symmetric 3x3 matrix by Jacobi rotations. The columns of `vecs` are
// the eigenvectors (a proper rotation), `vals` the corresponding eigenvalues.
inline void eigSym3(const Mat3& A, Vec3* vals, Mat3* vecs) {
  Mat3 a = A, v;
  for (int sweep = 0; sweep < 50; ++sweep) {
    double off = a(0, 1) * a(0, 1) + a(0, 2) * a(0, 2) + a(1, 2) * a(1, 2);
    if (off < 1e-30) break;
    for (int p = 0; p < 2; ++p) {
      for (int q = p + 1; q < 3; ++q) {
        double apq = a(p, q);
        if (std::fabs(apq) < 1e-300) continue;
        double theta = (a(q, q) - a(p, p)) / (2.0 * apq);
        double t = (theta >= 0 ? 1.0 : -1.0) / (std::fabs(theta) + std::sqrt(theta * theta + 1.0));
        double c = 1.0 / std::sqrt(t * t + 1.0), s = t * c;
        // A <- J^T A J with J(p,p)=J(q,q)=c, J(p,q)=s, J(q,p)=-s.
        for (int k = 0; k < 3; ++k) {
          double akp = a(k, p), akq = a(k, q);
          a(k, p) = c * akp - s * akq;
          a(k, q) = s * akp + c * akq;
        }
        for (int k = 0; k < 3; ++k) {
          double apk = a(p, k), aqk = a(q, k);
          a(p, k) = c * apk - s * aqk;
          a(q, k) = s * apk + c * aqk;
        }
        for (int k = 0; k < 3; ++k) {
          double vkp = v(k, p), vkq = v(k, q);
          v(k, p) = c * vkp - s * vkq;
          v(k, q) = s * vkp + c * vkq;
        }
      }
    }
  }
  // Make the eigenvector frame right-handed.
  Vec3 c0 = v.col(0), c1 = v.col(1), c2 = v.col(2);
  if (dot(cross(c0, c1), c2) < 0)
    for (int k = 0; k < 3; ++k) v(k, 2) = -v(k, 2);
  if (vals) *vals = {a(0, 0), a(1, 1), a(2, 2)};
  if (vecs) *vecs = v;
}

// Rigid transform (position + orientation).
struct Pose {
  Vec3 pos;
  Quat quat;
  Vec3 apply(const Vec3& p) const { return pos + rotate(quat, p); }
  Vec3 applyDir(const Vec3& d) const { return rotate(quat, d); }
};
inline Pose operator*(const Pose& a, const Pose& b) { return {a.apply(b.pos), normalize(a.quat * b.quat)}; }
inline Pose inverse(const Pose& p) {
  Quat qi = conj(p.quat);
  return {rotate(qi, -p.pos), qi};
}

struct Ray {
  Vec3 origin;
  Vec3 dir;  // unit length
  Vec3 at(double t) const { return origin + dir * t; }
};

struct Aabb {
  Vec3 lo{1e300, 1e300, 1e300};
  Vec3 hi{-1e300, -1e300, -1e300};
  bool valid() const { return lo.x <= hi.x; }
  void add(const Vec3& p) { lo = vmin(lo, p); hi = vmax(hi, p); }
  void add(const Aabb& b) { if (b.valid()) { add(b.lo); add(b.hi); } }
  Vec3 center() const { return (lo + hi) * 0.5; }
  Vec3 extent() const { return hi - lo; }
};

// Column-major 4x4 float matrix for OpenGL.
struct Mat4 {
  float m[16] = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
  float& operator()(int r, int c) { return m[c * 4 + r]; }
  float operator()(int r, int c) const { return m[c * 4 + r]; }
  static Mat4 identity() { return {}; }
  Mat4 operator*(const Mat4& o) const {
    Mat4 r;
    for (int i = 0; i < 4; ++i)
      for (int j = 0; j < 4; ++j) {
        float s = 0;
        for (int k = 0; k < 4; ++k) s += (*this)(i, k) * o(k, j);
        r(i, j) = s;
      }
    return r;
  }
  Vec3 transformPoint(const Vec3& p) const {
    double x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
    double y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
    double z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
    double w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
    if (std::fabs(w) > 1e-12) { x /= w; y /= w; z /= w; }
    return {x, y, z};
  }
  // Rigid transform with per-axis scale applied in the local frame.
  static Mat4 fromPose(const Pose& p, const Vec3& scale = {1, 1, 1}) {
    Mat3 r = toMat3(p.quat);
    Mat4 o;
    for (int i = 0; i < 3; ++i) {
      o(i, 0) = float(r(i, 0) * scale.x);
      o(i, 1) = float(r(i, 1) * scale.y);
      o(i, 2) = float(r(i, 2) * scale.z);
      o(i, 3) = float(p.pos[i]);
    }
    return o;
  }
  static Mat4 perspective(double fovyRad, double aspect, double znear, double zfar) {
    Mat4 o;
    double f = 1.0 / std::tan(fovyRad * 0.5);
    for (float& v : o.m) v = 0;
    o(0, 0) = float(f / aspect);
    o(1, 1) = float(f);
    o(2, 2) = float((zfar + znear) / (znear - zfar));
    o(2, 3) = float(2 * zfar * znear / (znear - zfar));
    o(3, 2) = -1;
    return o;
  }
  static Mat4 ortho(double l, double r, double b, double t, double n, double f) {
    Mat4 o;
    o(0, 0) = float(2 / (r - l));
    o(1, 1) = float(2 / (t - b));
    o(2, 2) = float(-2 / (f - n));
    o(0, 3) = float(-(r + l) / (r - l));
    o(1, 3) = float(-(t + b) / (t - b));
    o(2, 3) = float(-(f + n) / (f - n));
    return o;
  }
  static Mat4 lookAt(const Vec3& eye, const Vec3& target, const Vec3& up) {
    Vec3 f = normalize(target - eye);
    Vec3 s = normalize(cross(f, up), perpendicular(f));
    Vec3 u = cross(s, f);
    Mat4 o;
    o(0, 0) = float(s.x); o(0, 1) = float(s.y); o(0, 2) = float(s.z);
    o(1, 0) = float(u.x); o(1, 1) = float(u.y); o(1, 2) = float(u.z);
    o(2, 0) = float(-f.x); o(2, 1) = float(-f.y); o(2, 2) = float(-f.z);
    o(0, 3) = float(-dot(s, eye));
    o(1, 3) = float(-dot(u, eye));
    o(2, 3) = float(dot(f, eye));
    return o;
  }
  Mat4 inverted() const;
};

inline Mat4 Mat4::inverted() const {
  const float* a = m;
  float inv[16];
  inv[0] = a[5] * a[10] * a[15] - a[5] * a[11] * a[14] - a[9] * a[6] * a[15] + a[9] * a[7] * a[14] + a[13] * a[6] * a[11] - a[13] * a[7] * a[10];
  inv[4] = -a[4] * a[10] * a[15] + a[4] * a[11] * a[14] + a[8] * a[6] * a[15] - a[8] * a[7] * a[14] - a[12] * a[6] * a[11] + a[12] * a[7] * a[10];
  inv[8] = a[4] * a[9] * a[15] - a[4] * a[11] * a[13] - a[8] * a[5] * a[15] + a[8] * a[7] * a[13] + a[12] * a[5] * a[11] - a[12] * a[7] * a[9];
  inv[12] = -a[4] * a[9] * a[14] + a[4] * a[10] * a[13] + a[8] * a[5] * a[14] - a[8] * a[6] * a[13] - a[12] * a[5] * a[10] + a[12] * a[6] * a[9];
  inv[1] = -a[1] * a[10] * a[15] + a[1] * a[11] * a[14] + a[9] * a[2] * a[15] - a[9] * a[3] * a[14] - a[13] * a[2] * a[11] + a[13] * a[3] * a[10];
  inv[5] = a[0] * a[10] * a[15] - a[0] * a[11] * a[14] - a[8] * a[2] * a[15] + a[8] * a[3] * a[14] + a[12] * a[2] * a[11] - a[12] * a[3] * a[10];
  inv[9] = -a[0] * a[9] * a[15] + a[0] * a[11] * a[13] + a[8] * a[1] * a[15] - a[8] * a[3] * a[13] - a[12] * a[1] * a[11] + a[12] * a[3] * a[9];
  inv[13] = a[0] * a[9] * a[14] - a[0] * a[10] * a[13] - a[8] * a[1] * a[14] + a[8] * a[2] * a[13] + a[12] * a[1] * a[10] - a[12] * a[2] * a[9];
  inv[2] = a[1] * a[6] * a[15] - a[1] * a[7] * a[14] - a[5] * a[2] * a[15] + a[5] * a[3] * a[14] + a[13] * a[2] * a[7] - a[13] * a[3] * a[6];
  inv[6] = -a[0] * a[6] * a[15] + a[0] * a[7] * a[14] + a[4] * a[2] * a[15] - a[4] * a[3] * a[14] - a[12] * a[2] * a[7] + a[12] * a[3] * a[6];
  inv[10] = a[0] * a[5] * a[15] - a[0] * a[7] * a[13] - a[4] * a[1] * a[15] + a[4] * a[3] * a[13] + a[12] * a[1] * a[7] - a[12] * a[3] * a[5];
  inv[14] = -a[0] * a[5] * a[14] + a[0] * a[6] * a[13] + a[4] * a[1] * a[14] - a[4] * a[2] * a[13] - a[12] * a[1] * a[6] + a[12] * a[2] * a[5];
  inv[3] = -a[1] * a[6] * a[11] + a[1] * a[7] * a[10] + a[5] * a[2] * a[11] - a[5] * a[3] * a[10] - a[9] * a[2] * a[7] + a[9] * a[3] * a[6];
  inv[7] = a[0] * a[6] * a[11] - a[0] * a[7] * a[10] - a[4] * a[2] * a[11] + a[4] * a[3] * a[10] + a[8] * a[2] * a[7] - a[8] * a[3] * a[6];
  inv[11] = -a[0] * a[5] * a[11] + a[0] * a[7] * a[9] + a[4] * a[1] * a[11] - a[4] * a[3] * a[9] - a[8] * a[1] * a[7] + a[8] * a[3] * a[5];
  inv[15] = a[0] * a[5] * a[10] - a[0] * a[6] * a[9] - a[4] * a[1] * a[10] + a[4] * a[2] * a[9] + a[8] * a[1] * a[6] - a[8] * a[2] * a[5];
  float det = a[0] * inv[0] + a[1] * inv[4] + a[2] * inv[8] + a[3] * inv[12];
  Mat4 r;
  if (std::fabs(det) < 1e-30f) return r;
  float id = 1.0f / det;
  for (int i = 0; i < 16; ++i) r.m[i] = inv[i] * id;
  return r;
}

}  // namespace mjf
