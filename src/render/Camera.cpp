#include "render/Camera.h"

#include <cmath>

namespace mjf {

Vec3 OrbitCamera::eye() const {
  double az = deg2rad(azimuth), el = deg2rad(elevation);
  Vec3 dir{std::cos(el) * std::cos(az), std::cos(el) * std::sin(az), std::sin(el)};
  return target + dir * distance;
}

Vec3 OrbitCamera::right() const { return normalize(cross(forward(), {0, 0, 1}), {1, 0, 0}); }
Vec3 OrbitCamera::up() const { return normalize(cross(right(), forward())); }

double OrbitCamera::znear() const { return clampv(distance * 0.005, 0.0005, 0.5); }
double OrbitCamera::zfar() const { return std::max(200.0, distance * 200.0); }

Mat4 OrbitCamera::view() const { return Mat4::lookAt(eye(), target, {0, 0, 1}); }

Mat4 OrbitCamera::proj(double aspect) const {
  if (ortho) {
    double h = distance * std::tan(deg2rad(fovy) * 0.5);
    return Mat4::ortho(-h * aspect, h * aspect, -h, h, -zfar(), zfar());
  }
  return Mat4::perspective(deg2rad(fovy), aspect, znear(), zfar());
}

Ray OrbitCamera::ray(double x, double y, double w, double h) const {
  double nx = (2.0 * x / w) - 1.0, ny = 1.0 - (2.0 * y / h);
  double th = std::tan(deg2rad(fovy) * 0.5);
  double aspect = w / std::max(1.0, h);
  Vec3 f = forward(), r = right(), u = up();
  if (ortho) {
    double hh = distance * th;
    Vec3 origin = eye() + r * (nx * hh * aspect) + u * (ny * hh) - f * zfar() * 0.5;
    return {origin, f};
  }
  Vec3 d = normalize(f + r * (nx * th * aspect) + u * (ny * th));
  return {eye(), d};
}

double OrbitCamera::pixelSize(double viewportHeight) const {
  return 2.0 * distance * std::tan(deg2rad(fovy) * 0.5) / std::max(1.0, viewportHeight);
}

void OrbitCamera::orbit(double dAz, double dEl) {
  azimuth = std::fmod(azimuth + dAz + 360.0, 360.0);
  elevation = clampv(elevation + dEl, -89.0, 89.0);
}

void OrbitCamera::pan(double dx, double dy, double vh) {
  double s = pixelSize(vh);
  target += right() * (-dx * s) + up() * (dy * s);
}

void OrbitCamera::zoom(double factor) { distance = clampv(distance * factor, 0.02, 2000.0); }

void OrbitCamera::frame(const Vec3& center, double radius) {
  target = center;
  double r = std::max(radius * 1.25, 0.12);
  distance = r / std::sin(deg2rad(fovy) * 0.5);
}

}  // namespace mjf
