// Orbit camera for a z-up world.
#pragma once

#include "core/Math.h"

namespace mjf {

struct OrbitCamera {
  Vec3 target{0, 0, 0.3};
  double distance = 3.5;
  double azimuth = 225;   // degrees, around +z; 0 looks along +x... see eye()
  double elevation = 22;  // degrees above the xy plane
  double fovy = 40;       // degrees
  bool ortho = false;

  Vec3 eye() const;
  Vec3 forward() const { return normalize(target - eye()); }
  Vec3 right() const;
  Vec3 up() const;
  double znear() const;
  double zfar() const;
  Mat4 view() const;
  Mat4 proj(double aspect) const;
  // Ray through a pixel (x right, y down) of a w x h viewport.
  Ray ray(double x, double y, double w, double h) const;
  // World-space size of one pixel at the target distance.
  double pixelSize(double viewportHeight) const;

  void orbit(double dAzimuthDeg, double dElevationDeg);
  void pan(double dxPixels, double dyPixels, double viewportHeight);
  void zoom(double factor);
  // Frames a bounding sphere.
  void frame(const Vec3& center, double radius);
};

}  // namespace mjf
