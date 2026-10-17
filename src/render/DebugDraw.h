// Immediate-mode overlay geometry: thick anti-aliased lines and translucent triangles, in two
// layers (depth-tested "world" and always-visible "overlay").
#pragma once

#include <vector>

#include "render/GL.h"

namespace mjf {

class DebugDraw {
 public:
  enum Layer { World = 0, Overlay = 1 };

  bool init();
  void release();
  void clear();

  void line(const Vec3& a, const Vec3& b, const Color& c, float widthPx = 2.0f, Layer layer = World);
  void triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Color& col, Layer layer = World);
  // Circle in the plane orthogonal to `normal`.
  void circle(const Vec3& center, const Vec3& normal, double radius, const Color& c, float widthPx = 2.0f,
              Layer layer = World, int segments = 48);
  // Arc around `normal` starting at direction `from` (orthogonal to normal), angles in radians.
  void arc(const Vec3& center, const Vec3& normal, const Vec3& from, double radius, double a0, double a1,
           const Color& c, float widthPx = 2.0f, Layer layer = World);
  // Filled pie slice.
  void sector(const Vec3& center, const Vec3& normal, const Vec3& from, double radius, double a0, double a1,
              const Color& c, Layer layer = World);
  void arrow(const Vec3& from, const Vec3& to, const Color& c, float widthPx = 2.5f, Layer layer = World,
             double headFrac = 0.18);
  void axes(const Pose& pose, double size, float widthPx = 2.0f, Layer layer = World, float alpha = 1.0f);
  void box(const Pose& pose, const Vec3& halfSize, const Color& c, float widthPx = 1.5f, Layer layer = World);
  void cross3(const Vec3& p, double size, const Color& c, float widthPx = 2.0f, Layer layer = World);

  void render(const Mat4& viewProj, int viewportW, int viewportH);

 private:
  struct LineVert {
    float a[3], b[3];
    float color[4];
    float side, end, width;
  };
  struct TriVert {
    float p[3];
    float color[4];
  };
  void renderLayer(int layer, const Mat4& viewProj, int w, int h);

  std::vector<LineVert> lines_[2];
  std::vector<TriVert> tris_[2];
  Shader lineShader_, triShader_;
  GLuint lineVao_ = 0, lineVbo_ = 0, triVao_ = 0, triVbo_ = 0;
};

}  // namespace mjf
