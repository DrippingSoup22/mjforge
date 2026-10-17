// Scene renderer: lit primitives with shadows, checker planes, sky, grid and overlays.
#pragma once

#include <vector>

#include "render/Camera.h"
#include "render/DebugDraw.h"
#include "render/GL.h"
#include "render/Mesh.h"

namespace mjf {

enum class Shape : uint8_t { Sphere, Capsule, Cylinder, Box, Ellipsoid, Plane, Cone, Custom };

struct DrawItem {
  Shape shape = Shape::Sphere;
  Pose pose;
  Vec3 size{1, 1, 1};  // MuJoCo semantics: sphere r; capsule/cylinder r, half-length; box half-sizes; plane half-xy
  Color color;
  uint8_t highlight = 0;  // 0 none, 1 hovered, 2 selected, 3 primary selection
  bool castShadow = true;
  float specular = 0.5f;
  float shininess = 0.5f;
  float emission = 0.0f;
  // Checker texture (MuJoCo builtin "checker"); squares per meter along x/y in the local frame.
  bool checker = false;
  Color checker2;
  float checkerDensity = 2.0f;
  bool checkerEdges = false;
  Color edgeColor;
  // Custom mesh (height fields): unit mesh scaled per axis by `size`.
  const Mesh* mesh = nullptr;
  bool heightTint = false;  // shade by height with contour lines (height fields)
  Vec3 localOffset;  // set by the renderer for camera-following infinite planes
  uint8_t lod = 0;   // set by the renderer from the on-screen size
};

struct FrameParams {
  int width = 1, height = 1;
  bool shadows = true;
  bool grid = true;
  float gridAlpha = 1.0f;
  float xray = 0.0f;  // 0 = opaque, >0 = make bodies translucent with this alpha
  Vec3 lightDir{-0.35, -0.22, -1.0};
  Color skyTop{0.30f, 0.36f, 0.45f, 1};
  Color skyBottom{0.13f, 0.14f, 0.17f, 1};
};

class Renderer {
 public:
  bool init();
  void release();
  void render(const std::vector<DrawItem>& items, const OrbitCamera& cam, const FrameParams& fp, DebugDraw* dbg);
  PrimitiveMeshes& meshes() { return meshes_; }

 private:
  void drawItem(Shader& sh, const DrawItem& it, bool depthOnly);
  void renderShadowMap(const std::vector<DrawItem>& items, const Mat4& lightViewProj);

  PrimitiveMeshes meshes_;
  Shader lit_, depth_, grid_, sky_;
  GLuint shadowFbo_ = 0, shadowTex_ = 0;
  int shadowSize_ = 2048;
  GLuint emptyVao_ = 0;
  GLuint gridVao_ = 0, gridVbo_ = 0;
};

}  // namespace mjf
