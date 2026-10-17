// GPU meshes for MuJoCo primitives. All are unit-sized and shaped in the vertex shader:
// spheres/ellipsoids/boxes/cylinders by a per-axis scale, capsules by radius + half-length
// (the "side" attribute moves each hemisphere along z), so one mesh serves every size.
#pragma once

#include <cstdint>
#include <vector>

#include "render/GL.h"

namespace mjf {

struct Mesh {
  GLuint vao = 0, vbo = 0, ibo = 0;
  GLsizei count = 0;
  bool valid() const { return vao != 0 && count > 0; }
  // Vertex layout: position(3) normal(3) side(1).
  void upload(const std::vector<float>& verts, const std::vector<uint32_t>& idx);
  void draw() const;
  void release();
};

struct PrimitiveMeshes {
  Mesh sphere, capsule, cylinder, box, plane, cone;
  void build();
  void release();
};

// Builds a mesh for a height field (normalized elevations in [0,1], row 0 = +Y edge as in MJCF),
// spanning [-1,1]^2 in xy and [0,1] in z, with a box base down to z = -base/elevation_z.
void buildHfieldMesh(Mesh& mesh, int nrow, int ncol, const std::vector<float>& data, double baseRatio);

}  // namespace mjf
