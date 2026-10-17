// Analytic geometry for MuJoCo primitives: volume, inertia, extents and ray intersection.
#pragma once

#include "core/Document.h"

namespace mjf {

// Volume of a primitive (0 for plane / heightfield / mesh).
double primitiveVolume(GeomType type, const Vec3& size);
// Principal moments of inertia about the primitive's center, in its local frame, for mass m.
// Matches the formulas used by MuJoCo's compiler for solid bodies.
Vec3 primitiveInertia(GeomType type, const Vec3& size, double mass);
// Whether MuJoCo would let this geom contribute mass (planes and height fields never do;
// geoms outside the default inertiagrouprange 0..5 don't either).
bool geomHasMass(const Geom& g);
double geomMass(const Geom& g);

// Half-extents along the world axes of a primitive with rotation `rot`.
Vec3 primitiveWorldHalfExtents(GeomType type, const Vec3& size, const Mat3& rot);
Aabb primitiveWorldAabb(GeomType type, const Vec3& size, const Pose& pose);

// Ray intersection in world space. Returns the hit distance along the ray or a negative value.
// `pose` is the primitive's world pose. Planes are treated as finite rectangles (size x/y, or
// `infiniteExtent` when the size is zero).
double rayPrimitive(const Ray& ray, GeomType type, const Vec3& size, const Pose& pose,
                    double infiniteExtent = 50.0, Vec3* normal = nullptr);
// Mass properties of a closed triangle mesh per unit density, in the mesh frame (exact volume
// integration over signed tetrahedra; inside-out meshes are handled). Cached per vertex buffer.
struct MeshMass {
  double volume = 0;
  Vec3 center;                   // center of volume
  Mat3 inertia = Mat3::zero();   // about the center, for density 1
};
const MeshMass& meshMassProps(const MeshAsset& mesh);
// Bounds of a mesh in its own frame (cached).
const Aabb& meshBounds(const MeshAsset& mesh);

// Geom helpers that also handle meshes (the primitive versions above can't see mesh assets).
double geomMass(const Document& doc, const Geom& g);
Aabb geomWorldAabb(const Document& doc, const Geom& g, const Pose& world);
double rayGeom(const Document& doc, const Ray& ray, const Geom& g, const Pose& world, double infiniteExtent = 50.0,
               Vec3* normal = nullptr);

// Distance between a ray and a segment, plus the parameters of the closest points.
double raySegmentDistance(const Ray& ray, const Vec3& a, const Vec3& b, double* tRay, double* tSeg);

}  // namespace mjf
