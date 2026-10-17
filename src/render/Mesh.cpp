#include "render/Mesh.h"

#include <cmath>

namespace mjf {

void Mesh::upload(const std::vector<float>& verts, const std::vector<uint32_t>& idx) {
  release();
  glGenVertexArrays(1, &vao);
  glBindVertexArray(vao);
  glGenBuffers(1, &vbo);
  glBindBuffer(GL_ARRAY_BUFFER, vbo);
  glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(verts.size() * sizeof(float)), verts.data(), GL_STATIC_DRAW);
  glGenBuffers(1, &ibo);
  glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
  glBufferData(GL_ELEMENT_ARRAY_BUFFER, GLsizeiptr(idx.size() * sizeof(uint32_t)), idx.data(), GL_STATIC_DRAW);
  const GLsizei stride = 7 * sizeof(float);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 1, GL_FLOAT, GL_FALSE, stride, (void*)(6 * sizeof(float)));
  glBindVertexArray(0);
  count = GLsizei(idx.size());
}

void Mesh::draw() const {
  if (!vao) return;
  glBindVertexArray(vao);
  glDrawElements(GL_TRIANGLES, count, GL_UNSIGNED_INT, nullptr);
}

void Mesh::release() {
  if (ibo) glDeleteBuffers(1, &ibo);
  if (vbo) glDeleteBuffers(1, &vbo);
  if (vao) glDeleteVertexArrays(1, &vao);
  vao = vbo = ibo = 0;
  count = 0;
}

namespace {

struct Builder {
  std::vector<float> v;
  std::vector<uint32_t> i;
  uint32_t vert(const Vec3& p, const Vec3& n, float side = 0) {
    v.insert(v.end(), {float(p.x), float(p.y), float(p.z), float(n.x), float(n.y), float(n.z), side});
    return uint32_t(v.size() / 7 - 1);
  }
  void tri(uint32_t a, uint32_t b, uint32_t c) { i.insert(i.end(), {a, b, c}); }
  void quad(uint32_t a, uint32_t b, uint32_t c, uint32_t d) { tri(a, b, c); tri(a, c, d); }
};

// Rings of a UV sphere from the north pole (phi=0) to the south pole (phi=pi). When `capsule`
// is true the equator ring is duplicated: the upper copy gets side=+1, the lower side=-1, and
// the band between them becomes the capsule's cylinder.
void buildSphereLike(Builder& b, int slices, int stacks, bool capsule) {
  std::vector<std::vector<uint32_t>> rings;
  for (int s = 0; s <= stacks; ++s) {
    double phi = kPi * s / stacks;
    double z = std::cos(phi), r = std::sin(phi);
    auto addRing = [&](float side) {
      std::vector<uint32_t> ring;
      for (int k = 0; k <= slices; ++k) {
        double th = 2 * kPi * k / slices;
        Vec3 p{r * std::cos(th), r * std::sin(th), z};
        ring.push_back(b.vert(p, p, side));
      }
      rings.push_back(ring);
    };
    if (capsule) {
      if (2 * s == stacks) {
        addRing(1.0f);
        addRing(-1.0f);
      } else {
        addRing(2 * s < stacks ? 1.0f : -1.0f);
      }
    } else {
      addRing(0.0f);
    }
  }
  for (size_t r = 0; r + 1 < rings.size(); ++r)
    for (int k = 0; k < slices; ++k)
      b.quad(rings[r][k], rings[r + 1][k], rings[r + 1][k + 1], rings[r][k + 1]);
}

}  // namespace

void PrimitiveMeshes::build() {
  {
    Builder b;
    buildSphereLike(b, 48, 24, false);
    sphere.upload(b.v, b.i);
  }
  {
    Builder b;
    buildSphereLike(b, 40, 24, true);
    capsule.upload(b.v, b.i);
  }
  {  // cylinder, radius 1, z in [-1, 1]
    Builder b;
    const int n = 48;
    std::vector<uint32_t> top, bot;
    for (int k = 0; k <= n; ++k) {
      double th = 2 * kPi * k / n;
      Vec3 d{std::cos(th), std::sin(th), 0};
      top.push_back(b.vert({d.x, d.y, 1}, d));
      bot.push_back(b.vert({d.x, d.y, -1}, d));
    }
    for (int k = 0; k < n; ++k) b.quad(bot[k], bot[k + 1], top[k + 1], top[k]);
    for (int cap = 0; cap < 2; ++cap) {
      double z = cap ? 1 : -1;
      Vec3 nrm{0, 0, z};
      uint32_t c = b.vert({0, 0, z}, nrm);
      std::vector<uint32_t> ring;
      for (int k = 0; k <= n; ++k) {
        double th = 2 * kPi * k / n;
        ring.push_back(b.vert({std::cos(th), std::sin(th), z}, nrm));
      }
      for (int k = 0; k < n; ++k) {
        if (cap) b.tri(c, ring[k], ring[k + 1]);
        else b.tri(c, ring[k + 1], ring[k]);
      }
    }
    cylinder.upload(b.v, b.i);
  }
  {  // box, half-size 1
    Builder b;
    for (int axis = 0; axis < 3; ++axis) {
      for (int sgn = -1; sgn <= 1; sgn += 2) {
        Vec3 n{};
        n[axis] = sgn;
        Vec3 u{}, w{};
        u[(axis + 1) % 3] = 1;
        w[(axis + 2) % 3] = 1;
        if (sgn < 0) std::swap(u, w);
        uint32_t a = b.vert(n - u - w, n), c = b.vert(n + u - w, n), d = b.vert(n + u + w, n), e = b.vert(n - u + w, n);
        b.quad(a, c, d, e);
      }
    }
    box.upload(b.v, b.i);
  }
  {  // plane: [-1,1]^2 at z = 0 facing +z
    Builder b;
    Vec3 n{0, 0, 1};
    uint32_t a = b.vert({-1, -1, 0}, n), c = b.vert({1, -1, 0}, n), d = b.vert({1, 1, 0}, n), e = b.vert({-1, 1, 0}, n);
    b.quad(a, c, d, e);
    plane.upload(b.v, b.i);
  }
  {  // cone: base radius 1 at z = 0, tip at z = 1
    Builder b;
    const int n = 32;
    double slope = 1.0 / std::sqrt(2.0);
    for (int k = 0; k < n; ++k) {
      double t0 = 2 * kPi * k / n, t1 = 2 * kPi * (k + 1) / n, tm = (t0 + t1) / 2;
      Vec3 p0{std::cos(t0), std::sin(t0), 0}, p1{std::cos(t1), std::sin(t1), 0};
      Vec3 n0{std::cos(t0) * slope, std::sin(t0) * slope, slope}, n1{std::cos(t1) * slope, std::sin(t1) * slope, slope};
      Vec3 nm{std::cos(tm) * slope, std::sin(tm) * slope, slope};
      uint32_t a = b.vert(p0, n0), c = b.vert(p1, n1), tip = b.vert({0, 0, 1}, nm);
      b.tri(a, c, tip);
      uint32_t ca = b.vert({0, 0, 0}, {0, 0, -1}), cb = b.vert(p1, {0, 0, -1}), cc = b.vert(p0, {0, 0, -1});
      b.tri(ca, cb, cc);
    }
    cone.upload(b.v, b.i);
  }
}

void PrimitiveMeshes::release() {
  sphere.release();
  capsule.release();
  cylinder.release();
  box.release();
  plane.release();
  cone.release();
}

void buildHfieldMesh(Mesh& mesh, int nrow, int ncol, const std::vector<float>& data, double baseRatio) {
  if (nrow < 2 || ncol < 2 || int(data.size()) < nrow * ncol) {
    mesh.release();
    return;
  }
  Builder b;
  auto height = [&](int r, int c) { return double(data[size_t(r) * ncol + c]); };
  auto pos = [&](int r, int c) {
    double x = -1.0 + 2.0 * c / (ncol - 1);
    double y = 1.0 - 2.0 * r / (nrow - 1);  // row 0 is the +Y edge
    return Vec3{x, y, height(r, c)};
  };
  std::vector<uint32_t> idx(size_t(nrow) * ncol);
  for (int r = 0; r < nrow; ++r) {
    for (int c = 0; c < ncol; ++c) {
      double hl = height(r, std::max(0, c - 1)), hr = height(r, std::min(ncol - 1, c + 1));
      double hu = height(std::max(0, r - 1), c), hd = height(std::min(nrow - 1, r + 1), c);
      double dx = 2.0 / (ncol - 1) * (std::min(ncol - 1, c + 1) - std::max(0, c - 1));
      double dy = 2.0 / (nrow - 1) * (std::min(nrow - 1, r + 1) - std::max(0, r - 1));
      // Normal in the unit-scaled space; the shader rescales by the per-axis size.
      // +y is towards smaller row indices, so dh/dy = (hu - hd) / dy.
      Vec3 n = normalize(Vec3{-(hr - hl) / dx, (hd - hu) / dy, 1.0});
      idx[size_t(r) * ncol + c] = b.vert(pos(r, c), n);
    }
  }
  for (int r = 0; r + 1 < nrow; ++r)
    for (int c = 0; c + 1 < ncol; ++c) {
      uint32_t a = idx[size_t(r) * ncol + c], bb = idx[size_t(r) * ncol + c + 1];
      uint32_t cc = idx[size_t(r + 1) * ncol + c + 1], d = idx[size_t(r + 1) * ncol + c];
      b.tri(a, d, cc);
      b.tri(a, cc, bb);
    }
  // Base box (skirt) so the terrain looks solid from the sides.
  double zb = -std::max(0.0, baseRatio);
  auto skirt = [&](int r0, int c0, int r1, int c1, const Vec3& n) {
    Vec3 p0 = pos(r0, c0), p1 = pos(r1, c1);
    uint32_t a = b.vert(p0, n), bb = b.vert(p1, n), cc = b.vert({p1.x, p1.y, zb}, n), d = b.vert({p0.x, p0.y, zb}, n);
    b.quad(a, d, cc, bb);
  };
  for (int c = 0; c + 1 < ncol; ++c) {
    skirt(0, c + 1, 0, c, {0, 1, 0});
    skirt(nrow - 1, c, nrow - 1, c + 1, {0, -1, 0});
  }
  for (int r = 0; r + 1 < nrow; ++r) {
    skirt(r, 0, r + 1, 0, {-1, 0, 0});
    skirt(r + 1, ncol - 1, r, ncol - 1, {1, 0, 0});
  }
  mesh.upload(b.v, b.i);
}

void buildTriangleMesh(Mesh& mesh, const std::vector<float>& vert, const std::vector<int>& face) {
  Builder b;
  size_t nv = vert.size() / 3;
  auto P = [&](int i) { return Vec3{vert[3 * size_t(i)], vert[3 * size_t(i) + 1], vert[3 * size_t(i) + 2]}; };
  for (size_t t = 0; t + 2 < face.size(); t += 3) {
    if (size_t(face[t]) >= nv || size_t(face[t + 1]) >= nv || size_t(face[t + 2]) >= nv) continue;
    Vec3 a = P(face[t]), c1 = P(face[t + 1]), c2 = P(face[t + 2]);
    Vec3 n = normalize(cross(c1 - a, c2 - a), {0, 0, 1});
    b.tri(b.vert(a, n), b.vert(c1, n), b.vert(c2, n));
  }
  if (b.i.empty()) {
    mesh.release();
    return;
  }
  mesh.upload(b.v, b.i);
}

}  // namespace mjf
