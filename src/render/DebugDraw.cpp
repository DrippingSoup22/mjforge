#include "render/DebugDraw.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace mjf {

namespace {

const char* kLineVS = R"(#version 330 core
layout(location=0) in vec3 aA;
layout(location=1) in vec3 aB;
layout(location=2) in vec4 aColor;
layout(location=3) in vec3 aParams;  // side (-1/+1), end (0 = A, 1 = B), width px
uniform mat4 uViewProj;
uniform vec2 uViewport;
out vec4 vColor;
out float vEdge;
out float vHalf;
void main() {
  vec4 ca = uViewProj * vec4(aA, 1.0);
  vec4 cb = uViewProj * vec4(aB, 1.0);
  // Clip the segment against the near plane so both ends are in front of the camera.
  float eps = 1e-5;
  if (ca.w < eps) ca = mix(ca, cb, (eps - ca.w) / (cb.w - ca.w));
  if (cb.w < eps) cb = mix(cb, ca, (eps - cb.w) / (ca.w - cb.w));
  vec2 sa = ca.xy / ca.w * uViewport * 0.5;
  vec2 sb = cb.xy / cb.w * uViewport * 0.5;
  vec2 d = sb - sa;
  float len = length(d);
  vec2 dir = len > 1e-6 ? d / len : vec2(1.0, 0.0);
  vec2 nrm = vec2(-dir.y, dir.x);
  float hw = aParams.z * 0.5 + 1.0;
  vec4 c = aParams.y < 0.5 ? ca : cb;
  vec2 ext = dir * (aParams.y < 0.5 ? -1.0 : 1.0) * 0.5;  // tiny cap extension
  c.xy += (nrm * aParams.x * hw + ext) / (uViewport * 0.5) * c.w;
  gl_Position = c;
  vColor = aColor;
  vEdge = aParams.x * hw;
  vHalf = aParams.z * 0.5;
}
)";

const char* kLineFS = R"(#version 330 core
in vec4 vColor;
in float vEdge;
in float vHalf;
out vec4 fragColor;
void main() {
  float a = clamp(vHalf + 0.5 - abs(vEdge), 0.0, 1.0);
  fragColor = vec4(vColor.rgb, vColor.a * a);
}
)";

const char* kTriVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec4 aColor;
uniform mat4 uViewProj;
out vec4 vColor;
void main() { vColor = aColor; gl_Position = uViewProj * vec4(aPos, 1.0); }
)";

const char* kTriFS = R"(#version 330 core
in vec4 vColor;
out vec4 fragColor;
void main() { fragColor = vColor; }
)";

}  // namespace

bool DebugDraw::init() {
  if (!lineShader_.build(kLineVS, kLineFS) || !triShader_.build(kTriVS, kTriFS)) return false;
  glGenVertexArrays(1, &lineVao_);
  glGenBuffers(1, &lineVbo_);
  glBindVertexArray(lineVao_);
  glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
  const GLsizei ls = sizeof(LineVert);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, ls, (void*)offsetof(LineVert, a));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, ls, (void*)offsetof(LineVert, b));
  glEnableVertexAttribArray(2);
  glVertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, ls, (void*)offsetof(LineVert, color));
  glEnableVertexAttribArray(3);
  glVertexAttribPointer(3, 3, GL_FLOAT, GL_FALSE, ls, (void*)offsetof(LineVert, side));
  glGenVertexArrays(1, &triVao_);
  glGenBuffers(1, &triVbo_);
  glBindVertexArray(triVao_);
  glBindBuffer(GL_ARRAY_BUFFER, triVbo_);
  const GLsizei ts = sizeof(TriVert);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, ts, (void*)offsetof(TriVert, p));
  glEnableVertexAttribArray(1);
  glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, ts, (void*)offsetof(TriVert, color));
  glBindVertexArray(0);
  return true;
}

void DebugDraw::release() {
  if (lineVbo_) glDeleteBuffers(1, &lineVbo_);
  if (lineVao_) glDeleteVertexArrays(1, &lineVao_);
  if (triVbo_) glDeleteBuffers(1, &triVbo_);
  if (triVao_) glDeleteVertexArrays(1, &triVao_);
  lineVbo_ = lineVao_ = triVbo_ = triVao_ = 0;
}

void DebugDraw::clear() {
  for (int l = 0; l < 2; ++l) {
    lines_[l].clear();
    tris_[l].clear();
  }
}

void DebugDraw::line(const Vec3& a, const Vec3& b, const Color& c, float w, Layer layer) {
  auto mk = [&](float side, float end) {
    LineVert v;
    v.a[0] = float(a.x); v.a[1] = float(a.y); v.a[2] = float(a.z);
    v.b[0] = float(b.x); v.b[1] = float(b.y); v.b[2] = float(b.z);
    v.color[0] = c.r; v.color[1] = c.g; v.color[2] = c.b; v.color[3] = c.a;
    v.side = side;
    v.end = end;
    v.width = w;
    return v;
  };
  auto& L = lines_[layer];
  LineVert v0 = mk(-1, 0), v1 = mk(1, 0), v2 = mk(1, 1), v3 = mk(-1, 1);
  L.insert(L.end(), {v0, v1, v2, v0, v2, v3});
}

void DebugDraw::triangle(const Vec3& a, const Vec3& b, const Vec3& c, const Color& col, Layer layer) {
  auto mk = [&](const Vec3& p) {
    TriVert v;
    v.p[0] = float(p.x); v.p[1] = float(p.y); v.p[2] = float(p.z);
    v.color[0] = col.r; v.color[1] = col.g; v.color[2] = col.b; v.color[3] = col.a;
    return v;
  };
  tris_[layer].insert(tris_[layer].end(), {mk(a), mk(b), mk(c)});
}

void DebugDraw::circle(const Vec3& center, const Vec3& normal, double r, const Color& c, float w, Layer layer, int seg) {
  Vec3 n = normalize(normal);
  Vec3 u = perpendicular(n), v = cross(n, u);
  Vec3 prev = center + u * r;
  for (int i = 1; i <= seg; ++i) {
    double t = 2 * kPi * i / seg;
    Vec3 p = center + (u * std::cos(t) + v * std::sin(t)) * r;
    line(prev, p, c, w, layer);
    prev = p;
  }
}

void DebugDraw::arc(const Vec3& center, const Vec3& normal, const Vec3& from, double r, double a0, double a1,
                    const Color& c, float w, Layer layer) {
  Vec3 n = normalize(normal);
  Vec3 u = normalize(from - n * dot(from, n), perpendicular(n));
  Vec3 v = cross(n, u);
  int seg = std::max(4, int(std::fabs(a1 - a0) / (2 * kPi) * 64));
  Vec3 prev = center + (u * std::cos(a0) + v * std::sin(a0)) * r;
  for (int i = 1; i <= seg; ++i) {
    double t = a0 + (a1 - a0) * i / seg;
    Vec3 p = center + (u * std::cos(t) + v * std::sin(t)) * r;
    line(prev, p, c, w, layer);
    prev = p;
  }
}

void DebugDraw::sector(const Vec3& center, const Vec3& normal, const Vec3& from, double r, double a0, double a1,
                       const Color& c, Layer layer) {
  Vec3 n = normalize(normal);
  Vec3 u = normalize(from - n * dot(from, n), perpendicular(n));
  Vec3 v = cross(n, u);
  int seg = std::max(4, int(std::fabs(a1 - a0) / (2 * kPi) * 64));
  Vec3 prev = center + (u * std::cos(a0) + v * std::sin(a0)) * r;
  for (int i = 1; i <= seg; ++i) {
    double t = a0 + (a1 - a0) * i / seg;
    Vec3 p = center + (u * std::cos(t) + v * std::sin(t)) * r;
    triangle(center, prev, p, c, layer);
    prev = p;
  }
}

void DebugDraw::arrow(const Vec3& from, const Vec3& to, const Color& c, float w, Layer layer, double headFrac) {
  Vec3 d = to - from;
  double len = length(d);
  if (len < 1e-9) return;
  Vec3 dir = d / len;
  double hl = len * headFrac;
  Vec3 base = to - dir * hl;
  line(from, base, c, w, layer);
  Vec3 u = perpendicular(dir), v = cross(dir, u);
  double hr = hl * 0.45;
  const int n = 12;
  for (int i = 0; i < n; ++i) {
    double t0 = 2 * kPi * i / n, t1 = 2 * kPi * (i + 1) / n;
    Vec3 p0 = base + (u * std::cos(t0) + v * std::sin(t0)) * hr;
    Vec3 p1 = base + (u * std::cos(t1) + v * std::sin(t1)) * hr;
    triangle(p0, p1, to, c, layer);
    triangle(p1, p0, base, Color{c.r * 0.7f, c.g * 0.7f, c.b * 0.7f, c.a}, layer);
  }
}

void DebugDraw::axes(const Pose& pose, double size, float w, Layer layer, float alpha) {
  line(pose.pos, pose.apply({size, 0, 0}), {0.92f, 0.30f, 0.30f, alpha}, w, layer);
  line(pose.pos, pose.apply({0, size, 0}), {0.40f, 0.85f, 0.35f, alpha}, w, layer);
  line(pose.pos, pose.apply({0, 0, size}), {0.35f, 0.55f, 0.95f, alpha}, w, layer);
}

void DebugDraw::box(const Pose& pose, const Vec3& h, const Color& c, float w, Layer layer) {
  Vec3 p[8];
  for (int i = 0; i < 8; ++i) p[i] = pose.apply({(i & 1) ? h.x : -h.x, (i & 2) ? h.y : -h.y, (i & 4) ? h.z : -h.z});
  const int e[12][2] = {{0, 1}, {2, 3}, {4, 5}, {6, 7}, {0, 2}, {1, 3}, {4, 6}, {5, 7}, {0, 4}, {1, 5}, {2, 6}, {3, 7}};
  for (const auto& k : e) line(p[k[0]], p[k[1]], c, w, layer);
}

void DebugDraw::cross3(const Vec3& p, double s, const Color& c, float w, Layer layer) {
  line(p - Vec3{s, 0, 0}, p + Vec3{s, 0, 0}, c, w, layer);
  line(p - Vec3{0, s, 0}, p + Vec3{0, s, 0}, c, w, layer);
  line(p - Vec3{0, 0, s}, p + Vec3{0, 0, s}, c, w, layer);
}

void DebugDraw::renderLayer(int layer, const Mat4& viewProj, int w, int h) {
  if (!tris_[layer].empty()) {
    triShader_.use();
    triShader_.set("uViewProj", viewProj);
    glBindVertexArray(triVao_);
    glBindBuffer(GL_ARRAY_BUFFER, triVbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(tris_[layer].size() * sizeof(TriVert)), tris_[layer].data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(tris_[layer].size()));
  }
  if (!lines_[layer].empty()) {
    lineShader_.use();
    lineShader_.set("uViewProj", viewProj);
    lineShader_.set("uViewport", float(w), float(h));
    glBindVertexArray(lineVao_);
    glBindBuffer(GL_ARRAY_BUFFER, lineVbo_);
    glBufferData(GL_ARRAY_BUFFER, GLsizeiptr(lines_[layer].size() * sizeof(LineVert)), lines_[layer].data(), GL_STREAM_DRAW);
    glDrawArrays(GL_TRIANGLES, 0, GLsizei(lines_[layer].size()));
  }
}

void DebugDraw::render(const Mat4& viewProj, int w, int h) {
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glDisable(GL_CULL_FACE);
  glDepthMask(GL_FALSE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LEQUAL);
  renderLayer(World, viewProj, w, h);
  glDisable(GL_DEPTH_TEST);
  renderLayer(Overlay, viewProj, w, h);
  glEnable(GL_DEPTH_TEST);
  glDepthMask(GL_TRUE);
  glBindVertexArray(0);
}

}  // namespace mjf
