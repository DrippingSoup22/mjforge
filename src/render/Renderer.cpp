#include "render/Renderer.h"

#include <algorithm>
#include <cmath>

namespace mjf {

namespace {

const char* kLitVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
layout(location=1) in vec3 aNormal;
layout(location=2) in float aSide;
uniform mat4 uModel;
uniform mat4 uViewProj;
uniform mat4 uLightViewProj;
uniform vec3 uScale;
uniform vec3 uLocalOffset;
uniform vec2 uCapsule;
uniform int uMode;
out vec3 vWorld;
out vec3 vNormal;
out vec3 vLocal;
out vec4 vLight;
void main() {
  vec3 p, n;
  if (uMode == 1) {
    p = aPos * uCapsule.x + vec3(0.0, 0.0, aSide * uCapsule.y);
    n = aNormal;
  } else {
    p = aPos * uScale + uLocalOffset;
    n = aNormal / max(abs(uScale), vec3(1e-9)) * sign(uScale);
  }
  vec4 w = uModel * vec4(p, 1.0);
  vWorld = w.xyz;
  vNormal = mat3(uModel) * n;
  vLocal = p;
  vLight = uLightViewProj * w;
  gl_Position = uViewProj * w;
}
)";

const char* kLitFS = R"(#version 330 core
in vec3 vWorld;
in vec3 vNormal;
in vec3 vLocal;
in vec4 vLight;
uniform vec4 uColor;
uniform vec3 uCamPos;
uniform vec3 uLightDir;
uniform float uSpecular;
uniform float uShininess;
uniform float uEmission;
uniform int uChecker;
uniform vec4 uChecker2;
uniform float uCheckerDensity;
uniform int uEdges;
uniform vec4 uEdgeColor;
uniform int uHighlight;
uniform int uHeightTint;
uniform float uHeightMax;
uniform int uShadows;
uniform sampler2DShadow uShadowMap;
uniform float uShadowTexel;
out vec4 fragColor;

float shadowFactor(vec3 N, vec3 L) {
  vec3 p = vLight.xyz / vLight.w * 0.5 + 0.5;
  if (p.x <= 0.0 || p.x >= 1.0 || p.y <= 0.0 || p.y >= 1.0 || p.z >= 1.0) return 1.0;
  float bias = 0.0008 + 0.0025 * (1.0 - max(dot(N, L), 0.0));
  float s = 0.0;
  for (int x = -1; x <= 1; ++x)
    for (int y = -1; y <= 1; ++y)
      s += texture(uShadowMap, vec3(p.xy + vec2(x, y) * uShadowTexel, p.z - bias));
  return s / 9.0;
}

void main() {
  vec3 N = normalize(vNormal);
  if (!gl_FrontFacing) N = -N;
  vec3 V = normalize(uCamPos - vWorld);
  vec3 L = normalize(-uLightDir);
  vec3 base = uColor.rgb;
  if (uChecker == 1) {
    vec2 q = vLocal.xy * uCheckerDensity;
    vec2 f = floor(q);
    float parity = mod(f.x + f.y, 2.0);
    // Fade the pattern out where it would alias.
    float blur = clamp(max(fwidth(q).x, fwidth(q).y) - 0.3, 0.0, 1.0);
    base = mix(mix(uColor.rgb, uChecker2.rgb, parity), (uColor.rgb + uChecker2.rgb) * 0.5, blur);
    if (uEdges == 1) {
      vec2 tile = q * 0.5;
      vec2 g = abs(fract(tile - 0.5) - 0.5) / max(fwidth(tile), vec2(1e-6));
      float line = (1.0 - min(min(g.x, g.y), 1.0)) * (1.0 - blur);
      base = mix(base, uEdgeColor.rgb, line * 0.75);
    }
  }
  if (uHeightTint == 1) {
    // Height fields: darker valleys, contour lines and a 0.5 m grid make the relief readable.
    float h = clamp(vLocal.z / max(uHeightMax, 1e-6), 0.0, 1.0);
    base *= mix(0.55, 1.2, h);
    float c = h * 8.0;
    float gc = abs(fract(c - 0.5) - 0.5) / max(fwidth(c), 1e-6);
    base = mix(base, base * 0.6, (1.0 - min(gc, 1.0)) * 0.5);
    vec2 q = vLocal.xy * 2.0;
    vec2 gg = abs(fract(q - 0.5) - 0.5) / max(fwidth(q), vec2(1e-6));
    base = mix(base, vec3(0.92), (1.0 - min(min(gg.x, gg.y), 1.0)) * 0.22);
  }
  float sh = uShadows == 1 ? shadowFactor(N, L) : 1.0;
  float diff = max(dot(N, L), 0.0);
  vec3 H = normalize(L + V);
  float specPow = mix(8.0, 96.0, clamp(uShininess, 0.0, 1.0));
  float spec = pow(max(dot(N, H), 0.0), specPow) * uSpecular * 0.55;
  float hemi = 0.5 + 0.5 * N.z;
  vec3 ambient = mix(vec3(0.17, 0.16, 0.16), vec3(0.38, 0.41, 0.46), hemi);
  float head = 0.20 * max(dot(N, V), 0.0);
  vec3 col = base * (ambient + head + diff * sh * vec3(0.90, 0.87, 0.82)) + vec3(spec * sh) + base * uEmission;
  if (uHighlight > 0) {
    vec3 hc = uHighlight == 1 ? vec3(0.55, 0.78, 1.0) : vec3(1.0, 0.56, 0.20);
    float rim = pow(1.0 - max(dot(N, V), 0.0), 2.0);
    float amt = uHighlight == 1 ? 0.55 : (uHighlight == 3 ? 1.0 : 0.8);
    col = mix(col, hc, 0.20 * amt) + hc * rim * 0.85 * amt;
  }
  fragColor = vec4(col, uColor.a);
}
)";

const char* kDepthFS = R"(#version 330 core
void main() {}
)";

const char* kSkyVS = R"(#version 330 core
out vec2 vUv;
void main() {
  vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2);
  vUv = p;
  gl_Position = vec4(p * 2.0 - 1.0, 0.0, 1.0);
}
)";

const char* kSkyFS = R"(#version 330 core
in vec2 vUv;
uniform vec4 uTop;
uniform vec4 uBottom;
out vec4 fragColor;
void main() {
  float t = smoothstep(0.0, 1.0, vUv.y);
  fragColor = vec4(mix(uBottom.rgb, uTop.rgb, t), 1.0);
}
)";

const char* kGridVS = R"(#version 330 core
layout(location=0) in vec3 aPos;
uniform mat4 uViewProj;
uniform vec3 uCenter;
uniform float uExtent;
out vec3 vWorld;
void main() {
  vec3 p = vec3(uCenter.xy + aPos.xy * uExtent, 0.0);
  vWorld = p;
  gl_Position = uViewProj * vec4(p, 1.0);
}
)";

const char* kGridFS = R"(#version 330 core
in vec3 vWorld;
uniform vec3 uCamPos;
uniform float uAlpha;
uniform float uFade;
out vec4 fragColor;
float gridLine(vec2 coord, float width) {
  vec2 g = abs(fract(coord - 0.5) - 0.5) / max(fwidth(coord), vec2(1e-6));
  return 1.0 - min(min(g.x, g.y) / width, 1.0);
}
void main() {
  vec2 p = vWorld.xy;
  vec2 q = p * 10.0;
  float minorVisible = 1.0 - smoothstep(0.15, 0.45, max(fwidth(q).x, fwidth(q).y));
  float minor = gridLine(q, 1.0) * minorVisible;
  float major = gridLine(p, 1.3);
  float a = max(minor * 0.16, major * 0.40);
  vec3 col = vec3(0.66, 0.70, 0.76);
  vec2 fw = max(fwidth(p), vec2(1e-6));
  float ax = 1.0 - min(abs(p.y) / (fw.y * 1.6), 1.0);
  float ay = 1.0 - min(abs(p.x) / (fw.x * 1.6), 1.0);
  if (ax > 0.0) { col = mix(col, vec3(0.92, 0.32, 0.32), ax); a = max(a, ax * 0.8); }
  if (ay > 0.0) { col = mix(col, vec3(0.38, 0.82, 0.36), ay); a = max(a, ay * 0.8); }
  float fade = 1.0 - smoothstep(uFade * 0.35, uFade, length(uCamPos - vWorld));
  fragColor = vec4(col, a * fade * uAlpha);
}
)";

}  // namespace

bool Renderer::init() {
  if (!lit_.build(kLitVS, kLitFS) || !depth_.build(kLitVS, kDepthFS) || !sky_.build(kSkyVS, kSkyFS) ||
      !grid_.build(kGridVS, kGridFS))
    return false;
  meshes_.build();
  glGenVertexArrays(1, &emptyVao_);

  // Shadow map.
  glGenTextures(1, &shadowTex_);
  glBindTexture(GL_TEXTURE_2D, shadowTex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_DEPTH_COMPONENT24, shadowSize_, shadowSize_, 0, GL_DEPTH_COMPONENT, GL_FLOAT, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_MODE, GL_COMPARE_REF_TO_TEXTURE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_COMPARE_FUNC, GL_LEQUAL);
  glGenFramebuffers(1, &shadowFbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, shadowTex_, 0);
  glDrawBuffer(GL_NONE);
  glReadBuffer(GL_NONE);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  // Grid quad.
  const float quad[] = {-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, -1, 0, 1, 1, 0, -1, 1, 0};
  glGenVertexArrays(1, &gridVao_);
  glGenBuffers(1, &gridVbo_);
  glBindVertexArray(gridVao_);
  glBindBuffer(GL_ARRAY_BUFFER, gridVbo_);
  glBufferData(GL_ARRAY_BUFFER, sizeof quad, quad, GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), nullptr);
  glBindVertexArray(0);
  return true;
}

void Renderer::release() {
  meshes_.release();
  if (shadowFbo_) glDeleteFramebuffers(1, &shadowFbo_);
  if (shadowTex_) glDeleteTextures(1, &shadowTex_);
  if (emptyVao_) glDeleteVertexArrays(1, &emptyVao_);
  if (gridVbo_) glDeleteBuffers(1, &gridVbo_);
  if (gridVao_) glDeleteVertexArrays(1, &gridVao_);
  shadowFbo_ = shadowTex_ = emptyVao_ = gridVbo_ = gridVao_ = 0;
}

void Renderer::drawItem(Shader& sh, const DrawItem& it, bool depthOnly) {
  const Mesh* mesh = nullptr;
  Vec3 scale{1, 1, 1}, offset{};
  int mode = 0;
  // Shadows are soft anyway: never the finest level there.
  const int lod = std::min(kLodLevels - 1, depthOnly ? std::max(1, int(it.lod)) : int(it.lod));
  switch (it.shape) {
    case Shape::Sphere: mesh = &meshes_.sphere[lod]; scale = {it.size.x, it.size.x, it.size.x}; break;
    case Shape::Ellipsoid: mesh = &meshes_.sphere[lod]; scale = it.size; break;
    case Shape::Capsule: mesh = &meshes_.capsule[lod]; mode = 1; break;
    case Shape::Cylinder: mesh = &meshes_.cylinder[lod]; scale = {it.size.x, it.size.x, it.size.y}; break;
    case Shape::Box: mesh = &meshes_.box; scale = it.size; break;
    case Shape::Cone: mesh = &meshes_.cone; scale = it.size; break;
    case Shape::Plane:
      mesh = &meshes_.plane;
      scale = {it.size.x, it.size.y, 1};
      offset = it.localOffset;  // infinite planes: quad follows the camera, pattern stays put
      break;
    case Shape::Custom: mesh = it.mesh; scale = it.size; break;
  }
  if (!mesh || !mesh->valid()) return;
  sh.set("uModel", Mat4::fromPose(it.pose));
  sh.set("uMode", mode);
  sh.set("uCapsule", float(it.size.x), float(it.size.y));
  sh.set("uScale", scale);
  sh.set("uLocalOffset", offset);
  if (!depthOnly) {
    sh.set("uColor", it.color);
    sh.set("uSpecular", it.specular);
    sh.set("uShininess", it.shininess);
    sh.set("uEmission", it.emission);
    sh.set("uChecker", it.checker ? 1 : 0);
    sh.set("uHeightTint", it.heightTint ? 1 : 0);
    sh.set("uHeightMax", float(it.size.z));
    sh.set("uChecker2", it.checker2);
    sh.set("uCheckerDensity", it.checkerDensity);
    sh.set("uEdges", it.checkerEdges ? 1 : 0);
    sh.set("uEdgeColor", it.edgeColor);
    sh.set("uHighlight", int(it.highlight));
  }
  mesh->draw();
}

void Renderer::renderShadowMap(const std::vector<DrawItem>& items, const Mat4& lightViewProj) {
  glBindFramebuffer(GL_FRAMEBUFFER, shadowFbo_);
  glViewport(0, 0, shadowSize_, shadowSize_);
  glClear(GL_DEPTH_BUFFER_BIT);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glEnable(GL_POLYGON_OFFSET_FILL);
  glPolygonOffset(1.5f, 2.0f);
  depth_.use();
  depth_.set("uViewProj", lightViewProj);
  depth_.set("uLightViewProj", lightViewProj);
  for (const DrawItem& it : items) {
    if (!it.castShadow || it.shape == Shape::Plane || it.color.a < 0.5f) continue;
    drawItem(depth_, it, true);
  }
  glDisable(GL_POLYGON_OFFSET_FILL);
}

void Renderer::render(const std::vector<DrawItem>& items_, const OrbitCamera& cam, const FrameParams& fp, DebugDraw* dbg) {
  GLint targetFbo = 0;
  glGetIntegerv(GL_FRAMEBUFFER_BINDING, &targetFbo);
  const int w = std::max(1, fp.width), h = std::max(1, fp.height);
  const double aspect = double(w) / h;
  const Mat4 view = cam.view();
  const Mat4 proj = cam.proj(aspect);
  const Mat4 viewProj = proj * view;
  const Vec3 eye = cam.eye();
  const Vec3 lightDir = normalize(fp.lightDir);

  // Infinite planes become large quads centered under the camera target.
  std::vector<DrawItem> items = items_;
  // Level of detail from the projected radius in pixels.
  const double pixelsPerUnit = h / (2.0 * std::tan(deg2rad(cam.fovy) * 0.5));
  for (DrawItem& it : items) {
    double r = 0;
    switch (it.shape) {
      case Shape::Sphere: r = it.size.x; break;
      case Shape::Ellipsoid: r = std::max({it.size.x, it.size.y, it.size.z}); break;
      case Shape::Capsule: r = it.size.x + it.size.y; break;
      case Shape::Cylinder: r = std::hypot(it.size.x, it.size.y); break;
      default: continue;
    }
    double d = cam.ortho ? cam.distance : std::max(1e-3, length(it.pose.pos - eye));
    double px = r / d * pixelsPerUnit;
    it.lod = px > 60 ? 0 : px > 14 ? 1 : 2;
  }
  const double planeExtent = std::max(60.0, cam.distance * 30.0);
  for (DrawItem& it : items) {
    if (it.shape != Shape::Plane) continue;
    if (it.size.x <= 0 || it.size.y <= 0) {
      Vec3 lt = inverse(it.pose).apply(cam.target);
      if (it.size.x <= 0) { it.size.x = planeExtent; it.localOffset.x = std::round(lt.x); }
      if (it.size.y <= 0) { it.size.y = planeExtent; it.localOffset.y = std::round(lt.y); }
    }
  }

  // Shadow map fitted around the camera target.
  Mat4 lightViewProj;
  if (fp.shadows) {
    double R = clampv(cam.distance * 1.4, 1.5, 60.0);
    Vec3 c = cam.target;
    Vec3 up = std::fabs(lightDir.z) > 0.95 ? Vec3{0, 1, 0} : Vec3{0, 0, 1};
    Mat4 lv = Mat4::lookAt(c - lightDir * (R * 3), c, up);
    Mat4 lp = Mat4::ortho(-R, R, -R, R, 0.01, R * 6);
    lightViewProj = lp * lv;
    renderShadowMap(items, lightViewProj);
  }

  glBindFramebuffer(GL_FRAMEBUFFER, GLuint(targetFbo));
  glViewport(0, 0, w, h);
  glClearColor(fp.skyBottom.r, fp.skyBottom.g, fp.skyBottom.b, 1);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  // Sky gradient.
  glDisable(GL_DEPTH_TEST);
  glDepthMask(GL_FALSE);
  sky_.use();
  sky_.set("uTop", fp.skyTop);
  sky_.set("uBottom", fp.skyBottom);
  glBindVertexArray(emptyVao_);
  glDrawArrays(GL_TRIANGLES, 0, 3);
  glDepthMask(GL_TRUE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_LESS);
  glDisable(GL_CULL_FACE);
  glDisable(GL_BLEND);

  lit_.use();
  lit_.set("uViewProj", viewProj);
  lit_.set("uLightViewProj", lightViewProj);
  lit_.set("uCamPos", eye);
  lit_.set("uLightDir", lightDir);
  lit_.set("uShadows", fp.shadows ? 1 : 0);
  lit_.set("uShadowTexel", 1.0f / float(shadowSize_));
  glActiveTexture(GL_TEXTURE0);
  glBindTexture(GL_TEXTURE_2D, shadowTex_);
  lit_.set("uShadowMap", 0);

  // Planes have local coordinates centered at their origin; for camera-following planes we keep
  // the plane origin and simply draw a big quad, which is exact for the checker pattern.
  std::vector<const DrawItem*> transparent;
  for (const DrawItem& it : items) {
    bool translucent = it.color.a < 0.999f || (fp.xray > 0 && it.shape != Shape::Plane && it.highlight == 0);
    if (translucent) {
      transparent.push_back(&it);
      continue;
    }
    drawItem(lit_, it, false);
  }

  // Editor grid.
  if (fp.grid && fp.gridAlpha > 0.01f) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glEnable(GL_POLYGON_OFFSET_FILL);
    glPolygonOffset(-1.0f, -4.0f);
    grid_.use();
    grid_.set("uViewProj", viewProj);
    double ext = std::max(30.0, cam.distance * 12.0);
    grid_.set("uCenter", Vec3{std::round(cam.target.x), std::round(cam.target.y), 0});
    grid_.set("uExtent", float(ext));
    grid_.set("uCamPos", eye);
    grid_.set("uAlpha", fp.gridAlpha);
    grid_.set("uFade", float(std::max(12.0, cam.distance * 6.0)));
    glBindVertexArray(gridVao_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glDisable(GL_POLYGON_OFFSET_FILL);
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  // Translucent geometry, back to front.
  if (!transparent.empty()) {
    std::sort(transparent.begin(), transparent.end(), [&](const DrawItem* a, const DrawItem* b) {
      return length2(a->pose.pos - eye) > length2(b->pose.pos - eye);
    });
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    lit_.use();
    for (const DrawItem* it : transparent) {
      DrawItem copy = *it;
      if (fp.xray > 0 && copy.color.a >= 0.999f) copy.color.a = fp.xray;
      drawItem(lit_, copy, false);
    }
    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
  }

  if (dbg) dbg->render(viewProj, w, h);
  glBindVertexArray(0);
}

}  // namespace mjf
