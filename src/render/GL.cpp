#include "render/GL.h"

#include <algorithm>

#include <cstdio>
#include <vector>

namespace mjf {

Shader::~Shader() {
  if (prog_) glDeleteProgram(prog_);
}

static GLuint compileStage(GLenum type, const char* src, std::string* log) {
  GLuint s = glCreateShader(type);
  glShaderSource(s, 1, &src, nullptr);
  glCompileShader(s);
  GLint ok = 0;
  glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
  if (!ok) {
    char buf[4096];
    glGetShaderInfoLog(s, sizeof buf, nullptr, buf);
    if (log) *log += buf;
    std::fprintf(stderr, "Shader compile error:\n%s\n", buf);
    glDeleteShader(s);
    return 0;
  }
  return s;
}

bool Shader::build(const char* vs, const char* fs, std::string* log) {
  GLuint v = compileStage(GL_VERTEX_SHADER, vs, log);
  GLuint f = compileStage(GL_FRAGMENT_SHADER, fs, log);
  if (!v || !f) return false;
  GLuint p = glCreateProgram();
  glAttachShader(p, v);
  glAttachShader(p, f);
  glLinkProgram(p);
  glDeleteShader(v);
  glDeleteShader(f);
  GLint ok = 0;
  glGetProgramiv(p, GL_LINK_STATUS, &ok);
  if (!ok) {
    char buf[4096];
    glGetProgramInfoLog(p, sizeof buf, nullptr, buf);
    if (log) *log += buf;
    std::fprintf(stderr, "Shader link error:\n%s\n", buf);
    glDeleteProgram(p);
    return false;
  }
  if (prog_) glDeleteProgram(prog_);
  prog_ = p;
  locs_.clear();
  return true;
}

GLint Shader::loc(const char* name) {
  auto it = locs_.find(name);
  if (it != locs_.end()) return it->second;
  GLint l = glGetUniformLocation(prog_, name);
  locs_[name] = l;
  return l;
}

RenderTarget::~RenderTarget() { release(); }

void RenderTarget::release() {
  if (msFbo_) glDeleteFramebuffers(1, &msFbo_);
  if (msColor_) glDeleteRenderbuffers(1, &msColor_);
  if (msDepth_) glDeleteRenderbuffers(1, &msDepth_);
  if (resolveFbo_) glDeleteFramebuffers(1, &resolveFbo_);
  if (resolveTex_) glDeleteTextures(1, &resolveTex_);
  msFbo_ = msColor_ = msDepth_ = resolveFbo_ = resolveTex_ = 0;
}

void RenderTarget::resize(int w, int h, int samples) {
  w = std::max(1, w);
  h = std::max(1, h);
  if (w == w_ && h == h_ && samples == samples_ && msFbo_) return;
  release();
  w_ = w;
  h_ = h;
  samples_ = samples;
  GLint maxSamples = 0;
  glGetIntegerv(GL_MAX_SAMPLES, &maxSamples);
  int s = std::min(samples, int(maxSamples));

  glGenFramebuffers(1, &msFbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, msFbo_);
  glGenRenderbuffers(1, &msColor_);
  glBindRenderbuffer(GL_RENDERBUFFER, msColor_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, s, GL_RGBA8, w, h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_RENDERBUFFER, msColor_);
  glGenRenderbuffers(1, &msDepth_);
  glBindRenderbuffer(GL_RENDERBUFFER, msDepth_);
  glRenderbufferStorageMultisample(GL_RENDERBUFFER, s, GL_DEPTH24_STENCIL8, w, h);
  glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_STENCIL_ATTACHMENT, GL_RENDERBUFFER, msDepth_);
  if (glCheckFramebufferStatus(GL_FRAMEBUFFER) != GL_FRAMEBUFFER_COMPLETE)
    std::fprintf(stderr, "Viewport framebuffer incomplete\n");

  glGenTextures(1, &resolveTex_);
  glBindTexture(GL_TEXTURE_2D, resolveTex_);
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA8, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
  glGenFramebuffers(1, &resolveFbo_);
  glBindFramebuffer(GL_FRAMEBUFFER, resolveFbo_);
  glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, resolveTex_, 0);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void RenderTarget::bind() {
  glBindFramebuffer(GL_FRAMEBUFFER, msFbo_);
  glViewport(0, 0, w_, h_);
}

void RenderTarget::resolve() {
  glBindFramebuffer(GL_READ_FRAMEBUFFER, msFbo_);
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, resolveFbo_);
  glBlitFramebuffer(0, 0, w_, h_, 0, 0, w_, h_, GL_COLOR_BUFFER_BIT, GL_NEAREST);
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

}  // namespace mjf
