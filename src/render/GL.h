// OpenGL include and small RAII helpers.
#pragma once

#include <glad/gl.h>

#include <string>
#include <unordered_map>

#include "core/Math.h"

namespace mjf {

class Shader {
 public:
  Shader() = default;
  ~Shader();
  Shader(const Shader&) = delete;
  Shader& operator=(const Shader&) = delete;

  bool build(const char* vs, const char* fs, std::string* log = nullptr);
  void use() const { glUseProgram(prog_); }
  GLuint id() const { return prog_; }
  GLint loc(const char* name);

  void set(const char* n, int v) { glUniform1i(loc(n), v); }
  void set(const char* n, float v) { glUniform1f(loc(n), v); }
  void set(const char* n, float x, float y) { glUniform2f(loc(n), x, y); }
  void set(const char* n, float x, float y, float z) { glUniform3f(loc(n), x, y, z); }
  void set(const char* n, float x, float y, float z, float w) { glUniform4f(loc(n), x, y, z, w); }
  void set(const char* n, const Vec3& v) { glUniform3f(loc(n), float(v.x), float(v.y), float(v.z)); }
  void set(const char* n, const Color& c) { glUniform4f(loc(n), c.r, c.g, c.b, c.a); }
  void set(const char* n, const Mat4& m) { glUniformMatrix4fv(loc(n), 1, GL_FALSE, m.m); }

 private:
  GLuint prog_ = 0;
  std::unordered_map<std::string, GLint> locs_;
};

// Multisampled color+depth render target with a resolved texture for display in ImGui.
class RenderTarget {
 public:
  ~RenderTarget();
  void resize(int w, int h, int samples);
  void bind();
  void resolve();  // MSAA -> texture
  GLuint texture() const { return resolveTex_; }
  int width() const { return w_; }
  int height() const { return h_; }

 private:
  void release();
  int w_ = 0, h_ = 0, samples_ = 0;
  GLuint msFbo_ = 0, msColor_ = 0, msDepth_ = 0;
  GLuint resolveFbo_ = 0, resolveTex_ = 0;
};

}  // namespace mjf
