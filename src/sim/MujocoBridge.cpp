#include "sim/MujocoBridge.h"

#include <chrono>
#include <cstring>
#include <filesystem>
#include <iterator>
#include <stdexcept>

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#else
#include <unistd.h>
#endif

namespace mjf {

namespace {

thread_local std::vector<std::string> g_warnings;

void errorHandler(const char* msg) { throw std::runtime_error(std::string("MuJoCo error: ") + msg); }

void warningHandler(const char* msg) {
  if (g_warnings.size() < 64) g_warnings.emplace_back(msg);
}

}  // namespace

void installMujocoErrorHandlers() {
  mju_user_error = errorHandler;
  mju_user_warning = warningHandler;
}

namespace {
std::filesystem::path executableDir() {
#if defined(_WIN32)
  wchar_t buf[4096];
  DWORD n = GetModuleFileNameW(nullptr, buf, DWORD(std::size(buf)));
  if (n == 0 || n >= std::size(buf)) return {};
  return std::filesystem::path(std::wstring(buf, n)).parent_path();
#elif defined(__APPLE__)
  char buf[4096];
  uint32_t size = sizeof buf;
  if (_NSGetExecutablePath(buf, &size) != 0) return {};
  std::error_code ec;
  std::filesystem::path p = std::filesystem::canonical(buf, ec);
  return (ec ? std::filesystem::path(buf) : p).parent_path();
#else
  char buf[4096];
  ssize_t n = readlink("/proc/self/exe", buf, sizeof buf - 1);
  if (n <= 0) return {};
  return std::filesystem::path(std::string(buf, size_t(n))).parent_path();
#endif
}
int g_pluginLibraries = 0;
}  // namespace

int loadMujocoPlugins() {
  std::filesystem::path dir = executableDir() / "mujoco_plugin";
  std::error_code ec;
  if (dir.empty() || !std::filesystem::is_directory(dir, ec)) return 0;
  g_pluginLibraries = 0;
  try {  // MuJoCo takes a narrow path; skip plugins rather than fail if the folder name has no such form
    mj_loadAllPluginLibraries(dir.string().c_str(), [](const char*, int, int count) { g_pluginLibraries += count > 0; });
  } catch (const std::exception&) {
  }
  return g_pluginLibraries;
}

std::vector<std::string> takeMujocoWarnings() {
  std::vector<std::string> w;
  w.swap(g_warnings);
  return w;
}

ModelPtr makeModelPtr(mjModel* m) {
  return ModelPtr(m, [](mjModel* p) { if (p) mj_deleteModel(p); });
}

DataPtr makeData(const mjModel* m) { return DataPtr(m ? mj_makeData(m) : nullptr); }

CompileResult compileMjcf(const std::string& xml, const std::vector<std::pair<std::string, std::string>>& files) {
  CompileResult r;
  auto t0 = std::chrono::steady_clock::now();
  mjVFS vfs;
  mj_defaultVFS(&vfs);
  const char* kMain = "mjforge_model.xml";
  mj_addBufferVFS(&vfs, kMain, xml.data(), int(xml.size()));
  for (const auto& [name, content] : files) mj_addBufferVFS(&vfs, name.c_str(), content.data(), int(content.size()));
  char err[4096] = "";
  mjModel* m = nullptr;
  try {
    m = mj_loadXML(kMain, &vfs, err, sizeof err);
  } catch (const std::exception& e) {
    std::snprintf(err, sizeof err, "%s", e.what());
    m = nullptr;
  }
  mj_deleteVFS(&vfs);
  r.compileMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
  if (!m) {
    r.ok = false;
    r.error = err[0] ? err : "Unknown MuJoCo compile error";
    return r;
  }
  if (err[0]) r.warnings.emplace_back(err);
  r.ok = true;
  r.model = makeModelPtr(m);

  // Evaluate the reference configuration.
  try {
    DataPtr d = makeData(m);
    if (d) {
      if (m->nkey > 0) mj_resetDataKeyframe(m, d.get(), 0);  // the start state
      mj_forward(m, d.get());
      r.nefc0 = d->nefc;
      for (int i = 0; i < d->ncon; ++i) {
        const mjContact& c = d->contact[i];
        InitialContact ic;
        ic.geom1 = c.geom[0];
        ic.geom2 = c.geom[1];
        ic.dist = c.dist;
        ic.pos = {c.pos[0], c.pos[1], c.pos[2]};
        r.initialContacts.push_back(ic);
      }
      r.subtreeMass.resize(m->nbody);
      r.subtreeCom.resize(m->nbody);
      for (int b = 0; b < m->nbody; ++b) {
        r.subtreeMass[b] = m->body_subtreemass[b];
        r.subtreeCom[b] = {d->subtree_com[3 * b], d->subtree_com[3 * b + 1], d->subtree_com[3 * b + 2]};
      }
    }
  } catch (const std::exception& e) {
    r.warnings.emplace_back(e.what());
  }
  for (auto& w : takeMujocoWarnings()) r.warnings.push_back(w);
  return r;
}

BackgroundCompiler::BackgroundCompiler() { thread_ = std::thread([this] { loop(); }); }

BackgroundCompiler::~BackgroundCompiler() {
  {
    std::lock_guard<std::mutex> lk(mu_);
    quit_ = true;
  }
  cv_.notify_all();
  if (thread_.joinable()) thread_.join();
}

void BackgroundCompiler::request(std::string xml, ExportMap map, uint64_t revision) {
  {
    std::lock_guard<std::mutex> lk(mu_);
    jobXml_ = std::move(xml);
    jobMap_ = std::move(map);
    jobRevision_ = revision;
    hasJob_ = true;
  }
  cv_.notify_all();
}

bool BackgroundCompiler::poll(CompileResult& out) {
  std::lock_guard<std::mutex> lk(mu_);
  if (!hasResult_) return false;
  out = std::move(result_);
  result_ = {};
  hasResult_ = false;
  return true;
}

bool BackgroundCompiler::busy() const {
  std::lock_guard<std::mutex> lk(mu_);
  return working_ || hasJob_;
}

void BackgroundCompiler::loop() {
  for (;;) {
    std::string xml;
    ExportMap map;
    uint64_t rev = 0;
    {
      std::unique_lock<std::mutex> lk(mu_);
      cv_.wait(lk, [this] { return quit_ || hasJob_; });
      if (quit_) return;
      xml = std::move(jobXml_);
      map = std::move(jobMap_);
      rev = jobRevision_;
      hasJob_ = false;
      working_ = true;
    }
    CompileResult r = compileMjcf(xml);
    r.map = std::move(map);
    r.revision = rev;
    {
      std::lock_guard<std::mutex> lk(mu_);
      working_ = false;
      // Drop the result if a newer request already arrived; it will be recompiled.
      if (!hasJob_) {
        result_ = std::move(r);
        hasResult_ = true;
      }
    }
    if (onResult_) onResult_();
  }
}

}  // namespace mjf
