// Thin layer over the MuJoCo C library: compiling MJCF from memory, error handling and a
// background compile worker used for live validation.
#pragma once

#include <condition_variable>
#include <functional>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <mujoco/mujoco.h>

#include "core/Math.h"
#include "core/MjcfWriter.h"

namespace mjf {

using ModelPtr = std::shared_ptr<mjModel>;
struct DataDeleter {
  void operator()(mjData* d) const { if (d) mj_deleteData(d); }
};
using DataPtr = std::unique_ptr<mjData, DataDeleter>;

// Makes MuJoCo errors throw std::runtime_error (instead of exiting) and collects warnings.
// Call once at startup.
void installMujocoErrorHandlers();
// Warnings emitted by MuJoCo on the calling thread since the last call.
std::vector<std::string> takeMujocoWarnings();

ModelPtr makeModelPtr(mjModel* m);
DataPtr makeData(const mjModel* m);

struct InitialContact {
  int geom1 = -1, geom2 = -1;
  double dist = 0;  // negative = penetration
  Vec3 pos;
};

struct CompileResult {
  bool ok = false;
  std::string error;                  // compile error (when !ok)
  std::vector<std::string> warnings;  // compiler / runtime warnings
  ModelPtr model;
  ExportMap map;
  uint64_t revision = 0;
  double compileMs = 0;
  // Evaluated at the start state (keyframe 0, else qpos0) with mj_forward.
  std::vector<InitialContact> initialContacts;
  int nefc0 = 0;
  std::vector<double> subtreeMass;  // per compiled body
  std::vector<Vec3> subtreeCom;     // per compiled body
};

// Compiles MJCF text. `files` are additional virtual files (path -> content) for meshes etc.
CompileResult compileMjcf(const std::string& xml,
                          const std::vector<std::pair<std::string, std::string>>& files = {});

// Compiles on a worker thread. Only the most recent request is processed.
class BackgroundCompiler {
 public:
  BackgroundCompiler();
  ~BackgroundCompiler();
  BackgroundCompiler(const BackgroundCompiler&) = delete;
  BackgroundCompiler& operator=(const BackgroundCompiler&) = delete;

  void request(std::string xml, ExportMap map, uint64_t revision);
  // Called from the worker thread whenever a result becomes available (e.g. to wake the UI).
  void setResultCallback(std::function<void()> cb) { onResult_ = std::move(cb); }
  // Returns true and fills `out` when a result newer than the last poll is available.
  bool poll(CompileResult& out);
  bool busy() const;

 private:
  void loop();

  std::thread thread_;
  mutable std::mutex mu_;
  std::condition_variable cv_;
  bool quit_ = false;
  bool hasJob_ = false;
  bool working_ = false;
  bool hasResult_ = false;
  std::string jobXml_;
  ExportMap jobMap_;
  uint64_t jobRevision_ = 0;
  CompileResult result_;
  std::function<void()> onResult_;
};

}  // namespace mjf
