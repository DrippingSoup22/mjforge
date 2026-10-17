// MJForge entry point: GUI by default, plus headless commands for scripting and CI.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#ifdef _WIN32
#define NOMINMAX
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

#include "app/App.h"
#include "app/Cli.h"
#include "sim/MujocoBridge.h"

namespace {

// A release build on Windows is a windowed program with no console of its own: command-line
// results go to the console it was started from, unless they are redirected to a file or pipe.
void attachParentConsole() {
#ifdef _WIN32
  DWORD type = GetFileType(GetStdHandle(STD_OUTPUT_HANDLE));
  if (type == FILE_TYPE_DISK || type == FILE_TYPE_PIPE || !AttachConsole(ATTACH_PARENT_PROCESS)) return;
  FILE* f = nullptr;
  freopen_s(&f, "CONOUT$", "w", stdout);
  freopen_s(&f, "CONOUT$", "w", stderr);
#endif
}

}  // namespace

int main(int argc, char** argv) {
  mjf::installMujocoErrorHandlers();
  int plugins = mjf::loadMujocoPlugins();
  mjf::CliOptions cli;
  std::string err;
  if (!mjf::parseCli(argc, argv, cli, err)) {
    attachParentConsole();
    std::fprintf(stderr, "%s\n\n%s", err.c_str(), mjf::cliUsage());
    return 2;
  }
  if (cli.help) {
    attachParentConsole();
    std::printf("%s", mjf::cliUsage());
    return 0;
  }
  if (cli.version) {
    attachParentConsole();
    std::printf("MJForge %s, MuJoCo %s (%d plugin librar%s loaded)\n", MJFORGE_VERSION, mj_versionString(), plugins,
                plugins == 1 ? "y" : "ies");
    return 0;
  }
  if (cli.headless()) {
    attachParentConsole();
    return mjf::runHeadless(cli);
  }
  mjf::App app;
  return app.run(cli);
}
