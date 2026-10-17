// MJForge entry point: GUI by default, plus headless commands for scripting and CI.
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#include "app/App.h"
#include "app/Cli.h"
#include "sim/MujocoBridge.h"

int main(int argc, char** argv) {
  mjf::installMujocoErrorHandlers();
  mjf::CliOptions cli;
  std::string err;
  if (!mjf::parseCli(argc, argv, cli, err)) {
    std::fprintf(stderr, "%s\n\n%s", err.c_str(), mjf::cliUsage());
    return 2;
  }
  if (cli.help) {
    std::printf("%s", mjf::cliUsage());
    return 0;
  }
  if (cli.headless()) return mjf::runHeadless(cli);
  mjf::App app;
  return app.run(cli);
}
