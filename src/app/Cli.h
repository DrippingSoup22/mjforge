// Command line options.
#pragma once

#include <string>
#include <vector>

namespace mjf {

struct CliOptions {
  std::string input;          // project (.mjforge) or MJCF (.xml) to open
  std::string exportPath;     // headless: write MJCF here
  std::string savePath;       // headless: write the project (.mjforge) here
  std::string scope = "all";  // all | models | environment
  std::string angle = "radian";
  bool validate = false;      // headless: compile + analysis report
  bool help = false;
  // GUI testing helpers
  std::string screenshot;     // save a PNG after `frames` frames and exit
  int frames = 8;
  int width = 1600, height = 960;
  std::string templateName;   // start from a built-in template
  std::vector<std::string> select;  // element names to select at startup
  std::vector<std::string> actions; // UI actions to run at startup (testing), e.g. "simulate"
  bool headless() const { return screenshot.empty() && (!exportPath.empty() || !savePath.empty() || validate); }
};

bool parseCli(int argc, char** argv, CliOptions& out, std::string& error);
const char* cliUsage();
int runHeadless(const CliOptions& cli);

}  // namespace mjf
