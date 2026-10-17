#include "app/Cli.h"

#include <algorithm>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "analysis/Analysis.h"
#include "core/MjcfWriter.h"
#include "core/ProjectIO.h"
#include "core/Templates.h"
#include "sim/MjcfImport.h"
#include "sim/MujocoBridge.h"
#include "sim/WarpLoader.h"

namespace mjf {

const char* cliUsage() {
  return "MJForge " MJFORGE_VERSION " - visual MuJoCo / MuJoCo Warp model & environment editor\n"
         "\n"
         "Usage: mjforge [file.mjforge | model.xml] [options]\n"
         "\n"
         "Headless commands:\n"
         "  --export <out.xml>     Export the project to MJCF and exit\n"
         "  --scope <s>            Export scope: all | models | environment (default all)\n"
         "  --angle <u>            Angle unit in the MJCF: radian | degree (default radian)\n"
         "  --loader               With --export: also write <name>_mjwarp.py (MuJoCo Warp loader)\n"
         "  --validate             Compile with MuJoCo, print a report and exit (non-zero on errors)\n"
         "  --save <out.mjforge>   Save the project (with --template, or to convert an MJCF file)\n"
         "\n"
         "GUI options:\n"
         "  --template <name>      Start from a built-in template\n"
         "  --size <W>x<H>         Window size (default 1600x960)\n"
         "  --select <name>        Select an element by name at startup (repeatable)\n"
         "  --action <name>        Run a UI action at startup (repeatable, for testing)\n"
         "  --screenshot <out.png> Render a few frames, save a screenshot and exit\n"
         "  --frames <n>           Frames to render before the screenshot (default 8)\n"
         "  -h, --help             Show this help\n";
}

bool parseCli(int argc, char** argv, CliOptions& o, std::string& error) {
  auto need = [&](int& i, const char* flag) -> const char* {
    if (i + 1 >= argc) {
      error = std::string("Missing value for ") + flag;
      return nullptr;
    }
    return argv[++i];
  };
  for (int i = 1; i < argc; ++i) {
    std::string a = argv[i];
    const char* v = nullptr;
    if (a == "-h" || a == "--help") o.help = true;
    else if (a == "--validate") o.validate = true;
    else if (a == "--loader") o.loader = true;
    else if (a == "--export") { if (!(v = need(i, "--export"))) return false; o.exportPath = v; }
    else if (a == "--save") { if (!(v = need(i, "--save"))) return false; o.savePath = v; }
    else if (a == "--scope") { if (!(v = need(i, "--scope"))) return false; o.scope = v; }
    else if (a == "--angle") { if (!(v = need(i, "--angle"))) return false; o.angle = v; }
    else if (a == "--screenshot") { if (!(v = need(i, "--screenshot"))) return false; o.screenshot = v; }
    else if (a == "--frames") { if (!(v = need(i, "--frames"))) return false; o.frames = std::max(1, std::atoi(v)); }
    else if (a == "--template") { if (!(v = need(i, "--template"))) return false; o.templateName = v; }
    else if (a == "--select") { if (!(v = need(i, "--select"))) return false; o.select.push_back(v); }
    else if (a == "--action") { if (!(v = need(i, "--action"))) return false; o.actions.push_back(v); }
    else if (a == "--size") {
      if (!(v = need(i, "--size"))) return false;
      if (std::sscanf(v, "%dx%d", &o.width, &o.height) != 2) { error = "Bad --size, expected WxH"; return false; }
    } else if (!a.empty() && a[0] == '-') {
      error = "Unknown option: " + a;
      return false;
    } else {
      o.input = a;
    }
  }
  if (o.scope != "all" && o.scope != "models" && o.scope != "environment") {
    error = "Bad --scope: " + o.scope;
    return false;
  }
  return true;
}

int runHeadless(const CliOptions& cli) {
  Document doc;
  std::string err;
  if (!cli.input.empty()) {
    std::string ext = cli.input.size() > 4 ? cli.input.substr(cli.input.size() - 4) : "";
    bool ok;
    if (ext == ".xml") {
      ImportReport rep;
      ok = importMjcfFile(cli.input, doc, &err, &rep);
      for (const std::string& n : rep.notes) std::printf("NOTE   %s\n", n.c_str());
    } else {
      ok = loadProject(doc, cli.input, &err);
    }
    if (!ok) {
      std::fprintf(stderr, "Failed to open %s: %s\n", cli.input.c_str(), err.c_str());
      return 1;
    }
  } else if (!cli.templateName.empty()) {
    if (!makeTemplate(cli.templateName, doc)) {
      std::fprintf(stderr, "Unknown template: %s\n", cli.templateName.c_str());
      return 2;
    }
  } else {
    std::fprintf(stderr, "A project file or --template is required for headless commands.\n");
    return 2;
  }
  if (!cli.savePath.empty()) {
    if (!saveProject(doc, cli.savePath, &err)) {
      std::fprintf(stderr, "%s\n", err.c_str());
      return 1;
    }
    std::printf("Saved %s\n", cli.savePath.c_str());
  }
  ExportOptions opt;
  opt.scope = cli.scope == "models" ? ExportOptions::Scope::Models
            : cli.scope == "environment" ? ExportOptions::Scope::Environment
                                         : ExportOptions::Scope::All;
  opt.angle = cli.angle == "degree" ? AngleUnit::Degree : AngleUnit::Radian;
  std::string xml = writeMjcf(doc, opt);
  int rc = 0;
  if (cli.validate) {
    CompileResult r = compileMjcf(xml);
    if (!r.ok) {
      std::printf("ERROR  MuJoCo compile failed: %s\n", r.error.c_str());
      rc = 1;
    } else {
      std::printf("OK     compiled in %.1f ms: nbody=%d njnt=%d ngeom=%d nu=%d nsensor=%d nq=%d nv=%d\n", r.compileMs,
                  int(r.model->nbody), int(r.model->njnt), int(r.model->ngeom), int(r.model->nu),
                  int(r.model->nsensor), int(r.model->nq), int(r.model->nv));
      for (const auto& w : r.warnings) std::printf("WARN   %s\n", w.c_str());
    }
    auto t0 = std::chrono::steady_clock::now();
    AnalysisReport rep = analyzeModel(doc, &r, true);
    double analysisMs = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    for (const Issue& is : rep.issues) {
      std::printf("%-7s [%s] %s\n", severityName(is.severity), areaName(is.area), is.title.c_str());
      if (!is.detail.empty()) {
        std::string d = is.detail;
        for (size_t pos = 0; (pos = d.find('\n', pos)) != std::string::npos; pos += 9) d.replace(pos, 1, "\n        ");
        std::printf("        %s\n", d.c_str());
      }
      if (is.fix) std::printf("        fix available in the editor: %s\n", is.fixLabel.c_str());
      if (is.severity == Severity::Error) rc = 1;
    }
    std::printf("Summary: %d errors, %d warnings, %d info, %d tips. %d legs, %d contact pairs, %d MJWarp broadphase "
                "pairs at rest. Analysis took %.0f ms.\n",
                rep.counts[0], rep.counts[1], rep.counts[2], rep.counts[3], rep.legCount, rep.collisionPairs,
                rep.broadphaseAtRest, analysisMs);
  }
  if (!cli.exportPath.empty()) {
    std::vector<std::string> written;
    if (!exportMjcfFile(doc, cli.exportPath, opt, &err, "", &written)) {
      std::fprintf(stderr, "%s\n", err.c_str());
      return 1;
    }
    if (written.size() > 1) std::printf("Wrote %d asset files (meshes, textures)\n", int(written.size() - 1));
    std::printf("Wrote %s\n", cli.exportPath.c_str());
    if (cli.loader) {
      std::string path = cli.exportPath, file = path.substr(path.find_last_of("/\\") + 1);
      std::string stem = file.substr(0, file.find_last_of('.'));
      std::string dir = path.substr(0, path.size() - file.size());
      std::string py = dir + stem + "_mjwarp.py";
      if (!writeTextFile(py, mjwarpLoaderScript(doc, file, nullptr), &err)) {
        std::fprintf(stderr, "%s\n", err.c_str());
        return 1;
      }
      std::printf("Wrote %s\n", py.c_str());
    }
  }
  return rc;
}

}  // namespace mjf
