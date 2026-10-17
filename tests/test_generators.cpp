#include <doctest/doctest.h>

#include "analysis/Analysis.h"
#include "core/Generators.h"
#include "core/MjcfWriter.h"
#include "core/ProjectIO.h"
#include "core/Templates.h"
#include "sim/MujocoBridge.h"
#include "sim/Simulator.h"

using namespace mjf;

TEST_CASE("generators: every environment piece compiles and is static") {
  for (const GeneratorInfo& g : generatorList()) {
    INFO(g.type);
    Document doc;
    REQUIRE(makeTemplate("empty", doc));
    Id b = addGenerated(doc, g.type, {1, 0, 0}, 0.3);
    REQUIRE(b);
    CHECK(doc.body(b)->role == Role::Environment);
    int geoms = 0;
    for (const Geom& x : doc.geoms) geoms += x.body == b;
    CHECK(geoms > 0);
    CompileResult cr = compileMjcf(writeMjcf(doc));
    INFO(cr.error);
    REQUIRE(cr.ok);
    CHECK(cr.model->nv == 0);  // generated pieces never add degrees of freedom
    AnalysisReport r = analyzeModel(doc, &cr, true);
    CHECK(r.counts[int(Severity::Error)] == 0);
  }
}

TEST_CASE("generators: parameters regenerate, persist and keep assets") {
  Document doc;
  REQUIRE(makeTemplate("empty", doc));
  Id stairs = addGenerated(doc, "stairs", {1, 0, 0});
  auto countGeoms = [&](Id b) {
    int n = 0;
    for (const Geom& x : doc.geoms) n += x.body == b;
    return n;
  };
  int base = countGeoms(stairs);
  doc.body(stairs)->genParams[0] = 10;  // steps
  REQUIRE(regenerate(doc, stairs));
  CHECK(countGeoms(stairs) == base + 4);
  CHECK(genParam(*doc.body(stairs), "steps") == 10);
  // Out-of-range values are clamped.
  doc.body(stairs)->genParams[0] = 1e6;
  regenerate(doc, stairs);
  CHECK(genParam(*doc.body(stairs), "steps") == 60);

  Id rough = addGenerated(doc, "rough", {0, 0, 0});
  Id hf = kNoId;
  for (const Geom& x : doc.geoms) if (x.body == rough) hf = x.hfield;
  REQUIRE(hf);
  auto data = doc.hfield(hf)->data;
  doc.body(rough)->genParams[7] = 42;  // seed
  regenerate(doc, rough);
  Id hf2 = kNoId;
  for (const Geom& x : doc.geoms) if (x.body == rough) hf2 = x.hfield;
  CHECK(hf2 == hf);                          // same asset
  CHECK(doc.hfield(hf)->data.get() != data.get());  // new data
  CHECK(doc.hfields.size() == 1);
  // The spawn area is flat: the center sample is zero.
  const Hfield& h = *doc.hfield(hf);
  CHECK((*h.data)[size_t(h.nrow / 2) * h.ncol + h.ncol / 2] == doctest::Approx(0.0));

  std::string err;
  Document loaded;
  REQUIRE(projectFromJson(loaded, projectToJson(doc, ""), &err, nullptr));
  const Body* lb = loaded.body(rough);
  REQUIRE(lb);
  CHECK(lb->generator == "rough");
  CHECK(genParam(*lb, "seed") == 42);
  CHECK(writeMjcf(loaded) == writeMjcf(doc));
}

TEST_CASE("generators: a creature walks onto generated terrain without blowing up") {
  for (const char* type : {"rough", "stairs", "stones"}) {
    INFO(type);
    Document doc;
    REQUIRE(makeTemplate("quadruped", doc));
    addGenerated(doc, type, {type == std::string("rough") ? 0.0 : 0.35, 0, 0});
    ExportMap map;
    CompileResult cr = compileMjcf(writeMjcf(doc, {}, &map));
    cr.map = map;
    REQUIRE(cr.ok);
    Simulator sim;
    REQUIRE(sim.start(cr.model, cr.map));
    sim.mode = CtrlMode::Sine;
    for (int i = 0; i < 1500 && !sim.diverged(); ++i) sim.stepOnce();
    CHECK_FALSE(sim.diverged());
    CHECK(sim.sizing().peakBroadphase >= sim.sizing().peakContacts / 4);
  }
}
