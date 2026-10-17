#include <doctest/doctest.h>

#include <cmath>
#include <unordered_set>

#include <nlohmann/json.hpp>

#include "analysis/Analysis.h"
#include "core/Generators.h"
#include "core/MjcfWriter.h"
#include "core/Ops.h"
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

TEST_CASE("generators: regenerating keeps edits, separate terrains and valid parameters") {
  Document doc;
  REQUIRE(makeTemplate("quadruped", doc));
  Id ramp = addGenerated(doc, "ramp", {2, 0, 0});
  auto geomNamed = [&](const std::string& n) -> Geom* {
    for (Geom& g : doc.geoms)
      if (g.name == n) return &g;
    return nullptr;
  };
  std::string base = doc.body(ramp)->name;
  Geom* slope = geomNamed(base + "_slope");
  REQUIRE(slope);
  Id slopeId = slope->id;
  slope->friction.x = 0.05;  // an icy slope
  Id foot = kNoId;
  for (const Geom& g : doc.geoms)
    if (g.name == "FL_foot") foot = g.id;
  REQUIRE(foot);
  Id pair = addContactPair(doc, foot, slopeId);
  // A steeper ramp: the slope keeps its ID, its friction and the pair that refers to it.
  doc.body(ramp)->genParams[2] = 12;
  REQUIRE(regenerate(doc, ramp));
  slope = geomNamed(base + "_slope");
  REQUIRE(slope);
  CHECK(slope->id == slopeId);
  CHECK(slope->friction.x == doctest::Approx(0.05));
  CHECK(doc.pair(pair) != nullptr);
  // A flat ramp (angle 0, the inspector's minimum) still compiles: its platform is a slab.
  doc.body(ramp)->genParams[2] = 0;
  REQUIRE(regenerate(doc, ramp));
  CompileResult cr = compileMjcf(writeMjcf(doc));
  INFO(cr.error);
  CHECK(cr.ok);

  // A duplicated terrain gets its own height field when its parameters change.
  Id rough = addGenerated(doc, "rough", {-4, 0, 0});
  std::vector<Id> copies = duplicate(doc, {rough}, {0, 8, 0});
  REQUIRE(copies.size() == 1);
  Id hOriginal = kNoId;
  for (const Geom& g : doc.geoms)
    if (g.body == rough && g.type == GeomType::Hfield) hOriginal = g.hfield;
  REQUIRE(hOriginal);
  std::vector<float> before = *doc.hfield(hOriginal)->data;
  doc.body(copies[0])->genParams[7] = 42;  // seed
  REQUIRE(regenerate(doc, copies[0]));
  CHECK(*doc.hfield(hOriginal)->data == before);
  Id hCopy = kNoId;
  for (const Geom& g : doc.geoms)
    if (g.body == copies[0] && g.type == GeomType::Hfield) hCopy = g.hfield;
  CHECK(hCopy != hOriginal);

  // NaN (typed into a field) and parameters missing from an older file take their defaults.
  Id stairs = addGenerated(doc, "stairs", {0, -4, 0});
  doc.body(stairs)->genParams = {4, std::nan(""), 0.3, 0.08};  // no platform / descend entries
  REQUIRE(regenerate(doc, stairs));
  CHECK(doc.body(stairs)->genParams[1] == doctest::Approx(1.2));  // width default
  CHECK(doc.body(stairs)->genParams[4] == doctest::Approx(1.0));  // platform default, not 0
  CHECK(compileMjcf(writeMjcf(doc)).ok);

  // The largest stepping-stone field (100 x 100) regenerates with unique names.
  Id stones = addGenerated(doc, "stones", {0, 4, 0});
  doc.body(stones)->genParams[0] = 100;
  doc.body(stones)->genParams[1] = 100;
  REQUIRE(regenerate(doc, stones));
  std::unordered_set<std::string> names;
  int n = 0;
  bool unique = true;
  for (const Geom& g : doc.geoms) {
    if (!g.name.empty()) unique = names.insert(g.name).second && unique;
    n += g.body == stones;
  }
  CHECK(unique);
  CHECK(n == 10000);
  CHECK(compileMjcf(writeMjcf(doc)).ok);
}

TEST_CASE("generators: pieces are recognized by key after copies, renames and older files") {
  Document doc;
  REQUIRE(makeTemplate("empty", doc));
  auto piecesOf = [&](Id body) {
    std::vector<const Geom*> out;
    for (const Geom& g : doc.geoms)
      if (g.body == body) out.push_back(&g);
    return out;
  };
  auto keyed = [&](Id body, const std::string& key) -> Geom* {
    for (Geom& g : doc.geoms)
      if (g.body == body && g.genKey == key) return &g;
    return nullptr;
  };
  auto idsUnique = [&] {
    std::unordered_set<Id> ids;
    for (const Geom& g : doc.geoms)
      if (!ids.insert(g.id).second) return false;
    return true;
  };

  // A duplicated ramp keeps the icy slope it was copied with when its own angle changes.
  Id ramp = addGenerated(doc, "ramp", {0, 0, 0});
  keyed(ramp, "_slope")->friction.x = 0.05;
  Id copy = duplicate(doc, {ramp}, {0, 3, 0}).at(0);
  REQUIRE(keyed(copy, "_slope"));
  Id copySlope = keyed(copy, "_slope")->id;
  doc.body(copy)->genParams[2] = 20;
  REQUIRE(regenerate(doc, copy));
  REQUIRE(keyed(copy, "_slope"));
  CHECK(keyed(copy, "_slope")->id == copySlope);
  CHECK(keyed(copy, "_slope")->friction.x == doctest::Approx(0.05));
  CHECK(piecesOf(copy).size() == piecesOf(ramp).size());

  // Renaming the body (or a piece) changes nothing about which piece is which.
  doc.body(ramp)->name = "hill";
  keyed(ramp, "_top")->name = "landing";
  size_t count = piecesOf(ramp).size();
  doc.body(ramp)->genParams[3] = 2.0;  // a longer platform
  REQUIRE(regenerate(doc, ramp));
  CHECK(piecesOf(ramp).size() == count);
  REQUIRE(keyed(ramp, "_top"));
  CHECK(keyed(ramp, "_top")->name == "landing");
  CHECK(keyed(ramp, "_top")->size.x == doctest::Approx(1.0));

  // A geom added by hand stays; a copy of a piece becomes hand-made and stays too.
  Id marker = addGeom(doc, ramp, GeomType::Sphere, {0.1, 0, 0}, {{0, 0, 1}, {}}, "marker");
  Id slope = keyed(ramp, "_slope")->id;
  Id slopeCopy = duplicate(doc, {slope}, {0, 0, 0.5}).at(0);
  REQUIRE(regenerate(doc, ramp));
  CHECK(doc.geom(marker) != nullptr);
  REQUIRE(doc.geom(slopeCopy) != nullptr);
  CHECK(doc.geom(slopeCopy)->genKey.empty());
  CHECK(keyed(ramp, "_slope")->id == slope);
  CHECK(idsUnique());
  REQUIRE(regenerate(doc, ramp));  // stable: nothing doubles on a second pass
  CHECK(piecesOf(ramp).size() == count + 2);
  CHECK(idsUnique());

  // A piece that no longer exists takes the contact pair on it along; the model still compiles.
  Id stairs = addGenerated(doc, "stairs", {3, 0, 0});
  Id ball = addGeom(doc, kWorldId, GeomType::Sphere, {0.1, 0, 0}, {{3, 0, 1}, {}}, "ball");
  Id pair = addContactPair(doc, ball, keyed(stairs, "_step6")->id);
  doc.body(stairs)->genParams[0] = 3;
  REQUIRE(regenerate(doc, stairs));
  CHECK(keyed(stairs, "_step6") == nullptr);
  CHECK(doc.pair(pair) == nullptr);
  CompileResult cr = compileMjcf(writeMjcf(doc));
  INFO(cr.error);
  CHECK(cr.ok);

  // Keys are saved.
  Document back;
  std::string err;
  REQUIRE(projectFromJson(back, projectToJson(doc), &err));
  CHECK(back.geom(keyed(stairs, "_step2")->id)->genKey == "_step2");
  // A file from before keys existed: pieces are known by name, and a geom on the body that is not
  // named like one is replaced as it was then.
  Id extra = addGeom(doc, stairs, GeomType::Box, {0.1, 0.1, 0.1}, {{0, 0, 2}, {}}, "leftover");
  for (Geom& g : doc.geoms)
    if (g.name == "stairs_step2") g.friction.x = 0.3;
  nlohmann::json old = nlohmann::json::parse(projectToJson(doc));
  for (auto& g : old["geoms"]) g.erase("genKey");
  REQUIRE(projectFromJson(back, old.dump(), &err));
  CHECK(back.geom(extra)->genKey == "?" + std::to_string(extra));
  back.body(stairs)->genParams[3] = 0.1;  // a higher rise
  REQUIRE(regenerate(back, stairs));
  const Geom* step2 = nullptr;
  for (const Geom& g : back.geoms)
    if (g.name == "stairs_step2") step2 = &g;
  REQUIRE(step2);
  CHECK(step2->genKey == "_step2");
  CHECK(step2->friction.x == doctest::Approx(0.3));
  CHECK(step2->size.z == doctest::Approx(0.1));  // (2 x 0.1) / 2
  CHECK(back.geom(extra) == nullptr);  // older files: everything on the body was generated
}
