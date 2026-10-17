#include "core/Generators.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>

#include "core/Ops.h"

namespace mjf {

namespace {

const Color kConcrete{0.62f, 0.64f, 0.67f, 1};
const Color kStone{0.48f, 0.47f, 0.45f, 1};
const Color kWall{0.55f, 0.58f, 0.63f, 1};
const Color kTerrain{0.45f, 0.55f, 0.42f, 1};

// Small deterministic RNG (PCG32) so generated environments are reproducible from their seed.
struct Rng {
  uint64_t state;
  explicit Rng(uint64_t seed) : state(seed * 6364136223846793005ULL + 1442695040888963407ULL) { next(); }
  uint32_t next() {
    uint64_t old = state;
    state = old * 6364136223846793005ULL + 1442695040888963407ULL;
    uint32_t xorshifted = uint32_t(((old >> 18u) ^ old) >> 27u);
    uint32_t rot = uint32_t(old >> 59u);
    return (xorshifted >> rot) | (xorshifted << ((32 - rot) & 31));
  }
  double uniform() { return next() / 4294967296.0; }
  double uniform(double a, double b) { return a + (b - a) * uniform(); }
};

// Smooth value noise in [0, 1] with fractal octaves.
double hash2(int x, int y, uint32_t seed) {
  uint32_t h = uint32_t(x) * 374761393u + uint32_t(y) * 668265263u + seed * 2246822519u;
  h = (h ^ (h >> 13)) * 1274126177u;
  h ^= h >> 16;
  return (h & 0xFFFFFF) / double(0xFFFFFF);
}
double valueNoise(double x, double y, uint32_t seed) {
  int x0 = int(std::floor(x)), y0 = int(std::floor(y));
  double fx = x - x0, fy = y - y0;
  auto s = [](double t) { return t * t * (3 - 2 * t); };
  double a = hash2(x0, y0, seed), b = hash2(x0 + 1, y0, seed);
  double c = hash2(x0, y0 + 1, seed), d = hash2(x0 + 1, y0 + 1, seed);
  double u = s(fx), v = s(fy);
  return (a * (1 - u) + b * u) * (1 - v) + (c * (1 - u) + d * u) * v;
}
double fractalNoise(double x, double y, int octaves, uint32_t seed) {
  double sum = 0, amp = 1, norm = 0, f = 1;
  for (int o = 0; o < octaves; ++o) {
    sum += amp * valueNoise(x * f, y * f, seed + uint32_t(o) * 101u);
    norm += amp;
    amp *= 0.5;
    f *= 2;
  }
  return sum / norm;
}

struct Ctx {
  Document& doc;
  Id body;
  const Body& b;
  double p(const char* k) const { return genParam(b, k); }
  int pi(const char* k) const { return int(std::lround(genParam(b, k))); }
  Id box(const Vec3& half, const Vec3& pos, const Quat& q, const Color& c, const std::string& name = "") {
    Id g = addGeom(doc, body, GeomType::Box, half, {pos, q}, name);
    doc.geom(g)->rgba = c;
    return g;
  }
};

void genStairs(Ctx& c) {
  int n = std::max(1, c.pi("steps"));
  double w = c.p("width"), d = c.p("tread"), r = c.p("rise"), top = c.p("top");
  bool down = c.p("descend") > 0.5;
  std::string base = c.b.name;
  for (int i = 0; i < n; ++i) {
    double h = (i + 1) * r;
    c.box({d / 2, w / 2, h / 2}, {i * d + d / 2, 0, h / 2}, {}, kConcrete, base + "_step" + std::to_string(i + 1));
  }
  double H = n * r, x = n * d;
  if (top > 0) c.box({top / 2, w / 2, H / 2}, {x + top / 2, 0, H / 2}, {}, kConcrete, base + "_top");
  if (down) {
    double x0 = x + top;
    for (int i = 0; i < n; ++i) {
      double h = (n - i) * r;
      c.box({d / 2, w / 2, h / 2}, {x0 + i * d + d / 2, 0, h / 2}, {}, kConcrete,
            base + "_down" + std::to_string(i + 1));
    }
  }
}

void genRamp(Ctx& c) {
  double L = c.p("length"), w = c.p("width"), th = deg2rad(c.p("angle")), top = c.p("top"), t = 0.05;
  bool down = c.p("descend") > 0.5;
  std::string base = c.b.name;
  // The top surface runs from (0, 0, 0) up to (L cos, 0, L sin).
  auto slope = [&](double x0, double sign, const std::string& name) {
    Vec3 mid{x0 + sign * L / 2 * std::cos(th), 0, L / 2 * std::sin(th)};
    Vec3 n{-sign * std::sin(th), 0, std::cos(th)};
    c.box({L / 2, w / 2, t / 2}, mid - n * (t / 2), axisAngle({0, 1, 0}, -sign * th), kConcrete, name);
  };
  slope(0, 1, base + "_slope");
  double H = L * std::sin(th), x = L * std::cos(th);
  if (top > 0) c.box({top / 2, w / 2, H / 2}, {x + top / 2, 0, H / 2}, {}, kConcrete, base + "_top");
  if (down) slope(x + top + x, -1, base + "_slope_down");
}

// Height fields are centered on the body; heights are normalized to [0, 1] of the elevation.
void setHfield(Ctx& c, Id reuse, int nrow, int ncol, double sx, double sy, double elevation,
               const std::vector<float>& data, const std::string& name) {
  Hfield* h = c.doc.hfield(reuse);
  if (!h) {
    Hfield nh;
    nh.id = c.doc.newId();
    nh.name = c.doc.uniqueName(Kind::Hfield, name);
    c.doc.hfields.push_back(nh);
    c.doc.touch();
    h = c.doc.hfield(nh.id);
  }
  h->nrow = nrow;
  h->ncol = ncol;
  h->size = Vec4(sx / 2, sy / 2, std::max(1e-3, elevation), std::max(0.02, 0.25 * elevation));
  h->data = std::make_shared<const std::vector<float>>(data);
  // 1 mm above the body so its flat parts don't coincide with a floor plane at the same height.
  Id g = addGeom(c.doc, c.body, GeomType::Hfield, {sx / 2, sy / 2, elevation}, {{0, 0, 0.001}, {}}, c.b.name + "_surface");
  c.doc.geom(g)->hfield = h->id;
  c.doc.geom(g)->rgba = kTerrain;
}

// Samples f(x, y) in meters (body frame) on an nrow x ncol grid, flattening a spawn disc at the
// center. Rows run from +Y to -Y as in MJCF.
template <class F>
void sampleTerrain(Ctx& c, Id reuse, double sx, double sy, double spacing, double flat, F&& f, const std::string& name) {
  int ncol = std::clamp(int(std::lround(sx / spacing)) + 1, 2, 1024);
  int nrow = std::clamp(int(std::lround(sy / spacing)) + 1, 2, 1024);
  std::vector<float> data(size_t(nrow) * ncol);
  for (int r = 0; r < nrow; ++r) {
    double y = sy / 2 - sy * r / (nrow - 1);
    for (int k = 0; k < ncol; ++k) {
      double x = -sx / 2 + sx * k / (ncol - 1);
      double v = std::clamp(f(x, y), 0.0, 1.0);
      if (flat > 0) {
        double d = std::hypot(x, y);
        double blend = std::clamp((d - flat) / std::max(0.05, flat * 0.5), 0.0, 1.0);
        v *= blend * blend * (3 - 2 * blend);
      }
      data[size_t(r) * ncol + k] = float(v);
    }
  }
  setHfield(c, reuse, nrow, ncol, sx, sy, c.p("height"), data, name);
}

void genRough(Ctx& c, Id reuse) {
  double feature = std::max(0.02, c.p("feature"));
  int oct = std::clamp(c.pi("octaves"), 1, 8);
  uint32_t seed = uint32_t(std::max(0, c.pi("seed")));
  sampleTerrain(c, reuse, c.p("sizex"), c.p("sizey"), std::max(0.01, c.p("spacing")), c.p("flat"),
                [&](double x, double y) { return fractalNoise(x / feature, y / feature, oct, seed); },
                c.b.name + "_hfield");
}

void genWaves(Ctx& c, Id reuse) {
  double lambda = std::max(0.05, c.p("wavelength"));
  double cross = std::clamp(c.p("cross"), 0.0, 1.0);
  sampleTerrain(c, reuse, c.p("sizex"), c.p("sizey"), std::max(0.01, c.p("spacing")), c.p("flat"),
                [&](double x, double y) {
                  double a = 0.5 + 0.5 * std::sin(2 * kPi * x / lambda);
                  double b = 0.5 + 0.5 * std::sin(2 * kPi * y / lambda);
                  return (1 - cross) * a + cross * a * b;
                },
                c.b.name + "_hfield");
}

void genStones(Ctx& c) {
  int rows = std::max(1, c.pi("rows")), cols = std::max(1, c.pi("cols"));
  double s = c.p("stone"), gap = c.p("gap"), h = c.p("height"), var = c.p("variation");
  Rng rng(uint64_t(std::max(0, c.pi("seed"))) + 7);
  double pitch = s + gap;
  for (int r = 0; r < rows; ++r)
    for (int k = 0; k < cols; ++k) {
      double hi = std::max(0.01, h + rng.uniform(-var, var));
      Vec3 pos{s / 2 + r * pitch, (k - (cols - 1) / 2.0) * pitch, hi / 2};
      c.box({s / 2, s / 2, hi / 2}, pos, {}, kStone, c.b.name + "_" + std::to_string(r + 1) + "_" + std::to_string(k + 1));
    }
}

void genObstacles(Ctx& c) {
  int n = std::max(0, c.pi("count"));
  double ax = c.p("areax"), ay = c.p("areay"), smin = c.p("minsize"), smax = std::max(smin, c.p("maxsize"));
  double clear = c.p("clear");
  Rng rng(uint64_t(std::max(0, c.pi("seed"))) + 13);
  int placed = 0;
  for (int attempt = 0; placed < n && attempt < n * 50; ++attempt) {
    Vec3 p{rng.uniform(-ax / 2, ax / 2), rng.uniform(-ay / 2, ay / 2), 0};
    double s = rng.uniform(smin, smax);
    if (std::hypot(p.x, p.y) < clear + s) continue;  // keep the spawn area free
    double hgt = rng.uniform(smin, smax);
    std::string name = c.b.name + "_" + std::to_string(++placed);
    if (rng.uniform() < 0.6) {
      Id g = c.box({s, s * rng.uniform(0.5, 1.0), hgt}, {p.x, p.y, hgt}, axisAngle({0, 0, 1}, rng.uniform(0, kPi)), kStone, name);
      (void)g;
    } else {
      Id g = addGeom(c.doc, c.body, GeomType::Cylinder, {s, hgt, 0}, {{p.x, p.y, hgt}, {}}, name);
      c.doc.geom(g)->rgba = kStone;
    }
  }
}

void genArena(Ctx& c) {
  double sx = c.p("sizex"), sy = c.p("sizey"), h = c.p("wallheight"), t = c.p("thickness");
  std::string base = c.b.name;
  c.box({sx / 2 + t, t / 2, h / 2}, {0, sy / 2 + t / 2, h / 2}, {}, kWall, base + "_north");
  c.box({sx / 2 + t, t / 2, h / 2}, {0, -sy / 2 - t / 2, h / 2}, {}, kWall, base + "_south");
  c.box({t / 2, sy / 2, h / 2}, {sx / 2 + t / 2, 0, h / 2}, {}, kWall, base + "_east");
  c.box({t / 2, sy / 2, h / 2}, {-sx / 2 - t / 2, 0, h / 2}, {}, kWall, base + "_west");
}

}  // namespace

const std::vector<GeneratorInfo>& generatorList() {
  static const std::vector<GeneratorInfo> list = {
      {"stairs", "Stairs", "Solid steps up to a platform (and optionally down again).",
       {{"steps", "Steps", 6, 1, 60, 0.1, nullptr, true, "Number of steps."},
        {"width", "Width", 1.2, 0.1, 20, 0.01, "m", false, "Width of the staircase."},
        {"tread", "Tread depth", 0.3, 0.05, 2, 0.005, "m", false, "Depth of each step."},
        {"rise", "Rise", 0.08, 0.005, 0.5, 0.002, "m", false, "Height of each step. Small robots manage a few cm."},
        {"top", "Platform", 1.0, 0, 10, 0.01, "m", false, "Depth of the platform at the top (0: none)."},
        {"descend", "Down side", 0, 0, 1, 1, nullptr, true, "1: add steps going down after the platform."}}},
      {"ramp", "Ramp", "An incline to a platform (and optionally down again).",
       {{"length", "Slope length", 2.0, 0.1, 20, 0.01, "m", false, "Length along the slope."},
        {"width", "Width", 1.2, 0.1, 20, 0.01, "m", false, ""},
        {"angle", "Angle", 10, 0, 60, 0.1, "deg", false, "Incline. Friction limits what a robot can climb."},
        {"top", "Platform", 1.0, 0, 10, 0.01, "m", false, "Depth of the platform at the top (0: none)."},
        {"descend", "Down side", 0, 0, 1, 1, nullptr, true, "1: add a slope going down after the platform."}}},
      {"rough", "Rough terrain (height field)", "Fractal-noise terrain as one height field geom: cheap in MJWarp.",
       {{"sizex", "Size X", 6, 0.5, 100, 0.05, "m", false, "Full extent along X."},
        {"sizey", "Size Y", 6, 0.5, 100, 0.05, "m", false, "Full extent along Y."},
        {"height", "Max height", 0.08, 0.001, 5, 0.002, "m", false, "Highest point above the base."},
        {"feature", "Feature size", 0.4, 0.02, 10, 0.01, "m", false, "Size of the bumps (noise wavelength)."},
        {"octaves", "Detail", 3, 1, 8, 0.05, nullptr, true, "Noise octaves: more adds finer detail."},
        {"spacing", "Grid spacing", 0.05, 0.01, 1, 0.002, "m", false, "Resolution of the height field."},
        {"flat", "Flat spawn radius", 0.6, 0, 10, 0.01, "m", false, "Flat area at the center for spawning."},
        {"seed", "Seed", 1, 0, 100000, 0.2, nullptr, true, "Change for a different random terrain."}}},
      {"waves", "Waves (height field)", "Sinusoidal terrain, along X or as an egg-crate.",
       {{"sizex", "Size X", 6, 0.5, 100, 0.05, "m", false, ""},
        {"sizey", "Size Y", 6, 0.5, 100, 0.05, "m", false, ""},
        {"height", "Amplitude", 0.06, 0.001, 5, 0.002, "m", false, "Crest height above the troughs."},
        {"wavelength", "Wavelength", 1.0, 0.05, 20, 0.01, "m", false, ""},
        {"cross", "Egg-crate", 0, 0, 1, 0.01, nullptr, false, "0: waves along X only, 1: bumps in both directions."},
        {"spacing", "Grid spacing", 0.05, 0.01, 1, 0.002, "m", false, ""},
        {"flat", "Flat spawn radius", 0.6, 0, 10, 0.01, "m", false, ""}}},
      {"stones", "Stepping stones", "A grid of pillars with gaps, heights randomized.",
       {{"rows", "Rows", 8, 1, 100, 0.1, nullptr, true, "Along X."},
        {"cols", "Columns", 3, 1, 100, 0.1, nullptr, true, "Along Y."},
        {"stone", "Stone size", 0.25, 0.02, 5, 0.005, "m", false, ""},
        {"gap", "Gap", 0.1, 0, 5, 0.005, "m", false, ""},
        {"height", "Height", 0.1, 0.01, 5, 0.005, "m", false, ""},
        {"variation", "Height variation", 0.02, 0, 2, 0.002, "m", false, ""},
        {"seed", "Seed", 1, 0, 100000, 0.2, nullptr, true, ""}}},
      {"obstacles", "Obstacle field", "Random static boxes and cylinders, keeping the spawn area clear.",
       {{"count", "Count", 20, 0, 500, 0.2, nullptr, true, "Each obstacle adds collision pairs in MJWarp."},
        {"areax", "Area X", 5, 0.5, 100, 0.05, "m", false, ""},
        {"areay", "Area Y", 5, 0.5, 100, 0.05, "m", false, ""},
        {"minsize", "Min size", 0.04, 0.005, 5, 0.002, "m", false, "Half-size of the smallest obstacle."},
        {"maxsize", "Max size", 0.15, 0.005, 5, 0.002, "m", false, ""},
        {"clear", "Clear radius", 0.6, 0, 50, 0.01, "m", false, "No obstacles within this distance of the center."},
        {"seed", "Seed", 1, 0, 100000, 0.2, nullptr, true, ""}}},
      {"arena", "Arena walls", "Four walls around a rectangular area.",
       {{"sizex", "Size X", 6, 0.5, 200, 0.05, "m", false, "Inner size along X."},
        {"sizey", "Size Y", 6, 0.5, 200, 0.05, "m", false, "Inner size along Y."},
        {"wallheight", "Wall height", 0.5, 0.01, 20, 0.01, "m", false, ""},
        {"thickness", "Thickness", 0.1, 0.01, 5, 0.005, "m", false, ""}}},
  };
  return list;
}

const GeneratorInfo* findGenerator(const std::string& type) {
  for (const GeneratorInfo& g : generatorList())
    if (type == g.type) return &g;
  return nullptr;
}

double genParam(const Body& b, const char* key) {
  const GeneratorInfo* info = findGenerator(b.generator);
  if (!info) return 0;
  for (size_t i = 0; i < info->params.size(); ++i)
    if (std::string(info->params[i].key) == key)
      return i < b.genParams.size() ? b.genParams[i] : info->params[i].def;
  return 0;
}

Id addGenerated(Document& doc, const std::string& type, const Vec3& origin, double yaw, const std::vector<double>* params) {
  const GeneratorInfo* info = findGenerator(type);
  if (!info) return kNoId;
  Id b = addBody(doc, kWorldId, {origin, axisAngle({0, 0, 1}, yaw)}, type, Role::Environment);
  Body* body = doc.body(b);
  body->generator = type;
  body->genParams.clear();
  for (size_t i = 0; i < info->params.size(); ++i)
    body->genParams.push_back(params && i < params->size() ? (*params)[i] : info->params[i].def);
  regenerate(doc, b);
  return b;
}

bool regenerate(Document& doc, Id bodyId) {
  Body* body = doc.body(bodyId);
  if (!body) return false;
  const GeneratorInfo* info = findGenerator(body->generator);
  if (!info) return false;
  body->genParams.resize(info->params.size());
  for (size_t i = 0; i < info->params.size(); ++i)
    body->genParams[i] = std::clamp(body->genParams[i], info->params[i].min, info->params[i].max);
  // Replace the body's own geoms; a height field asset is reused so references stay valid.
  std::vector<Id> old;
  Id hfield = kNoId;
  for (const Geom& g : doc.geoms)
    if (g.body == bodyId) {
      old.push_back(g.id);
      if (g.type == GeomType::Hfield) hfield = g.hfield;
    }
  deleteElements(doc, old);
  Body copy = *doc.body(bodyId);  // stable reference while geoms are added
  Ctx c{doc, bodyId, copy};
  const std::string& t = copy.generator;
  if (t == "stairs") genStairs(c);
  else if (t == "ramp") genRamp(c);
  else if (t == "rough") genRough(c, hfield);
  else if (t == "waves") genWaves(c, hfield);
  else if (t == "stones") genStones(c);
  else if (t == "obstacles") genObstacles(c);
  else if (t == "arena") genArena(c);
  doc.touch();
  return true;
}

}  // namespace mjf
