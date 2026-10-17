# MJForge — Plan & Design

MJForge is a fast, native, visual editor for building **MuJoCo models (creatures/robots)** and
**environments (terrain, obstacles, props)**, focused on producing MJCF that loads cleanly in
**MuJoCo Warp (MJWarp)** for large-scale RL training.

This document is the roadmap. It is kept up to date as milestones are completed.

---

## 1. Goals

1. **Total freedom with simple building blocks.** Everything is built from MuJoCo primitives
   (box, sphere, capsule, cylinder, ellipsoid, plane, heightfield, later meshes). No polygon
   modeling — this is not Blender. Every MJCF attribute that matters for physics is editable.
2. **Output always loads in MuJoCo / MJWarp.** The editor compiles the model with the real
   MuJoCo library in the background on every change and checks MJWarp-specific restrictions,
   so problems show up while you edit, not when training starts.
3. **Physics-aware guidance.** The editor understands mass, inertia, gravity, actuator strength
   and timestep stability, and warns about things like "this 10 kg creature has knee motors that
   can deliver 2 µN·m" before you ever press play.
4. **See how it behaves.** A one-click simulation preview (MuJoCo C physics, our own renderer)
   with actuator sliders, test signals, perturbations and plots.
5. **Fast and responsive.** Native C++/OpenGL, immediate-mode UI, no heavy rendering stack.
   Idle CPU use near zero; heavy work (compile/validation) runs off the UI thread.
6. **Model and environment separately or together.** Build either or both, save them as
   projects, combine them, and export a single scene file or separate files.

## 2. Key decisions

| Decision | Choice | Why |
|---|---|---|
| Language | **C++17** | MuJoCo is C/C++; Dear ImGui is C++. Native speed. |
| UI | **Dear ImGui (docking)** + ImGuizmo + ImPlot | The standard for responsive tool UIs (MuJoCo's own new Studio uses it). Dockable panels, gizmos, live plots. |
| Window/GL | **GLFW 3.4 + OpenGL 3.3 core** (glad) | Portable (Linux/Windows/macOS), light. |
| Renderer | **Our own** (not MuJoCo's) | Lightweight; editor overlays (gizmos, joint arcs, COM, selection) are first-class. |
| Physics | **MuJoCo C library** (linked) | Used only for compile/validation, analysis and the preview sim. Same library MJWarp uses to load XML (`mujoco.MjModel.from_xml_path` → `mjw.put_model`). |
| Document | **Own data model** (flat, ID-based) | Editor-friendly (stable IDs, undo snapshots, metadata), independent of mjSpec API churn. |
| Project file | **JSON** (`.mjforge`) | Human-readable, versioned, diff-able. |
| Export | **MJCF XML** written by MJForge | Clean, minimal (only non-default attributes), deterministic. |
| Import | **MJCF via MuJoCo's parser (mjSpec)** | Exact MuJoCo semantics (defaults, includes, orientations). |
| MuJoCo version | 3.16.0 by default; can use the copy inside your Python `mujoco` package | Guarantees the editor validates with the *same* MuJoCo that MJWarp uses. |

### MuJoCo vs MJWarp: what the editor enforces

MJWarp 3.16 supports nearly all of MuJoCo. Verified from `mujoco_warp/_src/io.py` (`put_model`):

- **Unsupported (hard errors in MJWarp):** PGS solver; noslip solver (`noslip_iterations>0`);
  body/actuator/sensor plugins; `pid` and `orientation` actuators (PID gain/dyn, SO3
  transmission/bias); `distance` and `flexvert` equality; disabling `midphase`/`autoreset`;
  enabling `override`/`fwdinv`/`diagexact`/`ipc`; contact-sensor linvel/angvel data;
  dense Jacobian with `nv > 60`; sleeping without Newton solver.
- **Margin trap:** with default flags (`multiccd`/`nativeccd` enabled) any colliding pair of
  box/mesh geoms with nonzero `margin` raises an error.
- **Performance model:** collisions are either *primitive* (fast) or *CCD* (slower, more
  memory, longer compile). Primitive: plane vs sphere/capsule/ellipsoid/cylinder/box/mesh;
  sphere–sphere/capsule/cylinder/box; capsule–capsule/box. CCD: anything with ellipsoid, mesh,
  heightfield, cylinder–cylinder/box/capsule, box–box. The editor lists which CCD pair types
  your model creates and suggests cheaper alternatives.
- **Buffer sizes:** MJWarp preallocates contacts/constraints per world (`nconmax`, `njmax`).
  The editor measures peak contacts/constraints during the preview simulation and recommends
  values (exported as a Python loader snippet).

### Model + environment in one file?

MuJoCo loads exactly one model, so for training the creature and the world end up in one
`mjModel`. MJForge supports all the standard ways of getting there:

1. **Single file** (default): environment + model(s) merged in one `scene.xml`. Most robust.
2. **Split files:** `model.xml` (standalone, loadable on its own) + `scene.xml` that does
   `<include file="model.xml"/>` and adds floor, lights, obstacles (the MuJoCo Menagerie layout).
3. **Combine in the editor:** open an environment project, *Import* a model project (or any
   MJCF), place it, export.

## 3. Architecture

```
src/
  app/        main.cpp, App (window, main loop, CLI: --export/--validate/--screenshot)
  core/       Math, Document (data model), Ops (add/delete/duplicate/mirror/reparent),
              History (undo/redo), Kinematics & MassProps, MjcfWriter, ProjectIO (JSON),
              MjcfImport (mjSpec), Templates, Generators
  analysis/   Validator (structure, physics plausibility, MJWarp rules), Physics (torques,
              effective inertia, stability), Issue list with auto-fixes
  sim/        MujocoBridge (VFS compile, error handling, worker thread), Simulator
  render/     GL helpers, Shader, primitive Meshes, Renderer (lit geoms, checker planes,
              infinite grid, shadows), DebugDraw (lines/arcs/arrows), Camera, Picking
  ui/         Editor state, Viewport, Outliner, Inspector + property widgets, Actuators,
              Sensors, Issues, Stats, XML preview, Simulation panel, Scene settings,
              File dialog, Export dialog, Theme, MJCF attribute docs (tooltips)
```

- **Document:** flat vectors of plain structs (`Body`, `Geom`, `Joint`, `Site`, `Camera`,
  `Light`, `Actuator`, `Sensor`, `Material`, `Texture`, `Exclude`, …) with stable 32-bit IDs.
  References use IDs, converted to names on export, so renames never break references.
  Body 0 is the world. Heavy payloads (heightfield/mesh data) are shared copy-on-write so undo
  snapshots stay cheap.
- **Undo/redo:** whole-document snapshots (documents are small); continuous edits (drags,
  sliders) are coalesced into one step.
- **Validation pipeline:** every change bumps a revision → debounced export to XML → worker
  thread compiles with `mj_loadXML` from an in-memory VFS → results (errors, compiled masses,
  initial contacts) flow back to the Issues and Stats panels. The UI thread never blocks.
- **Viewport:** scene rendered into a multisampled FBO shown in a dockable ImGui window.
  CPU ray picking against analytic primitives. ImGuizmo for translate/rotate/scale (scale edits
  geom size — MuJoCo has no scale).
- **Simulation preview:** compiles the current document, steps MuJoCo on the main thread within
  a per-frame time budget, renders from `mjData`. Parameter edits during simulation trigger a
  recompile with state transfer, so you can tune `kp`, damping, friction, … live.

## 4. Milestones

Each milestone ends in a working, committed application.

### M0 — Foundations
- [x] Research MuJoCo 3.16 / MJWarp 3.16 feature support and sizing API
- [x] Repo, CMake (FetchContent: GLFW, ImGui docking, ImGuizmo, ImPlot, nlohmann/json;
      vendored glad + stb), MuJoCo discovery (`MUJOCO_DIR` / Python package / release download)
- [x] App shell: window, docking layout, theme, fonts, HiDPI, idle-friendly main loop
- [x] `--screenshot` / scripted frames for headless testing (Xvfb)

### M1 — Document, viewport, editing
- [x] Data model + undo/redo + selection
- [x] Renderer: primitives (shader-stretched capsules), checker planes, infinite grid,
      lighting, shadows, selection highlight; joint/COM/site/light/camera overlays
- [x] Orbit camera, view presets, frame-selected; picking; ImGuizmo with snapping
- [x] Outliner (tree, drag & drop reparent, context menus, rename, visibility)
- [x] Inspector (property grid, MuJoCo defaults with reset, units, doc tooltips,
      multi-selection editing)
- [x] Add menu (bodies, geoms, joints, sites, lights, cameras)
- [x] Save/Open project (JSON), Export MJCF, recent files

### M2 — Physics authoring & validation
- [x] Complete geom / joint / body / inertial attributes
- [x] Actuators: motor, position, velocity, intvelocity, damper, cylinder, muscle, adhesion,
      general, dcmotor (+ table panel, "add actuators to all joints")
- [x] Sensors, sites, contact excludes, scene options (timestep, integrator, solver, …)
- [ ] Explicit contact `<pair>`s
- [x] Background MuJoCo compile; Issues panel with click-to-select and auto-fix buttons
- [x] Physics analysis: subtree mass/COM, effective joint inertia, gravity torque vs actuator
      strength, servo sag, PD/timestep stability, mass ratios, initial penetration
- [x] MJWarp rules + performance hints; live XML preview

### M3 — Simulation preview

Done: Play (Space) runs MuJoCo on a private copy of the compiled model and draws it with the
editor's renderer. Control modes: *Hold pose* (servos target the start pose; torque motors get a
built-in, stability-limited PD), *Manual* sliders, *Sine test*, *Limp*. Drag bodies to push them
(spring like MuJoCo's own perturbation). Contacts and contact forces are drawn; the panel shows
per-actuator control, position, tracking error and force vs capacity, with an ImPlot chart of
target/actual/force. Edits while running recompile in the background and swap the model in,
keeping the state when the structure is unchanged. Blow-ups are detected (NaN or MuJoCo's
bad-qacc warnings), attributed to a joint and explained. Peak contacts, constraint rows and
MJWarp broadphase candidates give `nconmax`/`njmax` (copyable Python snippet).

- [x] Play/pause/step/reset, speed control, instability detection
- [x] Actuator sliders, test signals (sine sweep per actuator), perturbation by dragging
- [x] Contact points/forces overlay, live plots (joint vs target, force vs limit)
- [x] Live parameter tuning (recompile + state transfer)
- [x] Peak `ncon`/`nefc` measurement → `nconmax`/`njmax` recommendation

### M4 — Modeling tools, environments, import/combine

Done: MJCF import through MuJoCo's own parser/compiler (verified on Menagerie robots), File >
Import into scene to combine projects or MJCF (models and/or environment, replace environment,
automatic placement, name de-duplication), keyframes (start pose), environment generators with
live parameters (stairs, ramp, rough/wave height fields, stepping stones, obstacles, arena), the
add-limb tool with auto-mirroring, linear/radial arrays, the hopper template, and a MuJoCo Warp
loader script written next to exported XML (buffer sizes measured in the preview or a short
headless settle).
- [x] Add-limb tool (click a surface to attach a jointed limb), extend limb
- [x] Mirror subtree (left/right naming, symmetric joint axes), duplicate
- [x] Linear/radial array
- [x] Drop to floor, snapping
- [ ] Align / distribute
- [x] Creature templates (quadruped, hexapod, biped, arm)
- [x] Hopper template
- [x] Environment generators: stairs, ramps, rough terrain, heightfield (noise), arena walls,
      obstacle fields; materials & textures; lights & cameras
- [x] Import MJCF (mjSpec); import a model/environment from another project
- [x] Split export (`scene.xml` + `model.xml`)
- [x] Python MJWarp loader snippet on export

### M5 — Polish
- [x] Keyframes (per-joint storage, start pose, capture from simulation)
- [x] Pose mode (P): edit the start pose in the viewport with joint rings and sliders, mirror
      left/right (axis-aware), place on the floor; position servos get matching keyframe controls
- [x] Mesh assets: imported with MJCF files and loaded from STL/OBJ/MSH through MuJoCo, exact
      volume, mass and inertia, rendering and picking; projects store them as base64, exports write
      big meshes as STL files and copy texture images next to the XML
- [x] Keyboard & mouse help (F1), README with screenshots, user guide (docs/USER_GUIDE.md)
- [x] Tendons (fixed, spatial with wrapping and pulleys), equality constraints (connect, weld,
      joint, tendon) and contact pairs: editing, drawing, import/export, merge, analysis checks,
      tendon motors in the simulation; Menagerie robots using them round-trip exactly
- [x] CI on Linux, Windows and macOS
- [ ] Preferences window
- [x] Unit/integration tests (export → MuJoCo compile, JSON and MJCF round trips, analysis,
      simulator, generators, merge), `tools/check_mjwarp.py`, example projects verified in MJWarp,
      GitHub Actions CI (build, tests, headless validation, MJWarp CPU check)

## 5. Testing strategy

- **C++ unit tests:** math, document operations, mass properties vs MuJoCo's compiled
  `body_mass`/`body_inertia`, writer output compiles in MuJoCo, project round-trip.
- **MJWarp verification:** `tools/check_mjwarp.py file.xml` loads with `mujoco`, runs
  `mjw.put_model` + `make_data` + a few `mjw.step` (works on CPU, faster on GPU).
- **GUI:** headless runs under Xvfb with `--screenshot` for visual checks of every milestone.

## 6. Risks & mitigations

| Risk | Mitigation |
|---|---|
| MuJoCo version mismatch between editor and the user's MJWarp | Build against the Python package's `libmujoco`; XML stays conservative (only non-default attributes). |
| Ambiguous angle units when files are combined | Always write `<compiler angle="radian" autolimits="true"/>` (degree option available). |
| Compile cost on big models | Debounced background compile; UI never waits. |
| Scope creep | Milestones above; each one ships a usable app. |
