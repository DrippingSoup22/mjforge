# MJForge user guide

MJForge edits a MuJoCo model as a tree of bodies with geoms, joints, sites, cameras and lights,
plus actuators and sensors. Everything is saved in a project file (`.mjforge`, JSON) and exported to
MJCF. While you edit, MuJoCo compiles the model in the background and an analysis checks it.

## The window

| Panel | What it is for |
|---|---|
| **Outliner** (left) | The body tree. Drag a body onto another to re-parent it, right-click for actions, double-click to frame, F2 or the context menu to rename. The eye icon hides a subtree in the viewport. |
| **Viewport** (center) | Your scene, drawn by MJForge's own renderer. Gizmos move, rotate and resize the selection. Quick toggles at the top left: joints, centers of mass, x-ray, sites, names, frames. |
| **Inspector** (right) | Every property of the selection, grouped in sections. Orange labels are values that differ from MuJoCo's default; the arrow button resets them. Tooltips quote the MuJoCo documentation. With several elements of the same type selected, edits apply to all of them. |
| **Scene** (right, tab) | Simulation options (timestep, integrator, solver, friction cone, gravity, flags), compiler settings, visual settings for MuJoCo's viewers, presets, and keyframes. |
| **Issues** (bottom) | The analysis: errors, warnings, info and tips, each with an explanation and, when possible, a fix button. Click the element chips to select them. |
| **Actuators / Sensors** | Tables of all actuators and sensors, with buttons to add them in bulk. |
| **MJCF** | The XML that will be exported, updated live. |
| **Simulation** | Controls and readouts while simulating (see below). |

The toolbar holds the gizmo modes, snapping, the pick mode (bodies or geoms), buttons to add things,
the add-limb tool, mirror, the simulation controls, and the validation status on the right (click it
to open the Issues panel). The status bar shows counts, the model mass and compile time.

## Building a creature

**From a template.** *File > New* offers a quadruped, hexapod, biped, hopper and a robot arm. They
compile without warnings and stand (or hold their pose) in the simulator, so they make a good
starting point to modify.

**From scratch.** *Add > Model (creature root)* creates a free-floating body with a torso sphere.
Then:

1. Change the torso geom: select it (Alt+click, or pick mode *Geoms*), change its type and size in
   the Inspector, or press R and drag the resize gizmo.
2. Press **G** for the add-limb tool and click on the torso where a leg should start. The panel in
   the viewport sets segments, length, radius, joint range, direction, actuators and the foot. With
   *Mirror* on, the limb appears on the other side too. Press Esc when done.
3. *Tools > Place models on the floor*, then press Space to see whether it stands.

Every limb segment is a body with a hinge joint (local Y axis) and a capsule. Joints rotate their
body relative to the parent; the joint's arc in the viewport shows its range, and the needle shows
its current angle while simulating.

**Roles.** Every top-level body is either part of a *model* (creature, robot) or of the
*environment* (prop, obstacle, terrain). The role decides where the body goes when you export models
and environment separately, and the analysis treats models differently (legs, balance, strength).

**Arrays and mirrors.** *Edit > Array* makes linear or radial copies: select one hexapod leg and
make 5 radial copies around Z with a total angle of 360 degrees to get six legs 60 degrees apart.
*Ctrl+M* mirrors a body subtree left/right with names (`left` becomes `right`) and joint axes that
move symmetrically.

**Collisions.** *Tools > Collisions* has presets for a selected model: everything collides, no
self-collisions, or only the feet touch the world (the cheapest setup for locomotion training).
*Tools > Exclude contacts between selected bodies* adds `<exclude>` pairs.

**Sensors.** *Tools > Add sensors* adds an IMU at the model root or touch sensors on the feet; the
Sensors panel adds joint position and velocity sensors in one click.

## Physics properties

- **Mass.** By default MuJoCo computes mass and inertia from the geoms and their density
  (1000 kg/m³, water). Set a geom's density or mass, or give a body an explicit inertial.
- **Joints.** Range, damping (resists velocity), armature (extra rotor inertia: makes stiff servos
  stable at larger timesteps), friction loss, and springs.
- **Actuators.** *Motor*: the control is a torque (times the gear). *Position*: the control is the
  target angle, the actuator applies `kp * (target - angle)` with damping from `kv` or a damping
  ratio. *Velocity*: target speed. Control range limits the command, force range limits the output.
- **Contacts.** Friction (sliding, torsional, rolling), contact dimension, solver softness, margin,
  and the `contype`/`conaffinity` bit masks deciding which geoms collide.
- **Scene.** Timestep, integrator (implicitfast is a good default and runs in MJWarp), solver
  (Newton), and the friction cone: elliptic cones with impratio 10–100 keep standing feet from
  slowly creeping.

## Reading the analysis

Issues are grouped by area:

- **Model**: structural problems MuJoCo would reject or silently misinterpret (missing targets,
  inverted ranges, massless moving bodies).
- **Physics**: whether the actuators can do their job. MJForge estimates the load on every joint:
  lifting the moved subtree against gravity, and supporting the body while walking (about half the
  feet on the ground). It compares that with the actuator's capacity (force range, or control range
  times gain and gear). Position servos also get a sag estimate: how far the joint gives way under
  the body weight with the current `kp`.
- **Stability**: stiffness against the timestep. A servo with natural frequency ω and timestep dt is
  accurate when ω·dt is below about 0.5 and explodes above 2. The fix adds armature.
- **Start pose**: the model floating above or starting inside the floor, the center of mass outside
  the feet, overlapping geoms. Evaluated at the start pose (the first keyframe if there is one).
- **MuJoCo Warp**: features MJWarp rejects, with fixes.
- **Performance**: tips for training speed, such as which contact pairs need MJWarp's slower convex
  collider and how many self-collision pairs your creature has.

Selecting a joint shows its *Physics check* in the Inspector: capacity against the estimated need
as a bar, the sag, the stiffness margin, and the effective inertia it moves.

## Simulating

Press **Space** (or the green Play button). The simulation uses MuJoCo on the CPU; the viewport
gets a colored frame and a badge with the simulated time.

- **Control modes** (Simulation panel): *Hold pose* keeps the start pose: position servos target
  it, torque motors use a built-in PD controller limited by their control range. It answers "can
  my actuators carry this body?". *Manual* gives a slider per actuator: the raw control a policy would
  output. *Sine test* sweeps every joint through part of its range. *Limp* turns actuation off.
- **Push**: left-drag any moving body; a spring pulls the grabbed point toward the mouse.
- **Live tuning**: edit anything in the Inspector while it runs. The model recompiles in the
  background and the simulation continues from the same state. Structural changes (adding a body)
  restart from the start pose.
- **Readouts**: per actuator the control, position, tracking error (target minus actual) and force
  against capacity; the bar turns red at the limit. Click a row to plot it and to select the
  actuator in the Inspector.
- **Instability**: if the dynamics blow up, the simulation pauses, names the joint where it
  happened, and lists the usual causes.
- **Keyframes**: *Save pose as keyframe* stores the current pose as the start pose (keyframe 0)
  of every later simulation and of the exported model.
- **Keys**: Space play/pause, Shift+Space stop, Ctrl+R reset, Right arrow single step while paused.

Static parts of the environment stay editable while simulating: move an obstacle and the running
simulation picks it up.

## Environments

*Add > Environment* creates generated pieces: stairs, ramp, rough terrain, waves, stepping stones,
obstacle field, arena walls. Each one is a static body; select it to edit its parameters in the
Inspector (the geometry regenerates as you drag), move it with the gizmo, or press *Bake* to turn it
into ordinary geoms. Rough terrain and waves are single height-field geoms with a flat spawn area at
the center: much cheaper for MuJoCo Warp than many boxes, because every geom that can touch the
robot adds broadphase work in every world.

*Add > Environment prop* adds single static or dynamic objects (crates, balls).

## Combining and importing

*File > Import into scene* (Ctrl+I) reads another project or an MJCF file and lets you choose:

- **Models**: the creatures/robots, with their actuators, sensors and keyframes, placed next to
  yours (adjust the offset).
- **Environment**: floor, props, terrain, lights and sky. *Replace my current environment* removes
  yours first; otherwise a second floor is skipped.
- **Simulation settings**: take over the other file's timestep, gravity, solver...

Clashing names are made unique. *File > Open* also opens MJCF files directly as a new project.

MJCF is read with MuJoCo's own parser and compiler, so defaults classes, `<include>`, Euler angles,
`fromto` and degree units are resolved exactly. Not yet supported: mesh geoms are replaced by boxes
of the same extent (the bodies keep MuJoCo's exact mass and inertia), tendons, equality constraints,
explicit contact pairs and flex objects are reported and skipped.

## Exporting

*File > Export MJCF* (Ctrl+E):

- **Scope**: everything in one file, only the models, or only the environment.
- **Split**: writes `scene.xml` (environment, includes the model) and `scene_model.xml` (the model,
  loadable on its own), like MuJoCo Menagerie.
- **Angle unit**: radians (default) or degrees in the file. Keyframes and control ranges are raw
  values in either case, as MuJoCo expects.
- **MuJoCo Warp loader**: writes `name_mjwarp.py`, which loads the file with `mujoco_warp`, resets
  to the start keyframe and sets `nconmax`/`njmax` from the contact peaks of your last simulation
  (or of a short settle computed during export).

## Files and settings

Projects are JSON (`.mjforge`) and keep editor metadata the MJCF can't (roles, generator
parameters, hidden/locked flags, the camera). Preferences and recent files are stored in
`$XDG_CONFIG_HOME/mjforge` (usually `~/.config/mjforge`).

## Keyboard and mouse

| Mouse | |
|---|---|
| Left click / Shift / Ctrl | select / add / toggle (Alt picks geoms instead of bodies) |
| Double click | frame the clicked element |
| Right drag / right click | orbit / context menu |
| Middle drag, Shift + right drag | pan |
| Wheel | zoom toward the cursor |
| Left drag while simulating | push a body |

| Keys | |
|---|---|
| Q W E R | select, move, rotate, resize |
| Tab | gizmo in world / local frame (Ctrl inverts snapping while dragging) |
| G | add-limb tool (Esc leaves it) |
| F, Home | frame selection, frame everything |
| Numpad 1 / 3 / 7 / 5 | front, side, top view, orthographic |
| Ctrl+D, Ctrl+M, Del | duplicate, mirror left/right, delete |
| Ctrl+Z, Ctrl+Y (Ctrl+Shift+Z) | undo, redo |
| Space, Shift+Space | play/pause, stop simulating |
| Ctrl+R, Right arrow | reset simulation, single step while paused |
| X, J, C, L | x-ray, joints, centers of mass, names |
| Ctrl+O, Ctrl+S, Ctrl+Shift+S | open (project or MJCF), save, save as (*File > New* for templates) |
| Ctrl+I, Ctrl+E, Ctrl+Q | import into scene, export MJCF, quit |
