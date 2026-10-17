A native visual editor for building creatures, robots and environments for MuJoCo and MuJoCo Warp, aimed at reinforcement learning. MuJoCo 3.16.0 is included: nothing else to install.

## Download

| System | File |
|---|---|
| Windows 10 or 11, 64-bit | `MJForge-@VERSION@-windows-x86_64.zip` |
| macOS 11 or newer, Apple silicon | `MJForge-@VERSION@-macos-arm64.zip` |
| Linux x86_64 with glibc 2.35+ (Ubuntu 22.04, Debian 12, Fedora 36 or newer) | `MJForge-@VERSION@-linux-x86_64.tar.gz` |

Unpack it anywhere and start it from the folder:

- **Windows:** run `mjforge.exe`. The app is not code-signed, so Windows may say it protected your PC: click *More info*, then *Run anyway*.
- **macOS:** the app is not notarized by Apple, so macOS refuses to open it the first time. Open Terminal, type `xattr -dr com.apple.quarantine ` (with a space at the end), drag `MJForge.app` into the Terminal window and press Return; then open the app normally. (MuJoCo 3.16 has no Intel Mac version.)
- **Linux:** run `./mjforge`. It needs an X11 or XWayland desktop with OpenGL 3.3.

The folder also holds five example scenes (`examples/`), the user guide (`docs/USER_GUIDE.md`), a quick start (`QUICKSTART.txt`) and the licenses of the included software (`THIRD_PARTY_LICENSES.txt`).

## What you can do

- **Build** from MuJoCo's primitives with gizmos, snapping and undo: bodies, geoms, joints, sites, cameras, lights, every actuator type (including MuJoCo 3.16's DC motor), sensors, tendons, equality constraints and contact pairs. Templates: quadruped, hexapod, biped, hopper, robot arm.
- **Generate environments** with live parameters: stairs, ramps, rough terrain, waves, stepping stones, obstacle fields, arena walls.
- **Understand** the model: every edit is compiled by MuJoCo in the background, and the Issues panel explains problems in plain language with one-click fixes: actuators too weak for their load, unstable stiffness, start poses inside the floor, models MuJoCo or MuJoCo Warp would refuse.
- **Simulate** in the editor (Space) and **export** clean MJCF plus a ready-to-run MuJoCo Warp loader script.

The same program works from the command line: `mjforge model.xml --validate`, `mjforge project.mjforge --export out.xml --loader`, `mjforge --help`.
