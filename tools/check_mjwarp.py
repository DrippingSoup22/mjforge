#!/usr/bin/env python3
"""Checks that an MJCF file exported by MJForge loads and simulates in MuJoCo Warp.

Usage:
  python tools/check_mjwarp.py scene.xml [--steps 200] [--nworld 8] [--device cuda|cpu]
                               [--nconmax N] [--njmax N]

It runs the same rollout (zero controls) in MuJoCo C and in MJWarp, then reports:
  * whether mujoco_warp.put_model accepts the model (unsupported features raise here),
  * NaN / divergence, contact and constraint buffer overflows,
  * the peak number of contacts and constraint rows (to size nconmax / njmax),
  * how far the MJWarp trajectory is from MuJoCo C (informative; solvers differ slightly).
Exit code 0 means the model is usable in MJWarp.
"""
import argparse
import math
import sys
import time

import numpy as np

import mujoco


def round_up(n: int) -> int:
  """Next 'nice' buffer size (16, 24, 32, 48, 64, ...)."""
  n = max(int(n), 1)
  size = 16
  while size < n:
    size = size * 3 // 2 if (size & (size - 1)) == 0 else (size // 3) * 4
  return size


def main() -> int:
  ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
  ap.add_argument("xml")
  ap.add_argument("--steps", type=int, default=200)
  ap.add_argument("--nworld", type=int, default=8)
  ap.add_argument("--device", default=None, help="Warp device, e.g. cuda:0 or cpu (default: Warp's default)")
  ap.add_argument("--nconmax", type=int, default=None)
  ap.add_argument("--njmax", type=int, default=None)
  args = ap.parse_args()

  mjm = mujoco.MjModel.from_xml_path(args.xml)
  print(f"MuJoCo {mujoco.__version__}: nbody={mjm.nbody} nq={mjm.nq} nv={mjm.nv} nu={mjm.nu} "
        f"ngeom={mjm.ngeom} nsensor={mjm.nsensor} timestep={mjm.opt.timestep}")

  # Reference rollout in MuJoCo C, tracking peak contacts / constraints.
  mjd = mujoco.MjData(mjm)
  if mjm.nkey > 0:
    mujoco.mj_resetDataKeyframe(mjm, mjd, 0)
  qpos_ref = []
  peak_ncon, peak_nefc = 0, 0
  t0 = time.time()
  for _ in range(args.steps):
    mujoco.mj_step(mjm, mjd)
    peak_ncon = max(peak_ncon, mjd.ncon)
    peak_nefc = max(peak_nefc, mjd.nefc)
    qpos_ref.append(mjd.qpos.copy())
  print(f"MuJoCo C: {args.steps} steps in {time.time() - t0:.2f}s, peak ncon={peak_ncon}, peak nefc={peak_nefc}")
  if not np.all(np.isfinite(mjd.qpos)):
    print("FAIL: MuJoCo C simulation produced NaN/Inf (unstable model).")
    return 1

  try:
    import warp as wp
    import mujoco_warp as mjw
  except ImportError as e:
    print(f"SKIP: mujoco_warp is not installed ({e}). pip install mujoco-warp")
    return 0

  nconmax = args.nconmax or round_up(max(peak_ncon * 1.5, 8))
  njmax = args.njmax or round_up(max(peak_nefc * 1.5, 16))
  device = args.device
  ctx = wp.ScopedDevice(device) if device else wp.ScopedDevice(wp.get_device())
  with ctx:
    try:
      m = mjw.put_model(mjm)
    except NotImplementedError as e:
      print(f"FAIL: MuJoCo Warp does not support a feature of this model: {e}")
      return 1
    except ValueError as e:
      print(f"FAIL: MuJoCo Warp rejected the model: {e}")
      return 1
    d = mjw.make_data(mjm, nworld=args.nworld, nconmax=nconmax, njmax=njmax)
    if mjm.nkey > 0:
      mjw.reset_data_keyframe(m, d, 0)
    t0 = time.time()
    worst = 0.0
    peak_pairs = 0  # broadphase candidate pairs per world: this is what nconmax must cover
    peak_contacts = 0
    for i in range(args.steps):
      mjw.step(m, d)
      if i < 20 or i == args.steps - 1 or (i + 1) % 10 == 0:
        if hasattr(d, "ncollision"):
          peak_pairs = max(peak_pairs, int(math.ceil(int(d.ncollision.numpy()[0]) / args.nworld)))
        if hasattr(d, "nacon"):
          peak_contacts = max(peak_contacts, int(math.ceil(int(d.nacon.numpy()[0]) / args.nworld)))
      if i == args.steps - 1 or (i + 1) % 50 == 0:
        qpos = d.qpos.numpy()
        if not np.all(np.isfinite(qpos)):
          print(f"FAIL: MJWarp produced NaN/Inf at step {i + 1}.")
          return 1
        worst = max(worst, float(np.max(np.abs(qpos[0] - qpos_ref[i]))))
    wp.synchronize()
    dt = time.time() - t0
    overflow = 0
    if hasattr(d, "overflow"):
      overflow = int(np.bitwise_or.reduce(d.overflow.numpy().astype(np.int64)))
  print(f"MJWarp ({wp.get_device()}): put_model OK, {args.steps} steps x {args.nworld} worlds in {dt:.2f}s "
        f"(includes kernel compilation)")
  print(f"Max |qpos(MJWarp) - qpos(MuJoCo C)| over the rollout: {worst:.3g}")
  print(f"Peak per world: broadphase pairs={peak_pairs}, contacts={peak_contacts}, constraint rows={peak_nefc}")
  sugg_ncon = round_up(max(peak_pairs, peak_contacts * 1.5, peak_ncon * 1.5, 8))
  sugg_njmax = round_up(max(peak_nefc * 1.5, 16))
  if overflow:
    names = []
    if hasattr(mjw, "OverflowType"):
      names = [e.name for e in mjw.OverflowType if overflow & int(e.value)]
    print(f"WARN: buffer overflow ({', '.join(names) or hex(overflow)}) with nconmax={nconmax}, njmax={njmax}. "
          f"MJWarp's broadphase needs nconmax >= the number of candidate geom pairs, not just contacts.")
  print(f"Suggested sizes: mjw.make_data(mjm, nworld=..., nconmax={sugg_ncon}, njmax={sugg_njmax})")
  print("PASS")
  return 0


if __name__ == "__main__":
  sys.exit(main())
