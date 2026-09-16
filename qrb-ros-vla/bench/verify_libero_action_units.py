#!/usr/bin/env python3
"""Rung 1b: feed the dataset's own actions into the sim and check the proprio tracks.

Rung 1a (bench/verify_libero_contract.py) proved we can READ the simulator's state in
the same units and order the policy was calibrated on. This proves we can WRITE to it:
that the 7-D action vector the dataset records is directly feedable to `env.step()`
with no hidden rescaling, and that we are stepping at the right cadence.

Why this has to come before any policy claim: if the action units or the control rate
are wrong, a closed-loop rollout still runs, still renders, and still reports a success
rate -- just a meaningless one. Replaying the *recorded human actions* separates "our
bridge is wrong" from "the policy is bad", because the recorded actions are known to
complete the task. If they do not track here, nothing downstream is interpretable.

THE CADENCE QUESTION THIS SETTLES. The LeRobot dataset declares `fps: 10`, while
robosuite's LIBERO envs default to `control_freq = 20`. Those cannot both describe one
action per `env.step()`. Either the dataset was decimated from a 20 Hz recording (so
each recorded action should be applied for TWO env steps), or the env should be
constructed at 10 Hz. Guessing wrong halves or doubles every commanded motion, which
looks like a sluggish or overshooting policy rather than a units bug. So the script
sweeps `--repeat` and reports which cadence actually tracks the recording.

Expect drift regardless: this is open-loop integration of delta commands, so error
accumulates. The signal is not "zero error", it is which cadence tracks *dramatically*
better over the first tens of frames, and whether position error stays in millimetres
rather than exploding immediately.

Environment (all three mandatory):
    export MUJOCO_GL=osmesa
    export LIBERO_CONFIG_PATH=<dir containing config.yaml>
    export PYTHONPATH=<LIBERO repo root>

Usage:
    bench/verify_libero_action_units.py --episode artifacts/libero_ep0.npz
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from torchfree_init import load_init_states  # noqa: E402
from verify_libero_contract import TASK_STEM, libero_dir, quat_to_rotvec  # noqa: E402

# Artifact defaults are anchored to the repository, not the caller's working directory: these
# scripts are invoked by workshop/scripts/fastforward.sh and by hand from various places, and
# a bare relative default fails with FileNotFoundError depending on where you happen to be.
# An explicitly passed path is still interpreted relative to the cwd, which is what a user
# typing one expects.
_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


DIMS = ["x", "y", "z", "rx", "ry", "rz", "g1", "g2"]


def sim_state(obs: dict, previous: np.ndarray | None) -> np.ndarray:
    """The dataset's 8-D state layout, rebuilt from robosuite obs (rung 1a verdict).

    `previous` is threaded through so the axis-angle branch is chosen for continuity;
    LIBERO's home pose sits at an angle of ~pi, where a fixed sign test flips by 2*pi.
    """
    prev_rot = None if previous is None else previous[3:6]
    return np.concatenate([
        np.asarray(obs["robot0_eef_pos"], dtype=np.float64),
        quat_to_rotvec(obs["robot0_eef_quat"], prev_rot),
        np.asarray(obs["robot0_gripper_qpos"], dtype=np.float64),
    ])


def replay(env, inits, init_index, settle, actions, truth, repeat):
    """Apply each recorded action `repeat` times; return the tracked state trajectory."""
    env.reset()
    obs = env.set_init_state(inits[init_index])

    # Settling: the recording begins after the scene and gripper come to rest. Rung 1a
    # measured ~10 steps to converge on the dataset's opening state. Gripper -1 = open.
    neutral = np.zeros(7)
    neutral[6] = -1.0
    for _ in range(settle):
        obs, _, _, _ = env.step(neutral)

    traj = []
    prev = sim_state(obs, None)
    traj.append(prev)
    for t in range(len(actions)):
        for _ in range(repeat):
            obs, _, _, _ = env.step(np.asarray(actions[t], dtype=np.float64))
        prev = sim_state(obs, prev)
        traj.append(prev)
    return np.stack(traj)


def summarize(traj: np.ndarray, truth: np.ndarray, label: str) -> dict:
    """Compare a replayed trajectory against the recording, position error in mm."""
    n = min(len(traj), len(truth))
    a, b = traj[:n], truth[:n]
    pos_err = np.linalg.norm(a[:, :3] - b[:, :3], axis=1) * 1000.0  # mm
    rot_err = np.linalg.norm(a[:, 3:6] - b[:, 3:6], axis=1)
    grip_err = np.abs(a[:, 6] - b[:, 6])

    def at(k):
        return float(pos_err[k]) if k < n else float("nan")

    print(f"\n--- {label} ---")
    print(f"  frames compared      : {n}")
    print(f"  eef position error mm: t=1 {at(1):.2f}  t=10 {at(10):.2f}  "
          f"t=50 {at(50):.2f}  t=100 {at(100):.2f}  final {pos_err[-1]:.2f}")
    print(f"  eef position error mm: mean {pos_err.mean():.2f}  median "
          f"{np.median(pos_err):.2f}  max {pos_err.max():.2f}")
    print(f"  rotvec error (rad)   : mean {rot_err.mean():.4f}  max {rot_err.max():.4f}")
    print(f"  gripper error        : mean {grip_err.mean():.4f}  max {grip_err.max():.4f}")
    if rot_err.max() > 3.0:
        print("  WARNING: rotvec error exceeds 3 rad -- axis-angle branch flip, not "
              "a tracking failure")
    return {
        "label": label,
        "frames": n,
        "pos_err_mm_mean": float(pos_err.mean()),
        "pos_err_mm_median": float(np.median(pos_err)),
        "pos_err_mm_max": float(pos_err.max()),
        "pos_err_mm_at_10": at(10),
        "pos_err_mm_at_50": at(50),
        "rot_err_mean": float(rot_err.mean()),
        "rot_err_max": float(rot_err.max()),
        "grip_err_mean": float(grip_err.mean()),
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", default=_repo_path("artifacts/libero_ep0.npz"))
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--bddl-dir", default="",
                    help="default: resolved from LIBERO's own config")
    ap.add_argument("--init-dir", default="",
                    help="default: resolved from LIBERO's own config")
    ap.add_argument("--init-index", type=int, default=7, help="rung 1a best match")
    ap.add_argument("--settle", type=int, default=10, help="idle steps before replay")
    ap.add_argument("--frames", type=int, default=0, help="0 = whole episode")
    ap.add_argument("--repeat", type=int, nargs="+", default=[1, 2],
                    help="env steps per recorded action; sweeps the cadence question")
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    for var in ("MUJOCO_GL", "LIBERO_CONFIG_PATH", "PYTHONPATH"):
        if not os.environ.get(var):
            sys.exit(f"{var} is not set; see this script's docstring")

    args.bddl_dir = args.bddl_dir or libero_dir("bddl_files", args.suite)
    args.init_dir = args.init_dir or libero_dir("init_states", args.suite)

    z = np.load(args.episode, allow_pickle=False)
    truth = np.asarray(z["state"], dtype=np.float64)
    actions = np.asarray(z["actions"], dtype=np.float64)
    n = len(actions) if not args.frames else min(len(actions), args.frames)
    actions, truth = actions[:n], truth[: n + 1]
    print(f"episode: {n} actions, action range "
          f"[{actions[:, :6].min():+.3f}, {actions[:, :6].max():+.3f}] on pose dims, "
          f"gripper in {sorted(set(np.sign(actions[:, 6])))}")

    inits = load_init_states(
        str(Path(args.init_dir) / f"{TASK_STEM}.pruned_init")
    )
    inits = np.asarray(inits).reshape(len(inits), -1)

    from libero.libero.envs import OffScreenRenderEnv

    # use_camera_obs=False: rendering is 340 ms/observation on this board's software
    # rasterizer and contributes nothing to a proprio-tracking check.
    env = OffScreenRenderEnv(
        bddl_file_name=str(Path(args.bddl_dir) / f"{TASK_STEM}.bddl"),
        camera_heights=84,
        camera_widths=84,
        use_camera_obs=False,
    )
    inner = env.env
    print(f"env control_freq={getattr(inner, 'control_freq', '?')} "
          f"dataset fps={float(z['fps'])}")
    ctrl = getattr(inner, "robots", [None])[0]
    if ctrl is not None:
        c = getattr(ctrl, "controller", None)
        print(f"controller={type(c).__name__ if c else '?'} "
              f"action_dim={getattr(ctrl, 'action_dim', '?')}")
    print(f"action space: low={inner.action_spec[0][:7]}\n"
          f"              high={inner.action_spec[1][:7]}")

    results = []
    for rep in args.repeat:
        traj = replay(env, inits, args.init_index, args.settle, actions, truth, rep)
        results.append(summarize(traj, truth, f"repeat={rep} "
                                 f"({float(z['fps']) * rep:.0f} Hz effective)"))

    best = min(results, key=lambda r: r["pos_err_mm_at_50"])
    print(f"\nVERDICT: '{best['label']}' tracks best "
          f"(position error at t=50: {best['pos_err_mm_at_50']:.2f} mm)")
    for r in results:
        if r is not best:
            ratio = r["pos_err_mm_at_50"] / max(best["pos_err_mm_at_50"], 1e-9)
            print(f"         vs '{r['label']}': {r['pos_err_mm_at_50']:.2f} mm "
                  f"({ratio:.1f}x worse)")

    if args.json:
        Path(args.json).write_text(
            json.dumps({"episode": args.episode, "init_index": args.init_index,
                        "settle": args.settle, "results": results,
                        "verdict": best["label"]}, indent=2) + "\n")
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
