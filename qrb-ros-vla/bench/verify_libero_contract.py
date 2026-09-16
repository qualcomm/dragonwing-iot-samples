#!/usr/bin/env python3
"""Resolve the LIBERO simulator's observation contract against the recorded dataset.

This is rung 1 of the closed-loop verification ladder. Before any task success rate
can mean anything, we have to prove that the observations we hand the policy from a
live sim are the SAME QUANTITIES, in the same order and orientation, as the ones the
policy was calibrated on. Two conventions silently destroy a VLA's accuracy while
everything still appears to run:

  1. The 8-D proprio state layout. Pi0.5 discretizes state into the *language prompt*
     (docs/DESIGN.md section 4), so a permuted or mis-scaled state produces a garbage
     prompt, not an error.
  2. Camera orientation. robosuite returns frames whose vertical axis is flipped
     relative to what the LeRobot dataset stores, and openvla/openpi LIBERO evals
     compensate. Get it wrong and the policy degrades with no diagnostic at all.

Rather than trusting documentation for either, this script compares the live sim
against artifacts/libero_ep0.npz -- the actual recorded episode whose task string
matches LIBERO-10 task
`LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_yellow_and_white_mug_on_the_right_plate`
exactly. Ground truth is on disk, so both questions are decidable, not arguable.

Method:
  * scan the task's recorded init states, reset the sim to each, and score candidate
    state reconstructions built from robosuite observation keys against the dataset's
    state[0]. The winning (init index, reconstruction) pair identifies BOTH which
    initial condition the recording used and how to rebuild the 8-D vector.
  * for that init, render agentview + wrist and correlate each against the dataset's
    stored frames under all four dihedral orientations. The winner is the transform a
    live client must apply.

Environment (all three are mandatory; see the LIBERO install notes):
    export MUJOCO_GL=osmesa
    export LIBERO_CONFIG_PATH=<dir containing config.yaml>
    export PYTHONPATH=<LIBERO repo root>

Usage:
    bench/verify_libero_contract.py --episode artifacts/libero_ep0.npz
"""

from __future__ import annotations

import argparse
import itertools
import json
import os
import sys
import time
from pathlib import Path

import numpy as np

# Artifact defaults are anchored to the repository, not the caller's working directory: these
# scripts are invoked by workshop/scripts/fastforward.sh and by hand from various places, and
# a bare relative default fails with FileNotFoundError depending on where you happen to be.
# An explicitly passed path is still interpreted relative to the cwd, which is what a user
# typing one expects.
_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


TASK_STEM = (
    "LIVING_ROOM_SCENE5_put_the_white_mug_on_the_left_plate_and_put_the_"
    "yellow_and_white_mug_on_the_right_plate"
)

# The four dihedral transforms that could plausibly relate a robosuite render to a
# stored dataset frame. Horizontal-only mirroring is included so that a false match
# cannot masquerade as the vertical flip we expect.
ORIENTATIONS = {
    "identity": lambda a: a,
    "flipud (vertical)": np.flipud,
    "fliplr (horizontal)": np.fliplr,
    "rot180": lambda a: np.rot90(a, 2),
}


def quat_to_rotvec(
    quat_xyzw: np.ndarray, previous: np.ndarray | None = None
) -> np.ndarray:
    """Axis-angle (rotation vector) from an (x, y, z, w) quaternion.

    Hand-rolled rather than via scipy so the script has one fewer hard dependency, and
    so the branch convention is explicit instead of inherited.

    THE BRANCH CHOICE IS NOT COSMETIC. LIBERO's Panda home pose points the gripper
    straight down, which puts the rotation angle at almost exactly pi -- the precise
    point where the axis-angle representation is discontinuous. Pinning the branch by
    the sign of the scalar part (`w >= 0`) is the obvious thing to do and it is WRONG
    here: measured on this board, a sim rollout flips from +3.14 to -3.14 partway
    through, a jump of 2*pi, while representing the same physical orientation.

    That flip is silent and destructive. State is spliced into the language prompt as
    256 discretized bins (docs/DESIGN.md section 4), so a sign flip reads to the policy
    as the wrist having spun 360 degrees between two consecutive control steps.

    The recorded dataset shows no such flip: over all 200 frames of episode 0 its first
    rotvec component stays within [+2.95, +3.20] and the largest step-to-step change is
    0.021. So the correct rule is CONTINUITY -- pick whichever of the two equivalent
    representatives lies nearer the previous state -- not a fixed sign test. Pass
    `previous` on every call after the first.
    """
    q = np.asarray(quat_xyzw, dtype=np.float64)
    q = q / (np.linalg.norm(q) + 1e-12)
    x, y, z, w = q
    if w < 0.0:  # same rotation, shorter path -- a starting point, not the answer
        x, y, z, w = -x, -y, -z, -w
    vec_norm = float(np.linalg.norm([x, y, z]))
    if vec_norm < 1e-9:
        return np.zeros(3)
    axis = np.array([x, y, z]) / vec_norm
    angle = 2.0 * np.arctan2(vec_norm, w)
    rotvec = axis * angle
    if previous is None:
        return rotvec
    # The equivalent representative on the far side of the wrap: same axis, angle
    # reduced by a full turn, which negates the direction of travel.
    alt = axis * (angle - 2.0 * np.pi)
    prev = np.asarray(previous, dtype=np.float64)
    return alt if np.linalg.norm(alt - prev) < np.linalg.norm(rotvec - prev) else rotvec


def libero_dir(kind: str, suite: str) -> str:
    """Resolve LIBERO's own asset directories instead of hardcoding a path.

    `get_libero_path` reads the config that LIBERO_CONFIG_PATH points at, which
    scripts/install-libero.sh writes. Hardcoding an install location here would make these
    harnesses depend on one machine's scratch directory -- and a claim that a number is
    reproducible from bench/ is false if the default arguments only work on the board it
    was first run on.
    """
    from libero.libero import get_libero_path

    return str(Path(get_libero_path(kind)) / suite)


def read_init_states(init_dir: Path, stem: str) -> np.ndarray:
    """Load a task's recorded init states without importing torch.

    LIBERO ships these as torch.save archives. Pulling in torch to read what is
    ultimately a float array would drag the entire CUDA sbsa wheel stack onto a board
    with no NVIDIA GPU, so bench/torchfree_init.py reads the zip+pickle directly.
    """
    for suffix in (".pruned_init", ".init"):
        path = init_dir / f"{stem}{suffix}"
        if path.exists():
            break
    else:
        sys.exit(f"no init-state file for {stem} under {init_dir}")

    sys.path.insert(0, str(Path(__file__).resolve().parent))
    from torchfree_init import load_init_states as _load

    arr = np.asarray(_load(str(path)))
    return arr.reshape(arr.shape[0], -1) if arr.ndim > 1 else arr[None, :]


def candidate_states(obs: dict) -> dict[str, np.ndarray]:
    """Every plausible 8-D reconstruction of the dataset's `state` from robosuite obs."""
    pos = np.asarray(obs["robot0_eef_pos"], dtype=np.float64)
    quat = np.asarray(obs["robot0_eef_quat"], dtype=np.float64)
    grip = np.asarray(obs["robot0_gripper_qpos"], dtype=np.float64)
    rotvec = quat_to_rotvec(quat)
    rotvec_neg = quat_to_rotvec(-quat)

    out = {
        "eef_pos + rotvec + gripper_qpos": np.concatenate([pos, rotvec, grip]),
        "eef_pos + rotvec(-q branch) + gripper_qpos": np.concatenate(
            [pos, rotvec_neg, grip]
        ),
        "eef_pos + rotvec + gripper_qpos[::-1]": np.concatenate(
            [pos, rotvec, grip[::-1]]
        ),
    }
    if "robot0_eef_quat_site" in obs:
        out["eef_pos + rotvec(site quat) + gripper_qpos"] = np.concatenate(
            [pos, quat_to_rotvec(np.asarray(obs["robot0_eef_quat_site"])), grip]
        )
    return out


def ncc(a: np.ndarray, b: np.ndarray) -> float:
    """Normalized cross-correlation over flattened float images, in [-1, 1]."""
    x = a.astype(np.float64).ravel()
    y = b.astype(np.float64).ravel()
    x -= x.mean()
    y -= y.mean()
    denom = np.linalg.norm(x) * np.linalg.norm(y)
    return float(x @ y / denom) if denom > 1e-12 else 0.0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", default=_repo_path("artifacts/libero_ep0.npz"))
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--bddl-dir", default="",
                    help="default: resolved from LIBERO's own config")
    ap.add_argument("--init-dir", default="",
                    help="default: resolved from LIBERO's own config")
    ap.add_argument("--max-inits", type=int, default=50, help="0 = all")
    ap.add_argument("--res", type=int, default=256)
    ap.add_argument("--json", default="", help="write the verdict here")
    args = ap.parse_args()

    if not Path(args.episode).exists():
        sys.exit(f"missing {args.episode}")
    for var in ("MUJOCO_GL", "LIBERO_CONFIG_PATH", "PYTHONPATH"):
        if not os.environ.get(var):
            sys.exit(f"{var} is not set; see this script's docstring")

    # Resolved after the env check above, since it needs LIBERO importable.
    args.bddl_dir = args.bddl_dir or libero_dir("bddl_files", args.suite)
    args.init_dir = args.init_dir or libero_dir("init_states", args.suite)

    z = np.load(args.episode, allow_pickle=False)
    ds_state = np.asarray(z["state"], dtype=np.float64)
    ds_image = np.asarray(z["image"])
    ds_wrist = np.asarray(z["wrist_image"])
    task = str(z["task"])
    print(f"dataset : {len(ds_state)} frames, state{ds_state.shape[1:]} "
          f"images {ds_image.shape[1:]}, task '{task}'")
    print(f"state[0]: {np.array2string(ds_state[0], precision=4, floatmode='fixed')}")

    bddl = Path(args.bddl_dir) / f"{TASK_STEM}.bddl"
    if not bddl.exists():
        sys.exit(f"missing bddl {bddl}")

    inits = read_init_states(Path(args.init_dir), TASK_STEM)
    n_inits = len(inits) if not args.max_inits else min(len(inits), args.max_inits)
    print(f"init states: {len(inits)} available, scanning {n_inits}")

    from libero.libero.envs import OffScreenRenderEnv

    t0 = time.time()
    env = OffScreenRenderEnv(
        bddl_file_name=str(bddl),
        camera_heights=args.res,
        camera_widths=args.res,
        camera_names=["agentview", "robot0_eye_in_hand"],
    )
    print(f"env construct: {time.time() - t0:.2f}s | instruction: "
          f"'{env.language_instruction}'")

    env.seed(0)
    obs = env.reset()

    print("\n=== OBSERVATION DICT (live sim) ===")
    for k in sorted(obs):
        v = obs[k]
        if isinstance(v, np.ndarray):
            print(f"  {k:32s} {str(v.shape):14s} {v.dtype}")

    missing = [
        k for k in ("robot0_eef_pos", "robot0_eef_quat", "robot0_gripper_qpos")
        if k not in obs
    ]
    if missing:
        sys.exit(f"observation dict lacks {missing}; cannot resolve the state layout")

    # --- state layout: scan (init state, reconstruction) jointly -----------------
    print(f"\n=== STATE LAYOUT SCAN ({n_inits} inits x candidates) ===")
    best = None
    for idx in range(n_inits):
        env.reset()
        obs = env.set_init_state(inits[idx])
        for name, vec in candidate_states(obs).items():
            if vec.shape != ds_state[0].shape:
                continue
            err = float(np.abs(vec - ds_state[0]).max())
            if best is None or err < best["max_abs_err"]:
                best = {
                    "init_index": idx,
                    "reconstruction": name,
                    "max_abs_err": err,
                    "sim_state": vec.tolist(),
                }
    if best is None:
        sys.exit("no candidate reconstruction matched the dataset's 8-D shape")

    print(f"  best init index      : {best['init_index']}")
    print(f"  best reconstruction  : {best['reconstruction']}")
    print(f"  max |sim - dataset|  : {best['max_abs_err']:.6f}")
    print(f"  sim   : {np.array2string(np.array(best['sim_state']), precision=4, floatmode='fixed')}")
    print(f"  data  : {np.array2string(ds_state[0], precision=4, floatmode='fixed')}")
    layout_ok = best["max_abs_err"] < 0.05
    print(f"  VERDICT: {'CONFIRMED' if layout_ok else 'NOT CONFIRMED'} "
          f"(threshold 0.05 on max abs error)")

    # --- image orientation, at the winning init ---------------------------------
    env.reset()
    obs = env.set_init_state(inits[best["init_index"]])
    print("\n=== IMAGE ORIENTATION (normalized cross-correlation vs dataset) ===")
    orient = {}
    for cam_key, ds_frame, label in (
        ("agentview_image", ds_image[0], "agentview"),
        ("robot0_eye_in_hand_image", ds_wrist[0], "wrist"),
    ):
        if cam_key not in obs:
            print(f"  {label}: '{cam_key}' absent from obs, skipped")
            continue
        sim_frame = np.asarray(obs[cam_key])
        scores = {n: ncc(f(sim_frame), ds_frame) for n, f in ORIENTATIONS.items()}
        winner = max(scores, key=scores.get)
        orient[label] = {"winner": winner, "scores": scores}
        print(f"  {label}:")
        for n, s in sorted(scores.items(), key=lambda kv: -kv[1]):
            print(f"      {n:22s} {s:+.4f}{'   <-- best' if n == winner else ''}")

    verdict = {
        "task": task,
        "bddl": str(bddl),
        "state_layout": best,
        "state_layout_confirmed": layout_ok,
        "image_orientation": orient,
        "mujoco_gl": os.environ.get("MUJOCO_GL"),
    }
    if args.json:
        Path(args.json).write_text(json.dumps(verdict, indent=2) + "\n")
        print(f"\nwrote {args.json}")

    print("\nSUMMARY")
    print(f"  state    : {best['reconstruction']} @ init {best['init_index']} "
          f"(max err {best['max_abs_err']:.4f})")
    for label, info in orient.items():
        print(f"  {label:9s}: apply '{info['winner']}' to the sim render")
    return 0 if layout_ok else 1


if __name__ == "__main__":
    sys.exit(main())
