#!/usr/bin/env python3
"""Does a LIBERO task physically fit inside the VUPN2355's reach envelope?

THE FIRST GO/NO-GO, AND IT IS FREE. A task whose objects sit outside the arm's reach is
infeasible at any policy quality: no amount of fine-tuning, calibration or IK makes a 300 mm
arm touch something 800 mm away. So this runs before any of the closed-loop ablations and
before anything is wired. No NPU, no rendering, no bundle mapped -- just env resets, the same
cheap pattern bench/verify_libero_null_policy.py uses.

WHY THIS SUPERSEDES THE SCREEN IN analyze_arm_envelope.py. That one measures radial distance
from the world z-axis, because it works from a recorded state trajectory and has no way to
know where the robot stands. LIBERO's Panda base is actually at (-0.75, 0, 0.912), so the
world-axis figure understates the true reach requirement by roughly the base offset. This
script reads the base pose from the live sim and is the authoritative number.

WHAT COUNTS AS "REQUIRED REACH". Every position the gripper must visit to satisfy the goal:
the initial end-effector pose, plus every object the task registers -- manipulated objects and
fixtures alike, since a fixture like a caddy or a plate is a place-target the gripper has to
reach into. Taking all registered objects rather than parsing the BDDL goal predicates is the
conservative direction only for *distractors*; LIBERO's obj_body_id is task-scoped, so in
practice the two agree and this needs no BDDL parsing.

Reported per task as the linear scale factor by which the scene exceeds the arm. That is the
useful form: a factor of 1.2 might be recoverable by remounting the arm or shrinking props, a
factor of 2.7 means the embodiment is simply wrong for the task.

Environment (all three mandatory):
    source ~/libero/libero-env.sh

Usage:
    bench/verify_arm_workspace.py
    bench/verify_arm_workspace.py --suite libero_10 --tasks 0 1 2 --episodes 5
"""

from __future__ import annotations

import argparse
import json
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from arm_embodiment import ArmSpec  # noqa: E402
from torchfree_init import load_init_states  # noqa: E402

_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


def install_torch_stub() -> None:
    """Same stub as the other harnesses: LIBERO torch.loads its .pruned_init files, and
    installing real torch drags the NVIDIA CUDA sbsa stack onto a board with no NVIDIA GPU."""
    if "torch" in sys.modules:
        return
    stub = types.ModuleType("torch")
    stub.load = lambda path, *a, **k: load_init_states(path)  # type: ignore[attr-defined]
    sys.modules["torch"] = stub


def probe_task(env, inits: np.ndarray, init_indices: list[int],
               settle: int) -> list[dict]:
    """Reach requirement for each initial state of one task."""
    e = env.env
    root = e.robots[0].robot_model.root_body
    rows = []

    for idx in init_indices:
        env.reset()
        obs = env.set_init_state(inits[idx])
        neutral = np.zeros(7)
        neutral[6] = -1.0
        for _ in range(settle):
            obs, _, _, _ = env.step(neutral)

        base = np.asarray(e.sim.data.get_body_xpos(root), dtype=np.float64)
        points = {"eef_start": np.asarray(obs["robot0_eef_pos"], dtype=np.float64)}
        for name, bid in e.obj_body_id.items():
            points[name] = np.asarray(e.sim.data.body_xpos[bid], dtype=np.float64)

        # Horizontal radius is the figure that matters for a desktop arm: the shoulder is
        # above the table, so vertical offset is taken up by the pitch joints, whereas
        # horizontal distance is what the link lengths must actually span.
        detail = {}
        for name, p in points.items():
            rel = p - base
            detail[name] = {
                "radial_mm": round(float(np.linalg.norm(rel[:2])) * 1000.0, 1),
                "dist3d_mm": round(float(np.linalg.norm(rel)) * 1000.0, 1),
                "dz_mm": round(float(rel[2]) * 1000.0, 1),
            }
        radial = [d["radial_mm"] for d in detail.values()]
        rows.append({
            "init_index": int(idx),
            "base_xyz": [round(float(v), 4) for v in base],
            "radial_min_mm": min(radial),
            "radial_max_mm": max(radial),
            "dist3d_max_mm": max(d["dist3d_mm"] for d in detail.values()),
            "points": detail,
        })
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--tasks", type=int, nargs="+",
                    default=list(range(10)))
    ap.add_argument("--episodes", type=int, default=5,
                    help="init states per task, starting at --init-start")
    ap.add_argument("--init-start", type=int, default=0)
    ap.add_argument("--settle", type=int, default=10)
    ap.add_argument("--spec", default=_repo_path("bench/arm-spec-measured.json"))
    ap.add_argument("--json", default=_repo_path("bench/arm-workspace-screen.json"))
    args = ap.parse_args()

    spec = ArmSpec.load(args.spec)

    install_torch_stub()
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs.env_wrapper import ControlEnv

    bm = benchmark.get_benchmark_dict()[args.suite]()
    bddl_root = Path(get_libero_path("bddl_files"))

    tasks = []
    for ti in args.tasks:
        task = bm.get_task(ti)
        bddl = str(bddl_root / task.problem_folder / task.bddl_file)
        inits = np.asarray(bm.get_task_init_states(ti))
        idxs = [i for i in range(args.init_start, args.init_start + args.episodes)
                if i < inits.shape[0]]

        # No cameras, no renderer at all: positions come from the physics state.
        env = ControlEnv(
            bddl_file_name=bddl, camera_heights=64, camera_widths=64,
            camera_names=["agentview"], use_camera_obs=False,
            has_offscreen_renderer=False, has_renderer=False,
        )
        env.seed(0)
        try:
            rows = probe_task(env, inits, idxs, args.settle)
        finally:
            env.close()

        radial_max = max(r["radial_max_mm"] for r in rows)
        radial_min = min(r["radial_min_mm"] for r in rows)
        dist_max = max(r["dist3d_max_mm"] for r in rows)
        scale = radial_max / spec.reach_max_mm
        tasks.append({
            "task_index": ti,
            "language": task.language,
            "inits": rows,
            "radial_max_mm": radial_max,
            "radial_min_mm": radial_min,
            "dist3d_max_mm": dist_max,
            "arm_reach_max_mm": spec.reach_max_mm,
            "scale_factor_needed": round(scale, 2),
            "fits": bool(radial_max <= spec.reach_max_mm),
        })
        print(f"  task {ti}: radial {radial_min:.0f}..{radial_max:.0f} mm  "
              f"needs {scale:.2f}x the arm's reach  "
              f"{'FITS' if radial_max <= spec.reach_max_mm else 'DOES NOT FIT'}"
              f"  -- {task.language}")

    n_fit = sum(1 for t in tasks if t["fits"])
    worst = max(t["scale_factor_needed"] for t in tasks)
    best = min(t["scale_factor_needed"] for t in tasks)

    out = {
        "suite": args.suite,
        "arm_spec_status": spec.status,
        "arm_spec_digest": spec.digest,
        "arm_reach_max_mm": spec.reach_max_mm,
        "arm_reach_min_mm": spec.reach_min_mm,
        "episodes_per_task": args.episodes,
        "init_start": args.init_start,
        "settle": args.settle,
        "tasks_fitting": n_fit,
        "tasks_total": len(tasks),
        "scale_factor_range": [best, worst],
        "verdict": (
            f"{n_fit}/{len(tasks)} tasks fit within a {spec.reach_max_mm:.0f} mm reach. "
            f"Scene exceeds the arm by {best:.2f}x to {worst:.2f}x."
        ),
        "tasks": tasks,
    }
    Path(args.json).write_text(json.dumps(out, indent=2) + "\n")

    print()
    print(f"arm spec: {spec.status.upper()} (digest {spec.digest}), "
          f"reach {spec.reach_max_mm:.0f} mm")
    print(out["verdict"])
    print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
