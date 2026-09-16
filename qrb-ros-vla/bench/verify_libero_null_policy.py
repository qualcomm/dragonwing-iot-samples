#!/usr/bin/env python3
"""Negative control: confirm LIBERO's success predicate cannot be satisfied by doing nothing.

A closed-loop success rate is only evidence about the policy if a policy-shaped hole in the
same harness scores near zero. If `check_success()` were sticky-true at reset, or evaluated
against a stale state, or trivially satisfied by the arm flailing, then a high success rate
would be measuring the harness rather than Pi0.5 -- and it would look exactly the same from
the outside.

So this runs the identical episode loop with the policy replaced by:
  * zeros    -- hold still, gripper open. The degenerate case.
  * random   -- uniform in the controller's [-1, 1] action space. Tests whether merely
                disturbing the scene can trip the goal predicate.

Both must score 0. Anything else invalidates the closed-loop numbers in
docs/DESIGN.md section 10 and has to be understood before they are published.

Needs no NPU and no rendering (proprio and the success predicate are enough), so it is cheap
and safe to run alongside other work.

Environment:
    export MUJOCO_GL=osmesa
    export LIBERO_CONFIG_PATH=<dir containing config.yaml>
    export PYTHONPATH=<LIBERO repo root>

Usage:
    bench/verify_libero_null_policy.py --tasks 0 1 2 3 --episodes 3
"""

from __future__ import annotations

import argparse
import json
import sys
import types
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from torchfree_init import load_init_states  # noqa: E402


def install_torch_stub() -> None:
    if "torch" in sys.modules:
        return
    stub = types.ModuleType("torch")
    stub.load = lambda path, *a, **k: load_init_states(path)  # type: ignore[attr-defined]
    sys.modules["torch"] = stub


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--tasks", type=int, nargs="+", default=[0, 1, 2, 3])
    ap.add_argument("--episodes", type=int, default=3)
    ap.add_argument("--max-steps", type=int, default=520)
    ap.add_argument("--settle", type=int, default=10)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--json", default="")
    args = ap.parse_args()

    install_torch_stub()
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs.env_wrapper import ControlEnv

    bench_cls = benchmark.get_benchmark_dict()[args.suite]()
    rng = np.random.default_rng(args.seed)
    rows = []

    for task_index in args.tasks:
        task = bench_cls.get_task(task_index)
        bddl = str(Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file)
        inits = np.asarray(bench_cls.get_task_init_states(task_index))
        # No cameras: the success predicate reads simulator state, not pixels, so rendering
        # would only add 340 ms per observation for nothing.
        env = ControlEnv(
            bddl_file_name=bddl, camera_heights=84, camera_widths=84,
            use_camera_obs=False, has_offscreen_renderer=False, has_renderer=False,
        )
        env.seed(args.seed)

        for policy in ("zeros", "random"):
            for init_index in range(min(args.episodes, len(inits))):
                env.reset()
                env.set_init_state(inits[init_index])

                neutral = np.zeros(7)
                neutral[6] = -1.0
                for _ in range(args.settle):
                    env.step(neutral)

                # Assert the predicate is false BEFORE acting. A predicate already true at
                # the start would make every episode a trivial success.
                success_at_start = bool(env.check_success())

                success = False
                steps = 0
                while steps < args.max_steps and not success:
                    if policy == "zeros":
                        action = neutral
                    else:
                        action = rng.uniform(-1.0, 1.0, size=7)
                    _, _, done, _ = env.step(action)
                    steps += 1
                    if env.check_success():
                        success = True
                    elif done:
                        break

                rows.append({
                    "task_index": task_index, "policy": policy,
                    "init_index": init_index, "success": success,
                    "success_at_start": success_at_start, "steps": steps,
                })
                flag = "  <-- PROBLEM" if (success or success_at_start) else ""
                print(f"task {task_index} {policy:6s} init {init_index}: "
                      f"success={success} at_start={success_at_start} "
                      f"steps={steps}{flag}")
        env.close()

    n = len(rows)
    n_succ = sum(r["success"] for r in rows)
    n_start = sum(r["success_at_start"] for r in rows)
    print(f"\nnull-policy successes: {n_succ}/{n}   predicate-true-at-reset: {n_start}/{n}")
    for policy in ("zeros", "random"):
        sub = [r for r in rows if r["policy"] == policy]
        print(f"  {policy:6s}: {sum(r['success'] for r in sub)}/{len(sub)}")

    ok = n_succ == 0 and n_start == 0
    print("VERDICT: " + ("PASS - the predicate requires actually doing the task, so the "
                         "closed-loop success rate is evidence about the policy"
                         if ok else
                         "FAIL - a null policy or the reset state satisfies the goal "
                         "predicate; the closed-loop success rate is NOT trustworthy"))
    if args.json:
        Path(args.json).write_text(json.dumps(
            {"suite": args.suite, "max_steps": args.max_steps, "seed": args.seed,
             "null_successes": n_succ, "true_at_reset": n_start, "trials": n,
             "pass": ok, "rows": rows}, indent=2) + "\n")
        print(f"wrote {args.json}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
