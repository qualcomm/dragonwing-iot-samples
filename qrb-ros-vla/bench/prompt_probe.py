#!/usr/bin/env python3
"""Does the VLA generalize past LIBERO's instruction vocabulary? Measure it.

The deployed bundle is `lerobot/pi05_libero` (docs/DESIGN.md section 2): Pi0.5
fine-tuned on LIBERO, whose instructions are all pick / place / open / close /
turn-on. Asking it to "wave hi" is out of distribution, and the interesting
question for a live demo is not whether it *emits* actions -- it always does --
but whether the resulting motion is distinguishable per prompt and legible.

This runs a set of prompts through the real ROS + NPU path in one scene and
reports what the arm actually did, so the demo can be honest about which
commands it advertises.

Run (VLA node already up):
  source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
  source /home/ubuntu/libero/libero-env.sh
  $LIBERO_VENV_PYTHON bench/prompt_probe.py --chunks 4 \
      --prompts "pick up the black bowl" "wave hi" "say hello" "raise your arm"
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "osmesa")
os.environ.setdefault("LIBERO_CONFIG_PATH", "/home/ubuntu/libero/config")
_REPO = Path(__file__).resolve().parent.parent
for _p in (
    "/opt/ros/jazzy/lib/python3.12/site-packages",
    str(_REPO / "ros2_ws/install/qrb_ros_vla_msgs/lib/python3.12/site-packages"),
    "/home/ubuntu/libero/LIBERO",
    str(_REPO / "demo"),
    str(_REPO / "bench"),
):
    if Path(_p).exists() and _p not in sys.path:
        sys.path.insert(0, _p)

import numpy as np  # noqa: E402
import rclpy  # noqa: E402
from rclpy.executors import SingleThreadedExecutor  # noqa: E402
from std_msgs.msg import String  # noqa: E402

from libero_closed_loop import (  # noqa: E402
    ClosedLoopRunner, build_state, install_torch_stub, rot180,
)


def motion_signature(eef: np.ndarray, finger_gap: np.ndarray, actions: np.ndarray) -> dict[str, float]:
    """Summarize a trajectory in terms a presenter can defend on stage.

    ``finger_gap`` must be the *separation* of the two Panda fingers, not the sum
    of their qpos: the joints move symmetrically, so the sum cancels and a fully
    closing gripper looks motionless.
    """
    steps = np.diff(eef, axis=0)
    path = float(np.linalg.norm(steps, axis=1).sum())
    ptp = eef.max(axis=0) - eef.min(axis=0)
    # Lateral reversals: how many times the y-velocity changed sign by more than
    # 1 mm/step. A wave is oscillatory; a reach is not.
    vy = steps[:, 1]
    significant = np.sign(vy[np.abs(vy) > 1e-3])
    reversals = int(np.sum(significant[1:] != significant[:-1])) if significant.size > 1 else 0
    return {
        "path_length_m": round(path, 4),
        "net_displacement_m": round(float(np.linalg.norm(eef[-1] - eef[0])), 4),
        "ptp_x_m": round(float(ptp[0]), 4),
        "ptp_y_m": round(float(ptp[1]), 4),
        "ptp_z_m": round(float(ptp[2]), 4),
        "lateral_reversals": reversals,
        "finger_gap_range_m": round(float(finger_gap.max() - finger_gap.min()), 4),
        "mean_abs_action_xyz": [round(float(v), 4) for v in np.abs(actions[:, :3]).mean(axis=0)],
        "mean_action_gripper": round(float(actions[:, 6].mean()), 4),
    }


def finger_gap(obs: dict) -> float:
    """Distance between the two gripper fingers."""
    qpos = np.asarray(obs["robot0_gripper_qpos"], dtype=np.float64)
    return float(abs(qpos[0] - qpos[1])) if qpos.size >= 2 else float(qpos.sum())


def run_prompt(runner: ClosedLoopRunner, env, inits, prompt: str,
               args: argparse.Namespace) -> dict:
    env.reset()
    if inits is not None:
        env.set_init_state(inits[0])
    neutral = np.zeros(7)
    neutral[6] = -1.0
    obs = None
    for _ in range(args.settle):
        obs, _, _, _ = env.step(neutral)

    task_msg = String()
    task_msg.data = prompt
    runner.task_pub.publish(task_msg)
    time.sleep(0.2)

    sim = env.env.sim
    eef, gaps, applied, frames_out = [], [], [], []
    prev_state = None
    for chunk_idx in range(args.chunks):
        frames = [
            rot180(np.asarray(sim.render(width=args.res, height=args.res, camera_name=cam),
                              dtype=np.uint8))
            for cam in args.cameras
        ]
        if chunk_idx == 0 or args.save_frames:
            frames_out.append(frames[0])
        state8 = build_state(obs, prev_state)
        prev_state = state8
        runner.task_pub.publish(task_msg)  # prompt is rebuilt into tokens every step
        chunk = runner.request_chunk(state8, frames, args.timeout)
        if chunk is None:
            return {"prompt": prompt, "error": "timeout waiting for action chunk"}
        for i in range(min(args.horizon, len(chunk))):
            action = np.asarray(chunk[i], dtype=np.float64)
            obs, _, _, _ = env.step(action)
            eef.append(np.asarray(obs["robot0_eef_pos"], dtype=np.float64))
            gaps.append(finger_gap(obs))
            applied.append(action)
    frames_out.append(
        rot180(np.asarray(sim.render(width=args.res, height=args.res, camera_name=args.cameras[0]),
                          dtype=np.uint8))
    )
    record = {
        "prompt": prompt,
        "chunks": args.chunks,
        "steps": len(eef),
        **motion_signature(np.asarray(eef), np.asarray(gaps), np.asarray(applied)),
        "backend": (runner.latest_stats or {}).get("backend", "unknown"),
        "mean_chunk_ms": round(float(np.mean(runner.latencies[-args.chunks:])), 1)
        if runner.latencies else None,
    }
    if args.out_dir:
        from PIL import Image as PILImage
        strip = np.concatenate(frames_out, axis=1)
        slug = "".join(c if c.isalnum() else "_" for c in prompt)[:48]
        path = Path(args.out_dir) / f"probe_{slug}.png"
        PILImage.fromarray(strip).save(path)
        record["filmstrip"] = str(path)
    return record


def compare_prompts(runner: ClosedLoopRunner, env, inits, prompts: list[str],
                    args: argparse.Namespace) -> dict:
    """Language sensitivity from one frozen observation.

    Stepping the sim confounds "the prompt changed the plan" with "the arm ended
    up somewhere else". Here the state and both camera frames are captured once
    and reused for every request, so the only varying input is the prompt.

    The baseline that makes the comparison meaningful is repeat noise: the same
    prompt on the same observation, twice. A prompt effect smaller than repeat
    noise is not an effect.
    """
    env.reset()
    if inits is not None:
        env.set_init_state(inits[0])
    neutral = np.zeros(7)
    neutral[6] = -1.0
    obs = None
    for _ in range(args.settle):
        obs, _, _, _ = env.step(neutral)
    sim = env.env.sim
    frames = [
        rot180(np.asarray(sim.render(width=args.res, height=args.res, camera_name=cam),
                          dtype=np.uint8))
        for cam in args.cameras
    ]
    state8 = build_state(obs, None)

    task_msg = String()
    chunks: dict[str, np.ndarray] = {}
    order = [(p, 0) for p in prompts] + [(p, 1) for p in prompts[:2]]
    for prompt, repeat in order:
        task_msg.data = prompt
        runner.task_pub.publish(task_msg)
        time.sleep(0.2)
        chunk = runner.request_chunk(state8, frames, args.timeout)
        if chunk is None:
            return {"error": f"timeout on {prompt!r}"}
        chunks[f"{prompt}#{repeat}"] = np.asarray(chunk, dtype=np.float64)

    std = np.asarray(runner.action_std, dtype=np.float64)[:7]

    def norm_diff(a: np.ndarray, b: np.ndarray) -> float:
        n = min(len(a), len(b))
        return round(float(np.mean(np.abs(a[:n] - b[:n]) / (std + 1e-8))), 4)

    repeat_noise = [
        {"prompt": p, "normalized_mean_abs_diff": norm_diff(chunks[f"{p}#0"], chunks[f"{p}#1"])}
        for p in prompts[:2]
    ]
    pairs = []
    for i, a in enumerate(prompts):
        for b in prompts[i + 1:]:
            pairs.append({"a": a, "b": b,
                          "normalized_mean_abs_diff": norm_diff(chunks[f"{a}#0"], chunks[f"{b}#0"])})
    return {
        "mode": "same_state_compare",
        "note": "diffs are mean |a-b| per action dim, in units of the dataset action std",
        "repeat_noise": repeat_noise,
        "prompt_pairs": pairs,
        "first_action_per_prompt": {
            p: [round(float(v), 4) for v in chunks[f"{p}#0"][0]] for p in prompts
        },
        "backend": (runner.latest_stats or {}).get("backend", "unknown"),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--prompts", nargs="+", required=True)
    ap.add_argument("--scene", default=str(_REPO / "demo/scenes/sandbox_table.bddl"))
    ap.add_argument("--suite", default=None, help="load a LIBERO suite task instead of --scene")
    ap.add_argument("--task-index", type=int, default=0)
    ap.add_argument("--node", default="/qrb_ros_vla")
    ap.add_argument("--camera-topics", nargs="+",
                    default=["/vla/camera0/image_raw", "/vla/camera1/image_raw"])
    ap.add_argument("--cameras", nargs="+", default=["agentview", "robot0_eye_in_hand"])
    ap.add_argument("--res", type=int, default=256)
    ap.add_argument("--chunks", type=int, default=4)
    ap.add_argument("--horizon", type=int, default=6)
    ap.add_argument("--settle", type=int, default=10)
    ap.add_argument("--state-gap", type=float, default=0.05)
    ap.add_argument("--timeout", type=float, default=45.0)
    ap.add_argument("--stats-npz", default=str(_REPO / "artifacts/libero_ep0.npz"))
    ap.add_argument("--out-dir", default=str(_REPO / "bench/prompt-probe"))
    ap.add_argument("--save-frames", action="store_true")
    ap.add_argument("--publish-viz", action="store_true")
    ap.add_argument("--compare", action="store_true",
                    help="same-state language-sensitivity test instead of rollouts")
    ap.add_argument("--json", default=str(_REPO / "bench/prompt-probe/summary.json"))
    args = ap.parse_args()

    z = np.load(args.stats_npz, allow_pickle=False)
    stats = {k: np.asarray(z[k], dtype=np.float64)
             for k in ("state_mean", "state_std", "action_mean", "action_std")}
    if args.out_dir:
        Path(args.out_dir).mkdir(parents=True, exist_ok=True)

    install_torch_stub()
    from libero.libero.envs.env_wrapper import ControlEnv

    inits = None
    bddl = args.scene
    if args.suite:
        from libero.libero import benchmark, get_libero_path
        bm = benchmark.get_benchmark_dict()[args.suite]()
        task = bm.get_task(args.task_index)
        bddl = str(Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file)
        inits = np.asarray(bm.get_task_init_states(args.task_index))

    rclpy.init()
    runner = ClosedLoopRunner(args, stats)
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    import threading
    threading.Thread(target=executor.spin, daemon=True).start()

    env = ControlEnv(bddl_file_name=bddl, camera_heights=args.res, camera_widths=args.res,
                     camera_names=list(args.cameras), use_camera_obs=False,
                     has_offscreen_renderer=True, has_renderer=False)
    env.seed(0)
    records = []
    try:
        if args.compare:
            records.append(compare_prompts(runner, env, inits, list(args.prompts), args))
            print(json.dumps(records[-1], indent=2), flush=True)
        else:
            for prompt in args.prompts:
                record = run_prompt(runner, env, inits, prompt, args)
                records.append(record)
                print(json.dumps(record, indent=2), flush=True)
    finally:
        env.close()
        executor.shutdown()
        runner.destroy_node()
        rclpy.try_shutdown()

    if args.json:
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps({"scene": bddl, "records": records}, indent=2))
        print(f"wrote {args.json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
