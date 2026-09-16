#!/usr/bin/env python3
"""Rung 3: drive the LIBERO simulator closed-loop from Pi0.5 on the IQ-9075's NPUs.

Everything runs on the board. There is no laptop, no bridge and no cross-host transport:
MuJoCo renders through Mesa's llvmpipe software rasterizer (OpenGL 4.5 *compatibility*,
which is exactly why it works where Gazebo's Ogre2 demanded 3.3 *core* and failed --
docs/DESIGN.md section 9), and the sim talks to the C++ VLA node over local DDS.

This is what docs/DESIGN.md section 11 limitation #1 was waiting for. The existing
demo/libero_replay.py scores predicted actions against a human demonstration open-loop;
the policy never sees the consequences of its own actions. Here it does, so the output is
a real task success rate.

    ┌─ LIBERO / MuJoCo (this process) ──────────────────────────┐
    │  sim.render() x2  ──rot180──►  /vla/camera{0,1}/image_raw │──► qrb_ros_vla
    │  8-D proprio ──MEAN_STD──►     ~/state                    │      (NPU, ~1.1 s)
    │  task string  ──────────►      ~/task                     │          │
    │                                                            │          ▼
    │  env.step() x H  ◄──un-normalize──  ~/action_chunk  ◄──────┘   50x7 actions
    │  check_success()                                                        │
    └────────────────── repeat until success or max_steps ◄──────────────────┘

WHY RENDERING HAPPENS ONLY ON REPLAN. A 2-camera 256x256 observation costs 340 ms on this
board's software rasterizer while a physics step costs 31 ms. Rendering every step turns a
220-step episode into 84.8 s; rendering once per replan turns it into 8.7-15.3 s. That is
not a shortcut -- it is precisely what action chunking buys, since the policy only needs an
observation when it plans. On this board the *simulator* is the expensive half, not the 3B
VLA, which inverts the usual expectation.

CONVENTIONS THAT SILENTLY DESTROY THE RESULT IF WRONG. Each was resolved empirically
against the recorded dataset rather than taken from documentation; bench/
verify_libero_contract.py and bench/verify_libero_action_units.py are the authority, and
this module imports its helpers from there so there is exactly one definition:
  * images need rot180 (NCC +0.820 agentview vs -0.045 identity), applied to BOTH cameras
  * state is eef_pos(3) + rotvec(3) + gripper_qpos(2), MEAN_STD-normalized by us
  * the axis-angle branch must be chosen for CONTINUITY -- LIBERO's home pose sits at an
    angle of ~pi, so a fixed sign test flips by 2*pi mid-rollout
  * gripper -1 = open, +1 = closed
  * exactly ONE env.step() per predicted action (two is 25x worse)
  * ~10 settle steps after set_init_state before the first observation

Environment (all three mandatory):
    export MUJOCO_GL=osmesa
    export LIBERO_CONFIG_PATH=<dir containing config.yaml>
    export PYTHONPATH=<LIBERO repo root>

With the VLA node already running (it must own the NPUs exclusively):
    python3 demo/libero_closed_loop.py --suite libero_10 --task-index 0 --episodes 5 \
        --jsonl bench/libero-closed-loop.jsonl
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
import threading
import time
import types
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from qrb_ros_vla_msgs.msg import ActionChunk, InferenceStats

# bench/verify_libero_contract.py is where these conventions were empirically established
# against the recorded episode, so it is the single source of truth for them rather than a
# re-implementation here that could drift.
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "bench"))
from arm_embodiment import MODES as ARM_MODES  # noqa: E402
from arm_embodiment import ArmModel, ArmSpec  # noqa: E402
from torchfree_init import load_init_states  # noqa: E402
from verify_libero_contract import quat_to_rotvec  # noqa: E402

# Artifact defaults are anchored to the repository, not the caller's working directory: these
# scripts are invoked by workshop/scripts/fastforward.sh and by hand from various places, and
# a bare relative default fails with FileNotFoundError depending on where you happen to be.
# An explicitly passed path is still interpreted relative to the cwd, which is what a user
# typing one expects.
_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)



def install_torch_stub() -> None:
    """Let LIBERO's benchmark registry load init states without torch.

    LIBERO calls torch.load on its .pruned_init files. Installing torch for that one call
    would pull 26 packages including the entire NVIDIA CUDA sbsa stack onto a board with no
    NVIDIA GPU, so a stub backed by the zip+pickle reader stands in.
    """
    if "torch" in sys.modules:
        return
    stub = types.ModuleType("torch")
    stub.load = lambda path, *a, **k: load_init_states(path)  # type: ignore[attr-defined]
    sys.modules["torch"] = stub


def rot180(img: np.ndarray) -> np.ndarray:
    """The orientation correction, as one named thing so it cannot be applied twice."""
    return img[::-1, ::-1]


class FrameRecorder:
    """Write raw frames plus per-step metadata for an offline HUD video pass.

    Deliberately dumb: it saves pixels and numbers and composes nothing. The HUD is built
    afterwards by demo/make_rollout_video.py, so a rollout recorded here can be re-styled
    without re-running the policy, and so the compositing cost never lands inside a
    measured episode.

    Recording is still not free -- extra frames cost 340 ms each for the pair on this
    board's software rasterizer -- so a run with --save-frames is NOT a timing measurement.
    The harness prints that warning rather than leaving it implicit.
    """

    def __init__(self, root: Path, init_index: int, task: str,
                 args: argparse.Namespace):
        self.dir = root / f"task{args.task_index}_init{init_index}"
        (self.dir / "frames").mkdir(parents=True, exist_ok=True)
        self.meta_path = self.dir / "steps.jsonl"
        self.meta = self.meta_path.open("w")
        self.n = 0
        self.res = args.res
        self.cameras = list(args.cameras)
        (self.dir / "episode.json").write_text(json.dumps({
            "task": task, "task_index": args.task_index, "init_index": init_index,
            "replan_horizon": args.replan_horizon, "max_steps": args.max_steps,
            "res": args.res, "cameras": self.cameras,
            "video_stride": args.video_stride,
        }, indent=2) + "\n")

    def _write(self, images: list[np.ndarray], row: dict) -> None:
        from PIL import Image as PILImage

        for cam, img in zip(self.cameras, images):
            PILImage.fromarray(img).save(self.dir / "frames" / f"{self.n:05d}_{cam}.png")
        row["frame"] = self.n
        self.meta.write(json.dumps(row) + "\n")
        self.meta.flush()
        self.n += 1

    def on_replan(self, chunk_idx, steps, frames, actions, horizon, stats, success):
        self._write(frames, {
            "kind": "replan", "chunk": chunk_idx, "sim_step": steps,
            "action_in_chunk": 0, "horizon": int(horizon),
            "chunk_actions": np.asarray(actions[:, :7], dtype=float).round(4).tolist(),
            "stats": stats, "success": bool(success),
        })

    def on_step(self, chunk_idx, steps, i, sim, actions, horizon, stats, success):
        images = [
            rot180(np.asarray(sim.render(width=self.res, height=self.res,
                                         camera_name=cam), dtype=np.uint8))
            for cam in self.cameras
        ]
        self._write(images, {
            "kind": "step", "chunk": chunk_idx, "sim_step": steps,
            "action_in_chunk": int(i) + 1, "horizon": int(horizon),
            "stats": stats, "success": bool(success),
        })

    def close(self) -> None:
        self.meta.close()


class ClosedLoopRunner(Node):
    def __init__(self, args: argparse.Namespace, stats: dict):
        super().__init__("vla_libero_closed_loop")
        self.args = args
        self.state_mean = stats["state_mean"]
        self.state_std = stats["state_std"]
        self.action_mean = stats["action_mean"]
        self.action_std = stats["action_std"]

        self.cam_pubs = [self.create_publisher(Image, t, 10) for t in args.camera_topics]
        self.state_pub = self.create_publisher(Float32MultiArray, f"{args.node}/state", 10)
        self.task_pub = self.create_publisher(String, f"{args.node}/task", 10)
        self.create_subscription(
            ActionChunk, f"{args.node}/action_chunk", self.on_chunk, 10
        )
        self.create_subscription(
            InferenceStats, f"{args.node}/stats", self.on_stats, 10
        )
        # Rendered frames republished so a rollout can be watched or recorded to a rosbag
        # without re-running the policy. Costs nothing extra: the frames already exist.
        self.viz_pubs = (
            [self.create_publisher(Image, f"{args.node}/sim/{n}", 2)
             for n in ("agentview", "wrist")]
            if args.publish_viz else []
        )

        self.seq = 0
        self.last_stamp_ns = 0
        self.pending: tuple[int, int] | None = None
        self.chunk: np.ndarray | None = None
        self.chunk_ready = threading.Event()
        self.lock = threading.Lock()
        self.latest_stats: dict | None = None
        self.backends: set[str] = set()
        self.latencies: list[float] = []
        self.stale_chunks = 0

    # --- ROS plumbing ---------------------------------------------------------
    def on_chunk(self, msg: ActionChunk) -> None:
        got = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.lock:
            if self.pending is None or got != self.pending:
                # A chunk computed from an observation we are no longer waiting on -- e.g.
                # one that arrived after we gave up. Scoring it would silently apply a
                # stale plan to a new state, so it is counted and dropped.
                self.stale_chunks += 1
                return
            dof = msg.action_dof or 7
            actions = np.asarray(msg.actions, dtype=np.float32).reshape(
                msg.chunk_size, msg.action_dim
            )[:, :dof]
            if not msg.unnormalized:
                actions = actions * self.action_std[:dof] + self.action_mean[:dof]
            self.chunk = actions
            self.pending = None
        self.chunk_ready.set()

    def on_stats(self, msg: InferenceStats) -> None:
        self.latencies.append(msg.total_ms)
        self.backends.add(msg.backend)
        self.latest_stats = {
            "total_ms": msg.total_ms,
            "vision_encoder_ms": msg.vision_encoder_ms,
            "token_emb_ms": msg.token_emb_ms,
            "backbone_ms": msg.backbone_ms,
            "action_expert_ms": msg.action_expert_ms,
            "backend": msg.backend,
        }

    def publish_state(self, state8: np.ndarray) -> None:
        normalized = (state8 - self.state_mean) / (self.state_std + 1e-8)
        msg = Float32MultiArray()
        msg.data = [float(v) for v in normalized]
        self.state_pub.publish(msg)

    def _image_msg(self, frame: np.ndarray, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[0], frame.shape[1]
        msg.encoding = "rgb8"  # the only zero-copy encoding the node accepts
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def request_chunk(self, state8: np.ndarray, frames: list[np.ndarray],
                      timeout: float) -> np.ndarray | None:
        """One observation in, one action chunk out. Strictly one in flight.

        The node requires a fresh frame per camera per chunk and overwrites mid-inference
        arrivals latest-wins, so free-running would burn observations and could pair
        camera 0 from one step with camera 1 from another. Blocking here is what makes the
        loop correct, not merely polite.
        """
        self.chunk_ready.clear()
        self.seq += 1

        # State first, always. The node's Ready() gate ignores state entirely, and with no
        # state the tokenizer emits a structurally different Pi0-style prompt. Publishing
        # state first and pausing means the prompt is rebuilt with this step's state before
        # any image can trigger inference.
        self.publish_state(state8)
        time.sleep(self.args.state_gap)

        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            # The stamp is the correlation key, so a duplicate would make two observations
            # indistinguishable. Nudge rather than fail; the clock only needs to advance.
            stamp.nanosec = (stamp.nanosec + 1) % 1_000_000_000
            stamp_ns += 1
        self.last_stamp_ns = stamp_ns

        with self.lock:
            self.chunk = None
            self.pending = (stamp.sec, stamp.nanosec)

        for pub, frame in zip(self.cam_pubs, frames):
            pub.publish(self._image_msg(frame, stamp, f"obs{self.seq}"))
        for pub, frame in zip(self.viz_pubs, frames):
            pub.publish(self._image_msg(frame, stamp, f"obs{self.seq}"))

        if not self.chunk_ready.wait(timeout):
            with self.lock:
                self.pending = None  # so a late chunk is dropped, not misapplied
            return None
        with self.lock:
            return self.chunk


def build_state(obs: dict, previous: np.ndarray | None) -> np.ndarray:
    """The dataset's 8-D layout, rebuilt from robosuite keys (bench/ rung-1a verdict)."""
    prev_rot = None if previous is None else previous[3:6]
    return np.concatenate([
        np.asarray(obs["robot0_eef_pos"], dtype=np.float64),
        quat_to_rotvec(obs["robot0_eef_quat"], prev_rot),
        np.asarray(obs["robot0_gripper_qpos"], dtype=np.float64),
    ])


def run_episode(runner: ClosedLoopRunner, env, sim_box: dict, inits: np.ndarray,
                init_index: int, task_text: str, args: argparse.Namespace,
                arm: ArmModel | None = None) -> dict:
    """One closed-loop episode. Returns the audit record.

    `arm` is None on the published path (docs/DESIGN.md section 10). When supplied it degrades
    the loop to what the VUPN2355 hobby arm could actually execute -- see bench/arm_embodiment.py
    for why each stage exists and bench/arm-ablation.json for what each one costs.
    """
    env.reset()
    obs = env.set_init_state(inits[init_index])
    # hard_reset rebuilds MjSim, so a handle cached before reset is stale. Re-fetch.
    sim_box["sim"] = env.env.sim

    neutral = np.zeros(7)
    neutral[6] = -1.0  # gripper open
    for _ in range(args.settle):
        obs, _, _, _ = env.step(neutral)

    if arm is not None:
        # Seed on the settled pose, not the pre-settle one: a real arm is homed and
        # calibrated before a run, so it starts knowing where it is and only loses track
        # afterwards. Seeding earlier would fold a calibration error into the measurement.
        arm.reset(build_state(obs, None))

    prev_state = None
    success = False
    steps = 0
    chunks = 0
    timeouts = 0
    rec = None
    if args.save_frames:
        rec = FrameRecorder(Path(args.save_frames), init_index, task_text, args)
    t0 = time.time()

    while steps < args.max_steps and not success:
        sim = sim_box["sim"]
        frames = [
            rot180(np.asarray(sim.render(width=args.res, height=args.res,
                                         camera_name=cam), dtype=np.uint8))
            for cam in args.cameras
        ]
        state8 = build_state(obs, prev_state)
        prev_state = state8
        # What the policy is TOLD, which under an arm model is not what is true: either the
        # dataset mean (told nothing) or the dead-reckoned commanded pose (all a PWM servo
        # arm can supply). prev_state still tracks ground truth so the rotvec continuity
        # branch keeps working off a real trajectory.
        observed8 = state8 if arm is None else arm.state_estimate(state8)

        timeout = args.timeout_first if chunks == 0 else args.timeout
        actions = runner.request_chunk(observed8, frames, timeout)
        if actions is None:
            timeouts += 1
            runner.get_logger().warn(
                f"chunk timeout after {timeout:.0f}s (retry {timeouts}/{args.retries})"
            )
            if timeouts > args.retries:
                break
            continue
        timeouts = 0
        chunks += 1

        horizon = min(args.replan_horizon, len(actions), args.max_steps - steps)
        if rec is not None:
            # The observation the policy actually planned from, saved at the replan
            # boundary where it is free -- it has already been rendered and rotated.
            rec.on_replan(chunks, steps, frames, actions, horizon,
                          runner.latest_stats, success)
        for i in range(horizon):
            act = np.asarray(actions[i], dtype=np.float64)
            if arm is not None:
                act = arm.apply(act)
            obs, _, done, _ = env.step(act)
            steps += 1
            # Checked every step and latched: LIBERO success is a state predicate, and a
            # rollout that passes through the goal then drifts out still completed it.
            if env.check_success():
                success = True
            # Record on the stride, but ALWAYS record the step where success flips --
            # otherwise the one frame worth putting on a poster is the one most likely to
            # be skipped, since the loop breaks immediately afterwards.
            if rec is not None and (steps % args.video_stride == 0 or success):
                rec.on_step(chunks, steps, i, sim_box["sim"], actions, horizon,
                            runner.latest_stats, success)
            if success or done:
                break

    if rec is not None:
        rec.close()

    elapsed = time.time() - t0
    lat = np.asarray(runner.latencies[-chunks:]) if chunks else np.asarray([])
    record = {
        "task": task_text,
        "init_index": int(init_index),
        "success": bool(success),
        "steps": int(steps),
        "chunks": int(chunks),
        "timeouts": int(timeouts),
        "wall_s": round(elapsed, 2),
        "replan_horizon": int(args.replan_horizon),
        "max_steps": int(args.max_steps),
        "settle": int(args.settle),
        "backends": sorted(runner.backends),
        "latency_ms_mean": round(float(lat.mean()), 2) if lat.size else None,
        "latency_ms_p95": round(float(np.percentile(lat, 95)), 2) if lat.size else None,
        "stale_chunks": int(runner.stale_chunks),
    }
    if arm is not None:
        record.update(arm.audit())
    return record


def wilson_interval(k: int, n: int, z: float = 1.96) -> tuple[float, float]:
    """95% Wilson score interval for k successes in n trials.

    Deliberately not the textbook Wald interval (p +/- z*sqrt(p(1-p)/n)): at k=0 or k=n
    Wald collapses to a zero-width interval, so a 0/2 run prints as "0.0% +/- 0.0%" --
    false certainty, and precisely the opposite of what quoting an interval is for. Wilson
    stays sensible at the boundaries, which is where a small evaluation sweep usually is.
    """
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1.0 + z * z / n
    centre = (p + z * z / (2 * n)) / d
    half = z * float(np.sqrt(p * (1 - p) / n + z * z / (4 * n * n))) / d
    return (max(0.0, centre - half), min(1.0, centre + half))


def provenance(args: argparse.Namespace) -> dict:
    """Everything needed to reproduce the number, recorded next to it."""
    import mujoco
    import robosuite

    def git(*a):
        try:
            return subprocess.run(["git", *a], capture_output=True, text=True,
                                  timeout=10).stdout.strip() or None
        except Exception:
            return None

    return {
        "suite": args.suite,
        "task_index": args.task_index,
        "mujoco": mujoco.__version__,
        "robosuite": robosuite.__version__,
        "numpy": np.__version__,
        "repo_commit": git("rev-parse", "--short", "HEAD"),
        "res": args.res,
        "cameras": list(args.cameras),
        "image_orientation": "rot180 (both cameras)",
        "state_layout": "eef_pos(3)+rotvec(3)+gripper_qpos(2), MEAN_STD normalized",
        "steps_per_action": 1,
        "ablate": args.ablate,
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--suite", default="libero_10")
    ap.add_argument("--task-index", type=int, default=0)
    ap.add_argument("--episodes", type=int, default=5,
                    help="init states to run, starting at --init-start")
    ap.add_argument("--init-start", type=int, default=0)
    ap.add_argument("--replan-horizon", type=int, default=10,
                    help="actions executed per chunk before replanning; dominates both "
                         "success rate and wall clock")
    ap.add_argument("--max-steps", type=int, default=520)
    ap.add_argument("--settle", type=int, default=10)
    ap.add_argument("--res", type=int, default=256)
    ap.add_argument("--cameras", nargs="+",
                    default=["agentview", "robot0_eye_in_hand"])
    ap.add_argument("--stats-npz", default=_repo_path("artifacts/libero_ep0.npz"),
                    help="source of the MEAN_STD state/action statistics")
    ap.add_argument("--node", default="/qrb_ros_vla")
    ap.add_argument("--camera-topics", nargs="+",
                    default=["/vla/camera0/image_raw", "/vla/camera1/image_raw"])
    ap.add_argument("--state-gap", type=float, default=0.05)
    ap.add_argument("--timeout-first", type=float, default=30.0)
    ap.add_argument("--timeout", type=float, default=8.0)
    ap.add_argument("--retries", type=int, default=2)
    ap.add_argument("--save-frames", default="",
                    help="record raw frames + per-step metadata here for an offline HUD "
                         "video pass (demo/make_rollout_video.py). Adds render cost, so a "
                         "run with this set is NOT a timing measurement")
    ap.add_argument("--video-stride", type=int, default=1,
                    help="with --save-frames, render an extra frame every N sim steps; "
                         "1 is smooth and expensive, higher is choppier and cheaper")
    ap.add_argument("--publish-viz", action="store_true",
                    help="republish rendered frames as sensor_msgs/Image")
    ap.add_argument("--jsonl", default="", help="append per-episode records here")
    # --- hobby-arm feasibility ablations ------------------------------------------------
    # Default off, and with --ablate none the loop is the published path byte for byte.
    ap.add_argument("--arm-model", default="",
                    help="path to bench/arm-spec-measured.json. Required by every --ablate "
                         "mode except 'none'; degrades the loop to what the VUPN2355 hobby "
                         "arm could actually execute")
    ap.add_argument("--ablate", default="none", choices=sorted(ARM_MODES),
                    help="which physical limitation to reproduce: state-const tells the "
                         "policy nothing, state-deadreckon feeds back commanded pose as a "
                         "PWM servo arm would, dof5 removes the wrist DOF a 5-joint arm "
                         "lacks, quant applies the PWM count grid, slew caps joint speed, "
                         "composite applies all of them")
    ap.add_argument("--arm-frame-rate", type=float, default=None,
                    help="override the PWM frame rate in Hz. The 12-bit counts span one "
                         "PERIOD, so a faster frame rate gives FINER resolution -- see "
                         "bench/arm-envelope.json")
    args = ap.parse_args()

    if len(args.cameras) != len(args.camera_topics):
        sys.exit("--cameras and --camera-topics must have the same length")
    if len(args.cameras) != 2:
        sys.exit("send exactly 2 cameras: the graph's third slot is zero-filled by the "
                 "runner and a real third image would not match upstream's convention")

    if args.save_frames:
        print("NOTE: --save-frames renders extra frames (340 ms per pair on this board's\n"
              "      software rasterizer), so timings from this run are not comparable to\n"
              "      a clean sweep. Use it to make a video, not to measure.", file=sys.stderr)

    z = np.load(args.stats_npz, allow_pickle=False)
    stats = {k: np.asarray(z[k], dtype=np.float64)
             for k in ("state_mean", "state_std", "action_mean", "action_std")}

    install_torch_stub()
    from libero.libero import benchmark, get_libero_path
    from libero.libero.envs.env_wrapper import ControlEnv

    bm = benchmark.get_benchmark_dict()[args.suite]()
    task = bm.get_task(args.task_index)
    bddl = str(Path(get_libero_path("bddl_files")) / task.problem_folder / task.bddl_file)
    inits = np.asarray(bm.get_task_init_states(args.task_index))
    print(f"suite {args.suite} task {args.task_index}: {task.language!r}")
    print(f"init states: {inits.shape[0]}  bddl: {Path(bddl).name}")

    # use_camera_obs=False keeps the 340 ms render out of every physics step; the
    # offscreen renderer stays alive so we can render on demand at replan boundaries.
    env = ControlEnv(
        bddl_file_name=bddl,
        camera_heights=args.res, camera_widths=args.res,
        camera_names=list(args.cameras),
        use_camera_obs=False, has_offscreen_renderer=True, has_renderer=False,
    )
    env.seed(0)

    # --- arm model ---------------------------------------------------------------------
    arm = None
    if args.ablate != "none" or args.arm_model:
        if not args.arm_model:
            sys.exit(f"--ablate {args.ablate} needs --arm-model "
                     "bench/arm-spec-measured.json")
        spec = ArmSpec.load(args.arm_model, frame_rate_hz=args.arm_frame_rate)
        # Read the controller scaling off the LIVE controller rather than trusting the copy
        # in the spec JSON: every quantum in arm_embodiment.py is expressed as a fraction of
        # these, so a robosuite version that changed them would silently rescale the whole
        # ablation. Fall back to the OSC_POSE defaults only if the attribute moves.
        ctrl = getattr(env.env.robots[0], "controller", None)
        omax = getattr(ctrl, "output_max", None)
        if omax is not None and len(omax) >= 6:
            out_pos, out_rot = float(omax[0]), float(omax[3])
        else:
            out_pos, out_rot = 0.05, 0.5
            print("WARNING: could not read controller output_max; assuming OSC_POSE "
                  "defaults 0.05 m / 0.5 rad", file=sys.stderr)
        arm = ArmModel(spec, args.ablate, out_pos, out_rot,
                       state_mean=stats["state_mean"])
        print(f"arm model: {args.ablate}  spec={spec.status} ({spec.digest})  "
              f"pwm={spec.frame_rate_hz:.0f} Hz  "
              f"deg/count={spec.deg_per_count:.4f}  "
              f"scaling={out_pos} m / {out_rot} rad")
        print(f"           stages {arm.stages}")

    rclpy.init()
    runner = ClosedLoopRunner(args, stats)
    # An explicit executor rather than rclpy.spin(node), so shutdown can be ordered
    # deterministically in the finally block below -- see the comment there.
    executor = SingleThreadedExecutor()
    executor.add_node(runner)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    time.sleep(1.5)  # let publisher/subscriber matching settle before the first request

    msg = String()
    msg.data = task.language
    records: list[dict] = []
    try:
        sim_box: dict = {"sim": None}
        for n, init_index in enumerate(
            range(args.init_start, min(args.init_start + args.episodes, len(inits)))
        ):
            # The task is latched by the node, but it is republished per episode so a node
            # restarted mid-sweep does not silently inherit an empty prompt.
            runner.task_pub.publish(msg)
            rec = run_episode(runner, env, sim_box, inits, init_index,
                              task.language, args, arm)
            records.append(rec)
            print(f"[{n + 1}/{args.episodes}] init {init_index}: "
                  f"{'SUCCESS' if rec['success'] else 'fail   '} "
                  f"steps={rec['steps']:3d} chunks={rec['chunks']:2d} "
                  f"{rec['wall_s']:6.1f}s  lat={rec['latency_ms_mean']}")
    except KeyboardInterrupt:
        pass
    finally:
        env.close()
        # Stop spinning BEFORE tearing the node down, and wait for the thread to actually
        # leave spin(). Destroying a node while a thread is still inside the executor pulls
        # the executor's C++ side out from under it, which surfaces as
        # `terminate called without an active exception` and a core dump -- intermittently,
        # and only AFTER the episodes have already succeeded. That is worse than a
        # deterministic crash: under `set -o pipefail` in workshop/scripts/fastforward.sh
        # the abort fails the pipeline, `set -e` kills the escape hatch, and CP-8 never runs
        # even though the rollout worked. Observed on one of two otherwise identical runs.
        executor.shutdown()
        spin.join(timeout=5.0)
        runner.destroy_node()
        rclpy.try_shutdown()

    if not records:
        print("no episodes completed", file=sys.stderr)
        return 1

    n = len(records)
    k = sum(r["success"] for r in records)
    rate = k / n
    lo, hi = wilson_interval(k, n)
    print(f"\nsuccess {k}/{n} = {rate:.1%}  "
          f"(95% CI {lo:.1%}-{hi:.1%}, Wilson, on {n} episodes)")
    print(f"backends seen: {sorted(runner.backends)}  "
          f"stale chunks: {runner.stale_chunks}")
    if runner.latencies:
        lat = np.asarray(runner.latencies)
        print(f"NPU latency: mean {lat.mean():.1f} ms  p95 "
              f"{np.percentile(lat, 95):.1f} ms over {lat.size} chunks")
    if n < 20:
        print("NOTE: this is a REDUCED protocol, not the standard LIBERO benchmark. "
              "The CI above is wide; do not publish this as a suite success rate.")

    if args.jsonl:
        prov = provenance(args)
        with open(args.jsonl, "a") as fh:
            for r in records:
                fh.write(json.dumps({**prov, **r}) + "\n")
        print(f"appended {n} records to {args.jsonl}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
