#!/usr/bin/env python3
"""Pi0.5 flexibility lab: an arm in simulation, driven live from a browser.

What this is for: showing, in front of an audience, exactly how far one frozen
set of VLA weights stretches -- and where it stops. Nothing is pre-scripted
except the gesture lane, which says so on its own badge.

Three lanes, each labelled with what actually produced the motion:

  VLA       free-text prompt -> /qrb_ros_vla/task -> Pi0.5 on the Hexagon NPUs
            -> action chunk -> LIBERO steps the arm. Any wording is allowed.
  A/B       one frozen observation, two prompts differing by a single noun,
            plus the same prompt twice as a noise floor. Proves on stage that
            the words are doing work, with a number instead of a vibe.
  scripted  hand-written joint trajectories (demo/scripted_gestures.py). No
            model involved. This is the honest home for "wave hi": the deployed
            bundle is `lerobot/pi05_libero`, and measurement in
            bench/prompt-probe/compare.json shows non-manipulation prompts are
            indistinguishable from each other at the action level.

Any of the installed LIBERO worlds can be swapped in live from the scene picker
without restarting the model -- the weights never change, only the world does.
Free play has no goal: the benchmark success predicate is never evaluated and
never resets the scene. You reset it when you want to.

Cameras: the VLA always receives exactly its trained views (agentview +
robot0_eye_in_hand at 256px). The big presenter view is a separate `frontview`
render that the model never sees; it exists because the trained camera crops the
arm out of frame when it lifts.

Run:
  source /opt/ros/jazzy/setup.bash && source ros2_ws/install/setup.bash
  source /home/ubuntu/libero/libero-env.sh
  ros2 launch qrb_ros_vla vla.launch.py                    # terminal A
  $LIBERO_VENV_PYTHON demo/libero_web_demo.py --host 0.0.0.0 --port 8080

Then open http://<board-ip>:8080.
"""

from __future__ import annotations

import argparse
import io
import json
import os
import sys
import threading
import time
from dataclasses import dataclass, field
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any
from urllib.parse import parse_qs, urlparse

# Defaults match /home/ubuntu/libero/libero-env.sh so the script stays explicit
# about the external LIBERO project it drives. Source libero-env.sh for the venv.
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
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Image  # noqa: E402
from std_msgs.msg import Float32MultiArray, String  # noqa: E402

from qrb_ros_vla_msgs.msg import ActionChunk, InferenceStats  # noqa: E402

# Reuse the empirically verified LIBERO conventions already in this repository.
from libero_closed_loop import build_state, install_torch_stub, rot180  # noqa: E402
from language_probe import normalized_diff, verdict  # noqa: E402
from scene_catalog import Scene, build_catalog  # noqa: E402
from scripted_gestures import GESTURES, catalog as gesture_catalog  # noqa: E402

SCENES_DIR = _REPO / "demo/scenes"
INDEX_HTML = (Path(__file__).resolve().parent / "web" / "index.html")


@dataclass
class SharedState:
    """Everything the browser can see, plus the requests it can make.

    One lock, one condition. The sim thread owns the simulator; the HTTP threads
    only ever set request fields and read snapshots, which keeps the MuJoCo
    context single-threaded without a queue.
    """

    lock: threading.Lock = field(default_factory=threading.Lock)
    changed: threading.Condition = field(init=False)

    # what is happening
    lane: str = "idle"                  # idle | vla | scripted | ab
    provenance: str = "nothing running"
    task: str = ""
    gesture: str = ""
    command_id: int = 0
    scene_id: str = ""
    scene: dict[str, Any] = field(default_factory=dict)
    chunks: int = 0
    steps: int = 0
    first_action: list[float] = field(default_factory=list)
    action_range: list[float] | None = None
    stats: dict[str, float | str] | None = None
    ab: dict[str, Any] | None = None
    ab_running: bool = False
    note: str = "pick a lane"

    # requests from the browser, consumed by the sim thread
    want_scene: str = ""
    want_reset: str = ""            # "" | "scene" | "all"
    want_gesture: str = ""
    want_ab: tuple[str, str, bool] | None = None

    def __post_init__(self) -> None:
        self.changed = threading.Condition(self.lock)

    def snapshot(self) -> dict[str, Any]:
        with self.lock:
            return {
                "lane": self.lane,
                "provenance": self.provenance,
                "task": self.task,
                "gesture": self.gesture,
                "command_id": self.command_id,
                "scene_id": self.scene_id,
                "scene": self.scene,
                "chunks": self.chunks,
                "steps": self.steps,
                "first_action": self.first_action,
                "action_range": self.action_range,
                "stats": self.stats,
                "ab": self.ab,
                "ab_running": self.ab_running,
                "note": self.note,
                "want_scene": self.want_scene,
                "want_reset": self.want_reset,
                "want_gesture": self.want_gesture,
                "want_ab": self.want_ab,
            }

    def update(self, **values: Any) -> None:
        with self.changed:
            for key, value in values.items():
                setattr(self, key, value)
            self.changed.notify_all()

    def supersede(self, **values: Any) -> int:
        """Bump the command id and publish new state as one atomic step.

        Read-modify-write on command_id from separate HTTP threads would let two
        near-simultaneous clicks land on the same id, and the lane loops use that id
        as their only cancellation signal.
        """
        with self.changed:
            self.command_id += 1
            for key, value in values.items():
                setattr(self, key, value)
            self.changed.notify_all()
            return self.command_id


class FrameBus:
    """Latest-frame-wins JPEG hand-off from the sim thread to MJPEG readers."""

    def __init__(self) -> None:
        self._cond = threading.Condition()
        self._frames: dict[str, bytes] = {}
        self._seq = 0

    def publish(self, **named: bytes) -> None:
        with self._cond:
            self._frames.update(named)
            self._seq += 1
            self._cond.notify_all()

    def wait(self, kind: str, seq: int, timeout: float = 5.0) -> tuple[bytes | None, int]:
        with self._cond:
            if self._seq == seq:
                self._cond.wait(timeout)
            return self._frames.get(kind), self._seq


def jpeg(frame: np.ndarray, quality: int = 80) -> bytes:
    from PIL import Image as PILImage

    buf = io.BytesIO()
    PILImage.fromarray(frame).save(buf, format="JPEG", quality=quality)
    return buf.getvalue()


class RosLiberoDemo(Node):
    """The ROS half: publishes observations, waits for one action chunk."""

    def __init__(self, args: argparse.Namespace, shared: SharedState, stats: dict[str, np.ndarray]):
        super().__init__("vla_libero_web_demo")
        self.args = args
        self.shared = shared
        self.state_mean = stats["state_mean"]
        self.state_std = stats["state_std"]
        self.action_mean = stats["action_mean"]
        self.action_std = stats["action_std"]
        self.cam_pubs = [self.create_publisher(Image, t, 10) for t in args.camera_topics]
        self.state_pub = self.create_publisher(Float32MultiArray, f"{args.node}/state", 10)
        self.task_pub = self.create_publisher(String, f"{args.node}/task", 10)
        self.create_subscription(ActionChunk, f"{args.node}/action_chunk", self.on_chunk, 10)
        self.create_subscription(InferenceStats, f"{args.node}/stats", self.on_stats, 10)
        self.lock = threading.Lock()
        self.chunk_ready = threading.Event()
        self.pending: tuple[int, int] | None = None
        self.chunk: np.ndarray | None = None
        self.seq = 0
        self.last_stamp_ns = 0
        self.latest_stats: dict[str, float | str] | None = None
        # Inference runs on the ROS executor thread while the sim thread blocks in
        # request_chunk. Without an epoch, a chunk or stats message that lands just
        # after a reset repopulates the readout the reset just cleared. Every
        # superseding command bumps the epoch; callbacks from an older epoch are
        # dropped instead of written through.
        self.epoch = 0
        self.inflight_epoch = 0

    def abort(self) -> None:
        """Cancel any in-flight request and disown its results.

        Called by every command that supersedes another (new prompt, gesture,
        scene switch, probe, stop, reset). Waking `chunk_ready` is what lets the
        sim thread leave a 30 s timeout early instead of ignoring the button.
        """
        with self.lock:
            self.epoch += 1
            self.pending = None
            self.chunk = None
            self.latest_stats = None
        self.chunk_ready.set()

    def stale(self) -> bool:
        """True if the request currently in flight has been superseded.

        Only meaningful *after* request_chunk has run: it compares the epoch that
        request captured against the live one. Before issuing anything, the caller
        must instead compare against its own snapshot from `current_epoch`.
        """
        with self.lock:
            return self.inflight_epoch != self.epoch

    def current_epoch(self) -> int:
        with self.lock:
            return self.epoch

    def superseded(self, epoch: int) -> bool:
        """True if any command has landed since `epoch` was taken."""
        with self.lock:
            return self.epoch != epoch

    def publish_task(self, text: str) -> bool:
        if not rclpy.ok():
            return False
        msg = String()
        msg.data = text
        try:
            self.task_pub.publish(msg)
        except Exception:
            if not rclpy.ok():
                return False
            raise
        return True

    def on_chunk(self, msg: ActionChunk) -> None:
        got = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        with self.lock:
            if self.pending is None or got != self.pending:
                return  # a chunk for an observation we no longer wait on
            if self.inflight_epoch != self.epoch:
                self.pending = None  # superseded mid-inference; disown the result
                return
            dof = int(msg.action_dof or 7)
            actions = np.asarray(msg.actions, dtype=np.float32).reshape(
                msg.chunk_size, msg.action_dim)[:, :dof]
            if not msg.unnormalized:
                actions = actions * self.action_std[:dof] + self.action_mean[:dof]
            self.chunk = actions
            self.pending = None
        self.shared.update(
            first_action=[float(v) for v in actions[0]],
            action_range=[float(actions.min()), float(actions.max())],
        )
        self.chunk_ready.set()

    def on_stats(self, msg: InferenceStats) -> None:
        stats = {
            "backend": msg.backend,
            "vision_encoder_ms": msg.vision_encoder_ms,
            "token_emb_ms": msg.token_emb_ms,
            "backbone_ms": msg.backbone_ms,
            "action_expert_ms": msg.action_expert_ms,
            "host_packing_ms": msg.host_packing_ms,
            "total_ms": msg.total_ms,
            "chunks_per_second": msg.chunks_per_second,
        }
        with self.lock:
            if self.inflight_epoch != self.epoch:
                return  # telemetry for work the presenter already cancelled
            self.latest_stats = stats
        self.shared.update(stats=stats)

    def publish_state(self, state8: np.ndarray) -> bool:
        if not rclpy.ok():
            return False
        normalized = (state8 - self.state_mean) / (self.state_std + 1e-8)
        msg = Float32MultiArray()
        msg.data = [float(v) for v in normalized]
        try:
            self.state_pub.publish(msg)
        except Exception:
            if not rclpy.ok():
                return False
            raise
        return True

    def image_msg(self, frame: np.ndarray, stamp, frame_id: str) -> Image:
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[0], frame.shape[1]
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = frame.shape[1] * 3
        msg.data = frame.tobytes()
        return msg

    def request_chunk(self, task: str, state8: np.ndarray,
                      frames: list[np.ndarray]) -> np.ndarray | None:
        """One observation in, one action chunk out. Strictly one in flight.

        Pi0.5 has no state tensor: proprioception is discretized into the language
        prompt, so state must land before the image that triggers inference.

        Returns None both when cancelled and when timed out; the caller separates
        the two with `stale()`, because "the presenter pressed reset" and "the NPU
        never answered" need different messages on screen.
        """
        with self.lock:
            self.inflight_epoch = self.epoch
        if not self.publish_task(task):
            return None
        self.chunk_ready.clear()
        self.seq += 1
        if not self.publish_state(state8):
            return None
        time.sleep(self.args.state_gap)
        if not rclpy.ok() or self.stale():
            return None
        stamp = self.get_clock().now().to_msg()
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        if stamp_ns <= self.last_stamp_ns:
            stamp.nanosec = (stamp.nanosec + 1) % 1_000_000_000
            stamp_ns += 1
        self.last_stamp_ns = stamp_ns
        with self.lock:
            self.chunk = None
            self.pending = (stamp.sec, stamp.nanosec)
        try:
            for pub, frame in zip(self.cam_pubs, frames):
                pub.publish(self.image_msg(frame, stamp, f"web{self.seq}"))
        except Exception:
            if not rclpy.ok():
                return None
            raise
        if not self.chunk_ready.wait(self.args.timeout):
            with self.lock:
                self.pending = None
            self.get_logger().warn(f"no action chunk after {self.args.timeout:.1f}s")
            return None
        with self.lock:
            if self.inflight_epoch != self.epoch:
                return None
            return None if self.chunk is None else self.chunk.copy()


class EpisodeRecovered(RuntimeError):
    """The simulator refused a step and the scene was re-posed to keep going."""


class SceneRunner:
    """Owns the MuJoCo environment and knows how to swap the world underneath it."""

    def __init__(self, args: argparse.Namespace):
        install_torch_stub()
        from libero.libero.envs.env_wrapper import ControlEnv

        self._ControlEnv = ControlEnv
        self.args = args
        self.env = None
        self.scene: Scene | None = None
        self.inits: np.ndarray | None = None
        self.obs: dict | None = None
        self.prev_state: np.ndarray | None = None

    def _init_states(self, scene: Scene) -> np.ndarray | None:
        if scene.source != "libero":
            return None
        from libero.libero import benchmark

        bm = benchmark.get_benchmark_dict()[scene.suite]()
        return np.asarray(bm.get_task_init_states(scene.task_index))

    def load(self, scene: Scene) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None
        self.scene = scene
        self.inits = self._init_states(scene)
        self.env = self._ControlEnv(
            bddl_file_name=scene.bddl,
            camera_heights=self.args.res,
            camera_widths=self.args.res,
            camera_names=list(self.args.cameras) + [self.args.presenter_camera],
            use_camera_obs=False,
            has_offscreen_renderer=True,
            has_renderer=False,
            # Free play has no episode. Discarding the `done` flag at the call site is
            # not enough: robosuite sets `done = timestep >= horizon` (default 1000)
            # and then *raises* on the next step, which killed the sim thread and the
            # demo with it after ~100 s of continuous play. ignore_done is the only
            # switch that stops the episode from ever ending.
            ignore_done=True,
        )
        self.env.seed(self.args.seed)
        self.reset()

    def reset(self) -> None:
        assert self.env is not None
        self.env.reset()
        if self.inits is not None and len(self.inits):
            index = min(self.args.init_index, len(self.inits) - 1)
            self.env.set_init_state(self.inits[index])
        neutral = np.zeros(7)
        neutral[6] = -1.0
        obs = None
        for _ in range(self.args.settle):
            obs, _, _, _ = self.env.step(neutral)
        self.obs = obs
        self.prev_state = None

    def render(self, camera: str, res: int) -> np.ndarray:
        assert self.env is not None
        return rot180(np.asarray(
            self.env.env.sim.render(width=res, height=res, camera_name=camera),
            dtype=np.uint8))

    def model_frames(self) -> list[np.ndarray]:
        """Exactly the views the policy was trained on, in training order."""
        return [self.render(cam, self.args.res) for cam in self.args.cameras]

    def presenter_frame(self) -> np.ndarray:
        return self.render(self.args.presenter_camera, self.args.presenter_res)

    def state(self) -> np.ndarray:
        assert self.obs is not None
        state8 = build_state(self.obs, self.prev_state)
        self.prev_state = state8
        return state8

    def step(self, action: np.ndarray) -> None:
        """One control step. Raises EpisodeRecovered if the sim had to be re-posed.

        The benchmark success predicate is deliberately discarded: free play has no
        goal, so nothing here may end an episode on its own. `ignore_done=True` stops
        robosuite terminating on the horizon, but a simulator exception must still
        never take the demo down mid-talk: re-pose the scene, tell the presenter, and
        let them keep going.
        """
        assert self.env is not None
        try:
            self.obs, _, _, _ = self.env.step(np.asarray(action, dtype=np.float64))
        except (ValueError, RuntimeError) as exc:
            self.reset()
            raise EpisodeRecovered(str(exc)) from exc

    def close(self) -> None:
        if self.env is not None:
            self.env.close()
            self.env = None


def publish_frames(runner: SceneRunner, bus: FrameBus, model: list[np.ndarray] | None = None) -> None:
    frames = {"presenter": jpeg(runner.presenter_frame(), 82)}
    if model is not None:
        for name, frame in zip(("model0", "model1"), model):
            frames[name] = jpeg(frame, 78)
    bus.publish(**frames)


def run_ab_probe(runner: SceneRunner, node: RosLiberoDemo, prompt_a: str, prompt_b: str,
                 *, from_start: bool = True) -> dict[str, Any]:
    """Same pixels, same proprioception, one noun changed.

    Order is A, B, A: the third request is the noise floor, and putting it last
    means any slow drift in the runtime counts against the floor rather than
    inflating the prompt effect.

    ``from_start`` resets to the scene's settled start pose first, and defaults to
    true because the answer genuinely depends on the pose. Measured mid-grasp,
    with the gripper already around the bowl, both prompts return the same plan
    (0.9x floor): the visual context has already decided what happens next. From
    the start pose the same pair separates cleanly. A probe whose result depends
    on where the arm drifted to is not a proof, so the deterministic pose is the
    default and the pose used is reported alongside the number.
    """
    # Snapshot the epoch the probe starts from. `stale()` cannot be used here: the
    # command that requested this probe already bumped the epoch, so before the
    # first request the in-flight epoch is one behind and the probe would cancel
    # itself every time.
    epoch = node.current_epoch()
    if from_start:
        runner.reset()
    frames = runner.model_frames()
    state8 = runner.state()
    chunks: list[np.ndarray] = []
    for prompt in (prompt_a, prompt_b, prompt_a):
        if node.superseded(epoch):
            return {"cancelled": True}
        chunk = node.request_chunk(prompt, state8, frames)
        if chunk is None:
            # A probe holds the sim for three chunks, so it is the most likely lane
            # to be interrupted. Cancelled is not a failure and must not be shown
            # as one.
            return ({"cancelled": True} if node.superseded(epoch)
                    else {"error": f"timed out on {prompt!r}"})
        chunks.append(chunk)
    std = np.asarray(node.action_std, dtype=np.float64)
    return {
        "prompt_a": prompt_a,
        "prompt_b": prompt_b,
        "pose": "scene start pose" if from_start else "current pose",
        "first_action_a": [round(float(v), 4) for v in chunks[0][0]],
        "first_action_b": [round(float(v), 4) for v in chunks[1][0]],
        **verdict(normalized_diff(chunks[0], chunks[1], std),
                  normalized_diff(chunks[0], chunks[2], std)),
    }


def sim_loop(args: argparse.Namespace, node: RosLiberoDemo, shared: SharedState,
             bus: FrameBus, catalog) -> None:
    runner = SceneRunner(args)
    scene = catalog.get(args.scene) or catalog.ordered()[0]
    runner.load(scene)
    shared.update(scene_id=scene.scene_id, scene=scene.to_json(), steps=0, chunks=0,
                  note="ready - type a prompt, run the A/B probe, or fire a scripted gesture")
    publish_frames(runner, bus, runner.model_frames())

    step_period = 1.0 / args.control_hz if args.realtime and args.control_hz > 0 else 0.0
    try:
        while rclpy.ok():
            snap = shared.snapshot()

            if snap["want_scene"]:
                target = catalog.get(str(snap["want_scene"]))
                shared.update(want_scene="", lane="idle", provenance="loading world",
                              task="", gesture="", ab=None, chunks=0, steps=0,
                              note="loading world...")
                if target is not None:
                    runner.load(target)
                    shared.update(scene_id=target.scene_id, scene=target.to_json(),
                                  note=f"loaded {target.label} - same weights, new world")
                publish_frames(runner, bus, runner.model_frames())
                continue

            if snap["want_reset"]:
                scope = str(snap["want_reset"])
                if scope == "all":
                    target = catalog.get(args.scene) or catalog.ordered()[0]
                    shared.update(want_reset="", lane="idle", provenance="full reset",
                                  note="full reset - reloading the startup world...")
                    # Rebuild the environment rather than just re-posing it, so a world
                    # left in a wedged state (objects knocked off the table, drawer torn
                    # open) cannot survive the reset the presenter just asked for.
                    runner.load(target)
                    shared.update(scene_id=target.scene_id, scene=target.to_json(),
                                  lane="idle", provenance="nothing running", task="",
                                  gesture="", chunks=0, steps=0, ab=None, stats=None,
                                  first_action=[], action_range=None,
                                  note="full reset - startup world reloaded, every panel cleared")
                else:
                    runner.reset()
                    shared.update(want_reset="", lane="idle", provenance="nothing running",
                                  task="", gesture="", chunks=0, steps=0, ab=None,
                                  note="scene reset - arm and objects back to the start pose")
                publish_frames(runner, bus, runner.model_frames())
                continue

            if snap["want_gesture"]:
                name = str(snap["want_gesture"])
                gesture = GESTURES.get(name)
                command_id = int(snap["command_id"])
                shared.update(want_gesture="", gesture=name, lane="scripted",
                              provenance="scripted trajectory - no model in the loop",
                              note=f"playing scripted gesture: {gesture.label if gesture else name}")
                interrupted = False
                if gesture is not None:
                    next_at = time.monotonic()
                    for action in gesture.actions():
                        if int(shared.snapshot()["command_id"]) != command_id:
                            interrupted = True
                            break
                        if step_period:
                            now = time.monotonic()
                            if next_at > now:
                                time.sleep(next_at - now)
                            next_at = max(next_at + step_period, time.monotonic())
                        try:
                            runner.step(action)
                        except EpisodeRecovered as exc:
                            # Recovery is not a supersede: no other command is waiting to
                            # own the state, so this branch must return the lane to idle
                            # itself or the UI shows "scripted" forever with nothing running.
                            shared.update(steps=0, lane="idle", provenance="nothing running",
                                          note=f"simulator re-posed mid-gesture: {exc}")
                            interrupted = True
                            break
                        shared.update(steps=int(shared.snapshot()["steps"]) + 1)
                        publish_frames(runner, bus)
                # An interrupted gesture must not announce itself as finished, and
                # must not overwrite the lane/note the interrupting command just set.
                if not interrupted:
                    shared.update(lane="idle", provenance="nothing running",
                                  note="scripted gesture finished")
                publish_frames(runner, bus, runner.model_frames())
                continue

            if snap["want_ab"]:
                prompt_a, prompt_b, from_start = snap["want_ab"]  # type: ignore[misc]
                pose = "scene start pose" if from_start else "current pose"
                command_id = int(snap["command_id"])
                shared.update(want_ab=None, ab_running=True, lane="ab",
                              provenance=f"A/B language probe - 3 NPU chunks from the {pose}",
                              note=f"probing {prompt_a!r} vs {prompt_b!r} from the {pose}")
                result = run_ab_probe(runner, node, str(prompt_a), str(prompt_b),
                                      from_start=bool(from_start))
                superseded = int(shared.snapshot()["command_id"]) != command_id
                if superseded:
                    # A newer command owns every field now, including ab_running,
                    # which `issue` already cleared. Touch nothing on the way out.
                    publish_frames(runner, bus, runner.model_frames())
                    continue
                if result.get("cancelled"):
                    shared.update(ab_running=False)
                else:
                    shared.update(ab=result, ab_running=False, lane="idle",
                                  provenance="nothing running",
                                  note=str(result.get("summary", result.get("error", "probe done"))))
                publish_frames(runner, bus, runner.model_frames())
                continue

            if snap["lane"] != "vla":
                time.sleep(0.05)
                continue

            task_text = str(snap["task"])
            command_id = int(snap["command_id"])
            model = runner.model_frames()
            publish_frames(runner, bus, model)
            actions = node.request_chunk(task_text, runner.state(), model)
            if actions is None:
                # A cancelled request is the presenter pressing a button, not a fault;
                # only a real timeout should accuse the VLA node of being down.
                if not node.stale():
                    shared.update(note="no action chunk returned; is the VLA node up?")
                continue
            if int(shared.snapshot()["command_id"]) != command_id:
                continue
            shared.update(chunks=int(shared.snapshot()["chunks"]) + 1)
            horizon = min(args.replan_horizon, len(actions))
            next_at = time.monotonic()
            for i in range(horizon):
                current = shared.snapshot()
                if int(current["command_id"]) != command_id or current["lane"] != "vla":
                    break
                if step_period:
                    now = time.monotonic()
                    if next_at > now:
                        time.sleep(next_at - now)
                    next_at = max(next_at + step_period, time.monotonic())
                try:
                    runner.step(actions[i])
                except EpisodeRecovered as exc:
                    # Scene is already re-posed. Drop to idle so the presenter decides
                    # what happens next instead of the arm charging off from a pose the
                    # current chunk was never planned for. Publish before breaking: the
                    # loop goes idle straight after, so nothing else would refresh the
                    # view and the screen would show a pose the simulator has left.
                    shared.update(lane="idle", provenance="nothing running", steps=0,
                                  note=f"simulator re-posed and the lane stopped: {exc}")
                    publish_frames(runner, bus, runner.model_frames())
                    break
                shared.update(steps=int(shared.snapshot()["steps"]) + 1)
                publish_frames(runner, bus)
    finally:
        runner.close()


class DemoHandler(BaseHTTPRequestHandler):
    server_version = "Pi05FlexibilityLab/2.0"
    protocol_version = "HTTP/1.1"

    @property
    def shared(self) -> SharedState:
        return self.server.shared  # type: ignore[attr-defined]

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_bytes(INDEX_HTML.read_bytes(), "text/html; charset=utf-8")
        elif parsed.path == "/api/state":
            self.send_json(self.shared.snapshot())
        elif parsed.path == "/api/catalog":
            self.send_json({
                "scenes": self.server.catalog.to_json(),  # type: ignore[attr-defined]
                "gestures": gesture_catalog(),
            })
        elif parsed.path == "/mjpeg":
            kind = (parse_qs(parsed.query).get("kind") or ["presenter"])[0]
            self.stream_mjpeg(kind)
        elif parsed.path == "/stream":
            self.stream_events()
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def issue(self, **values: Any) -> None:
        """Supersede whatever is running, then publish the new state.

        The abort comes first: a chunk landing between the state write and the
        cancellation would write a readout for work that is already dead.

        `ab_running`, the action readout, the telemetry, and every queued `want_*`
        request are cleared here rather than by the lane being interrupted. Three
        reasons. A superseded probe must write nothing at all on its way out, so the
        only safe owner of the spinner flag is the command doing the superseding. The
        sim loop drains the queue in a fixed order (scene, reset, gesture, probe,
        VLA), so a gesture or probe queued just before a reset would otherwise be
        handled *after* it and replay onto the stage the presenter just cleared. And
        the readout describes one command's output only - a world switch that kept the
        previous world's last action vector on screen was a real bug found in testing.
        Net invariant: one queued request, and it is the most recent click.
        """
        self.server.ros_node.abort()  # type: ignore[attr-defined]
        for field, empty in (("ab_running", False), ("want_scene", ""), ("want_reset", ""),
                             ("want_gesture", ""), ("want_ab", None),
                             ("first_action", []), ("action_range", None), ("stats", None)):
            values.setdefault(field, empty)
        self.shared.supersede(**values)

    def do_POST(self) -> None:
        path = urlparse(self.path).path
        payload = self.read_json()
        if payload is None:
            return
        shared = self.shared

        if path == "/api/task":
            task = str(payload.get("task", "")).strip()
            if not task:
                self.send_error(HTTPStatus.BAD_REQUEST, "task must be non-empty")
                return
            self.issue(task=task, lane="vla", gesture="", chunks=0, ab=None,
                       provenance="Pi0.5 on the Hexagon NPUs",
                       note=f"VLA running: {task!r}")
            self.send_json({"ok": True, "task": task})
        elif path == "/api/gesture":
            name = str(payload.get("name", ""))
            if name not in GESTURES:
                self.send_error(HTTPStatus.BAD_REQUEST, f"unknown gesture {name!r}")
                return
            self.issue(want_gesture=name, task="", lane="idle")
            self.send_json({"ok": True, "gesture": name})
        elif path == "/api/scene":
            scene_id = str(payload.get("scene_id", ""))
            if self.server.catalog.get(scene_id) is None:  # type: ignore[attr-defined]
                self.send_error(HTTPStatus.BAD_REQUEST, f"unknown scene {scene_id!r}")
                return
            self.issue(want_scene=scene_id)
            self.send_json({"ok": True, "scene_id": scene_id})
        elif path == "/api/probe":
            default = (shared.snapshot().get("scene") or {}).get("ab_pair") or []
            prompt_a = str(payload.get("a") or (default[0] if len(default) > 1 else "")).strip()
            prompt_b = str(payload.get("b") or (default[1] if len(default) > 1 else "")).strip()
            if not prompt_a or not prompt_b:
                self.send_error(HTTPStatus.BAD_REQUEST, "need two prompts to compare")
                return
            from_start = not bool(payload.get("from_current"))
            self.issue(want_ab=(prompt_a, prompt_b, from_start), lane="idle", ab=None)
            self.send_json({"ok": True, "a": prompt_a, "b": prompt_b, "from_start": from_start})
        elif path == "/api/stop":
            self.issue(lane="idle", provenance="nothing running",
                         note="stopped; the scene is left exactly where it is")
            self.send_json({"ok": True})
        elif path == "/api/reset":
            scope = "all" if payload.get("all") else "scene"
            self.issue(want_reset=scope, lane="idle")
            self.send_json({"ok": True, "scope": scope})
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def read_json(self) -> dict[str, Any] | None:
        length = int(self.headers.get("Content-Length", "0"))
        if not length:
            return {}
        try:
            return json.loads(self.rfile.read(length).decode("utf-8"))
        except (TypeError, ValueError, json.JSONDecodeError):
            self.send_error(HTTPStatus.BAD_REQUEST, "expected a JSON object")
            return None

    def stream_mjpeg(self, kind: str) -> None:
        bus: FrameBus = self.server.bus  # type: ignore[attr-defined]
        boundary = "frameboundary"
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        seq = -1
        try:
            while True:
                frame, seq = bus.wait(kind, seq)
                if frame is None:
                    continue
                self.wfile.write(f"--{boundary}\r\nContent-Type: image/jpeg\r\n"
                                 f"Content-Length: {len(frame)}\r\n\r\n".encode())
                self.wfile.write(frame)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def stream_events(self) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", "text/event-stream")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        shared = self.shared
        last = ""
        try:
            while True:
                with shared.changed:
                    shared.changed.wait(timeout=1.0)
                snap = shared.snapshot()
                for key in ("want_scene", "want_reset", "want_gesture", "want_ab"):
                    snap.pop(key, None)
                payload = json.dumps(snap, separators=(",", ":"))
                if payload != last:
                    self.wfile.write(f"data: {payload}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    last = payload
        except (BrokenPipeError, ConnectionResetError, OSError):
            return

    def send_json(self, payload: Any) -> None:
        self.send_bytes(json.dumps(payload).encode("utf-8"), "application/json")

    def send_bytes(self, data: bytes, content_type: str) -> None:
        self.send_response(HTTPStatus.OK)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, fmt: str, *args: Any) -> None:
        return


def main() -> None:
    ap = argparse.ArgumentParser(description="Pi0.5 flexibility lab (ROS + LIBERO + browser)")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    ap.add_argument("--node", default="/qrb_ros_vla")
    ap.add_argument("--scene", default="sandbox:sandbox_table",
                    help="scene id to load at startup, e.g. libero_spatial:0")
    ap.add_argument("--suites", nargs="+",
                    default=["libero_spatial", "libero_object", "libero_goal", "libero_10"],
                    help="LIBERO suites to offer in the scene picker")
    ap.add_argument("--init-index", type=int, default=0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--res", type=int, default=256, help="resolution the policy sees")
    ap.add_argument("--presenter-res", type=int, default=448)
    ap.add_argument("--presenter-camera", default="frontview",
                    help="camera for the browser only; the policy never sees it")
    ap.add_argument("--cameras", nargs="+", default=["agentview", "robot0_eye_in_hand"])
    ap.add_argument("--camera-topics", nargs="+",
                    default=["/vla/camera0/image_raw", "/vla/camera1/image_raw"])
    ap.add_argument("--stats-npz", default=str(_REPO / "artifacts/libero_ep0.npz"))
    ap.add_argument("--replan-horizon", type=int, default=6)
    ap.add_argument("--settle", type=int, default=10)
    ap.add_argument("--state-gap", type=float, default=0.05)
    ap.add_argument("--timeout", type=float, default=30.0)
    ap.add_argument("--control-hz", type=float, default=10.0,
                    help="pace env.step to the LIBERO control rate")
    ap.add_argument("--no-realtime", dest="realtime", action="store_false",
                    help="step the simulator as fast as possible")
    ap.set_defaults(realtime=True)
    args = ap.parse_args()

    if len(args.cameras) != len(args.camera_topics):
        sys.exit("--cameras and --camera-topics must have the same length")
    if args.presenter_camera in args.cameras:
        sys.exit("--presenter-camera must differ from the policy's --cameras")
    if not Path(args.stats_npz).exists():
        sys.exit(f"missing {args.stats_npz}; run scripts/fetch-libero-episode.sh first")
    if not INDEX_HTML.exists():
        sys.exit(f"missing UI file {INDEX_HTML}")

    z = np.load(args.stats_npz, allow_pickle=False)
    stats = {k: np.asarray(z[k], dtype=np.float64)
             for k in ("state_mean", "state_std", "action_mean", "action_std")}

    install_torch_stub()
    catalog = build_catalog(SCENES_DIR, suites=tuple(args.suites))

    shared = SharedState()
    bus = FrameBus()
    rclpy.init()
    node = RosLiberoDemo(args, shared, stats)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    def spin_executor() -> None:
        try:
            executor.spin()
        except ExternalShutdownException:
            pass

    threading.Thread(target=spin_executor, daemon=True).start()

    httpd = ThreadingHTTPServer((args.host, args.port), DemoHandler)
    httpd.daemon_threads = True
    httpd.shared = shared        # type: ignore[attr-defined]
    httpd.bus = bus              # type: ignore[attr-defined]
    httpd.catalog = catalog      # type: ignore[attr-defined]
    httpd.ros_node = node        # type: ignore[attr-defined]
    threading.Thread(target=httpd.serve_forever, daemon=True).start()
    node.get_logger().info(
        f"flexibility lab on http://{args.host}:{args.port} - "
        f"{len(catalog.scenes)} scenes, {len(GESTURES)} scripted gestures")
    try:
        sim_loop(args, node, shared, bus, catalog)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        httpd.shutdown()
        httpd.server_close()
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
