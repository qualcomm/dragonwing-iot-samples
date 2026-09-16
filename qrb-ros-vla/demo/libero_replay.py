#!/usr/bin/env python3
"""Replay a real LIBERO episode through Pi0.5 on the NPU and score it against
the human demonstration.

This is the demo with an honest, checkable claim. Rather than asserting the
policy "works", it feeds the model the exact observations a human demonstrator
saw and measures how close the NPU's predicted action chunk is to what the
demonstrator actually did.

Pipeline per replanning step:

    parquet frame ─► base + wrist image ─► /vla/camera{0,1}/image_raw ─┐
                  └─ state[8] ─ MEAN_STD normalize ─► ~/state          ├─► qrb_ros_vla (NPU)
                     task string ──────────────────► ~/task            ┘
                                                                        │
    ground-truth actions[t : t+H]  ◄── compare ──  ~/action_chunk ──────┘
                                                   (un-normalized with
                                                    MEAN_STD action stats)

Normalization is not guesswork: lerobot/pi05_libero's policy_preprocessor.json
declares norm_map {VISUAL: IDENTITY, STATE: MEAN_STD, ACTION: MEAN_STD}, and the
mean/std come from the dataset's own meta/stats.json.

Fetch the inputs first:
    scripts/fetch-libero-episode.sh          # parquet + metadata (needs pyarrow)
    scripts/prepare-libero-npz.py            # -> .npz, numpy-only for the live path

Then, with ROS 2 and the workspace overlay sourced and the VLA node running:
    python3 demo/libero_replay.py --episode artifacts/libero_ep0.npz
"""

from __future__ import annotations

import argparse
import io
import json
import sys
import threading
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from qrb_ros_vla_msgs.msg import ActionChunk, InferenceStats

# Artifact defaults are anchored to the repository, not the caller's working directory: these
# scripts are invoked by workshop/scripts/fastforward.sh and by hand from various places, and
# a bare relative default fails with FileNotFoundError depending on where you happen to be.
# An explicitly passed path is still interpreted relative to the cwd, which is what a user
# typing one expects.
_REPO = Path(__file__).resolve().parent.parent


def _repo_path(rel: str) -> str:
    return str(_REPO / rel)


ACTION_NAMES = ["dx", "dy", "dz", "droll", "dpitch", "dyaw", "grip"]


def decode_image(cell) -> np.ndarray:
    """Return HxWx3 uint8 RGB.

    Accepts either an already-decoded array (the .npz path, numpy only) or
    LIBERO's parquet cell {'bytes': <encoded>, 'path': ...}.
    """
    if isinstance(cell, np.ndarray) and cell.ndim == 3:
        return cell
    raw = cell["bytes"] if isinstance(cell, dict) else cell
    try:
        from PIL import Image as PILImage

        return np.asarray(PILImage.open(io.BytesIO(raw)).convert("RGB"))
    except ImportError:
        import cv2

        arr = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
        if arr is None:
            raise RuntimeError("cannot decode image; install pillow or opencv")
        return cv2.cvtColor(arr, cv2.COLOR_BGR2RGB)


def load_episode(path: str, stats_path: str, tasks_path: str):
    """Load either the numpy-only .npz bundle or the original parquet."""
    if path.endswith(".npz"):
        z = np.load(path, allow_pickle=False)
        frames = {k: z[k] for k in ("image", "wrist_image", "state", "actions")}
        stats = {
            "state": {"mean": z["state_mean"], "std": z["state_std"]},
            "actions": {"mean": z["action_mean"], "std": z["action_std"]},
        }
        return frames, stats, str(z["task"]), float(z["fps"])

    import pyarrow.parquet as pq

    for p in (stats_path, tasks_path):
        if not Path(p).exists():
            sys.exit(f"missing {p}; run scripts/fetch-libero-episode.sh first")
    frames = pq.read_table(path).to_pydict()
    stats = json.loads(Path(stats_path).read_text())
    tasks = {
        json.loads(line)["task_index"]: json.loads(line)["task"]
        for line in Path(tasks_path).read_text().splitlines()
        if line.strip()
    }
    task = tasks.get(int(frames["task_index"][0]), "complete the manipulation task")
    return frames, stats, task, 10.0


class LiberoReplay(Node):
    def __init__(self, args: argparse.Namespace, frames: dict, stats: dict, task: str):
        super().__init__("vla_libero_replay")
        self.args = args
        self.frames = frames
        self.task = task
        self.n_frames = len(frames["state"])

        self.state_mean = np.asarray(stats["state"]["mean"], dtype=np.float32)
        self.state_std = np.asarray(stats["state"]["std"], dtype=np.float32)
        self.action_mean = np.asarray(stats["actions"]["mean"], dtype=np.float32)
        self.action_std = np.asarray(stats["actions"]["std"], dtype=np.float32)

        self.cam_pubs = [
            self.create_publisher(Image, t, 10) for t in args.camera_topics
        ]
        self.state_pub = self.create_publisher(Float32MultiArray, f"{args.node}/state", 10)
        self.task_pub = self.create_publisher(String, f"{args.node}/task", 10)
        self.create_subscription(ActionChunk, f"{args.node}/action_chunk", self.on_chunk, 10)
        self.create_subscription(InferenceStats, f"{args.node}/stats", self.on_stats, 10)

        self.frame_index = 0
        self.pending_index: int | None = None
        self.seq = 0
        self.pending_stamp: tuple[int, int] | None = None
        self.stale_chunks = 0
        self.results: list[dict] = []
        self.latencies: list[float] = []
        self.done = threading.Event()
        self.lock = threading.Lock()

        self.get_logger().info(
            f"episode: {self.n_frames} frames @ {args.fps} Hz | task: '{task}'"
        )
        # Give the node's subscriptions a moment to match, then start the loop.
        self.start_timer = self.create_timer(1.5, self.start)

    def start(self) -> None:
        self.start_timer.cancel()
        # Order matters, and not subtly. The node's Ready() gate (vla_node.cpp:366)
        # tests only "tokens are 200 long" AND "every camera has a fresh frame" -- robot
        # state is NOT a precondition. And with no state, Pi05Tokenizer::BuildPrompt
        # emits the Pi0-style "<task>\n" prompt instead of the Pi0.5
        # "Task: ..., State: ...;\nAction: " form this export was calibrated on.
        # So publish state FIRST: OnState stores it unconditionally before its early
        # return, and the subsequent OnTask then tokenizes task and state together in
        # one shot, correctly, with no window in which a state-less prompt is live.
        self.publish_state(0)
        msg = String()
        msg.data = self.task
        self.task_pub.publish(msg)
        self.publish_frame()

    def publish_state(self, idx: int) -> None:
        """Publish the MEAN_STD-normalized state for frame `idx`.

        The node expects state already normalized with the policy's training statistics
        and only bins it into 256 buckets for the prompt, so the statistics live here.
        """
        state = np.asarray(self.frames["state"][idx], dtype=np.float32)
        normalized = (state - self.state_mean) / (self.state_std + 1e-8)
        smsg = Float32MultiArray()
        smsg.data = [float(v) for v in normalized]
        self.state_pub.publish(smsg)

    def publish_frame(self) -> None:
        with self.lock:
            if self.frame_index >= self.n_frames or len(self.results) >= self.args.steps:
                self.done.set()
                return
            idx = self.frame_index
            self.pending_index = idx
            self.seq += 1
            seq = self.seq

        # State before images, every step. DDS guarantees per-topic ordering but not
        # cross-topic, so publishing images first lets the node fire on frame k while
        # tokens_ still encode state k-1 -- a silently stale prompt. The gap is free
        # against ~1.1 s of inference.
        self.publish_state(idx)
        time.sleep(self.args.state_gap)

        stamp = self.get_clock().now().to_msg()
        # An identical stamp across cameras is what makes one observation one unit, and
        # the node echoes stamp and frame_id from the triggering image into the
        # returned ActionChunk (vla_node.cpp:414-415) -- so this is the correlation key.
        self.pending_stamp = (stamp.sec, stamp.nanosec)
        for pub, key in zip(self.cam_pubs, ("image", "wrist_image")):
            frame = decode_image(self.frames[key][idx])
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = f"obs{seq}"
            msg.height, msg.width = frame.shape[0], frame.shape[1]
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = frame.shape[1] * 3
            msg.data = frame.tobytes()
            pub.publish(msg)

    def on_chunk(self, msg: ActionChunk) -> None:
        with self.lock:
            idx = self.pending_index
            if idx is None:
                return
            # The node echoes the triggering image's stamp, so a mismatch means this
            # chunk was computed from a different observation than the one we are about
            # to score it against. Counting these is the only way to notice; scoring
            # them would quietly corrupt the MAE.
            got = (msg.header.stamp.sec, msg.header.stamp.nanosec)
            if self.pending_stamp is not None and got != self.pending_stamp:
                self.stale_chunks += 1
                return
            self.pending_index = None

        dof = msg.action_dof or len(self.action_mean)
        pred = np.asarray(msg.actions, dtype=np.float32).reshape(
            msg.chunk_size, msg.action_dim
        )[:, :dof]
        # The model works in normalized action space; MEAN_STD back to real units.
        pred = pred * self.action_std[:dof] + self.action_mean[:dof]

        horizon = min(self.args.horizon, msg.chunk_size, self.n_frames - idx)
        truth = np.asarray(
            self.frames["actions"][idx : idx + horizon], dtype=np.float32
        )[:, :dof]
        pred_h = pred[:horizon]

        abs_err = np.abs(pred_h - truth)
        # A scale-free reference: how big is the error next to the spread of real
        # actions in this dataset? >1.0 means worse than predicting the mean.
        nmae = abs_err.mean(axis=0) / (self.action_std[:dof] + 1e-8)

        self.results.append(
            {
                "frame": idx,
                "mae": abs_err.mean(axis=0),
                "nmae": nmae,
                "truth0": truth[0],
                "pred0": pred_h[0],
            }
        )
        self.get_logger().info(
            f"frame {idx:4d} | MAE {abs_err.mean():.4f} | normalized MAE {nmae.mean():.3f} | "
            f"grip pred {pred_h[0][-1]:+.2f} truth {truth[0][-1]:+.2f}"
        )

        with self.lock:
            self.frame_index += self.args.stride
        self.publish_frame()

    def on_stats(self, msg: InferenceStats) -> None:
        self.latencies.append(msg.total_ms)

    def report(self) -> None:
        if not self.results:
            print("\nno chunks received - is the VLA node running?", file=sys.stderr)
            return
        mae = np.stack([r["mae"] for r in self.results])
        nmae = np.stack([r["nmae"] for r in self.results])
        print()
        print(f"LIBERO replay: {len(self.results)} replanning steps, "
              f"horizon {self.args.horizon}, task '{self.task}'")
        print(f"{'dim':>7}  {'MAE':>9}  {'MAE/std':>9}  {'action std':>11}")
        for i, name in enumerate(ACTION_NAMES[: mae.shape[1]]):
            print(f"{name:>7}  {mae[:, i].mean():9.4f}  {nmae[:, i].mean():9.3f}  "
                  f"{self.action_std[i]:11.3f}")
        print(f"{'ALL':>7}  {mae.mean():9.4f}  {nmae.mean():9.3f}")
        if self.stale_chunks:
            print(f"\ndiscarded {self.stale_chunks} chunk(s) whose echoed stamp did not "
                  "match the pending observation")
        if self.latencies:
            lat = np.asarray(self.latencies)
            print(f"\nNPU latency: mean {lat.mean():.1f} ms  p95 {np.percentile(lat, 95):.1f} ms  "
                  f"-> {1000.0 / lat.mean():.2f} chunks/s")
            motion_s = self.args.horizon / self.args.fps
            print(f"a {self.args.horizon}-step horizon is {motion_s:.1f} s of motion at "
                  f"{self.args.fps} Hz, produced in {lat.mean() / 1000:.2f} s "
                  f"({motion_s / (lat.mean() / 1000):.2f}x real time)")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", default=_repo_path("artifacts/libero_ep0.npz"))
    ap.add_argument("--stats", default="/tmp/libero/stats.json")
    ap.add_argument("--tasks", default="/tmp/libero/tasks.jsonl")
    ap.add_argument("--node", default="/qrb_ros_vla")
    ap.add_argument(
        "--camera-topics",
        nargs="+",
        default=["/vla/camera0/image_raw", "/vla/camera1/image_raw"],
    )
    ap.add_argument("--steps", type=int, default=10, help="replanning steps to run")
    ap.add_argument("--stride", type=int, default=20, help="frames advanced per replan")
    ap.add_argument("--horizon", type=int, default=10, help="chunk steps scored")
    ap.add_argument("--fps", type=float, default=0.0,
                    help="dataset control rate; 0 = take it from the episode")
    ap.add_argument("--state-gap", type=float, default=0.05,
                    help="seconds between publishing state and images, so the node "
                         "cannot fire on a fresh frame with a stale-state prompt")
    args = ap.parse_args()

    if not Path(args.episode).exists():
        sys.exit(
            f"missing {args.episode}; run scripts/fetch-libero-episode.sh then "
            "scripts/prepare-libero-npz.py"
        )

    frames, stats, task, fps = load_episode(args.episode, args.stats, args.tasks)
    if args.fps <= 0:
        args.fps = fps

    rclpy.init()
    node = LiberoReplay(args, frames, stats, task)
    try:
        while rclpy.ok() and not node.done.is_set():
            rclpy.spin_once(node, timeout_sec=0.5)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.report()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
