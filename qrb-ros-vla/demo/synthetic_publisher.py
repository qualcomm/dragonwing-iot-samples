#!/usr/bin/env python3
"""Drive qrb_ros_vla without a robot, an arm, or a simulator.

This is the armless smoke path: it publishes the exact topic set the VLA node
consumes -- N camera streams, normalized proprioceptive state, and a language
task -- then reports the action chunks that come back. Use it to prove the
NPU pipeline is live end to end before adding Gazebo or a real arm.

Frames are synthetic but not noise: a moving coloured block on a fixed
background, so successive chunks see genuinely different observations and a
frozen output is obvious.

Usage (with ROS 2 Jazzy and the workspace overlay sourced):
  python3 demo/synthetic_publisher.py --rate 1.0 --task "pick up the black bowl"
"""

from __future__ import annotations

import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, String

from qrb_ros_vla_msgs.msg import ActionChunk, InferenceStats

IMAGE_SIZE = 224


def synth_frame(t: float, cam: int) -> np.ndarray:
    """A moving block on a per-camera background, uint8 RGB HxWx3."""
    img = np.zeros((IMAGE_SIZE, IMAGE_SIZE, 3), dtype=np.uint8)
    img[:, :, cam % 3] = 40  # distinguish the cameras
    # Table-ish horizon so the frame is not degenerate.
    img[IMAGE_SIZE // 2 :, :, :] = 90
    # Moving block.
    cx = int((0.5 + 0.35 * math.sin(t + cam)) * IMAGE_SIZE)
    cy = int((0.6 + 0.15 * math.cos(t * 0.7 + cam)) * IMAGE_SIZE)
    half = 22
    x0, x1 = max(0, cx - half), min(IMAGE_SIZE, cx + half)
    y0, y1 = max(0, cy - half), min(IMAGE_SIZE, cy + half)
    img[y0:y1, x0:x1] = (220, 40, 40)
    return img


class SyntheticPublisher(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("vla_synthetic_publisher")
        self.args = args
        self.t0 = time.monotonic()
        self.chunks = 0

        self.image_pubs = [
            self.create_publisher(Image, topic, 10) for topic in args.camera_topics
        ]
        self.state_pub = self.create_publisher(Float32MultiArray, f"{args.node}/state", 10)
        self.task_pub = self.create_publisher(String, f"{args.node}/task", 10)

        self.create_subscription(
            ActionChunk, f"{args.node}/action_chunk", self.on_chunk, 10
        )
        self.create_subscription(InferenceStats, f"{args.node}/stats", self.on_stats, 10)

        # Latch the task a moment after startup so the node's subscription exists.
        self.task_timer = self.create_timer(1.0, self.publish_task)
        self.frame_timer = self.create_timer(1.0 / args.rate, self.publish_frame)
        self.get_logger().info(
            f"publishing {len(self.image_pubs)} camera streams at {args.rate} Hz "
            f"to {args.camera_topics}"
        )

    def publish_task(self) -> None:
        msg = String()
        msg.data = self.args.task
        self.task_pub.publish(msg)
        self.get_logger().info(f"task -> '{self.args.task}'")
        self.task_timer.cancel()

    def publish_frame(self) -> None:
        t = time.monotonic() - self.t0
        stamp = self.get_clock().now().to_msg()
        for cam, pub in enumerate(self.image_pubs):
            frame = synth_frame(t, cam)
            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = f"cam{cam}"
            msg.height, msg.width = IMAGE_SIZE, IMAGE_SIZE
            msg.encoding = "rgb8"
            msg.is_bigendian = 0
            msg.step = IMAGE_SIZE * 3
            msg.data = frame.tobytes()
            pub.publish(msg)

        # 8-DoF LIBERO-shaped state, already normalized to [-1, 1].
        state = Float32MultiArray()
        state.data = [
            float(0.6 * math.sin(t * 0.5 + i)) for i in range(self.args.state_dim)
        ]
        self.state_pub.publish(state)

    def on_chunk(self, msg: ActionChunk) -> None:
        self.chunks += 1
        actions = np.asarray(msg.actions, dtype=np.float32)
        if msg.action_dim > 0:
            grid = actions.reshape(msg.chunk_size, msg.action_dim)
            dof = grid[:, : msg.action_dof] if msg.action_dof else grid
        else:
            dof = actions
        self.get_logger().info(
            f"chunk #{self.chunks} task='{msg.task}' "
            f"{msg.chunk_size}x{msg.action_dof}dof "
            f"first={np.array2string(dof[0], precision=3, suppress_small=True)} "
            f"range=[{dof.min():.3f}, {dof.max():.3f}]"
        )

    def on_stats(self, msg: InferenceStats) -> None:
        self.get_logger().info(
            f"  {msg.backend}: vision={msg.vision_encoder_ms:.1f} "
            f"token_emb={msg.token_emb_ms:.1f} backbone={msg.backbone_ms:.1f} "
            f"expert={msg.action_expert_ms:.1f} pack={msg.host_packing_ms:.1f} "
            f"total={msg.total_ms:.1f} ms ({msg.chunks_per_second:.2f} chunk/s)"
        )


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--node", default="/qrb_ros_vla", help="VLA node namespace")
    ap.add_argument(
        "--camera-topics",
        nargs="+",
        default=["/vla/camera0/image_raw", "/vla/camera1/image_raw"],
    )
    ap.add_argument("--task", default="pick up the black bowl and place it on the plate")
    ap.add_argument("--rate", type=float, default=2.0, help="camera publish rate (Hz)")
    ap.add_argument("--state-dim", type=int, default=8)
    args = ap.parse_args()

    rclpy.init()
    node = SyntheticPublisher(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
