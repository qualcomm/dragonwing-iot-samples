# Copyright (c) 2026 QRB ROS VLA contributors.
# SPDX-License-Identifier: BSD-3-Clause
"""Bring up Pi0.5 on the Hexagon NPUs as a ROS 2 node.

One command, no arguments needed:

    ros2 launch qrb_ros_vla vla.launch.py

Defaults assume the repository layout (artifacts/ next to ros2_ws/). Override any
of them on the command line, e.g.

    ros2 launch qrb_ros_vla vla.launch.py task:="pick up the red block"
    ros2 launch qrb_ros_vla vla.launch.py htp_device_ids:="[0,0,0,0]"
"""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _default_repo_root() -> str:
    """Walk up from the installed share dir to the repository root.

    share/qrb_ros_vla -> share -> install/qrb_ros_vla -> install -> ros2_ws -> repo
    """
    share = Path(get_package_share_directory("qrb_ros_vla")).resolve()
    for parent in share.parents:
        if (parent / "artifacts").is_dir():
            return str(parent)
    return str(Path.home() / "QRB-ROS-VLA")


def generate_launch_description() -> LaunchDescription:
    repo = _default_repo_root()

    args = [
        DeclareLaunchArgument(
            "bundle_dir",
            default_value=f"{repo}/artifacts/pi05-qnn_context_binary-mixed-qualcomm_qcs9075",
            description="Directory holding the four Pi0.5 .bin context binaries",
        ),
        DeclareLaunchArgument(
            "tokenizer_model",
            default_value=f"{repo}/artifacts/paligemma_tokenizer.model",
            description="PaliGemma SentencePiece model (scripts/fetch-tokenizer.sh)",
        ),
        DeclareLaunchArgument(
            "task",
            default_value="pick up the black bowl and place it on the plate",
            description="Initial language instruction",
        ),
        DeclareLaunchArgument(
            "camera_topics",
            default_value="['/vla/camera0/image_raw', '/vla/camera1/image_raw']",
            description="Camera topics to subscribe to (1..3)",
        ),
        DeclareLaunchArgument(
            "htp_device_ids",
            default_value="[1, 1, 0, 0]",
            description="HTP device per component: vision, token_emb, backbone, action_expert",
        ),
        DeclareLaunchArgument(
            "denoise_steps", default_value="10",
            description="Flow-matching denoise steps; fewer is faster, less accurate",
        ),
        DeclareLaunchArgument(
            "action_dof", default_value="7",
            description="Real degrees of freedom for this embodiment (LIBERO Panda = 7)",
        ),
        DeclareLaunchArgument(
            "burst", default_value="true",
            description="Lock the HTP to burst/TURBO performance mode",
        ),
    ]

    return LaunchDescription(
        args
        + [
            # Without this the HTP runs at a default DCVS level and every stage is
            # slower. Upstream qrb_inference_manager reads it at context creation.
            SetEnvironmentVariable(
                "QNN_HTP_BURST",
                PythonExpression(
                    ["'1' if '", LaunchConfiguration("burst"), "'.lower() == 'true' else '0'"]
                ),
            ),
            Node(
                package="qrb_ros_vla",
                executable="vla_node",
                name="qrb_ros_vla",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {
                        "bundle_dir": LaunchConfiguration("bundle_dir"),
                        "tokenizer_model": LaunchConfiguration("tokenizer_model"),
                        "default_task": LaunchConfiguration("task"),
                        "camera_topics": LaunchConfiguration("camera_topics"),
                        "htp_device_ids": LaunchConfiguration("htp_device_ids"),
                        "denoise_steps": LaunchConfiguration("denoise_steps"),
                        "action_dof": LaunchConfiguration("action_dof"),
                    }
                ],
            ),
        ]
    )
