# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import launch
import launch_ros.actions


def generate_launch_description():
    mbot_launch = launch_ros.actions.IncludeLaunchDescription(
        package="makeblock", launch="mbot_launch.py"
    )
    return launch.LaunchDescription(
        [
            mbot_launch,
            Node(package="icm20948", executable="icm20948", name="icm20948_node"),
            Node(
                package="imu_complementary_filter",
                executable="complementary_filter_node",
                name="imu_filter_node",
            ),
            launch_ros.actions.Node(
                package="teleop_twist_keyboard",
                executable="teleop_twist_keyboard",
                name="keyboard_node",
            ),
        ]
    )
