# -----------------------------------------------------------------------------
#
# Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
# SPDX-License-Identifier: BSD-3-Clause
#
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from makeblock_interfaces.srv import SetMotors


def clamp(val, minimum, maximum):
    assert minimum < maximum
    val = val if val <= maximum else maximum
    val = val if val >= minimum else minimum
    return val


class mBot(Node):
    def __init__(self):
        super().__init__("mbot")

        # fl, fr, rl, rr
        self.motor_indices = [0, 3, 2, 1]
        self.motor_invert = [True, False, True, False]

        self.cmd_vel_sub = self.create_subscription(Twist, "/cmd_vel", self.move, 1)
        self.set_motors_client = self.create_client(SetMotors, "/set_motors")
        while not self.set_motors_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("/set_motors not available, waiting again...")

        self.logger = rclpy.logging.get_logger("logger")

    # convert speed in rad/s to a motor power in range [-255, 255]
    def speed_to_power(self, speed, invert=False):
        # to convert between rad/s and the [-255, 255] range that the mBot firmware
        # expects, we will first approximate that the max speed of mBot is 1.0 m/s
        #  1.0 m | 1 rotation | 2*pi rad
        # ------------------------------- = 33.333 rad/s = max wheel rotational speed
        #   1s  | pi*0.06 m  | 1 rotation
        # now, we assume that the function between motor input and wheel rotation
        # is linear
        #                    255
        # motor value = -------------- * input speed in rad/s
        #                33.333 rad/s
        speed_scale = 255 / 33.333
        motor_power = speed * speed_scale
        motor_power = motor_power if not invert else -motor_power
        return clamp(motor_power, -255, 255)

    def move(self, cmd_vel):
        # use kinematics as described by
        # https://ecam-eurobot.github.io/Tutorials/mechanical/mecanum.html
        # mBot is close enough to a 6.5" x 6.5" square that I make the
        # approximation lx == ly == 6.5" = 0.165 meters
        # mBot wheel radius is ~30mm
        r_inv = 1 / 0.03
        l = 0.165

        x = cmd_vel.linear.x  # m/s
        y = cmd_vel.linear.y  # m/s
        w = cmd_vel.angular.z  # rad/s

        fl = r_inv * (x - y - l * w)  # rad/s
        fr = r_inv * (x + y + l * w)  # rad/s
        rl = r_inv * (x + y - l * w)  # rad/s
        rr = r_inv * (x - y + l * w)  # rad/s
        self.logger.debug(
            f"Setting motors (rad/s): fr: {fl} fr: {fr} rl: {rl} rr: {rr}"
        )

        set_motors_req = SetMotors.Request()
        set_motors_req.index = self.motor_indices
        set_motors_req.power[0] = self.speed_to_power(fl, self.motor_invert[0])
        set_motors_req.power[1] = self.speed_to_power(fr, self.motor_invert[1])
        set_motors_req.power[2] = self.speed_to_power(rl, self.motor_invert[2])
        set_motors_req.power[3] = self.speed_to_power(rr, self.motor_invert[3])

        # TODO we should probably track whether all of these requests are being handled to avoid the queue growing without bound
        self.set_motors_client.call_async(set_motors_req)


def main(args=None):
    rclpy.init(args=args)
    mbot = mBot()
    rclpy.logging.get_logger("logger").info("mbot initialized")
    rclpy.spin(mbot)
    mbot.destroy_node()
    rclpy.logging.get_logger("logger").info("mbot shutdown")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
