import struct
import time
import serial

import rclpy
from rclpy.node import Node

from makeblock_interfaces.srv import SetMotors, SetServo


def short2bytes(sval):
    val = struct.pack("h", sval)
    return [val[0], val[1]]


class MegaPi(Node):
    def __init__(self):
        super().__init__("megapi")
        self.serial_file = "/dev/ttyUSB0"
        self.serial_port = None
        self.motor_ports = [2, 3, 10, 11]
        # TODO figure out other servo ports
        self.servo_ports = [69, -1, -1, -1, -1, -1, -1, -1, -1, -1]

        self.set_motors_srv = self.create_service(
            SetMotors, "set_motors", self.set_motors_callback
        )
        self.set_servo_srv = self.create_service(
            SetServo, "set_servo", self.set_servo_callback
        )

        self.logger = rclpy.logging.get_logger("logger")

    def start(self):
        self.serial_port = serial.Serial(self.serial_file, 115200, timeout=10)
        time.sleep(1)

    def stop(self):
        for i in range(len(self.motor_ports)):
            self.set_motor(i, 0)
        self.serial_port.close()

    def write_to_serial_port(self, byte_array):
        self.serial_port.write(byte_array)
        time.sleep(0.01)

    # set motor by index to a certain power
    # assumes range checking and error handling has already been done
    # power in range [-255,255]
    def set_motor(self, index, power):
        motor_port = self.motor_ports[index]
        self.write_to_serial_port(
            bytearray([0xFF, 0x55, 0x6, 0x0, 0x2, 0xA, motor_port] + short2bytes(power))
        )

    def set_motors_callback(self, request, response):
        for i in range(4):
            index = request.index[i]
            if index < 0 or index > 3:
                self.logger.error(f"Tried to set an out-of-range motor index: {index}")
                response.error_code = 1
                return response
            power = request.power[i]
            if power < -255 or power > 255:
                self.logger.error(f"Tried to set an out-of-range motor power: {power}")
                response.error_code = 2
                return response
        for i in range(4):
            index = request.index[i]
            power = request.power[i]
            self.logger.debug(f"Setting motor {index} to {power}")
            self.set_motor(index, power)
        response.error_code = 0
        return response

    # set a servo by index to a certain angle
    # assumes range checking and error handling has already been done
    def set_servo(self, index, angle):
        servo_port = self.servo_ports[index]
        self.write_to_serial_port(
            bytearray([0xFF, 0x55, 0x5, 0x0, 0x2, 33, servo_port, angle])
        )

    def set_servo_callback(self, request, response):
        index = request.index
        if index < 0 or index > (len(self.servo_ports) - 1):
            # TODO enum for error codes would be nice
            self.logger.error(f"Tried to set an out-of-range servo index: {index}")
            response.error_code = 1
            return response

        angle = request.angle
        if angle < 0 or angle > 180:
            self.logger.error(
                f"Tried to set an out-of-range servo angle (degrees): {angle}"
            )
            response.error_code = 2
            return response

        self.logger.debug(f"Setting servo {index} to {angle} degrees")
        self.set_servo(index, angle)
        response.error_code = 0
        return response


def main(args=None):
    rclpy.init(args=args)
    megapi = MegaPi()
    megapi.start()
    rclpy.logging.get_logger("logger").info("megapi initialized")

    rclpy.spin(megapi)
    megapi.stop()
    megapi.destroy_node()
    rclpy.logging.get_logger("logger").info("megapi shutdown")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
