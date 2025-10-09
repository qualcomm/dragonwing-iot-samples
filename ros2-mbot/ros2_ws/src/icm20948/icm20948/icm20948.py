from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_icm20x

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu, MagneticField


class ICM20948(Node):
    def __init__(self):
        super().__init__("icm20948")
        # TODO hardcoded imu i2c bus and address
        self.imu = adafruit_icm20x.ICM20948(I2C(1))
        self.imu_publisher = self.create_publisher(Imu, "imu/data_raw", 10)
        self.mag_publisher = self.create_publisher(MagneticField, "imu/mag", 10)
        timer_frequency = 50  # hz
        self.timer = self.create_timer(1 / timer_frequency, self.timer_callback)

    def timer_callback(self):
        # https://docs.circuitpython.org/projects/icm20x/en/latest/api.html
        acc = self.imu.acceleration
        gyro = self.imu.gyro
        mag = self.imu.magnetic

        timestamp = self.get_clock().now().to_msg()

        imu_msg = Imu()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.header.stamp = timestamp
        # flip y and z axes for ros conventions w.r.t our robot
        # TODO this transformation should be done elsewhere

        # returned values are already in rad/sec, no need to convert
        imu_msg.angular_velocity.x = gyro[0]
        imu_msg.angular_velocity.y = -gyro[1]
        imu_msg.angular_velocity.z = -gyro[2]
        # returned values are already in m/s^2, no need to convert
        imu_msg.linear_acceleration.x = acc[0]
        imu_msg.linear_acceleration.y = -acc[1]
        imu_msg.linear_acceleration.z = -acc[2]

        mag_msg = MagneticField()
        mag_msg.header.frame_id = "imu_link"
        mag_msg.header.stamp = timestamp
        # returned values in uT, need to convert to T
        mag_msg.magnetic_field.x = mag[0] / 1000000
        mag_msg.magnetic_field.y = -mag[1] / 1000000
        mag_msg.magnetic_field.z = -mag[2] / 1000000

        self.imu_publisher.publish(imu_msg)
        self.mag_publisher.publish(mag_msg)


def main(args=None):
    rclpy.init(args=args)
    icm20948 = ICM20948()
    rclpy.spin(icm20948)
    icm20948.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
