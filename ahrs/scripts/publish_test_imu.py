#!/usr/bin/env python3
"""
Publish test IMU data on /imu for development and testing.

Generates a slowly oscillating orientation (roll ±30°, pitch ±30°,
yaw 0-360°) plus steady angular velocity and linear acceleration.
Useful for testing the AHRS visualizer without physical hardware.

Usage::

    ros2 run ahrs publish_test_imu

    # Launch config
    ros2 run ahrs publish_test_imu --hz 200 --amp 0.5
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Header
import numpy as np
from scipy.spatial.transform import Rotation


class TestImuPublisher(Node):
    def __init__(self, rate_hz: float = 100.0, amplitude: float = 0.5) -> None:
        super().__init__("test_imu_publisher")
        self._publisher = self.create_publisher(Imu, "/imu", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self._t = 0.0
        self._dt = 1.0 / rate_hz
        self._amp = amplitude
        self.get_logger().info(
            f"Publishing test IMU on /imu at {rate_hz} Hz "
            f"(amplitude={amplitude})"
        )

    def _tick(self) -> None:
        self._t += self._dt

        roll = self._amp * math.sin(self._t * 0.5)
        pitch = self._amp * math.sin(self._t * 0.7)
        yaw = self._t * 0.3

        rot = Rotation.from_euler("xyz", [roll, pitch, yaw])
        q = rot.as_quat()  # [x, y, z, w]

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])

        msg.angular_velocity.x = float(0.1 * math.cos(self._t))
        msg.angular_velocity.y = float(0.1 * math.sin(self._t))
        msg.angular_velocity.z = 0.0

        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81

        msg.orientation_covariance[0] = 0.01
        msg.angular_velocity_covariance[0] = 0.01
        msg.linear_acceleration_covariance[0] = 0.01

        self._publisher.publish(msg)


def main(args: list[str] | None = None) -> None:
    parser = argparse.ArgumentParser(description="Publish test IMU data")
    parser.add_argument("--hz", type=float, default=100.0, help="Publish rate")
    parser.add_argument("--amp", type=float, default=0.5, help="Oscillation amplitude (rad)")
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = TestImuPublisher(rate_hz=parsed.hz, amplitude=parsed.amp)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
