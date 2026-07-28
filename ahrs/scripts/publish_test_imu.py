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

import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import Imu
from std_msgs.msg import Header


class TestImuPublisher(Node):
    def __init__(self, rate_hz: float = 100.0, amplitude: float = 0.5) -> None:
        super().__init__("test_imu_publisher")
        self._publisher = self.create_publisher(Imu, "/imu", 10)
        self._timer = self.create_timer(1.0 / rate_hz, self._tick)
        self._t = 0.0
        self._dt = 1.0 / rate_hz
        self._amp = amplitude
        self._prev_q = None
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

        # Specific force at rest: gravity expressed in the body frame. This is
        # physically consistent with the orientation above, so an AHRS that
        # fuses raw gyro+accel (like our visualizer) reproduces this motion
        # instead of sitting flat.
        la = rot.inv().apply(np.array([0.0, 0.0, 9.81]))

        # Body-frame angular velocity from the orientation delta.
        if self._prev_q is None:
            av = np.array([0.0, 0.0, 0.0])
        else:
            dq = self._prev_q.inv() * rot
            av = dq.as_rotvec() / self._dt
        self._prev_q = rot

        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        msg.orientation.x = float(q[0])
        msg.orientation.y = float(q[1])
        msg.orientation.z = float(q[2])
        msg.orientation.w = float(q[3])

        msg.angular_velocity.x = float(av[0])
        msg.angular_velocity.y = float(av[1])
        msg.angular_velocity.z = float(av[2])

        msg.linear_acceleration.x = float(la[0])
        msg.linear_acceleration.y = float(la[1])
        msg.linear_acceleration.z = float(la[2])

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
