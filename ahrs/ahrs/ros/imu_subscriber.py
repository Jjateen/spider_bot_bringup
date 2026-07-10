import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np

from ahrs.math_utils.orientation_filter import ComplementaryFilter
from ahrs.math_utils.quaternion import (
    quaternion_to_euler,
    quaternion_to_rotation_matrix,
)
from ahrs.ros.robot_state import SharedRobotState
from ahrs.utils.timing import RateTracker


class ImuSubscriber(Node):
    def __init__(
        self,
        topic: str,
        state: SharedRobotState,
        rate_tracker: RateTracker,
    ) -> None:
        super().__init__("imu_subscriber")
        self._state = state
        self._rate_tracker = rate_tracker
        self._filter = ComplementaryFilter(alpha=0.98)
        self._prev_ts: float | None = None
        sensor_qos = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self._subscription = self.create_subscription(
            Imu, topic, self._callback, sensor_qos
        )
        self.get_logger().info(f"Subscribed to IMU topic: {topic}")

    def _callback(self, msg: Imu) -> None:
        av = np.array(
            [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
            dtype=np.float64,
        )
        la = np.array(
            [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            dtype=np.float64,
        )
        ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

        if self._prev_ts is not None and ts > self._prev_ts:
            dt = ts - self._prev_ts
        else:
            dt = 0.01
        self._prev_ts = ts

        q = self._filter.update(
            la[0], la[1], la[2],
            av[0], av[1], av[2],
            dt,
        )
        R = quaternion_to_rotation_matrix(q)
        roll, pitch, yaw = quaternion_to_euler(q)

        self._state.update(
            quaternion=q,
            rotation_matrix=R,
            roll=roll,
            pitch=pitch,
            yaw=yaw,
            angular_velocity=av,
            linear_acceleration=la,
            timestamp=ts,
            imu_connected=True,
        )
        self._rate_tracker.tick()
