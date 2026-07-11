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
        alpha: float = 0.5,
        gyro_bias: np.ndarray | None = None,
        accel_bias: np.ndarray | None = None,
        imu_units: dict | None = None,
        use_sim_time: bool = False,
    ) -> None:
        # use_sim_time is auto-declared by rclpy on every node and is supplied
        # to the whole process via the launch file's `-p use_sim_time:=...`,
        # so an explicit override here is unnecessary (and rclpy requires
        # Parameter objects, not tuples, for parameter_overrides).
        super().__init__("imu_subscriber")
        self._state = state
        self._rate_tracker = rate_tracker
        self._filter = ComplementaryFilter(alpha=alpha)
        self.get_logger().info(f"Complementary filter alpha={alpha}")

        self._gyro_bias = (
            np.asarray(gyro_bias, dtype=np.float64)
            if gyro_bias is not None
            else np.zeros(3, dtype=np.float64)
        )
        self._accel_bias = (
            np.asarray(accel_bias, dtype=np.float64)
            if accel_bias is not None
            else np.zeros(3, dtype=np.float64)
        )
        units = imu_units or {}
        self._ang_unit = units.get("ang", "rad_s")
        self._lin_unit = units.get("lin", "m_s2")
        self._warned_zero = False

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
        try:
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

            # Reject non-finite samples so a single bad reading cannot
            # poison the orientation quaternion and NaN-out the mesh.
            if not (np.all(np.isfinite(av)) and np.all(np.isfinite(la))):
                self.get_logger().warn("IMU sample contained NaN/Inf; skipping")
                return
            if np.allclose(la, 0.0) and np.allclose(av, 0.0):
                if not self._warned_zero:
                    self.get_logger().warn(
                        "IMU all-zero sample; sensor may be missing"
                    )
                    self._warned_zero = True
                return

            # Unit conversion to ROS-standard rad/s and m/s^2.
            if self._ang_unit == "deg_s":
                av = np.deg2rad(av)
            if self._lin_unit == "g":
                la = la * 9.81

            av = av - self._gyro_bias
            la = la - self._accel_bias

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
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"IMU callback failed: {e}")
