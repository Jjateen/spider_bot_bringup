import numpy as np
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from ahrs.math_utils import G
from ahrs.math_utils.quaternion import quaternion_to_matrix_and_euler
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
            # Orientation is now supplied by the upstream filter (imu_filter_madgwick)
            # on /filtered/imu — we no longer estimate it here.
            # ROS msg uses [x, y, z, w]; the AHRS codebase uses [w, x, y, z]
            # (see robot_state.py default, orientation_filter.py return, and
            # test_orientation_filter.py _quat_wxyz helper).
            q = np.array(
                [
                    msg.orientation.w,
                    msg.orientation.x,
                    msg.orientation.y,
                    msg.orientation.z,
                ],
                dtype=np.float64,
            )
            if not np.all(np.isfinite(q)) or np.linalg.norm(q) < 1e-6:
                self.get_logger().warn("IMU orientation missing/invalid; skipping")
                return

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
                la = la * G

            av = av - self._gyro_bias
            la = la - self._accel_bias

            R, roll, pitch, yaw = quaternion_to_matrix_and_euler(q)

            ts = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9

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
