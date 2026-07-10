import threading

import rclpy

from ahrs.ros.imu_subscriber import ImuSubscriber
from ahrs.ros.joint_state_subscriber import (
    JointStateSubscriber,
    SharedJointState,
)
from ahrs.ros.robot_state import SharedRobotState
from ahrs.utils.timing import RateTracker


class ROSNode:
    def __init__(
        self,
        imu_topic: str,
        imu_state: SharedRobotState,
        joint_state: SharedJointState | None = None,
        joint_topic: str = "/joint_states",
    ) -> None:
        rclpy.init(args=None)
        self._rate_tracker = RateTracker()
        self._imu_node = ImuSubscriber(imu_topic, imu_state, self._rate_tracker)

        self._joint_state = joint_state
        self._joint_node: JointStateSubscriber | None = None
        if joint_state is not None:
            self._joint_node = JointStateSubscriber(joint_topic, joint_state)

        self._spin_thread: threading.Thread | None = None

    @property
    def rate_hz(self) -> float:
        return self._rate_tracker.rate_hz

    def start(self) -> None:
        self._spin_thread = threading.Thread(
            target=self._spin, daemon=True
        )
        self._spin_thread.start()

    def _spin(self) -> None:
        while rclpy.ok():
            rclpy.spin_once(self._imu_node, timeout_sec=0.005)
            if self._joint_node is not None:
                rclpy.spin_once(self._joint_node, timeout_sec=0.0)

    def shutdown(self) -> None:
        self._imu_node.destroy_node()
        if self._joint_node is not None:
            self._joint_node.destroy_node()
        rclpy.shutdown()
