import threading

import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import JointState


class SharedJointState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._positions: dict[str, float] = {}

    def update(self, positions: dict[str, float]) -> None:
        with self._lock:
            self._positions.update(positions)

    def read(self) -> dict[str, float]:
        with self._lock:
            return dict(self._positions)


class JointStateSubscriber(Node):
    def __init__(
        self,
        topic: str,
        state: SharedJointState,
    ) -> None:
        super().__init__("joint_state_subscriber")
        self._state = state
        self._subscription = self.create_subscription(
            JointState, topic, self._callback, 10
        )
        self.get_logger().info(f"Subscribed to joint states: {topic}")

    def _callback(self, msg: JointState) -> None:
        positions = dict(zip(msg.name, msg.position))
        self._state.update(positions)
