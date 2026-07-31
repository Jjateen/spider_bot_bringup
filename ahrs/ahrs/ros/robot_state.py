import threading
from dataclasses import dataclass, field, fields

import numpy as np


@dataclass
class RobotState:
    quaternion: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0, 0.0, 0.0]))
    rotation_matrix: np.ndarray = field(default_factory=lambda: np.eye(3, dtype=np.float64))
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    angular_velocity: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    linear_acceleration: np.ndarray = field(default_factory=lambda: np.zeros(3, dtype=np.float64))
    timestamp: float = 0.0
    imu_connected: bool = False


class SharedRobotState:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._state = RobotState()

    def update(self, **kwargs) -> None:
        with self._lock:
            current = self._state
            new_state = RobotState()
            for f in fields(RobotState):
                if f.name in kwargs:
                    val = kwargs[f.name]
                else:
                    val = getattr(current, f.name)
                if isinstance(val, np.ndarray):
                    val = val.copy()
                setattr(new_state, f.name, val)
            self._state = new_state

    def read(self) -> RobotState:
        with self._lock:
            state = self._state
        return RobotState(
            quaternion=state.quaternion.copy(),
            rotation_matrix=state.rotation_matrix.copy(),
            roll=state.roll,
            pitch=state.pitch,
            yaw=state.yaw,
            angular_velocity=state.angular_velocity.copy(),
            linear_acceleration=state.linear_acceleration.copy(),
            timestamp=state.timestamp,
            imu_connected=state.imu_connected,
        )
