import threading
import copy
from dataclasses import dataclass, field

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
            for key, value in kwargs.items():
                if hasattr(self._state, key):
                    setattr(self._state, key, value)

    def read(self) -> RobotState:
        with self._lock:
            return copy.deepcopy(self._state)
