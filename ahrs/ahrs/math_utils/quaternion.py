import warnings

import numpy as np
from scipy.spatial.transform import Rotation


def normalize_quaternion(q: np.ndarray) -> np.ndarray:
    if q.shape != (4,):
        return np.array([1.0, 0.0, 0.0, 0.0])
    norm = np.linalg.norm(q)
    if norm < 1e-12:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return q / norm


def validate_quaternion(q: np.ndarray) -> bool:
    if q.shape != (4,):
        return False
    norm = np.linalg.norm(q)
    return abs(norm - 1.0) < 1e-6


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    q = normalize_quaternion(q)
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    return rot.as_matrix()


def quaternion_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    q = normalize_quaternion(q)
    rot = Rotation.from_quat([q[1], q[2], q[3], q[0]])
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return tuple(rot.as_euler("xyz", degrees=False))
