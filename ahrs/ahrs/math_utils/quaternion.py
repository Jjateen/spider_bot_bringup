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


def _to_scipy_rotation(q: np.ndarray) -> Rotation:
    q = normalize_quaternion(q)
    return Rotation.from_quat([q[1], q[2], q[3], q[0]])


def quaternion_to_rotation_matrix(q: np.ndarray) -> np.ndarray:
    return _to_scipy_rotation(q).as_matrix()


def quaternion_to_matrix_and_euler(
    q: np.ndarray,
) -> tuple[np.ndarray, float, float, float]:
    rot = _to_scipy_rotation(q)
    R = rot.as_matrix()
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        roll, pitch, yaw = rot.as_euler("xyz", degrees=False)
    return R, roll, pitch, yaw


def quaternion_to_euler(q: np.ndarray) -> tuple[float, float, float]:
    rot = _to_scipy_rotation(q)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", UserWarning)
        return tuple(rot.as_euler("xyz", degrees=False))
