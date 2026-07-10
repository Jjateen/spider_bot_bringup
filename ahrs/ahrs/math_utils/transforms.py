import numpy as np


def identity_rotation() -> np.ndarray:
    return np.eye(3, dtype=np.float64)


def rotation_from_axis_angle(
    axis: np.ndarray, angle: float
) -> np.ndarray:
    from scipy.spatial.transform import Rotation

    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    rot = Rotation.from_rotvec(axis * angle)
    return rot.as_matrix()
