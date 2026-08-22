import numpy as np

from ahrs.graphics.robot_model import RobotModel


def compute_fk(
    model: RobotModel,
    joint_positions: dict[str, float],
    base_rotation: np.ndarray | None = None,
) -> dict[str, np.ndarray]:
    transforms: dict[str, np.ndarray] = {}
    transforms[model.root] = np.eye(4, dtype=np.float64)
    if base_rotation is not None:
        transforms[model.root][:3, :3] = base_rotation

    visited = {model.root}

    while True:
        added = False
        for jd in model.joints:
            if jd.parent in visited and jd.child not in visited:
                parent_T = transforms[jd.parent]
                joint_origin = model.joint_origin_map.get(jd.name, np.eye(4))
                angle = joint_positions.get(jd.name, 0.0)
                axis = model.joint_axis_map.get(jd.name, np.array([1.0, 0.0, 0.0]))

                rot_T = np.eye(4, dtype=np.float64)
                if jd.joint_type != "fixed" and abs(angle) > 1e-12:
                    from scipy.spatial.transform import Rotation
                    R = Rotation.from_rotvec(axis * angle).as_matrix()
                    rot_T[:3, :3] = R

                transforms[jd.child] = parent_T @ joint_origin @ rot_T
                visited.add(jd.child)
                added = True
        if not added:
            break

    return transforms
