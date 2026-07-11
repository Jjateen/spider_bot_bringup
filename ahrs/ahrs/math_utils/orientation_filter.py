import numpy as np
from scipy.spatial.transform import Rotation


class ComplementaryFilter:
    def __init__(self, alpha: float = 0.98):
        self._orientation = Rotation.from_quat([0, 0, 0, 1])
        self._alpha = alpha
        self._initialized = False

    def update(
        self,
        ax: float, ay: float, az: float,
        gx: float, gy: float, gz: float,
        dt: float,
    ) -> np.ndarray:
        rot_vec = np.array([gx, gy, gz], dtype=np.float64) * dt
        delta = Rotation.from_rotvec(rot_vec)
        q_gyro = (self._orientation * delta).as_quat()

        acc_norm = np.sqrt(ax * ax + ay * ay + az * az)
        if acc_norm > 1e-3:
            ax_n, ay_n, az_n = ax / acc_norm, ay / acc_norm, az / acc_norm
            roll = np.arctan2(ay_n, az_n)
            pitch = np.arctan2(-ax_n, np.sqrt(ay_n * ay_n + az_n * az_n))
            yaw = Rotation.from_quat(q_gyro).as_euler('xyz')[2]
            q_accel = Rotation.from_euler('xyz', [roll, pitch, yaw]).as_quat()

            # Seed the orientation from the first good gravity reading so the
            # absolute frame is gravity-referenced rather than startup-pose
            # relative. Yaw is taken from the (identity) gyro at that moment.
            conf = float(np.clip(1.0 - abs(acc_norm - 9.81) / 9.81, 0.0, 1.0))
            if not self._initialized and conf > 0.5:
                self._orientation = Rotation.from_quat(q_accel)
                self._initialized = True
                q_gyro = (self._orientation * delta).as_quat()

            dot = np.dot(q_gyro, q_accel)
            if dot < 0:
                q_accel = -q_accel
                dot = -dot
            # Confidence weight: full tilt correction near 1g, fades as the
            # measured acceleration departs from gravity (dynamic motion),
            # instead of the previous hard on/off gate at |a| in (8, 11).
            t = (1.0 - self._alpha) * conf
            if dot > 0.9995:
                q_fused = q_gyro + t * (q_accel - q_gyro)
            else:
                theta_0 = np.arccos(np.clip(dot, -1.0, 1.0))
                theta = theta_0 * t
                q_perp = q_accel - q_gyro * dot
                q_perp = q_perp / np.linalg.norm(q_perp)
                q_fused = q_gyro * np.cos(theta) + q_perp * np.sin(theta)
            q_fused = q_fused / np.linalg.norm(q_fused)
        else:
            q_fused = q_gyro

        self._orientation = Rotation.from_quat(q_fused)
        return np.array([q_fused[3], q_fused[0], q_fused[1], q_fused[2]])

    def reset(self) -> None:
        self._orientation = Rotation.from_quat([0, 0, 0, 1])
        self._initialized = False
