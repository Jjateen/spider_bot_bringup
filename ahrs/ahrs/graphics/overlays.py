import numpy as np
import open3d as o3d

from ahrs.graphics.hud import Hud
from ahrs.ros.robot_state import RobotState


class Overlay:
    def __init__(self) -> None:
        self._indicator = o3d.geometry.TriangleMesh.create_sphere(
            radius=0.08
        )
        self._indicator.compute_vertex_normals()
        self._indicator.translate(np.array([-5.0, 3.5, 0.0], dtype=np.float64))
        self._indicator.paint_uniform_color([0.5, 0.5, 0.5])
        self._hud = Hud()

    def update(self, state: RobotState, fps: float, imu_rate: float) -> None:
        # Alarms on live arrival rate (RateTracker decays to 0 when messages
        # stop), not just the one-time imu_connected flag.
        color = [0.0, 1.0, 0.0] if imu_rate > 0.5 else [1.0, 0.0, 0.0]
        self._indicator.paint_uniform_color(color)
        self._hud.update(state, fps, imu_rate)

    @property
    def geometries(self) -> list[o3d.geometry.Geometry]:
        return [self._indicator, self._hud.geometry]
