import numpy as np
import open3d as o3d


class CameraController:
    def __init__(
        self,
        initial_position: tuple[float, float, float] = (3.0, 3.0, 3.0),
        look_at: tuple[float, float, float] = (0.0, 0.0, 0.0),
    ) -> None:
        self._initial_position = np.array(initial_position, dtype=np.float64)
        self._look_at = np.array(look_at, dtype=np.float64)

    def reset(self, view_control: o3d.visualization.ViewControl) -> None:
        view_control.set_front(self._initial_position / np.linalg.norm(self._initial_position))
        view_control.set_lookat(self._look_at)
        view_control.set_up([0.0, 0.0, 1.0])
        view_control.set_zoom(0.5)
