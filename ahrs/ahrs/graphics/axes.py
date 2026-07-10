import numpy as np
import open3d as o3d


class Axes:
    def __init__(self, size: float = 0.5, origin: tuple[float, float, float] = (0.0, 0.0, 0.0)) -> None:
        self._geometry = self._build(size, origin)

    @staticmethod
    def _build(size: float, origin: tuple[float, float, float]) -> o3d.geometry.LineSet:
        ox, oy, oz = origin
        points = np.array(
            [
                [ox, oy, oz],
                [ox + size, oy, oz],
                [ox, oy, oz],
                [ox, oy + size, oz],
                [ox, oy, oz],
                [ox, oy, oz + size],
            ],
            dtype=np.float64,
        )
        lines = np.array([[0, 1], [2, 3], [4, 5]], dtype=np.int32)
        colors = np.array(
            [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        line_set.colors = o3d.utility.Vector3dVector(colors)
        return line_set

    @property
    def geometry(self) -> o3d.geometry.LineSet:
        return self._geometry
