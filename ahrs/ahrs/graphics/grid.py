import numpy as np
import open3d as o3d


class Grid:
    def __init__(self, size: float = 10.0, divisions: int = 20) -> None:
        self._geometry = self._build(size, divisions)

    @staticmethod
    def _build(size: float, divisions: int) -> o3d.geometry.LineSet:
        half = size / 2.0
        step = size / divisions
        points: list[list[float]] = []
        lines: list[tuple[int, int]] = []

        for i in range(divisions + 1):
            pos = -half + i * step
            p_start = len(points)
            points.append([pos, -half, 0.0])
            points.append([pos, half, 0.0])
            lines.append((p_start, p_start + 1))

            p_start = len(points)
            points.append([-half, pos, 0.0])
            points.append([half, pos, 0.0])
            lines.append((p_start, p_start + 1))

        line_set = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(np.array(points, dtype=np.float64)),
            lines=o3d.utility.Vector2iVector(np.array(lines, dtype=np.int32)),
        )
        line_set.paint_uniform_color([0.3, 0.3, 0.3])
        return line_set

    @property
    def geometry(self) -> o3d.geometry.LineSet:
        return self._geometry
