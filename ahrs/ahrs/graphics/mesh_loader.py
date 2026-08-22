import os

import numpy as np
import open3d as o3d

from ahrs.utils.logger import setup_logger

logger = setup_logger("ahrs.mesh")


def parse_scale(scale_str: str | None) -> float:
    if scale_str is None:
        return 1.0
    parts = scale_str.strip().split()
    return float(parts[0])


def load_mesh(
    filename: str, scale: float, default_color: tuple[float, float, float]
) -> o3d.geometry.TriangleMesh | None:
    if not os.path.exists(filename):
        logger.warning(f"Mesh not found: {filename}")
        return None
    mesh = o3d.io.read_triangle_mesh(filename)
    if len(mesh.vertices) == 0:
        logger.warning(f"Empty mesh: {filename}")
        return None
    if scale != 1.0:
        mesh.scale(scale, center=np.zeros(3))
    mesh.compute_vertex_normals()
    mesh.paint_uniform_color(default_color)
    return mesh
