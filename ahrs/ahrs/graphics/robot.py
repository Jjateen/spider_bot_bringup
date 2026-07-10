import copy

import numpy as np
import open3d as o3d

from ahrs.graphics.urdf_loader import (
    RobotModel,
    compute_fk,
    load_urdf_robot,
)


class Robot:
    def __init__(self, size: float = 0.3) -> None:
        self._model: RobotModel | None = None
        self._cube_mode = True
        self._size = size
        self._base_rotation = np.eye(3, dtype=np.float64)

        self._geometry = o3d.geometry.TriangleMesh.create_box(
            width=size, height=size, depth=size
        )
        self._geometry.compute_vertex_normals()
        self._geometry.paint_uniform_color([0.0, 0.6, 1.0])

        center_T = np.eye(4, dtype=np.float64)
        center_T[:3, 3] = -size / 2.0
        self._geometry.transform(center_T)

        self._original_vertices = copy.deepcopy(
            np.asarray(self._geometry.vertices, dtype=np.float64)
        )

        self._geometry_ref = o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=size * 0.8
        )
        self._original_ref_vertices = copy.deepcopy(
            np.asarray(self._geometry_ref.vertices, dtype=np.float64)
        )

    @classmethod
    def from_urdf(
        cls, xacro_path: str, mesh_dir: str | None = None
    ) -> "Robot":
        robot = cls.__new__(cls)
        robot._model = load_urdf_robot(xacro_path, mesh_dir)
        robot._cube_mode = False
        robot._size = 0.0
        robot._base_rotation = np.eye(3, dtype=np.float64)

        all_meshes = [lm.mesh for lm in robot._model.links]

        if len(all_meshes) > 0:
            robot._geometry = all_meshes[0]
            bb_min = all_meshes[0].get_min_bound()
            bb_max = all_meshes[0].get_max_bound()
            for m in all_meshes[1:]:
                bb_min = np.minimum(bb_min, m.get_min_bound())
                bb_max = np.maximum(bb_max, m.get_max_bound())
            extent = bb_max - bb_min
            ref_size = max(extent) * 0.3
        else:
            ref_size = 0.3

        robot._geometry_ref = (
            o3d.geometry.TriangleMesh.create_coordinate_frame(size=ref_size)
        )
        robot._original_ref_vertices = copy.deepcopy(
            np.asarray(robot._geometry_ref.vertices, dtype=np.float64)
        )

        return robot

    def update_pose(self, rotation_matrix: np.ndarray) -> None:
        self._base_rotation = rotation_matrix.copy()

        if self._cube_mode:
            rotated = self._original_vertices @ rotation_matrix.T
            np.asarray(self._geometry.vertices)[:] = rotated
            self._geometry.compute_vertex_normals()

            rotated_ref = self._original_ref_vertices @ rotation_matrix.T
            np.asarray(self._geometry_ref.vertices)[:] = rotated_ref

    def update_joints(self, joint_positions: dict[str, float]) -> None:
        if self._cube_mode or self._model is None:
            return

        transforms = compute_fk(
            self._model, joint_positions, self._base_rotation
        )

        for lm in self._model.links:
            if lm.name in transforms:
                T = transforms[lm.name]
                rotated = lm.original_vertices @ T[:3, :3].T + T[:3, 3]
                np.asarray(lm.mesh.vertices)[:] = rotated

        ref_T = transforms.get(
            self._model.root, np.eye(4, dtype=np.float64)
        )
        rotated_ref = self._original_ref_vertices @ ref_T[:3, :3].T + ref_T[:3, 3]
        np.asarray(self._geometry_ref.vertices)[:] = rotated_ref

    @property
    def geometries(self) -> list[o3d.geometry.Geometry]:
        if self._cube_mode:
            return [self._geometry, self._geometry_ref]
        meshes = [lm.mesh for lm in self._model.links]
        meshes.append(self._geometry_ref)
        return meshes

    @property
    def geometry(self) -> o3d.geometry.TriangleMesh:
        if self._cube_mode:
            return self._geometry
        if self._model and len(self._model.links) > 0:
            return self._model.links[0].mesh
        return self._geometry

    @property
    def geometry_ref(self) -> o3d.geometry.TriangleMesh:
        return self._geometry_ref
