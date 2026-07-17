import copy

import numpy as np
import open3d as o3d

from ahrs.graphics.robot_model import RobotModel
from ahrs.graphics.fk import compute_fk


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

        self._combined_mesh: o3d.geometry.TriangleMesh | None = None
        self._per_link_ranges: list[tuple[int, int, int, int]] = []
        self._total_vertices: int = 0
        self._last_joint_positions: dict[str, float] = {}

    @classmethod
    def from_model(cls, model: RobotModel) -> "Robot":
        robot = cls.__new__(cls)
        robot._model = model
        robot._cube_mode = False
        robot._size = 0.0
        robot._base_rotation = np.eye(3, dtype=np.float64)

        all_vertices = []
        all_triangles = []
        all_colors = []
        per_link_ranges = []
        vert_offset = 0
        tri_offset = 0

        for lm in model.links:
            verts = np.asarray(lm.mesh.vertices, dtype=np.float64)
            tris = np.asarray(lm.mesh.triangles, dtype=np.int32)
            cols = np.asarray(lm.mesh.vertex_colors, dtype=np.float64)

            n_verts = len(verts)
            n_tris = len(tris)

            if len(cols) == 0:
                cols = np.tile(np.array(lm.color, dtype=np.float64), (n_verts, 1))

            per_link_ranges.append(
                (vert_offset, vert_offset + n_verts, tri_offset, tri_offset + n_tris)
            )

            all_vertices.append(verts)
            all_triangles.append(tris + vert_offset)
            all_colors.append(cols)

            vert_offset += n_verts
            tri_offset += n_tris

        combined = o3d.geometry.TriangleMesh()
        combined.vertices = o3d.utility.Vector3dVector(np.vstack(all_vertices))
        combined.triangles = o3d.utility.Vector3iVector(np.vstack(all_triangles))
        combined.vertex_colors = o3d.utility.Vector3dVector(np.vstack(all_colors))
        combined.compute_vertex_normals()

        robot._combined_mesh = combined
        robot._per_link_ranges = per_link_ranges
        robot._total_vertices = vert_offset
        robot._geometry = combined
        robot._original_vertices = np.empty((0, 3), dtype=np.float64)
        robot._last_joint_positions = {}

        if len(model.links) > 0:
            bb_min = combined.get_min_bound()
            bb_max = combined.get_max_bound()
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
            self._geometry.vertices = o3d.utility.Vector3dVector(rotated)
            self._geometry.compute_vertex_normals()

            rotated_ref = self._original_ref_vertices @ rotation_matrix.T
            self._geometry_ref.vertices = o3d.utility.Vector3dVector(rotated_ref)
        else:
            # In URDF mode the base attitude must be applied even when the
            # joint-state path is empty/broken, so apply it here directly
            # instead of relying solely on update_joints.
            self._apply_fk(self._last_joint_positions)

    def update_joints(self, joint_positions: dict[str, float]) -> None:
        if self._cube_mode or self._model is None:
            return
        self._last_joint_positions = dict(joint_positions)
        self._apply_fk(joint_positions)

    def _apply_fk(self, joint_positions: dict[str, float]) -> None:
        if self._combined_mesh is None or self._model is None:
            return

        transforms = compute_fk(
            self._model, joint_positions, self._base_rotation
        )

        full_vertices = np.empty((self._total_vertices, 3), dtype=np.float64)
        for i, lm in enumerate(self._model.links):
            if lm.name in transforms:
                T = transforms[lm.name]
                v_start, v_end, _, _ = self._per_link_ranges[i]
                rotated = lm.original_vertices @ T[:3, :3].T + T[:3, 3]
                full_vertices[v_start:v_end] = rotated

        self._combined_mesh.vertices = o3d.utility.Vector3dVector(full_vertices)
        self._combined_mesh.compute_vertex_normals()

        ref_T = transforms.get(
            self._model.root, np.eye(4, dtype=np.float64)
        )
        rotated_ref = self._original_ref_vertices @ ref_T[:3, :3].T + ref_T[:3, 3]
        self._geometry_ref.vertices = o3d.utility.Vector3dVector(rotated_ref)

    @property
    def geometries(self) -> list[o3d.geometry.Geometry]:
        if self._cube_mode:
            return [self._geometry, self._geometry_ref]
        return [self._combined_mesh, self._geometry_ref]

    @property
    def geometry(self) -> o3d.geometry.TriangleMesh:
        if self._cube_mode:
            return self._geometry
        if self._combined_mesh is not None:
            return self._combined_mesh
        return self._geometry

    @property
    def geometry_ref(self) -> o3d.geometry.TriangleMesh:
        return self._geometry_ref
