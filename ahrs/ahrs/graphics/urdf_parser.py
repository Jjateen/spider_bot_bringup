import os
import xml.etree.ElementTree as ET

import numpy as np
import open3d as o3d

from ahrs.graphics.robot_model import RobotModel, LinkMesh
from ahrs.graphics.mesh_loader import parse_scale, load_mesh
from ahrs.utils.logger import setup_logger

logger = setup_logger("ahrs.urdf")


def _resolve_package_path(package_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)


def _parse_vec3(s: str) -> np.ndarray:
    parts = s.strip().split()
    return np.array([float(p) for p in parts], dtype=np.float64)


def rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", rpy).as_matrix()


def _compose_transform(xyz: np.ndarray, rpy: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def load_urdf_from_string(urdf_xml: str) -> RobotModel:
    root = ET.fromstring(urdf_xml)

    robot_name = root.get("name", "robot")
    logger.info(f"Parsing URDF robot: {robot_name}")

    model = RobotModel()

    for link_elem in root.findall("link"):
        name = link_elem.get("name")
        if name is None:
            continue

        for visual in link_elem.findall("visual"):
            origin_elem = visual.find("origin")
            xyz = np.zeros(3, dtype=np.float64)
            rpy = np.zeros(3, dtype=np.float64)
            if origin_elem is not None:
                if origin_elem.get("xyz"):
                    xyz = _parse_vec3(origin_elem.get("xyz"))
                if origin_elem.get("rpy"):
                    rpy = _parse_vec3(origin_elem.get("rpy"))
            visual_T = _compose_transform(xyz, rpy)

            color = (0.7, 0.7, 0.7)
            mat = visual.find("material")
            if mat is not None:
                col = mat.find("color")
                if col is not None:
                    rgba = col.get("rgba", "").strip().split()
                    if len(rgba) >= 3:
                        color = (float(rgba[0]), float(rgba[1]), float(rgba[2]))

            geom = visual.find("geometry")
            if geom is None:
                continue

            mesh = None

            box_elem = geom.find("box")
            if box_elem is not None:
                size_str = box_elem.get("size", "0.01 0.01 0.01")
                parts = size_str.strip().split()
                w, h, d = float(parts[0]), float(parts[1]), float(parts[2])
                if w > 0 and h > 0 and d > 0:
                    box = o3d.geometry.TriangleMesh.create_box(w, h, d)
                    box.compute_vertex_normals()
                    box.paint_uniform_color(color)
                    box.translate(-np.array([w / 2, h / 2, d / 2]))
                    mesh = box

            if mesh is None:
                mesh_elem = geom.find("mesh")
                if mesh_elem is not None:
                    mesh_path = mesh_elem.get("filename", "")
                    scale = parse_scale(mesh_elem.get("scale"))

                    actual_path = mesh_path
                    if mesh_path.startswith("package://"):
                        pkg_part = mesh_path[len("package://"):]
                        pkg_name, rel_path = pkg_part.split("/", 1)
                        pkg_dir = _resolve_package_path(pkg_name)
                        actual_path = os.path.join(pkg_dir, rel_path)

                    mesh = load_mesh(actual_path, scale, color)

            if mesh is None:
                continue

            vertices = np.asarray(mesh.vertices, dtype=np.float64)
            vertices[:] = vertices @ visual_T[:3, :3].T + visual_T[:3, 3]
            mesh.compute_vertex_normals()

            lm = LinkMesh(
                name=name,
                mesh=mesh,
                original_vertices=np.asarray(mesh.vertices, dtype=np.float64).copy(),
                color=color,
            )
            model.links.append(lm)

    logger.info(f"Loaded {len(model.links)} link meshes")

    for joint_elem in root.findall("joint"):
        name = joint_elem.get("name")
        joint_type = joint_elem.get("type", "fixed")
        origin_elem = joint_elem.find("origin")
        parent_elem = joint_elem.find("parent")
        child_elem = joint_elem.find("child")
        axis_elem = joint_elem.find("axis")

        if parent_elem is None or child_elem is None:
            continue

        parent = parent_elem.get("link")
        child = child_elem.get("link")
        if parent is None or child is None:
            continue

        xyz = np.zeros(3, dtype=np.float64)
        rpy = np.zeros(3, dtype=np.float64)
        if origin_elem is not None:
            if origin_elem.get("xyz"):
                xyz = _parse_vec3(origin_elem.get("xyz"))
            if origin_elem.get("rpy"):
                rpy = _parse_vec3(origin_elem.get("rpy"))

        T = _compose_transform(xyz, rpy)

        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if axis_elem is not None and axis_elem.get("xyz"):
            axis = _parse_vec3(axis_elem.get("xyz"))
            norm = np.linalg.norm(axis)
            if norm > 0:
                axis = axis / norm

        from ahrs.graphics.robot_model import JointData
        jd = JointData(
            name=name,
            parent=parent,
            child=child,
            joint_type=joint_type or "fixed",
            origin=T,
            axis=axis,
        )
        model.joints.append(jd)
        model.joint_order.append(name)
        model.child_to_parent[child] = parent
        model.joint_axis_map[name] = axis
        model.joint_origin_map[name] = T

        if parent not in model.parent_to_children:
            model.parent_to_children[parent] = []
        model.parent_to_children[parent].append((name, child))

    _child_links = set(model.child_to_parent.keys())
    _root_candidates = [
        lm.name for lm in model.links if lm.name not in _child_links
    ]
    if _root_candidates:
        model.root = _root_candidates[0]

    logger.info(
        f"Loaded {len(model.joints)} joints "
        f"(root: {model.root})"
    )
    return model
