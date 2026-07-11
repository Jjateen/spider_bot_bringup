import os
import subprocess
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field

import numpy as np
import open3d as o3d

from ahrs.utils.logger import setup_logger

logger = setup_logger("ahrs.urdf")


@dataclass
class LinkMesh:
    name: str
    mesh: o3d.geometry.TriangleMesh
    original_vertices: np.ndarray
    color: tuple[float, float, float] = (0.7, 0.7, 0.7)


@dataclass
class JointData:
    name: str
    parent: str
    child: str
    joint_type: str
    origin: np.ndarray
    axis: np.ndarray


@dataclass
class RobotModel:
    links: list[LinkMesh] = field(default_factory=list)
    joints: list[JointData] = field(default_factory=list)
    joint_order: list[str] = field(default_factory=list)
    child_to_parent: dict[str, str] = field(default_factory=dict)
    parent_to_children: dict[str, list[str]] = field(default_factory=dict)
    link_joint_origin: dict[str, np.ndarray] = field(default_factory=dict)
    joint_axis_map: dict[str, np.ndarray] = field(default_factory=dict)
    joint_origin_map: dict[str, np.ndarray] = field(default_factory=dict)
    root: str = "base_link"


def _resolve_package_path(package_name: str) -> str:
    from ament_index_python.packages import get_package_share_directory
    return get_package_share_directory(package_name)


def _run_xacro(xacro_path: str) -> str:
    import subprocess
    logger.info(f"Running xacro on {xacro_path}")
    cmd = [
        "xacro",
        xacro_path,
        "use_gz:=false",
    ]
    result = subprocess.run(
        cmd, capture_output=True, text=True, check=True
    )
    return result.stdout


def load_urdf_robot(
    xacro_path: str,
    mesh_dir: str | None = None,
) -> RobotModel:
    xml_str = _run_xacro(xacro_path)
    return load_urdf_from_string(xml_str)


def _parse_xyz(s: str) -> np.ndarray:
    parts = s.strip().split()
    return np.array([float(p) for p in parts], dtype=np.float64)


def _parse_rpy(s: str) -> np.ndarray:
    parts = s.strip().split()
    return np.array([float(p) for p in parts], dtype=np.float64)


def _rpy_to_matrix(rpy: np.ndarray) -> np.ndarray:
    from scipy.spatial.transform import Rotation
    return Rotation.from_euler("xyz", rpy).as_matrix()


def _compose_transform(
    xyz: np.ndarray, rpy: np.ndarray
) -> np.ndarray:
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = _rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def _parse_scale(scale_str: str | None) -> float:
    if scale_str is None:
        return 1.0
    parts = scale_str.strip().split()
    return float(parts[0])


def _load_mesh(
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


def load_urdf_from_string(
    urdf_xml: str,
) -> RobotModel:
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
                    xyz = _parse_xyz(origin_elem.get("xyz"))
                if origin_elem.get("rpy"):
                    rpy = _parse_rpy(origin_elem.get("rpy"))
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
                    scale_str = mesh_elem.get("scale")
                    scale = _parse_scale(scale_str)

                    actual_path = mesh_path
                    if mesh_path.startswith("package://"):
                        pkg_part = mesh_path[len("package://"):]
                        pkg_name, rel_path = pkg_part.split("/", 1)
                        pkg_dir = _resolve_package_path(pkg_name)
                        actual_path = os.path.join(pkg_dir, rel_path)

                    mesh = _load_mesh(actual_path, scale, color)

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
                xyz = _parse_xyz(origin_elem.get("xyz"))
            if origin_elem.get("rpy"):
                rpy = _parse_rpy(origin_elem.get("rpy"))

        T = _compose_transform(xyz, rpy)

        axis = np.array([1.0, 0.0, 0.0], dtype=np.float64)
        if axis_elem is not None and axis_elem.get("xyz"):
            axis = _parse_xyz(axis_elem.get("xyz"))
            norm = np.linalg.norm(axis)
            if norm > 0:
                axis = axis / norm

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

    # Derive the actual root link: a link that is never the child of any
    # joint. If the URDF names its root something other than the default
    # "base_link", compute_fk would otherwise set base_rotation on a link
    # that is never rendered, silently dropping the IMU-derived attitude.
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
