from dataclasses import dataclass, field

import numpy as np
import open3d as o3d


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
