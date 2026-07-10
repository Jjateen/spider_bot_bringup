import argparse
import os
import signal
import sys

import yaml

from ahrs.graphics.grid import Grid
from ahrs.graphics.axes import Axes
from ahrs.graphics.robot import Robot
from ahrs.graphics.overlays import Overlay
from ahrs.graphics.scene import Scene
from ahrs.graphics.viewer import Viewer
from ahrs.ros.joint_state_subscriber import SharedJointState
from ahrs.ros.robot_state import SharedRobotState
from ahrs.ros.ros_node import ROSNode
from ahrs.utils.logger import setup_logger

logger = setup_logger("ahrs")


def load_config(config_path: str | None = None) -> dict:
    if config_path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory("ahrs")
            config_path = os.path.join(pkg_dir, "config", "config.yaml")
        except Exception:
            config_path = os.path.join(
                os.path.dirname(__file__), "..", "config", "config.yaml"
            )

    config_path = os.path.abspath(config_path)
    if not os.path.exists(config_path):
        logger.warning(f"Config not found at {config_path}, using defaults")
        return {}

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)
    logger.info(f"Loaded config from {config_path}")
    return config or {}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="AHRS 3D Visualizer")
    parser.add_argument(
        "--config",
        type=str,
        default=None,
        help="Path to YAML configuration file",
    )
    parser.add_argument(
        "--topic",
        type=str,
        default=None,
        help="IMU ROS2 topic (overrides config)",
    )
    return parser.parse_args()


def _resolve_urdf_path(urdf_path: str) -> str | None:
    if not urdf_path:
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory("big_bertha_description")
            candidate = os.path.join(pkg_dir, "urdf", "big_bertha.urdf.xacro")
            if os.path.exists(candidate):
                logger.info(f"Auto-discovered URDF: {candidate}")
                return candidate
        except Exception:
            pass
        return None
    if urdf_path.startswith("package://"):
        parts = urdf_path[len("package://"):].split("/", 1)
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory(parts[0])
            return os.path.join(pkg_dir, parts[1])
        except Exception:
            logger.warning(f"Could not resolve package: {parts[0]}")
            return None
    if os.path.exists(urdf_path):
        return urdf_path
    logger.warning(f"URDF path not found: {urdf_path}")
    return None


def main() -> None:
    args = parse_args()
    config = load_config(args.config)

    imu_topic = args.topic or config.get("imu_topic", "/imu")
    win_cfg = config.get("window", {})
    grid_cfg = config.get("grid", {})
    cam_cfg = config.get("camera", {})
    bg = config.get("background_color", [0.12, 0.12, 0.12])
    robot_cfg = config.get("robot", {})

    logger.info(f"Using IMU topic: {imu_topic}")
    logger.info("Starting AHRS Visualizer")

    state = SharedRobotState()

    urdf_path = _resolve_urdf_path(robot_cfg.get("urdf_path", ""))
    joint_state: SharedJointState | None = None
    joint_topic = robot_cfg.get("joint_state_topic", "/joint_states")

    if urdf_path:
        logger.info(f"Loading URDF robot from: {urdf_path}")
        mesh_dir = robot_cfg.get("mesh_dir", "") or None
        robot = Robot.from_urdf(urdf_path, mesh_dir)
        joint_state = SharedJointState()
        logger.info(f"Will subscribe to joint states: {joint_topic}")
    else:
        logger.info("Using cube robot (no URDF path configured)")
        robot = Robot(size=robot_cfg.get("cube_size", 0.3))

    ros_node = ROSNode(
        imu_topic, state,
        joint_state=joint_state,
        joint_topic=joint_topic,
    )

    grid = Grid(
        size=grid_cfg.get("size", 10.0),
        divisions=grid_cfg.get("divisions", 20),
    )
    world_axes = Axes(size=0.8, origin=(0.0, 0.0, 0.0))
    overlay = Overlay()
    scene = Scene(
        grid=grid,
        world_axes=world_axes,
        robot=robot,
        overlay=overlay,
        background_color=tuple(bg),
    )

    viewer = Viewer(
        scene=scene,
        state=state,
        window_name=win_cfg.get("name", "AHRS Visualizer"),
        width=win_cfg.get("width", 1280),
        height=win_cfg.get("height", 720),
        imu_rate_fn=lambda: ros_node.rate_hz,
        joint_state=joint_state,
    )

    def shutdown(signum, frame):
        logger.info("Shutting down...")
        ros_node.shutdown()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)

    ros_node.start()
    logger.info("ROS node started, opening viewer...")
    viewer.run()


if __name__ == "__main__":
    main()
