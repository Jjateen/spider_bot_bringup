import argparse
import os
import signal
import sys

import rclpy
import yaml

from ahrs.graphics.grid import Grid
from ahrs.graphics.axes import Axes
from ahrs.graphics.robot import Robot
from ahrs.graphics.overlays import Overlay
from ahrs.graphics.scene import Scene
from ahrs.graphics.viewer import Viewer
from ahrs.ros.joint_state_subscriber import SharedJointState
from ahrs.ros.robot_description_subscriber import RobotDescriptionSubscriber
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

    rclpy.init(args=None)
    state = SharedRobotState()

    # Read use_sim_time, allowing a launch-provided ROS parameter to override
    # the config-file default. rclpy auto-declares `use_sim_time` on every
    # node, so read it rather than re-declaring (re-declare raises
    # ParameterAlreadyDeclaredException).
    _param_node = rclpy.create_node("ahrs_param_reader")
    use_sim_time = _param_node.get_parameter("use_sim_time").value
    _param_node.destroy_node()

    desc_sub = RobotDescriptionSubscriber()
    joint_state: SharedJointState | None = None
    joint_topic = robot_cfg.get("joint_state_topic", "/joint_states")

    logger.info("Waiting for /robot_description...")
    timeout = robot_cfg.get("description_timeout", 5.0)
    deadline = (rclpy.clock.Clock().now() + rclpy.duration.Duration(seconds=timeout)).nanoseconds
    while desc_sub.model is None and rclpy.ok():
        rclpy.spin_once(desc_sub, timeout_sec=0.1)
        if rclpy.clock.Clock().now().nanoseconds > deadline:
            break

    if desc_sub.model is not None:
        logger.info(
            f"Loaded URDF robot from /robot_description "
            f"({len(desc_sub.model.links)} links, {len(desc_sub.model.joints)} joints)"
        )
        robot = Robot.from_model(desc_sub.model)
        joint_state = SharedJointState()
        logger.info(f"Will subscribe to joint states: {joint_topic}")
    else:
        logger.warning(
            f"No /robot_description received after {timeout}s timeout, "
            "using cube robot"
        )
        robot = Robot(size=robot_cfg.get("cube_size", 0.3))
    desc_sub.destroy_node()

    filter_alpha = config.get("filter_alpha", 0.5)
    gyro_bias = config.get("gyro_bias", [0.0, 0.0, 0.0])
    accel_bias = config.get("accel_bias", [0.0, 0.0, 0.0])
    imu_units = config.get("imu_units", None)
    ros_node = ROSNode(
        imu_topic, state,
        joint_state=joint_state,
        joint_topic=joint_topic,
        filter_alpha=filter_alpha,
        gyro_bias=gyro_bias,
        accel_bias=accel_bias,
        imu_units=imu_units,
        use_sim_time=use_sim_time,
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
