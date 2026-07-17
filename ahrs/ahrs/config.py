import argparse
import os

import yaml

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
