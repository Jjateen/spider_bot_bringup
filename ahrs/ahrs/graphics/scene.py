import open3d as o3d

from ahrs.graphics.axes import Axes
from ahrs.graphics.grid import Grid
from ahrs.graphics.overlays import Overlay
from ahrs.graphics.robot import Robot
from ahrs.ros.robot_state import RobotState


class Scene:
    def __init__(
        self,
        grid: Grid,
        world_axes: Axes,
        robot: Robot,
        overlay: Overlay,
        background_color: tuple[float, float, float] = (0.12, 0.12, 0.12),
    ) -> None:
        self._robot = robot
        self._overlay = overlay
        self._background_color = background_color

        self._static_geometries: list[o3d.geometry.Geometry] = [
            grid.geometry,
            world_axes.geometry,
        ]

        self._dynamic_geometries: list[o3d.geometry.Geometry] = (
            robot.geometries + overlay.geometries
        )

    def update(
        self,
        state: RobotState,
        fps: float,
        imu_rate: float,
        joint_positions: dict[str, float] | None = None,
    ) -> None:
        self._robot.update_pose(state.rotation_matrix)
        if joint_positions is not None:
            self._robot.update_joints(joint_positions)
        self._overlay.update(state, fps, imu_rate)

    def geometries(self) -> list[o3d.geometry.Geometry]:
        return (
            self._static_geometries + self._dynamic_geometries
        )

    @property
    def background_color(self) -> tuple[float, float, float]:
        return self._background_color
