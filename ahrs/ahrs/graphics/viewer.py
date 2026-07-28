import time

import open3d as o3d

from ahrs.graphics.camera import CameraController
from ahrs.graphics.scene import Scene
from ahrs.ros.joint_state_subscriber import SharedJointState
from ahrs.ros.robot_state import SharedRobotState
from ahrs.utils.timing import FPSCounter


class Viewer:
    def __init__(
        self,
        scene: Scene,
        state: SharedRobotState,
        window_name: str = "AHRS Visualizer",
        width: int = 1280,
        height: int = 720,
        target_fps: int = 60,
        imu_rate_fn=None,
        joint_state: SharedJointState | None = None,
    ) -> None:
        self._scene = scene
        self._state = state
        self._joint_state = joint_state
        self._fps_counter = FPSCounter()
        self._target_fps = target_fps
        self._imu_rate_fn = imu_rate_fn or (lambda: 0.0)

        self._vis = o3d.visualization.Visualizer()
        self._vis.create_window(
            window_name=window_name, width=width, height=height
        )

        self._vis.get_render_option().background_color = (
            scene.background_color
        )
        self._vis.get_render_option().point_size = 1.0

        cam_ctrl = CameraController()
        cam_ctrl.reset(self._vis.get_view_control())

        for geom in scene.geometries():
            self._vis.add_geometry(geom)

    def run(self) -> None:
        frame_duration = 1.0 / self._target_fps

        try:
            while self._vis.poll_events():
                loop_start = time.perf_counter()

                robot_state = self._state.read()
                fps = self._fps_counter.tick()
                imu_rate = self._imu_rate_fn()

                joint_positions = (
                    self._joint_state.read()
                    if self._joint_state is not None
                    else None
                )

                self._scene.update(robot_state, fps, imu_rate, joint_positions)

                for geom in self._scene.geometries():
                    self._vis.update_geometry(geom)

                self._vis.update_renderer()

                elapsed = time.perf_counter() - loop_start
                sleep_time = frame_duration - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        finally:
            self._vis.destroy_window()
