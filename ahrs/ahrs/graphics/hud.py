import io

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np
import open3d as o3d

from ahrs.ros.robot_state import RobotState


class Hud:
    def __init__(self) -> None:
        self._fig, self._ax = plt.subplots(
            figsize=(7.0, 3.0), dpi=80, facecolor='black',
        )
        self._ax.set_facecolor('black')
        self._ax.axis('off')
        self._text = self._ax.text(
            0.03, 0.5, '', color='lime', fontfamily='monospace',
            fontsize=9, verticalalignment='center',
            transform=self._ax.transAxes,
        )
        plt.subplots_adjust(left=0, right=1, bottom=0, top=1)
        self._fig.canvas.draw()
        self._texture = o3d.geometry.Image(
            np.zeros((240, 560, 3), dtype=np.uint8),
        )
        self._mesh = self._create_panel()
        self._frame_counter = 0
        self._update_interval = 5

    def update(
        self, state: RobotState, fps: float, imu_rate: float,
    ) -> None:
        self._frame_counter += 1
        if self._frame_counter % self._update_interval != 0:
            return

        roll_deg = np.degrees(state.roll)
        pitch_deg = np.degrees(state.pitch)
        yaw_deg = np.degrees(state.yaw)
        av = state.angular_velocity
        la = state.linear_acceleration

        if state.imu_connected:
            status = 'CONNECTED'
            color = 'lime'
        else:
            status = 'DISCONNECTED'
            color = 'red'

        text = (
            f"Roll: {roll_deg:7.1f}°  Pitch: {pitch_deg:7.1f}°  Yaw: {yaw_deg:7.1f}°\n"
            f"Ang Vel: {av[0]:7.3f}  {av[1]:7.3f}  {av[2]:7.3f}  rad/s\n"
            f"Lin Acc: {la[0]:7.2f}  {la[1]:7.2f}  {la[2]:7.2f}  m/s²\n"
            f"IMU: {imu_rate:5.1f} Hz  |  FPS: {fps:5.0f}  |  [{status}]"
        )

        self._text.set_text(text)
        self._text.set_color(color)
        self._fig.canvas.draw()
        w, h = self._fig.canvas.get_width_height()
        buf = self._fig.canvas.tostring_argb()
        arr = np.frombuffer(buf, dtype=np.uint8).reshape(h, w, 4)[:, :, 1:].copy()
        self._texture = o3d.geometry.Image(arr)
        self._mesh.textures = [self._texture]

    def _create_panel(self) -> o3d.geometry.TriangleMesh:
        aspect = 7.0 / 3.0
        panel_width = 4.0
        panel_height = panel_width / aspect
        cx, cy, cz = -4.5, 2.5, 2.0

        verts = np.array([
            [cx - panel_width / 2, cy + panel_height / 2, cz],
            [cx + panel_width / 2, cy + panel_height / 2, cz],
            [cx + panel_width / 2, cy - panel_height / 2, cz],
            [cx - panel_width / 2, cy - panel_height / 2, cz],
        ], dtype=np.float64)

        tris = np.array([[0, 1, 2], [0, 2, 3]], dtype=np.int32)
        uvs = np.array([
            [0, 0], [1, 0], [1, 1],
            [0, 0], [1, 1], [0, 1],
        ], dtype=np.float64)

        mesh = o3d.geometry.TriangleMesh()
        mesh.vertices = o3d.utility.Vector3dVector(verts)
        mesh.triangles = o3d.utility.Vector3iVector(tris)
        mesh.triangle_uvs = o3d.utility.Vector2dVector(uvs)
        mesh.triangle_material_ids = o3d.utility.IntVector([0, 0])
        mesh.textures = [self._texture]
        return mesh

    @property
    def geometry(self) -> o3d.geometry.TriangleMesh:
        return self._mesh
