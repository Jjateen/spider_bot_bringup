import math

import numpy as np
from scipy.spatial.transform import Rotation

from ahrs.math_utils.orientation_filter import ComplementaryFilter


def _quat_wxyz(q: np.ndarray) -> np.ndarray:
    # Filter returns [w, x, y, z]; convert to scipy [x, y, z, w].
    return np.array([q[1], q[2], q[3], q[0]])


def test_identity_inputs_yield_identity():
    f = ComplementaryFilter(alpha=0.5)
    q = f.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dt=0.01)
    rot = Rotation.from_quat(_quat_wxyz(q))
    assert np.allclose(rot.as_matrix(), np.eye(3), atol=1e-6)
    assert np.allclose(q, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-6)


def test_constant_x_gyro_rotates_90_deg_about_x():
    # Isolate gyro integration: with zero acceleration the filter takes the
    # pure-integrator branch (q_fused = q_gyro), so a sustained X rotation
    # reaches the integrated angle. (Non-zero accel would fight it via the
    # complementary correction and settle short of 90 deg.)
    f = ComplementaryFilter(alpha=0.98)
    dt = 0.01
    omega = math.pi / 2.0
    steps = 100
    for _ in range(steps - 1):
        f.update(0.0, 0.0, 0.0, omega, 0.0, 0.0, dt=dt)
    q = f.update(0.0, 0.0, 0.0, omega, 0.0, 0.0, dt=dt)
    rot = Rotation.from_quat(_quat_wxyz(q))
    expected = Rotation.from_rotvec(
        np.array([omega * steps * dt, 0.0, 0.0])
    ).as_matrix()
    assert np.allclose(rot.as_matrix(), expected, atol=1e-2)


def test_level_accel_zero_roll_pitch():
    f = ComplementaryFilter(alpha=0.5)
    q = f.update(0.0, 0.0, 9.81, 0.0, 0.0, 0.0, dt=0.01)
    rot = Rotation.from_quat(_quat_wxyz(q))
    roll, pitch, _ = rot.as_euler("xyz")
    assert abs(roll) < 1e-3
    assert abs(pitch) < 1e-3


def test_first_accel_seed_levels_pitch():
    f = ComplementaryFilter(alpha=0.5)
    # Gravity along +X in the body frame corresponds to a pitch (about Y)
    # with zero roll. Seed from this reading and check it levels that way.
    q = f.update(9.81, 0.0, 0.0, 0.0, 0.0, 0.0, dt=0.01)
    rot = Rotation.from_quat(_quat_wxyz(q))
    roll, pitch, _ = rot.as_euler("xyz")
    assert abs(roll) < 1e-3
    assert abs(pitch + math.pi / 2.0) < 1e-2


def test_high_accel_no_nan():
    f = ComplementaryFilter(alpha=0.5)
    # ~2g dynamic acceleration: confidence ~0 but must not NaN/crash.
    q = f.update(0.0, 0.0, 2.0 * 9.81, 0.1, 0.0, 0.0, dt=0.01)
    assert np.all(np.isfinite(q))


def test_reset_clears_initialized():
    f = ComplementaryFilter(alpha=0.5)
    f.update(0.0, 9.81, 0.0, 0.0, 0.0, 0.0, dt=0.01)
    assert f._initialized is True
    f.reset()
    assert f._initialized is False
    q = f.update(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, dt=0.01)
    assert np.allclose(q, np.array([1.0, 0.0, 0.0, 0.0]), atol=1e-6)
