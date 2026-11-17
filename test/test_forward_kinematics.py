"""
test_forward_kinematics.py
Unit tests for SurgiTwin RobotKinematics engine.
"""

import math
import numpy as np
import pytest

from surgical_sim.robot_kinematics import RobotKinematics


@pytest.fixture
def kinematics():
    return RobotKinematics(tool_length=0.150)


def test_dh_matrix_identity(kinematics):
    """Test that zero DH parameters yield identity transformation."""
    t = kinematics.dh_matrix(0.0, 0.0, 0.0, 0.0)
    assert np.allclose(t, np.eye(4), atol=1e-9)


def test_dh_matrix_pure_translation(kinematics):
    """Test pure translation along z and x axes."""
    t = kinematics.dh_matrix(0.0, 0.25, 0.35, 0.0)
    assert np.isclose(t[0, 3], 0.35, atol=1e-9)
    assert np.isclose(t[2, 3], 0.25, atol=1e-9)
    assert np.allclose(t[:3, :3], np.eye(3), atol=1e-9)


def test_dh_matrix_pure_rotation(kinematics):
    """Test pure rotation about z and x axes."""
    t = kinematics.dh_matrix(math.pi / 2, 0.0, 0.0, math.pi / 2)
    expected_r = np.array([
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ])
    assert np.allclose(t[:3, :3], expected_r, atol=1e-9)


def test_forward_kinematics_zero_pose(kinematics):
    """Verify forward kinematics at zero joint angles."""
    t, p, q = kinematics.forward_kinematics(np.zeros(6))
    assert t.shape == (4, 4)
    assert p.shape == (3,)
    assert q.shape == (4,)
    # Quaternion norm must be 1.0
    assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-6)
    # Total arm length at zero pose extends along -x and +z
    assert np.isclose(p[2], 0.0628, atol=1e-3)  # Height above base
    assert p[0] < -0.8  # Extended along -x


def test_forward_kinematics_surgical_home(kinematics):
    """Verify forward kinematics at standard surgical home configuration."""
    t, p, q = kinematics.forward_kinematics(kinematics.SURGICAL_HOME_Q)
    assert np.isclose(np.linalg.norm(q), 1.0, atol=1e-6)
    # Surgical home position should be within valid physical reach
    reach = np.linalg.norm(p)
    assert 0.3 <= reach <= 0.85


def test_forward_kinematics_all_frames(kinematics):
    """Verify return_all_frames returns 7 frame transforms (base + 6 links)."""
    frames = kinematics.forward_kinematics(kinematics.SURGICAL_HOME_Q, return_all_frames=True)
    assert len(frames) == 7
    # Base frame must be identity
    assert np.allclose(frames[0], np.eye(4), atol=1e-9)
    # Each transformation must have det(R) = +1.0
    for i, frame in enumerate(frames):
        det_r = np.linalg.det(frame[:3, :3])
        assert np.isclose(det_r, 1.0, atol=1e-6), f"Frame {i} rotation determinant {det_r} is not +1.0"


def test_geometric_jacobian_dimensions(kinematics):
    """Verify geometric Jacobian is 6x6 and finite."""
    j = kinematics.compute_jacobian(kinematics.SURGICAL_HOME_Q)
    assert j.shape == (6, 6)
    assert not np.any(np.isnan(j))
    assert not np.any(np.isinf(j))


def test_rot_to_quaternion_round_trip(kinematics):
    """Verify rotation matrix <-> quaternion round trip fidelity."""
    angles = [0.1, -0.4, 0.7]
    # Construct test rotation matrix using Euler angles
    cx, sx = math.cos(angles[0]), math.sin(angles[0])
    cy, sy = math.cos(angles[1]), math.sin(angles[1])
    cz, sz = math.cos(angles[2]), math.sin(angles[2])
    rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    r_orig = rz @ ry @ rx

    quat = kinematics.rot_to_quaternion(r_orig)
    assert np.isclose(np.linalg.norm(quat), 1.0, atol=1e-9)

    r_recovered = kinematics.quaternion_to_rot(quat)
    assert np.allclose(r_orig, r_recovered, atol=1e-6)


def test_tool_length_extension():
    """Verify changing tool length extends the end-effector position along z-axis."""
    k_short = RobotKinematics(tool_length=0.0)
    k_long = RobotKinematics(tool_length=0.200)

    q = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    t_short, p_short, _ = k_short.forward_kinematics(q)
    t_long, p_long, _ = k_long.forward_kinematics(q)

    diff = np.linalg.norm(p_long - p_short)
    assert np.isclose(diff, 0.200, atol=1e-4)
