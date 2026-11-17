"""
test_joint_limits.py
Unit tests for joint limits validation and clamping.
"""

import numpy as np
import pytest

from surgical_sim.robot_kinematics import RobotKinematics
from surgical_sim.inverse_kinematics import InverseKinematicsSolver


@pytest.fixture
def kinematics():
    return RobotKinematics()


def test_is_within_limits_valid_home(kinematics):
    """Surgical home pose must be strictly within limits."""
    assert kinematics.is_within_limits(kinematics.SURGICAL_HOME_Q)


def test_is_within_limits_detects_violation(kinematics):
    """Detects joint angle exceeding positive or negative limits."""
    q_exceeded_pos = kinematics.SURGICAL_HOME_Q.copy()
    q_exceeded_pos[0] = np.pi + 0.1
    assert not kinematics.is_within_limits(q_exceeded_pos)

    q_exceeded_neg = kinematics.SURGICAL_HOME_Q.copy()
    q_exceeded_neg[2] = -np.pi - 0.1
    assert not kinematics.is_within_limits(q_exceeded_neg)


def test_clamp_joints_bounds(kinematics):
    """Clamping projects out-of-bounds angles back to limit boundaries."""
    q_out = np.array([4.0, -5.0, 3.5, -4.0, 6.0, -10.0])
    q_clamped = kinematics.clamp_joints(q_out)

    assert kinematics.is_within_limits(q_clamped)
    assert q_clamped[0] == kinematics.joint_limits[0, 1]
    assert q_clamped[1] == kinematics.joint_limits[1, 0]


def test_ik_solutions_respect_limits():
    """All IK solutions must satisfy joint limits."""
    solver = InverseKinematicsSolver()
    targets = [
        [0.0, 0.35, 0.20],
        [0.15, 0.40, 0.25],
        [-0.15, 0.30, 0.15],
    ]
    for target in targets:
        res = solver.solve_position(target)
        assert res.success
        assert solver.kinematics.is_within_limits(res.q)

