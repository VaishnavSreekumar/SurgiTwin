"""
test_inverse_kinematics.py
Unit tests for SurgiTwin InverseKinematicsSolver.
"""

import numpy as np
import pytest

from surgical_sim.robot_kinematics import RobotKinematics
from surgical_sim.inverse_kinematics import InverseKinematicsSolver


@pytest.fixture
def solver():
    kinematics = RobotKinematics(tool_length=0.150)
    return InverseKinematicsSolver(kinematics=kinematics, position_tolerance=0.0005)


def test_ik_workspace_center(solver):
    """Test IK convergence to the center of surgical workspace."""
    target_p = np.array([0.0, 0.35, 0.20])
    res = solver.solve_position(target_p)
    assert res.success, f"IK failed: {res.status_message}"
    assert res.position_error < 0.0005, f"Position error {res.position_error * 1000} mm exceeds 0.5 mm"

    # Forward kinematics confirmation
    _, p_fk, _ = solver.kinematics.forward_kinematics(res.q)
    assert np.allclose(p_fk, target_p, atol=0.0005)


def test_ik_multiple_surgical_points(solver):
    """Test IK convergence across a grid of reachable surgical targets."""
    targets = [
        [0.10, 0.30, 0.25],
        [-0.10, 0.40, 0.20],
        [0.05, 0.45, 0.15],
        [-0.15, 0.35, 0.18],
    ]
    for target in targets:
        target_p = np.array(target)
        res = solver.solve_position(target_p)
        assert res.success, f"Failed on target {target}: {res.status_message}"
        assert res.position_error < 0.0008, f"Error {res.position_error * 1000} mm for {target}"


def test_ik_round_trip_arbitrary_pose(solver):
    """Generate known joint angle, compute FK, then recover joints with IK."""
    q_known = np.array([0.4, -1.1, 1.7, -0.6, -1.57, 0.1])
    _, p_target, _ = solver.kinematics.forward_kinematics(q_known)

    res = solver.solve_position(p_target, q_seed=q_known + 0.1)
    assert res.success
    assert res.position_error < 0.0005


def test_ik_full_6d_pose(solver):
    """Test 6D pose inverse kinematics convergence with orientation."""
    q_known = np.array([0.2, -1.2, 1.6, -0.4, -1.57, 0.2])
    t_target, p_target, quat_target = solver.kinematics.forward_kinematics(q_known)

    res = solver.solve_pose(p_target, quat_target, q_seed=q_known + 0.1)
    assert res.success, f"6D IK failed: {res.status_message}"
    assert res.position_error < 0.001
    assert res.orientation_error < 0.02


def test_ik_unreachable_point_outside_max_reach(solver):
    """Reject target point outside maximum physical reach (>1.05 m)."""
    p_unreachable = np.array([1.5, 1.5, 1.0])
    res = solver.solve_position(p_unreachable)
    assert not res.success
    assert "exceeds maximum reach" in res.status_message


def test_ik_unreachable_point_inside_base_cylinder(solver):
    """Reject target point inside base exclusion radius (<0.10 m)."""
    p_inside_base = np.array([0.02, 0.02, 0.05])
    res = solver.solve_position(p_inside_base)
    assert not res.success
    assert "exclusion radius" in res.status_message

