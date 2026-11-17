"""
test_trajectory_generation.py
Unit tests for minimum-jerk quintic polynomial trajectory generation.
"""

import numpy as np
import pytest

from surgical_sim.robot_kinematics import RobotKinematics
from surgical_sim.trajectory_planner import TrajectoryPlanner


@pytest.fixture
def planner():
    kinematics = RobotKinematics(tool_length=0.150)
    return TrajectoryPlanner(
        kinematics=kinematics,
        max_cartesian_velocity=0.10,  # 100 mm/s
        max_joint_velocity=1.0,       # 1.0 rad/s
        max_joint_acceleration=2.0,   # 2.0 rad/s^2
    )


def test_trajectory_boundary_conditions(planner):
    """Verify boundary position, zero velocity, and zero acceleration."""
    q0 = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    q1 = np.array([0.2, -1.2, 1.8, -0.6, -np.pi / 2, 0.1])

    traj = planner.plan(q0, q1, dt=0.02)
    assert len(traj.points) >= 2

    first_pt = traj.points[0]
    last_pt = traj.points[-1]

    # Positions match
    assert np.allclose(first_pt.positions, q0, atol=1e-6)
    assert np.allclose(last_pt.positions, q1, atol=1e-6)

    # Velocities are exactly zero at start and end
    assert np.allclose(first_pt.velocities, np.zeros(6), atol=1e-6)
    assert np.allclose(last_pt.velocities, np.zeros(6), atol=1e-6)

    # Accelerations are zero at start and end
    assert np.allclose(first_pt.accelerations, np.zeros(6), atol=1e-6)
    assert np.allclose(last_pt.accelerations, np.zeros(6), atol=1e-6)


def test_trajectory_cartesian_velocity_limit(planner):
    """Verify end-effector Cartesian velocity never exceeds 100 mm/s limit."""
    q0 = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0])
    q1 = np.array([0.3, -1.2, 1.8, -0.6, -np.pi / 2, 0.1])

    traj = planner.plan(q0, q1, dt=0.02)

    cart_speeds = []
    for i in range(1, len(traj.points)):
        dp = np.linalg.norm(traj.points[i].cartesian_pos - traj.points[i - 1].cartesian_pos)
        dt = traj.points[i].time_from_start - traj.points[i - 1].time_from_start
        if dt > 1e-6:
            cart_speeds.append(dp / dt)

    max_cart_speed = np.max(cart_speeds)
    # Must strictly satisfy 100 mm/s (0.10 m/s) with numerical tolerance
    assert max_cart_speed <= 0.10 + 1e-4, f"Max speed {max_cart_speed * 1000} mm/s exceeds 100 mm/s"


def test_trajectory_joint_velocity_limit(planner):
    """Verify joint velocity never exceeds max_joint_vel."""
    q0 = np.zeros(6)
    q1 = np.array([1.5, -1.0, 1.2, 0.8, -0.5, 0.3])

    traj = planner.plan(q0, q1, dt=0.02)
    vels = np.array([pt.velocities for pt in traj.points])
    max_j_vel = np.max(np.abs(vels))

    assert max_j_vel <= planner.max_joint_vel + 1e-4


def test_trajectory_sampling_continuity(planner):
    """Verify trajectory sampling at fractional intermediate times."""
    q0 = np.zeros(6)
    q1 = np.ones(6) * 0.5
    traj = planner.plan(q0, q1, dt=0.02)

    sample_mid = traj.sample(traj.duration * 0.5)
    assert np.all(sample_mid.positions > q0)
    assert np.all(sample_mid.positions < q1)

    # Clamping tests
    sample_neg = traj.sample(-1.0)
    assert np.allclose(sample_neg.positions, q0, atol=1e-6)

    sample_over = traj.sample(traj.duration + 5.0)
    assert np.allclose(sample_over.positions, q1, atol=1e-6)

