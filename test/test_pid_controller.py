"""
test_pid_controller.py
Unit tests for PID controller and dynamic plant simulation.
"""

import numpy as np
import pytest

from surgical_sim.robot_kinematics import RobotKinematics
from surgical_sim.robot_controller import PIDJointController, RobotPlantSimulator
from surgical_sim.trajectory_planner import TrajectoryPlanner


def test_pid_zero_error_zero_effort():
    """With zero error, PID controller should output near-zero effort."""
    controller = PIDJointController()
    q = np.zeros(6)
    dq = np.zeros(6)
    tau = controller.compute(q, dq, q, dq, dt=0.02)
    assert np.allclose(tau, np.zeros(6), atol=1e-6)


def test_pid_proportional_torque():
    """Positive position error produces proportional positive torque."""
    controller = PIDJointController()
    q_des = np.array([0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    q_act = np.zeros(6)
    tau = controller.compute(q_des, np.zeros(6), q_act, np.zeros(6), dt=0.02)

    # Torque for joint 0 should be approx Kp[0] * 0.1
    expected_tau_0 = controller.kp[0] * 0.1 + controller.ki[0] * 0.1 * 0.02
    assert np.isclose(tau[0], expected_tau_0, atol=1e-3)
    assert tau[0] > 0.0


def test_pid_anti_windup_clamping():
    """Large steady state error clamps integral error to limit."""
    controller = PIDJointController(integral_limit=5.0)
    q_des = np.ones(6) * 10.0
    q_act = np.zeros(6)

    # Accumulate integral error over many steps
    for _ in range(500):
        controller.compute(q_des, np.zeros(6), q_act, np.zeros(6), dt=0.05)

    assert np.all(np.abs(controller.integral_error) <= 5.0 + 1e-6)


def test_plant_simulator_tracking_convergence():
    """Plant simulator tracks planned trajectory and settles with sub-millimeter error."""
    kinematics = RobotKinematics(tool_length=0.150)
    sim = RobotPlantSimulator(kinematics=kinematics)
    planner = TrajectoryPlanner(kinematics=kinematics)

    q0 = sim.q.copy()
    q_target = q0 + np.array([0.08, -0.08, 0.10, -0.05, 0.02, 0.0])

    traj = planner.plan(q0, q_target, dt=0.02)
    for pt in traj.points:
        sim.step(pt.positions, pt.velocities, dt=0.02)

    # Settle period
    for _ in range(30):
        sim.step(q_target, np.zeros(6), dt=0.02)

    metrics = sim.get_metrics()
    assert metrics["final_error_mm"] < 0.5, f"Final error {metrics['final_error_mm']} mm exceeds 0.5 mm"


def test_plant_simulator_estop_braking():
    """Emergency stop brings joint velocities to near-zero rapidly."""
    kinematics = RobotKinematics()
    sim = RobotPlantSimulator(kinematics=kinematics)

    # Give robot non-zero velocity
    sim.dq = np.array([0.8, -0.6, 0.5, 0.4, -0.3, 0.2])

    # Assert E-stop
    sim.set_estop(True)

    # Simulate 5 steps (100 ms)
    for _ in range(5):
        sim.step(sim.q, np.zeros(6), dt=0.02)

    # Velocity must be arrested
    max_vel = np.max(np.abs(sim.dq))
    assert max_vel < 0.05, f"Velocity {max_vel} rad/s not arrested after 100 ms of E-stop"

