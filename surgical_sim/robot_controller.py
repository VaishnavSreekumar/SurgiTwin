"""
robot_controller.py
SurgiTwin Joint Control & Robot Plant Simulator.

Implements multi-axis PID joint controllers, anti-windup clamping,
second-order mechanical plant dynamics, and active emergency brake dynamics
for the SurgiTwin 6-DOF surgical manipulator.
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple, Union
import numpy as np

from surgical_sim.robot_kinematics import RobotKinematics


@dataclass
class ControllerState:
    """Snapshot of robot controller and simulated plant state."""
    time: float
    q_actual: np.ndarray        # (6,) rad
    dq_actual: np.ndarray       # (6,) rad/s
    q_desired: np.ndarray       # (6,) rad
    dq_desired: np.ndarray      # (6,) rad/s
    joint_efforts: np.ndarray   # (6,) N*m
    cartesian_pos: np.ndarray   # (3,) m
    cartesian_error: float      # m
    is_estopped: bool


class PIDJointController:
    """
    Independent 6-axis PID joint controller with anti-windup.
    """

    DEFAULT_KP = np.array([120.0, 120.0, 90.0, 45.0, 35.0, 25.0], dtype=np.float64)
    DEFAULT_KD = np.array([15.0, 15.0, 12.0, 5.0, 4.0, 3.0], dtype=np.float64)
    DEFAULT_KI = np.array([2.0, 2.0, 1.5, 0.5, 0.5, 0.2], dtype=np.float64)
    DEFAULT_MAX_EFFORT = np.array([150.0, 150.0, 100.0, 40.0, 40.0, 20.0], dtype=np.float64)

    def __init__(
        self,
        kp: Optional[np.ndarray] = None,
        kd: Optional[np.ndarray] = None,
        ki: Optional[np.ndarray] = None,
        max_effort: Optional[np.ndarray] = None,
        integral_limit: float = 25.0,
    ):
        self.kp = np.asarray(kp, dtype=np.float64) if kp is not None else self.DEFAULT_KP.copy()
        self.kd = np.asarray(kd, dtype=np.float64) if kd is not None else self.DEFAULT_KD.copy()
        self.ki = np.asarray(ki, dtype=np.float64) if ki is not None else self.DEFAULT_KI.copy()
        self.max_effort = np.asarray(max_effort, dtype=np.float64) if max_effort is not None else self.DEFAULT_MAX_EFFORT.copy()
        self.integral_limit = float(integral_limit)

        self.integral_error = np.zeros(6, dtype=np.float64)
        self.prev_error = np.zeros(6, dtype=np.float64)

    def reset(self):
        """Reset internal integrator and derivative terms."""
        self.integral_error.fill(0.0)
        self.prev_error.fill(0.0)

    def compute(
        self,
        q_desired: np.ndarray,
        dq_desired: np.ndarray,
        q_actual: np.ndarray,
        dq_actual: np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """
        Compute joint torque efforts tau.

        :param q_desired: Target positions (6,) rad
        :param dq_desired: Target velocities (6,) rad/s
        :param q_actual: Current positions (6,) rad
        :param dq_actual: Current velocities (6,) rad/s
        :param dt: Time step (seconds)
        :return: Commanded torques (6,) N*m
        """
        error_pos = q_desired - q_actual
        error_vel = dq_desired - dq_actual

        if dt > 1e-6:
            self.integral_error += error_pos * dt
            # Anti-windup clamping
            self.integral_error = np.clip(self.integral_error, -self.integral_limit, self.integral_limit)

        p_term = self.kp * error_pos
        d_term = self.kd * error_vel
        i_term = self.ki * self.integral_error

        tau = p_term + d_term + i_term
        # Clamp to max actuator torque
        tau = np.clip(tau, -self.max_effort, self.max_effort)

        self.prev_error = error_pos.copy()
        return tau


class RobotPlantSimulator:
    """
    Dynamic simulation plant for 6-DOF surgical manipulator.

    Simulates:
      M * ddq + B * dq = tau
    with emergency braking:
      tau_brake = -B_estop * dq
    """

    # Link effective inertia (kg*m^2)
    INERTIA = np.array([2.5, 2.5, 1.8, 0.6, 0.4, 0.25], dtype=np.float64)
    # Joint viscous friction damping (N*m*s/rad)
    DAMPING = np.array([4.0, 4.0, 3.0, 1.2, 0.8, 0.5], dtype=np.float64)
    # E-stop active electromagnetic brake damping
    ESTOP_DAMPING = np.array([60.0, 60.0, 50.0, 20.0, 15.0, 10.0], dtype=np.float64)

    def __init__(
        self,
        initial_q: Optional[np.ndarray] = None,
        kinematics: Optional[RobotKinematics] = None,
        controller: Optional[PIDJointController] = None,
    ):
        self.kinematics = kinematics if kinematics is not None else RobotKinematics()
        self.controller = controller if controller is not None else PIDJointController()

        if initial_q is not None:
            self.q = np.asarray(initial_q, dtype=np.float64).copy()
        else:
            self.q = self.kinematics.SURGICAL_HOME_Q.copy()

        self.dq = np.zeros(6, dtype=np.float64)
        self.ddq = np.zeros(6, dtype=np.float64)
        self.current_time = 0.0
        self.is_estopped = False

        # Metrics tracking
        self.history: List[ControllerState] = []

    def set_estop(self, estopped: bool):
        """Set or release emergency stop."""
        self.is_estopped = estopped
        if estopped:
            self.controller.reset()

    def reset_state(self, q: Optional[np.ndarray] = None):
        """Reset robot joint position and velocities."""
        if q is not None:
            self.q = np.asarray(q, dtype=np.float64).copy()
        else:
            self.q = self.kinematics.SURGICAL_HOME_Q.copy()
        self.dq.fill(0.0)
        self.ddq.fill(0.0)
        self.controller.reset()
        self.is_estopped = False
        self.history.clear()

    def step(
        self,
        q_desired: np.ndarray,
        dq_desired: np.ndarray,
        dt: float = 0.02,
    ) -> ControllerState:
        """
        Advance simulation by dt seconds.

        :param q_desired: Commanded setpoint position (6,) rad
        :param dq_desired: Commanded setpoint velocity (6,) rad/s
        :param dt: Time interval (seconds)
        :return: ControllerState
        """
        if self.is_estopped:
            # Active emergency braking torque
            tau = -self.ESTOP_DAMPING * self.dq
            b_effective = self.ESTOP_DAMPING
        else:
            tau = self.controller.compute(q_desired, dq_desired, self.q, self.dq, dt)
            b_effective = self.DAMPING

        # Plant dynamics: M * ddq = tau - B * dq
        self.ddq = (tau - b_effective * self.dq) / self.INERTIA

        # Semi-implicit Euler integration (symplectic, energy-stable)
        self.dq += self.ddq * dt
        self.q += self.dq * dt

        # Enforce physical mechanical joint limits
        self.q = self.kinematics.clamp_joints(self.q)

        self.current_time += dt

        # Compute end-effector Cartesian pose and tracking error
        _, p_actual, _ = self.kinematics.forward_kinematics(self.q)
        _, p_desired, _ = self.kinematics.forward_kinematics(q_desired)
        cart_error = float(np.linalg.norm(p_desired - p_actual))

        state = ControllerState(
            time=self.current_time,
            q_actual=self.q.copy(),
            dq_actual=self.dq.copy(),
            q_desired=np.asarray(q_desired, dtype=np.float64).copy(),
            dq_desired=np.asarray(dq_desired, dtype=np.float64).copy(),
            joint_efforts=tau.copy(),
            cartesian_pos=p_actual.copy(),
            cartesian_error=cart_error,
            is_estopped=self.is_estopped,
        )

        self.history.append(state)
        return state

    def get_metrics(self) -> Dict[str, float]:
        """
        Compute tracking performance metrics over recorded history.
        """
        if not self.history:
            return {"max_error_mm": 0.0, "rms_error_mm": 0.0, "final_error_mm": 0.0}

        errors = np.array([s.cartesian_error for s in self.history], dtype=np.float64)
        max_err = float(np.max(errors)) * 1000.0
        rms_err = float(np.sqrt(np.mean(errors ** 2))) * 1000.0
        final_err = float(errors[-1]) * 1000.0

        return {
            "max_error_mm": max_err,
            "rms_error_mm": rms_err,
            "final_error_mm": final_err,
            "duration_s": float(self.history[-1].time - self.history[0].time),
            "sample_count": len(self.history),
        }

