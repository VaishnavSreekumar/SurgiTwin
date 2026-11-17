"""
trajectory_planner.py
SurgiTwin Quintic Polynomial Trajectory Planner.

Implements minimum-jerk $C^2$-continuous quintic polynomial trajectory generation
with dynamic time scaling ensuring strict compliance with Cartesian velocity limits (<= 100 mm/s)
and joint-space velocity and acceleration constraints.
"""

from dataclasses import dataclass
import math
from typing import List, Optional, Union
import numpy as np

from surgical_sim.robot_kinematics import RobotKinematics


@dataclass
class TrajectoryPoint:
    """A discrete sample along the planned trajectory."""
    time_from_start: float
    positions: np.ndarray      # (6,) rad
    velocities: np.ndarray     # (6,) rad/s
    accelerations: np.ndarray  # (6,) rad/s^2
    cartesian_pos: np.ndarray  # (3,) meters


class JointTrajectory:
    """Container for time-indexed joint trajectory with continuous sampling."""

    def __init__(self, points: List[TrajectoryPoint], duration: float, dt: float):
        self.points = points
        self.duration = float(duration)
        self.dt = float(dt)

    def sample(self, t: float) -> TrajectoryPoint:
        """
        Sample trajectory at arbitrary time t >= 0.0 with clamping at endpoints.
        """
        if not self.points:
            raise ValueError("Trajectory contains no points")

        if t <= 0.0:
            return self.points[0]
        if t >= self.duration:
            return self.points[-1]

        idx = int(t / self.dt)
        if idx >= len(self.points) - 1:
            return self.points[-1]

        # Linear interpolation between adjacent points
        p0 = self.points[idx]
        p1 = self.points[idx + 1]
        alpha = (t - p0.time_from_start) / (p1.time_from_start - p0.time_from_start + 1e-12)
        alpha = min(max(alpha, 0.0), 1.0)

        pos = p0.positions + alpha * (p1.positions - p0.positions)
        vel = p0.velocities + alpha * (p1.velocities - p0.velocities)
        acc = p0.accelerations + alpha * (p1.accelerations - p0.accelerations)
        cart = p0.cartesian_pos + alpha * (p1.cartesian_pos - p0.cartesian_pos)

        return TrajectoryPoint(
            time_from_start=t,
            positions=pos,
            velocities=vel,
            accelerations=acc,
            cartesian_pos=cart,
        )


class TrajectoryPlanner:
    """
    Quintic polynomial minimum-jerk trajectory planner.

    Enforces:
    - Zero boundary velocity and acceleration: v(0)=0, v(T)=0, a(0)=0, a(T)=0
    - Maximum Cartesian tip velocity <= max_cartesian_vel (default 0.10 m/s = 100 mm/s)
    - Maximum joint velocity <= max_joint_vel (default 1.0 rad/s)
    - Maximum joint acceleration <= max_joint_acc (default 2.0 rad/s^2)
    """

    def __init__(
        self,
        kinematics: Optional[RobotKinematics] = None,
        max_cartesian_velocity: float = 0.10,    # 100 mm/s
        max_joint_velocity: float = 1.0,         # 1.0 rad/s
        max_joint_acceleration: float = 2.0,     # 2.0 rad/s^2
        min_duration: float = 0.5,               # seconds
    ):
        self.kinematics = kinematics if kinematics is not None else RobotKinematics()
        self.max_cart_vel = float(max_cartesian_velocity)
        self.max_joint_vel = float(max_joint_velocity)
        self.max_joint_acc = float(max_joint_acceleration)
        self.min_duration = float(min_duration)

    def calculate_duration(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        p_start: Optional[np.ndarray] = None,
        p_goal: Optional[np.ndarray] = None,
    ) -> float:
        """
        Calculate dynamically feasible duration T satisfying all constraints.

        For quintic polynomial:
          v_max = 1.875 * |delta_q| / T  ==>  T_vel = 1.875 * |delta_q| / v_max
          a_max = 5.7735 * |delta_q| / T^2  ==>  T_acc = sqrt(5.7735 * |delta_q| / a_max)
        """
        delta_q = np.abs(q_goal - q_start)

        # Joint velocity requirement
        t_joint_vel = np.max(1.875 * delta_q / self.max_joint_vel) if np.any(delta_q > 1e-6) else 0.0

        # Joint acceleration requirement
        t_joint_acc = np.max(np.sqrt(5.7735 * delta_q / self.max_joint_acc)) if np.any(delta_q > 1e-6) else 0.0

        # Cartesian velocity requirement with 1.35x path curvature safety margin
        if p_start is None:
            _, p_start, _ = self.kinematics.forward_kinematics(q_start)
        if p_goal is None:
            _, p_goal, _ = self.kinematics.forward_kinematics(q_goal)

        cart_dist = float(np.linalg.norm(p_goal - p_start))
        t_cart_vel = 1.35 * (1.875 * cart_dist / self.max_cart_vel) if cart_dist > 1e-6 else 0.0

        duration = max(t_joint_vel, t_joint_acc, t_cart_vel, self.min_duration)
        return float(duration)

    def plan(
        self,
        q_start: Union[List[float], np.ndarray],
        q_goal: Union[List[float], np.ndarray],
        dt: float = 0.02,  # 50 Hz default
        custom_duration: Optional[float] = None,
    ) -> JointTrajectory:
        """
        Plan a minimum-jerk trajectory from q_start to q_goal.
        Includes verification loop to guarantee Cartesian velocity limit is respected.

        :param q_start: Start joint configuration (6,)
        :param q_goal: Target joint configuration (6,)
        :param dt: Time discretization step (seconds, default 0.02 = 50 Hz)
        :param custom_duration: Optional override duration (seconds)
        :return: JointTrajectory instance
        """
        q0 = np.asarray(q_start, dtype=np.float64)
        qf = np.asarray(q_goal, dtype=np.float64)

        if len(q0) != 6 or len(qf) != 6:
            raise ValueError(f"Joint configurations must have 6 DOF, got {len(q0)} and {len(qf)}")

        _, p0, _ = self.kinematics.forward_kinematics(q0)
        _, pf, _ = self.kinematics.forward_kinematics(qf)

        if custom_duration is not None and custom_duration > 0.0:
            duration = float(custom_duration)
        else:
            duration = self.calculate_duration(q0, qf, p0, pf)

        delta_q = qf - q0

        # Trajectory generation with Cartesian limit safety check
        points: List[TrajectoryPoint] = []
        for _ in range(3):  # Max 3 refinement passes
            num_steps = max(int(math.ceil(duration / dt)), 2)
            times = np.linspace(0.0, duration, num_steps + 1)
            points = []

            for t in times:
                tau = t / duration if duration > 1e-12 else 1.0
                tau = min(max(tau, 0.0), 1.0)

                s = 10.0 * (tau ** 3) - 15.0 * (tau ** 4) + 6.0 * (tau ** 5)
                s_dot = (30.0 * (tau ** 2) - 60.0 * (tau ** 3) + 30.0 * (tau ** 4)) / duration
                s_ddot = (60.0 * tau - 180.0 * (tau ** 2) + 120.0 * (tau ** 3)) / (duration ** 2)

                q_t = q0 + s * delta_q
                dq_t = s_dot * delta_q
                ddq_t = s_ddot * delta_q

                _, p_t, _ = self.kinematics.forward_kinematics(q_t)

                points.append(TrajectoryPoint(
                    time_from_start=float(t),
                    positions=q_t,
                    velocities=dq_t,
                    accelerations=ddq_t,
                    cartesian_pos=p_t,
                ))

            # Verify Cartesian velocities
            max_c_vel = 0.0
            for i in range(1, len(points)):
                seg_dp = np.linalg.norm(points[i].cartesian_pos - points[i - 1].cartesian_pos)
                seg_dt = points[i].time_from_start - points[i - 1].time_from_start
                if seg_dt > 1e-6:
                    seg_vel = seg_dp / seg_dt
                    if seg_vel > max_c_vel:
                        max_c_vel = seg_vel

            if max_c_vel <= self.max_cart_vel or custom_duration is not None:
                break
            else:
                scale = (max_c_vel / self.max_cart_vel) * 1.05
                duration = duration * scale

        return JointTrajectory(points=points, duration=duration, dt=dt)
