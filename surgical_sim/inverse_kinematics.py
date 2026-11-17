"""
inverse_kinematics.py
SurgiTwin 6-DOF Inverse Kinematics Engine.

Implements numerical Damped Least Squares (Levenberg-Marquardt) Jacobian inverse kinematics
with multi-start seeds, nullspace joint centering, and joint limit enforcement for the
SurgiTwin 6-DOF robotic manipulator.
"""

from dataclasses import dataclass
from typing import List, Optional, Tuple, Union
import numpy as np

from surgical_sim.robot_kinematics import RobotKinematics


@dataclass
class IKResult:
    """Result structure for inverse kinematics queries."""
    success: bool
    q: np.ndarray
    position_error: float
    orientation_error: float
    iterations: int
    status_message: str


class InverseKinematicsSolver:
    """
    Damped Least Squares (DLS) numerical Inverse Kinematics solver.

    Solves:
      dq = J^T (J J^T + lambda^2 I)^(-1) e
    with nullspace projection:
      dq_total = dq + (I - J^# J) * k_null * (q_home - q)
    """

    # Pre-defined candidate seeds for multi-start solver
    CANDIDATE_SEEDS = [
        np.array([0.0, -1.0, 1.8, -0.8, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([np.pi / 2, -1.2, 1.8, -0.6, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([-np.pi / 2, -1.2, 1.8, -0.6, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([0.5, -0.9, 1.5, -0.6, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([-0.5, -0.9, 1.5, -0.6, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([0.0, -1.5, 2.0, -0.5, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([1.0, -1.4, 1.9, -0.5, -np.pi / 2, 0.0], dtype=np.float64),
        np.array([-1.0, -1.4, 1.9, -0.5, -np.pi / 2, 0.0], dtype=np.float64),
    ]

    def __init__(
        self,
        kinematics: Optional[RobotKinematics] = None,
        damping: float = 0.02,
        max_iterations: int = 120,
        position_tolerance: float = 0.0005,  # 0.5 mm
        orientation_tolerance: float = 0.01,  # ~0.57 degrees
        max_reach: float = 1.05,
        min_reach: float = 0.10,
    ):
        """
        Initialize IK solver.

        :param kinematics: RobotKinematics instance (created if None).
        :param damping: Damping factor lambda for DLS.
        :param max_iterations: Maximum solver iterations per seed.
        :param position_tolerance: Convergence position error threshold (meters).
        :param orientation_tolerance: Convergence orientation error threshold (radians).
        :param max_reach: Maximum spherical workspace reach from base (meters).
        :param min_reach: Minimum spherical base exclusion radius (meters).
        """
        self.kinematics = kinematics if kinematics is not None else RobotKinematics()
        self.damping = float(damping)
        self.max_iterations = int(max_iterations)
        self.pos_tol = float(position_tolerance)
        self.rot_tol = float(orientation_tolerance)
        self.max_reach = float(max_reach)
        self.min_reach = float(min_reach)

    def is_reachable(self, target_position: Union[List[float], np.ndarray]) -> Tuple[bool, str]:
        """
        Pre-flight reachability check.
        """
        pos = np.asarray(target_position, dtype=np.float64)
        dist = np.linalg.norm(pos)
        if dist > self.max_reach:
            return False, f"Target distance ({dist:.3f} m) exceeds maximum reach ({self.max_reach:.3f} m)"
        if dist < self.min_reach:
            return False, f"Target distance ({dist:.3f} m) inside base collision exclusion radius ({self.min_reach:.3f} m)"
        return True, "Reachable"

    def solve_position(
        self,
        target_position: Union[List[float], np.ndarray],
        q_seed: Optional[Union[List[float], np.ndarray]] = None,
        nullspace_weight: float = 0.1,
    ) -> IKResult:
        """
        Solve inverse kinematics for target 3D Cartesian position.
        Uses position Jacobian (3x6) and projects joint centering towards home into the nullspace.

        :param target_position: Target (x, y, z) in robot base frame (meters).
        :param q_seed: Optional initial joint configuration guess.
        :param nullspace_weight: Weight for nullspace projection towards default home.
        :return: IKResult
        """
        target = np.asarray(target_position, dtype=np.float64)
        reachable, reason = self.is_reachable(target)
        if not reachable:
            return IKResult(
                success=False,
                q=np.zeros(6, dtype=np.float64),
                position_error=float("inf"),
                orientation_error=0.0,
                iterations=0,
                status_message=f"IK rejected: {reason}",
            )

        # Assemble list of seeds to try
        seeds = []
        if q_seed is not None:
            seeds.append(np.asarray(q_seed, dtype=np.float64))
        for cand in self.CANDIDATE_SEEDS:
            if not any(np.allclose(cand, s, atol=0.01) for s in seeds):
                seeds.append(cand)

        best_result: Optional[IKResult] = None
        min_err = float("inf")

        for seed in seeds:
            q = seed.copy()
            converged = False
            err_norm = float("inf")

            for it in range(1, self.max_iterations + 1):
                _, current_p, _ = self.kinematics.forward_kinematics(q)
                err_p = target - current_p
                err_norm = float(np.linalg.norm(err_p))

                if err_norm <= self.pos_tol:
                    converged = True
                    break

                # 3x6 translational Jacobian
                j_v = self.kinematics.compute_jacobian(q)[:3, :]

                # Damped Least Squares
                # dq = J^T (J J^T + lambda^2 I)^(-1) e
                j_jt = j_v @ j_v.T
                dls_inv = np.linalg.inv(j_jt + (self.damping ** 2) * np.eye(3))
                dq_task = j_v.T @ dls_inv @ err_p

                # Nullspace projection for joint centering
                j_pseudo = j_v.T @ dls_inv
                null_proj = np.eye(6) - (j_pseudo @ j_v)
                q_home = self.kinematics.SURGICAL_HOME_Q
                dq_null = nullspace_weight * (q_home - q)

                dq = dq_task + null_proj @ dq_null

                # Step size limiter to prevent wild overshoot
                step_norm = np.linalg.norm(dq)
                max_step = 0.25  # rad
                if step_norm > max_step:
                    dq = (max_step / step_norm) * dq

                q = q + dq

            # Check joint limits and FK validation
            q_clamped = self.kinematics.clamp_joints(q)
            _, fk_p, _ = self.kinematics.forward_kinematics(q_clamped)
            final_err = float(np.linalg.norm(target - fk_p))
            within_limits = self.kinematics.is_within_limits(q, tolerance=0.02)

            is_valid = converged and within_limits and (final_err <= self.pos_tol * 1.5)

            if is_valid:
                return IKResult(
                    success=True,
                    q=q_clamped,
                    position_error=final_err,
                    orientation_error=0.0,
                    iterations=it,
                    status_message=f"Converged in {it} iterations with error {final_err * 1000.0:.3f} mm",
                )

            if final_err < min_err:
                min_err = final_err
                best_result = IKResult(
                    success=False,
                    q=q_clamped,
                    position_error=final_err,
                    orientation_error=0.0,
                    iterations=it,
                    status_message=f"Failed to converge (residual error {final_err * 1000.0:.3f} mm)",
                )

        return best_result if best_result is not None else IKResult(
            success=False,
            q=np.zeros(6, dtype=np.float64),
            position_error=min_err,
            orientation_error=0.0,
            iterations=self.max_iterations,
            status_message="No seed converged within limits",
        )

    def solve_pose(
        self,
        target_position: Union[List[float], np.ndarray],
        target_orientation: Union[List[float], np.ndarray],
        q_seed: Optional[Union[List[float], np.ndarray]] = None,
    ) -> IKResult:
        """
        Solve full 6-DOF inverse kinematics for target position and rotation.

        :param target_position: Target (x, y, z) in robot base frame (meters).
        :param target_orientation: Target rotation as 3x3 matrix or 4-element quaternion [x, y, z, w].
        :param q_seed: Optional initial joint configuration guess.
        :return: IKResult
        """
        target_p = np.asarray(target_position, dtype=np.float64)
        reachable, reason = self.is_reachable(target_p)
        if not reachable:
            return IKResult(
                success=False,
                q=np.zeros(6, dtype=np.float64),
                position_error=float("inf"),
                orientation_error=float("inf"),
                iterations=0,
                status_message=f"IK rejected: {reason}",
            )

        orient = np.asarray(target_orientation, dtype=np.float64)
        if orient.shape == (3, 3):
            r_target = orient
        elif orient.size == 4:
            r_target = RobotKinematics.quaternion_to_rot(orient)
        else:
            raise ValueError("Orientation must be 3x3 matrix or 4-element quaternion")

        seeds = []
        if q_seed is not None:
            seeds.append(np.asarray(q_seed, dtype=np.float64))
        for cand in self.CANDIDATE_SEEDS:
            if not any(np.allclose(cand, s, atol=0.01) for s in seeds):
                seeds.append(cand)

        best_result: Optional[IKResult] = None
        min_err_total = float("inf")

        for seed in seeds:
            q = seed.copy()
            converged = False
            p_err_val = float("inf")
            r_err_val = float("inf")

            for it in range(1, self.max_iterations + 1):
                t_cur, p_cur, _ = self.kinematics.forward_kinematics(q)
                r_cur = t_cur[:3, :3]

                err_p = target_p - p_cur
                # Orientation error vector from cross-product of columns
                err_r = 0.5 * (
                    np.cross(r_cur[:, 0], r_target[:, 0])
                    + np.cross(r_cur[:, 1], r_target[:, 1])
                    + np.cross(r_cur[:, 2], r_target[:, 2])
                )

                p_err_val = float(np.linalg.norm(err_p))
                r_err_val = float(np.linalg.norm(err_r))

                if p_err_val <= self.pos_tol and r_err_val <= self.rot_tol:
                    converged = True
                    break

                err_6d = np.hstack([err_p, err_r])
                jacobian = self.kinematics.compute_jacobian(q)

                j_jt = jacobian @ jacobian.T
                dls_inv = np.linalg.inv(j_jt + (self.damping ** 2) * np.eye(6))
                dq = jacobian.T @ dls_inv @ err_6d

                step_norm = np.linalg.norm(dq)
                max_step = 0.20
                if step_norm > max_step:
                    dq = (max_step / step_norm) * dq

                q = q + dq

            q_clamped = self.kinematics.clamp_joints(q)
            t_final, fk_p, _ = self.kinematics.forward_kinematics(q_clamped)
            fk_r = t_final[:3, :3]

            final_pos_err = float(np.linalg.norm(target_p - fk_p))
            final_rot_err = float(np.linalg.norm(
                0.5 * (
                    np.cross(fk_r[:, 0], r_target[:, 0])
                    + np.cross(fk_r[:, 1], r_target[:, 1])
                    + np.cross(fk_r[:, 2], r_target[:, 2])
                )
            ))
            within_limits = self.kinematics.is_within_limits(q, tolerance=0.02)

            is_valid = converged and within_limits and (final_pos_err <= self.pos_tol * 2.0)

            if is_valid:
                return IKResult(
                    success=True,
                    q=q_clamped,
                    position_error=final_pos_err,
                    orientation_error=final_rot_err,
                    iterations=it,
                    status_message=f"Converged 6D in {it} iterations (pos {final_pos_err * 1000.0:.3f} mm, rot {final_rot_err:.4f} rad)",
                )

            total_err = final_pos_err + final_rot_err * 0.1
            if total_err < min_err_total:
                min_err_total = total_err
                best_result = IKResult(
                    success=False,
                    q=q_clamped,
                    position_error=final_pos_err,
                    orientation_error=final_rot_err,
                    iterations=it,
                    status_message=f"Failed to converge 6D (pos {final_pos_err * 1000.0:.3f} mm, rot {final_rot_err:.4f} rad)",
                )

        return best_result if best_result is not None else IKResult(
            success=False,
            q=np.zeros(6, dtype=np.float64),
            position_error=min_err_total,
            orientation_error=0.0,
            iterations=self.max_iterations,
            status_message="No seed converged 6D within limits",
        )

