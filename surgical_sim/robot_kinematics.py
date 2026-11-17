"""
robot_kinematics.py
SurgiTwin 6-DOF Robotic Manipulator Kinematics Engine.

Implements standard Denavit-Hartenberg (DH) forward kinematics, geometric Jacobian,
and kinematic transformation utilities for the SurgiTwin 6-DOF surgical manipulator.

Kinematic structure is modeled on a serial manipulator with surgical end-effector:
- 6 revolute joints (shoulder pan, shoulder lift, elbow, wrist 1, wrist 2, wrist 3)
- Rigid surgical tool attachment extending from the wrist flange (tool0)
"""

import math
from typing import Dict, List, Optional, Tuple, Union
import numpy as np


class RobotKinematics:
    """
    Kinematics engine for 6-DOF SurgiTwin surgical manipulator.

    Standard Denavit-Hartenberg parameters:
    i | theta_i | d_i (m) | a_i (m) | alpha_i (rad)
    --+---------+---------+---------+--------------
    1 |   q_1   |  0.1625 |    0    |    pi / 2
    2 |   q_2   |    0    | -0.4250 |      0
    3 |   q_3   |    0    | -0.3922 |      0
    4 |   q_4   |  0.1333 |    0    |    pi / 2
    5 |   q_5   |  0.0997 |    0    |   -pi / 2
    6 |   q_6   |  0.0996 |    0    |      0

    Tool offset:
    tool_length = 0.150 m along the end-effector z-axis.
    """

    JOINT_NAMES = [
        "shoulder_pan_joint",
        "shoulder_lift_joint",
        "elbow_joint",
        "wrist_1_joint",
        "wrist_2_joint",
        "wrist_3_joint",
    ]

    # Joint limits in radians [min, max]
    DEFAULT_JOINT_LIMITS = np.array([
        [-np.pi, np.pi],         # shoulder_pan
        [-np.pi, np.pi],         # shoulder_lift
        [-np.pi, np.pi],         # elbow
        [-np.pi, np.pi],         # wrist_1
        [-np.pi, np.pi],         # wrist_2
        [-2 * np.pi, 2 * np.pi], # wrist_3
    ], dtype=np.float64)

    # Standard surgical home pose [rad]
    SURGICAL_HOME_Q = np.array([0.0, -np.pi / 2, np.pi / 2, -np.pi / 2, -np.pi / 2, 0.0], dtype=np.float64)

    def __init__(self, tool_length: float = 0.150, joint_limits: Optional[np.ndarray] = None):
        """
        Initialize kinematics engine.

        :param tool_length: Rigid tool extension along tool z-axis (meters).
        :param joint_limits: Optional (6, 2) array of joint limits [min, max] in radians.
        """
        self.tool_length = float(tool_length)
        self.joint_limits = np.array(joint_limits, dtype=np.float64) if joint_limits is not None else self.DEFAULT_JOINT_LIMITS.copy()

        # Base DH parameters: (d, a, alpha)
        # End link incorporates tool_length into d_6
        self.dh_d = np.array([0.1625, 0.0, 0.0, 0.1333, 0.0997, 0.0996 + self.tool_length], dtype=np.float64)
        self.dh_a = np.array([0.0, -0.4250, -0.3922, 0.0, 0.0, 0.0], dtype=np.float64)
        self.dh_alpha = np.array([np.pi / 2, 0.0, 0.0, np.pi / 2, -np.pi / 2, 0.0], dtype=np.float64)

    @staticmethod
    def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
        """
        Compute standard 4x4 Denavit-Hartenberg transformation matrix A_i.

        A_i = Rot_z(theta) * Trans_z(d) * Trans_x(a) * Rot_x(alpha)
        """
        ct = math.cos(theta)
        st = math.sin(theta)
        ca = math.cos(alpha)
        sa = math.sin(alpha)

        return np.array([
            [ct, -st * ca,  st * sa, a * ct],
            [st,  ct * ca, -ct * sa, a * st],
            [0.0,     sa,       ca,      d],
            [0.0,    0.0,      0.0,    1.0],
        ], dtype=np.float64)

    def forward_kinematics(
        self,
        q: Union[List[float], np.ndarray],
        return_all_frames: bool = False,
    ) -> Union[Tuple[np.ndarray, np.ndarray, np.ndarray], List[np.ndarray]]:
        """
        Compute forward kinematics for joint configuration q.

        :param q: Array-like of 6 joint angles in radians.
        :param return_all_frames: If True, returns list of all cumulative link transforms [T_0, T_1, ..., T_6].
        :return: (T_tool, position (3,), quaternion (4,) [x, y, z, w]) or list of 4x4 matrices if return_all_frames=True.
        """
        q_arr = np.asarray(q, dtype=np.float64)
        if len(q_arr) != 6:
            raise ValueError(f"Expected 6 joint angles, got {len(q_arr)}")

        t_cumulative = np.eye(4, dtype=np.float64)
        transforms = [t_cumulative.copy()]

        for i in range(6):
            a_i = self.dh_matrix(q_arr[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            t_cumulative = t_cumulative @ a_i
            transforms.append(t_cumulative.copy())

        if return_all_frames:
            return transforms

        position = t_cumulative[:3, 3]
        rotation = t_cumulative[:3, :3]
        quaternion = self.rot_to_quaternion(rotation)

        return t_cumulative, position, quaternion

    def compute_jacobian(self, q: Union[List[float], np.ndarray]) -> np.ndarray:
        """
        Compute geometric 6x6 Jacobian matrix J(q).

        J(q) maps joint velocities dq to end-effector spatial twist v = [v_x, v_y, v_z, w_x, w_y, w_z]^T.
        For revolute joints:
          J_v_i = z_{i-1} x (p_ee - p_{i-1})
          J_w_i = z_{i-1}
        """
        q_arr = np.asarray(q, dtype=np.float64)
        frames = self.forward_kinematics(q_arr, return_all_frames=True)
        p_ee = frames[-1][:3, 3]

        jacobian = np.zeros((6, 6), dtype=np.float64)

        for i in range(6):
            t_prev = frames[i]  # Frame i-1 (base is frame 0)
            z_prev = t_prev[:3, 2]
            p_prev = t_prev[:3, 3]

            j_v = np.cross(z_prev, p_ee - p_prev)
            j_w = z_prev

            jacobian[:3, i] = j_v
            jacobian[3:, i] = j_w

        return jacobian

    def is_within_limits(self, q: Union[List[float], np.ndarray], tolerance: float = 1e-4) -> bool:
        """
        Check if joint configuration satisfies mechanical limits.
        """
        q_arr = np.asarray(q, dtype=np.float64)
        for i in range(6):
            if q_arr[i] < (self.joint_limits[i, 0] - tolerance) or q_arr[i] > (self.joint_limits[i, 1] + tolerance):
                return False
        return True

    def clamp_joints(self, q: Union[List[float], np.ndarray]) -> np.ndarray:
        """
        Clamp joint configuration to mechanical limits.
        """
        q_arr = np.asarray(q, dtype=np.float64)
        return np.clip(q_arr, self.joint_limits[:, 0], self.joint_limits[:, 1])

    @staticmethod
    def rot_to_quaternion(r: np.ndarray) -> np.ndarray:
        """
        Convert 3x3 rotation matrix to unit quaternion [x, y, z, w].
        Uses Shepperd's / Hughes algorithm for numerical stability across all angles.
        """
        tr = r[0, 0] + r[1, 1] + r[2, 2]
        if tr > 0.0:
            s = math.sqrt(tr + 1.0) * 2.0  # s = 4 * w
            w = 0.25 * s
            x = (r[2, 1] - r[1, 2]) / s
            y = (r[0, 2] - r[2, 0]) / s
            z = (r[1, 0] - r[0, 1]) / s
        elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
            s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0  # s = 4 * x
            w = (r[2, 1] - r[1, 2]) / s
            x = 0.25 * s
            y = (r[0, 1] + r[1, 0]) / s
            z = (r[0, 2] + r[2, 0]) / s
        elif r[1, 1] > r[2, 2]:
            s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0  # s = 4 * y
            w = (r[0, 2] - r[2, 0]) / s
            x = (r[0, 1] + r[1, 0]) / s
            y = 0.25 * s
            z = (r[1, 2] + r[2, 1]) / s
        else:
            s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0  # s = 4 * z
            w = (r[1, 0] - r[0, 1]) / s
            x = (r[0, 2] + r[2, 0]) / s
            y = (r[1, 2] + r[2, 1]) / s
            z = 0.25 * s

        q = np.array([x, y, z, w], dtype=np.float64)
        norm = np.linalg.norm(q)
        return q / norm if norm > 1e-12 else np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)

    @staticmethod
    def quaternion_to_rot(q: Union[List[float], np.ndarray]) -> np.ndarray:
        """
        Convert unit quaternion [x, y, z, w] to 3x3 rotation matrix.
        """
        x, y, z, w = q
        n = math.sqrt(x * x + y * y + z * z + w * w)
        if n < 1e-12:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / n, y / n, z / n, w / n

        return np.array([
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

