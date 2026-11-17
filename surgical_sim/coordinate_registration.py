"""
coordinate_registration.py: Camera-to-robot coordinate registration module.

Provides rigid-body homogeneous transformation matrices (4x4):
    P_base = T_base_camera * P_camera
Validates rotation orthogonality and determinant, handles pose orientation quaternions,
and provides YAML serialization for calibrated surgical extrinsic setups.
"""

from dataclasses import dataclass
import math
import os
from typing import Any, Dict, Tuple

import numpy as np
import yaml


@dataclass
class CoordinateRegistration:
    """Rigid 4x4 transformation from camera_optical_frame to robot base_link."""

    t_matrix: np.ndarray  # 4x4 homogeneous transformation matrix

    def __post_init__(self) -> None:
        """Validate transformation matrix upon creation."""
        self.validate()

    def validate(self) -> None:
        """Validate matrix shape, bottom row, rotation orthogonality, and unit determinant."""
        if self.t_matrix.shape != (4, 4):
            raise ValueError(f"Transformation matrix must be 4x4, got {self.t_matrix.shape}")

        # Check bottom row [0, 0, 0, 1]
        expected_bottom = np.array([0.0, 0.0, 0.0, 1.0])
        if not np.allclose(self.t_matrix[3, :], expected_bottom, atol=1e-5):
            raise ValueError(f"Bottom row must be [0, 0, 0, 1], got {self.t_matrix[3, :]}")

        # Extract 3x3 rotation matrix R
        r = self.rotation_matrix

        # Check determinant det(R) == 1.0 (proper rotation, not reflection)
        det = np.linalg.det(r)
        if not math.isclose(det, 1.0, abs_tol=1e-3):
            raise ValueError(f"Rotation matrix determinant must be +1.0, got {det:.6f}")

        # Check orthogonality: R * R^T == I
        identity = np.eye(3)
        ortho_check = np.dot(r, r.T)
        if not np.allclose(ortho_check, identity, atol=1e-3):
            raise ValueError("Rotation matrix is not orthogonal (R * R^T != I)")

    @property
    def rotation_matrix(self) -> np.ndarray:
        """3x3 rotation submatrix R."""
        return self.t_matrix[0:3, 0:3]

    @property
    def translation_vector(self) -> np.ndarray:
        """3x1 translation subvector t."""
        return self.t_matrix[0:3, 3]

    def transform_point(self, x_c: float, y_c: float, z_c: float) -> Tuple[float, float, float]:
        """
        Transform 3D point from camera frame to robot base frame.

        Formula:
            [X_base, Y_base, Z_base, 1]^T = T * [X_c, Y_c, Z_c, 1]^T
        """
        p_cam = np.array([x_c, y_c, z_c, 1.0], dtype=np.float64)
        p_base = np.dot(self.t_matrix, p_cam)
        return (float(p_base[0]), float(p_base[1]), float(p_base[2]))

    def transform_orientation(
        self, qx: float = 0.0, qy: float = 0.0, qz: float = 0.0, qw: float = 1.0
    ) -> Tuple[float, float, float, float]:
        """
        Transform orientation quaternion from camera frame into robot base frame.

        R_base = R_base_cam * R_cam
        """
        # Convert input quaternion to rotation matrix
        r_cam = quaternion_to_matrix(qx, qy, qz, qw)
        r_base = np.dot(self.rotation_matrix, r_cam)
        return matrix_to_quaternion(r_base)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize transformation to dictionary."""
        return {
            "source_frame": "camera_optical_frame",
            "target_frame": "base_link",
            "transformation_matrix": {
                "rows": 4,
                "cols": 4,
                "data": self.t_matrix.flatten().tolist(),
            },
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CoordinateRegistration":
        """Deserialize transformation from dictionary."""
        t_data = data["transformation_matrix"]["data"]
        mat = np.array(t_data, dtype=np.float64).reshape((4, 4))
        return cls(t_matrix=mat)

    def save_yaml(self, filepath: str) -> None:
        """Save extrinsic calibration to YAML file."""
        expanded = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(expanded) or ".", exist_ok=True)
        with open(expanded, "w", encoding="utf-8") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    @classmethod
    def load_yaml(cls, filepath: str) -> "CoordinateRegistration":
        """Load and validate extrinsic calibration from YAML file."""
        expanded = os.path.expanduser(filepath)
        if not os.path.isfile(expanded):
            raise FileNotFoundError(f"Registration file not found: {expanded}")
        with open(expanded, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    @classmethod
    def create_identity(cls) -> "CoordinateRegistration":
        """Create identity transformation matrix."""
        return cls(t_matrix=np.eye(4, dtype=np.float64))

    @classmethod
    def create_synthetic_overhead(
        cls,
        camera_pos: Tuple[float, float, float] = (0.0, 0.35, 0.80),
    ) -> "CoordinateRegistration":
        """
        Create a verified synthetic overhead surgical camera extrinsic matrix.

        Camera mounting convention:
        - Camera mounted at (0.0, 0.35, 0.80) m above robot workspace looking down.
        - Camera optical axes:
            X_cam (right) -> Robot +X (forward)
            Y_cam (down)  -> Robot -Y (right)
            Z_cam (view)  -> Robot -Z (down toward table)
        - Rotation matrix R has det(R) = +1.0.
        - Target at camera optical center at depth Z_c=0.60m maps exactly to home pose (0.0, 0.35, 0.20)m.
        """
        mat = np.eye(4, dtype=np.float64)
        mat[0:3, 0:3] = np.array(
            [[1.0, 0.0, 0.0], [0.0, -1.0, 0.0], [0.0, 0.0, -1.0]],
            dtype=np.float64,
        )
        mat[0, 3] = camera_pos[0]
        mat[1, 3] = camera_pos[1]
        mat[2, 3] = camera_pos[2]
        return cls(t_matrix=mat)


def quaternion_to_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert unit quaternion to 3x3 rotation matrix."""
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm > 0:
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    else:
        qw = 1.0

    return np.array(
        [
            [1.0 - 2.0 * (qy * qy + qz * qz), 2.0 * (qx * qy - qz * qw), 2.0 * (qx * qz + qy * qw)],
            [2.0 * (qx * qy + qz * qw), 1.0 - 2.0 * (qx * qx + qz * qz), 2.0 * (qy * qz - qx * qw)],
            [2.0 * (qx * qz - qy * qw), 2.0 * (qy * qz + qx * qw), 1.0 - 2.0 * (qx * qx + qy * qy)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(r: np.ndarray) -> Tuple[float, float, float, float]:
    """Convert 3x3 rotation matrix to unit quaternion (qx, qy, qz, qw)."""
    tr = r[0, 0] + r[1, 1] + r[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2.0
        qw = 0.25 * s
        qx = (r[2, 1] - r[1, 2]) / s
        qy = (r[0, 2] - r[2, 0]) / s
        qz = (r[1, 0] - r[0, 1]) / s
    elif (r[0, 0] > r[1, 1]) and (r[0, 0] > r[2, 2]):
        s = math.sqrt(1.0 + r[0, 0] - r[1, 1] - r[2, 2]) * 2.0
        qw = (r[2, 1] - r[1, 2]) / s
        qx = 0.25 * s
        qy = (r[0, 1] + r[1, 0]) / s
        qz = (r[0, 2] + r[2, 0]) / s
    elif r[1, 1] > r[2, 2]:
        s = math.sqrt(1.0 + r[1, 1] - r[0, 0] - r[2, 2]) * 2.0
        qw = (r[0, 2] - r[2, 0]) / s
        qx = (r[0, 1] + r[1, 0]) / s
        qy = 0.25 * s
        qz = (r[1, 2] + r[2, 1]) / s
    else:
        s = math.sqrt(1.0 + r[2, 2] - r[0, 0] - r[1, 1]) * 2.0
        qw = (r[1, 0] - r[0, 1]) / s
        qx = (r[0, 2] + r[2, 0]) / s
        qy = (r[1, 2] + r[2, 1]) / s
        qz = 0.25 * s

    return (float(qx), float(qy), float(qz), float(qw))
