"""
medical_registration.py
SurgiTwin Medical-Image-to-Robot Coordinate Registration Engine.

Transforms physical DICOM patient coordinates (LPS convention, millimeters)
into SurgiTwin robot base coordinates (meters) via rigorous 4x4 rigid SE(3) transformations.
Includes rotation orthogonality and determinant verification, YAML serialization,
and synthetic surgical table calibration defaults.
"""

import os
from typing import Any, Dict, List, Optional, Tuple, Union
import numpy as np
import yaml

from surgical_sim.anatomical_target import AnatomicalTarget


class MedicalRegistration:
    """
    Coordinate registration mapping DICOM patient space (mm) to Robot Base frame (m).

    Formula:
      [X_base, Y_base, Z_base, 1]^T = T_base_patient * [X_p/1000, Y_p/1000, Z_p/1000, 1]^T
    """

    def __init__(
        self,
        t_matrix: Union[List[List[float]], np.ndarray],
        source_frame: str = "patient_lps",
        target_frame: str = "base_link",
        is_simulated: bool = True,
    ):
        """
        Initialize registration transform.

        :param t_matrix: 4x4 homogeneous transformation matrix in meters.
        :param source_frame: Source coordinate frame identifier.
        :param target_frame: Target coordinate frame identifier.
        :param is_simulated: Flag indicating whether registration is simulated.
        """
        self.t_matrix = np.array(t_matrix, dtype=np.float64)
        if self.t_matrix.shape != (4, 4):
            raise ValueError(f"Transformation matrix must be 4x4, got {self.t_matrix.shape}")

        self.source_frame = source_frame
        self.target_frame = target_frame
        self.is_simulated = bool(is_simulated)

        self._validate_matrix()

    def _validate_matrix(self):
        """Validate SE(3) rigid-body properties: R*R^T = I and det(R) = +1.0."""
        r = self.t_matrix[:3, :3]
        bottom = self.t_matrix[3, :]

        if not np.allclose(bottom, [0.0, 0.0, 0.0, 1.0], atol=1e-5):
            raise ValueError(f"Bottom row of transform must be [0,0,0,1], got {bottom}")

        # Orthogonality
        r_rt = r @ r.T
        if not np.allclose(r_rt, np.eye(3), atol=1e-4):
            raise ValueError(f"Rotation matrix is not orthogonal: R*R^T =\n{r_rt}")

        # Proper rotation (no reflection)
        det_r = float(np.linalg.det(r))
        if not np.isclose(det_r, 1.0, atol=1e-4):
            raise ValueError(f"Rotation matrix determinant must be +1.0 (got {det_r:.4f})")

    @property
    def rotation_matrix(self) -> np.ndarray:
        return self.t_matrix[:3, :3].copy()

    @property
    def translation_vector(self) -> np.ndarray:
        return self.t_matrix[:3, 3].copy()

    @classmethod
    def create_synthetic_supine_table(
        cls,
        patient_center_on_table: Tuple[float, float, float] = (0.0, 0.35, 0.18),
    ) -> "MedicalRegistration":
        """
        Create a synthetic patient registration representing a patient lying supine on an operating table
        positioned directly in front of the SurgiTwin robot base.

        Patient orientation:
        - Patient Left (+X_p) aligns with Robot +X
        - Patient Posterior (+Y_p, back) aligns with Robot -Z (lying on table)
        - Patient Superior (+Z_p, head) aligns with Robot +Y (extending along table length)

        Rotation matrix:
          R = [[1,  0,  0],
               [0,  0,  1],
               [0, -1,  0]]
        """
        tx, ty, tz = patient_center_on_table
        t = np.array([
            [1.0,  0.0,  0.0, float(tx)],
            [0.0,  0.0,  1.0, float(ty)],
            [0.0, -1.0,  0.0, float(tz)],
            [0.0,  0.0,  0.0,       1.0],
        ], dtype=np.float64)

        return cls(t_matrix=t, is_simulated=True)

    def transform_point_mm(self, x_p: float, y_p: float, z_p: float) -> Tuple[float, float, float]:
        """
        Transform a physical DICOM coordinate (x, y, z) in mm to robot base frame (meters).
        """
        p_m = np.array([float(x_p) / 1000.0, float(y_p) / 1000.0, float(z_p) / 1000.0, 1.0], dtype=np.float64)
        p_base = self.t_matrix @ p_m
        return (float(p_base[0]), float(p_base[1]), float(p_base[2]))

    def inverse_transform_point_m(self, x_b: float, y_b: float, z_b: float) -> Tuple[float, float, float]:
        """
        Transform robot base frame coordinates (meters) back to physical DICOM coordinates (mm).
        """
        t_inv = np.linalg.inv(self.t_matrix)
        p_b = np.array([float(x_b), float(y_b), float(z_b), 1.0], dtype=np.float64)
        p_p_m = t_inv @ p_b
        return (float(p_p_m[0]) * 1000.0, float(p_p_m[1]) * 1000.0, float(p_p_m[2]) * 1000.0)

    def transform_target(self, target: AnatomicalTarget) -> Tuple[float, float, float]:
        """
        Transform an extracted AnatomicalTarget centroid into robot base coordinates (meters).
        """
        if not target.valid:
            raise ValueError(f"Cannot transform invalid anatomical target: {target.status_message}")

        return self.transform_point_mm(*target.centroid_patient_mm)

    def to_dict(self) -> Dict[str, Any]:
        """Serialize registration parameters to dictionary."""
        return {
            "source_frame": self.source_frame,
            "target_frame": self.target_frame,
            "is_simulated": self.is_simulated,
            "units": "meters",
            "matrix": self.t_matrix.tolist(),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "MedicalRegistration":
        """Construct registration from dictionary."""
        matrix = np.array(data["matrix"], dtype=np.float64)
        src = data.get("source_frame", "patient_lps")
        tgt = data.get("target_frame", "base_link")
        sim = data.get("is_simulated", True)
        return cls(t_matrix=matrix, source_frame=src, target_frame=tgt, is_simulated=sim)

    def save_yaml(self, filepath: str):
        """Save registration matrix to YAML configuration."""
        expanded = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(expanded) or ".", exist_ok=True)
        with open(expanded, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_dict(), f, default_flow_style=False)

    @classmethod
    def load_yaml(cls, filepath: str) -> "MedicalRegistration":
        """Load registration matrix from YAML configuration."""
        expanded = os.path.expanduser(filepath)
        if not os.path.exists(expanded):
            raise FileNotFoundError(f"Registration file not found: {expanded}")
        with open(expanded, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)
