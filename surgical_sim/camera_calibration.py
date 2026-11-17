"""
camera_calibration.py: Camera calibration and 3D back-projection module.

Provides camera intrinsic parameter modeling, checkerboard calibration using OpenCV,
YAML serialization, and mathematical pinhole projection/back-projection:
    X_camera = Z * K_inverse * [u, v, 1]^T
"""

from dataclasses import dataclass
import math
import os
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml


@dataclass
class CameraCalibration:
    """Camera intrinsic calibration parameters and distortion coefficients."""

    camera_matrix: np.ndarray  # 3x3 intrinsic matrix K
    dist_coeffs: np.ndarray    # 1x5 or 5-element distortion coefficients [k1, k2, p1, p2, k3]
    image_width: int
    image_height: int
    reprojection_error: float = 0.0

    def __post_init__(self) -> None:
        """Validate intrinsic parameters on instantiation."""
        self.validate()

    def validate(self) -> None:
        """Validate matrix shape, positive focal lengths, and principal point bounds."""
        if self.camera_matrix.shape != (3, 3):
            raise ValueError(f"Camera matrix must be 3x3, got {self.camera_matrix.shape}")
        if self.dist_coeffs.size < 4:
            raise ValueError(f"Distortion coefficients must have at least 4 elements, got {self.dist_coeffs.size}")
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError(f"Invalid image dimensions: {self.image_width}x{self.image_height}")

        fx = self.camera_matrix[0, 0]
        fy = self.camera_matrix[1, 1]
        cx = self.camera_matrix[0, 2]
        cy = self.camera_matrix[1, 2]

        if fx <= 0 or fy <= 0:
            raise ValueError(f"Focal lengths must be positive: fx={fx}, fy={fy}")
        if not (0 <= cx <= self.image_width):
            raise ValueError(f"Principal point cx ({cx}) outside image width [0, {self.image_width}]")
        if not (0 <= cy <= self.image_height):
            raise ValueError(f"Principal point cy ({cy}) outside image height [0, {self.image_height}]")

    @property
    def fx(self) -> float:
        """Focal length along X axis (pixels)."""
        return float(self.camera_matrix[0, 0])

    @property
    def fy(self) -> float:
        """Focal length along Y axis (pixels)."""
        return float(self.camera_matrix[1, 1])

    @property
    def cx(self) -> float:
        """Principal point X coordinate (pixels)."""
        return float(self.camera_matrix[0, 2])

    @property
    def cy(self) -> float:
        """Principal point Y coordinate (pixels)."""
        return float(self.camera_matrix[1, 2])

    def back_project(self, u: float, v: float, z: float) -> Tuple[float, float, float]:
        """Back-project 2D pixel coordinates (u, v) with known depth Z to 3D camera coordinates."""
        if z <= 0.0 or math.isnan(z) or math.isinf(z):
            raise ValueError(f"Invalid depth Z={z}. Depth must be strictly positive and finite.")

        x_c = (u - self.cx) * z / self.fx
        y_c = (v - self.cy) * z / self.fy
        return (float(x_c), float(y_c), float(z))

    def project(self, x_c: float, y_c: float, z_c: float) -> Tuple[float, float]:
        """
        Project 3D point in camera coordinates onto 2D image plane (pixels).

        Formula:
            u = fx * (X_c / Z_c) + cx
            v = fy * (Y_c / Z_c) + cy
        """
        if z_c <= 0.0:
            raise ValueError(f"Point must be in front of camera (Z_c > 0), got Z_c={z_c}")

        u = self.fx * (x_c / z_c) + self.cx
        v = self.fy * (y_c / z_c) + self.cy
        return (float(u), float(v))

    def to_dict(self) -> Dict[str, Any]:
        """Serialize calibration parameters to standard dictionary."""
        return {
            "image_width": int(self.image_width),
            "image_height": int(self.image_height),
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": self.camera_matrix.flatten().tolist(),
            },
            "distortion_coefficients": {
                "rows": 1,
                "cols": int(self.dist_coeffs.size),
                "data": self.dist_coeffs.flatten().tolist(),
            },
            "reprojection_error": float(self.reprojection_error),
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "CameraCalibration":
        """Construct CameraCalibration from dictionary structure."""
        width = int(data["image_width"])
        height = int(data["image_height"])
        c_data = data["camera_matrix"]["data"]
        k = np.array(c_data, dtype=np.float64).reshape((3, 3))
        d_data = data["distortion_coefficients"]["data"]
        d = np.array(d_data, dtype=np.float64).reshape((1, -1))
        err = float(data.get("reprojection_error", 0.0))
        return cls(camera_matrix=k, dist_coeffs=d, image_width=width, image_height=height, reprojection_error=err)

    def save_yaml(self, filepath: str) -> None:
        """Save camera calibration parameters to YAML file."""
        expanded = os.path.expanduser(filepath)
        os.makedirs(os.path.dirname(expanded) or ".", exist_ok=True)
        with open(expanded, "w", encoding="utf-8") as f:
            yaml.dump(self.to_dict(), f, default_flow_style=False)

    @classmethod
    def load_yaml(cls, filepath: str) -> "CameraCalibration":
        """Load and validate camera calibration from YAML file."""
        expanded = os.path.expanduser(filepath)
        if not os.path.isfile(expanded):
            raise FileNotFoundError(f"Calibration file not found: {expanded}")
        with open(expanded, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls.from_dict(data)

    @classmethod
    def create_synthetic(
        cls, width: int = 640, height: int = 480, hfov_deg: float = 60.0
    ) -> "CameraCalibration":
        """
        Create a verified synthetic camera calibration model.

        Focal length computed from horizontal field of view:
            fx = (width / 2) / tan(hfov / 2)
            fy = fx (square pixels assumed)
            cx = width / 2
            cy = height / 2
        """
        hfov_rad = math.radians(hfov_deg)
        fx = (width / 2.0) / math.tan(hfov_rad / 2.0)
        fy = fx
        cx = width / 2.0
        cy = height / 2.0

        k = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
        dist = np.zeros((1, 5), dtype=np.float64)
        return cls(camera_matrix=k, dist_coeffs=dist, image_width=width, image_height=height, reprojection_error=0.0)


def calibrate_checkerboard(
    images: List[np.ndarray],
    pattern_size: Tuple[int, int] = (9, 6),
    square_size_m: float = 0.025,
) -> CameraCalibration:
    """Perform camera calibration using a set of OpenCV checkerboard images."""
    if not images:
        raise ValueError("Cannot calibrate with empty image list")

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size_m

    objpoints = []
    imgpoints = []

    h, w = images[0].shape[:2]

    for img in images:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)
        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_sub = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            objpoints.append(objp)
            imgpoints.append(corners_sub)

    if len(objpoints) < 3:
        raise ValueError(
            f"Insufficient valid checkerboards detected ({len(objpoints)} found, at least 3 required)"
        )

    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, (w, h), None, None)
    return CameraCalibration(
        camera_matrix=mtx,
        dist_coeffs=dist,
        image_width=w,
        image_height=h,
        reprojection_error=float(ret),
    )
