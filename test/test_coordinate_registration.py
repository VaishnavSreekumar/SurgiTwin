"""
test_coordinate_registration.py.

Automated unit tests for homogeneous coordinate transformation matrices (4x4),
rotation matrix validity, determinant checks, and camera-to-robot frame mapping.
"""

import math
import tempfile
import numpy as np
import pytest

from surgical_sim.coordinate_registration import (
    CoordinateRegistration,
    matrix_to_quaternion,
    quaternion_to_matrix,
)


def test_identity_coordinate_transformation() -> None:
    """Scenario 10: Identity transformation leaves 3D coordinates unchanged."""
    reg = CoordinateRegistration.create_identity()
    px, py, pz = reg.transform_point(0.12, -0.34, 0.56)

    assert pytest.approx(px, abs=1e-6) == 0.12
    assert pytest.approx(py, abs=1e-6) == -0.34
    assert pytest.approx(pz, abs=1e-6) == 0.56


def test_pure_translation_transformation() -> None:
    """Scenario 11: Pure translation shifts point by exact vector offset."""
    t_mat = np.eye(4)
    t_mat[0, 3] = 0.50
    t_mat[1, 3] = -0.20
    t_mat[2, 3] = 1.00

    reg = CoordinateRegistration(t_mat)
    px, py, pz = reg.transform_point(0.10, 0.20, 0.30)

    assert pytest.approx(px, abs=1e-6) == 0.60
    assert pytest.approx(py, abs=1e-6) == 0.00
    assert pytest.approx(pz, abs=1e-6) == 1.30


def test_pure_rotation_transformation() -> None:
    """Scenario 12: 90 degree yaw rotation around Z axis."""
    # 90 deg around Z: X' = -Y, Y' = X, Z' = Z
    t_mat = np.array(
        [[0.0, -1.0, 0.0, 0.0], [1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 1.0]]
    )
    reg = CoordinateRegistration(t_mat)
    px, py, pz = reg.transform_point(1.0, 0.0, 0.5)

    assert pytest.approx(px, abs=1e-6) == 0.0
    assert pytest.approx(py, abs=1e-6) == 1.0
    assert pytest.approx(pz, abs=1e-6) == 0.5


def test_invalid_transformation_matrices_rejected() -> None:
    """Scenario 13: Wrong shapes, non-orthogonal rotations, and reflection matrices are rejected."""
    # Wrong dimension (3x3 instead of 4x4)
    with pytest.raises(ValueError, match="Transformation matrix must be 4x4"):
        CoordinateRegistration(np.eye(3))

    # Corrupted bottom row
    bad_bottom = np.eye(4)
    bad_bottom[3, 0] = 1.0
    with pytest.raises(ValueError, match="Bottom row must be"):
        CoordinateRegistration(bad_bottom)

    # Reflection matrix (det = -1.0)
    reflect_mat = np.eye(4)
    reflect_mat[0, 0] = -1.0  # det(R) = -1.0
    with pytest.raises(ValueError, match="determinant must be"):
        CoordinateRegistration(reflect_mat)

    # Non-orthogonal rotation
    skew_mat = np.eye(4)
    skew_mat[0, 1] = 0.5
    with pytest.raises(ValueError, match="not orthogonal"):
        CoordinateRegistration(skew_mat)


def test_overhead_surgical_camera_registration_to_robot_home() -> None:
    """Scenario 14: Overhead camera at (0, 0.35, 0.80) maps depth 0.60m to robot home (0, 0.35, 0.20)m."""
    reg = CoordinateRegistration.create_synthetic_overhead(camera_pos=(0.0, 0.35, 0.80))

    # Optical center target at Z=0.60m
    x_base, y_base, z_base = reg.transform_point(0.0, 0.0, 0.60)

    assert pytest.approx(x_base, abs=1e-5) == 0.00
    assert pytest.approx(y_base, abs=1e-5) == 0.35
    assert pytest.approx(z_base, abs=1e-5) == 0.20


def test_quaternion_matrix_conversion_round_trip() -> None:
    """Scenario 14b: Quaternion to matrix and matrix to quaternion conversions."""
    # 180 deg around X: q = [1, 0, 0, 0]
    r = quaternion_to_matrix(1.0, 0.0, 0.0, 0.0)
    qx, qy, qz, qw = matrix_to_quaternion(r)

    assert pytest.approx(abs(qx), abs=1e-4) == 1.0
    assert pytest.approx(qy, abs=1e-4) == 0.0
    assert pytest.approx(qz, abs=1e-4) == 0.0
    assert pytest.approx(qw, abs=1e-4) == 0.0


def test_extrinsic_yaml_serialization_round_trip() -> None:
    """Scenario 14c: Extrinsic registration YAML save/load."""
    reg = CoordinateRegistration.create_synthetic_overhead(camera_pos=(0.1, 0.4, 0.9))
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as tmp:
        reg.save_yaml(tmp.name)
        loaded = CoordinateRegistration.load_yaml(tmp.name)
        assert np.allclose(loaded.t_matrix, reg.t_matrix)

