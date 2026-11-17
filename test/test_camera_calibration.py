"""
test_camera_calibration.py.

Automated unit tests for camera intrinsic parameters, validation rules,
and YAML serialization in SurgiTwin.
"""

import tempfile
import numpy as np
import pytest

from surgical_sim.camera_calibration import CameraCalibration


@pytest.fixture
def synthetic_calib() -> CameraCalibration:
    """Fixture providing verified synthetic camera calibration."""
    return CameraCalibration.create_synthetic(width=640, height=480, hfov_deg=60.0)


def test_camera_intrinsic_matrix_validation(synthetic_calib: CameraCalibration) -> None:
    """Scenario 5: Validate intrinsic properties and principal point."""
    assert synthetic_calib.image_width == 640
    assert synthetic_calib.image_height == 480
    assert synthetic_calib.fx > 0.0
    assert synthetic_calib.fy > 0.0
    assert synthetic_calib.cx == 320.0
    assert synthetic_calib.cy == 240.0
    assert synthetic_calib.camera_matrix.shape == (3, 3)


def test_invalid_intrinsics_rejection() -> None:
    """Scenario 5b: Rejection of non-positive focal length and out-of-bounds principal point."""
    # Negative focal length fx
    k_bad_fx = np.array([[-500.0, 0.0, 320.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError, match="Focal lengths must be positive"):
        CameraCalibration(k_bad_fx, np.zeros((1, 5)), 640, 480)

    # Principal point outside image bounds
    k_bad_cx = np.array([[500.0, 0.0, 800.0], [0.0, 500.0, 240.0], [0.0, 0.0, 1.0]])
    with pytest.raises(ValueError, match="Principal point cx"):
        CameraCalibration(k_bad_cx, np.zeros((1, 5)), 640, 480)


def test_distortion_parameter_handling(synthetic_calib: CameraCalibration) -> None:
    """Scenario 6: Distortion coefficients must have at least 4 parameters."""
    assert synthetic_calib.dist_coeffs.size >= 4
    with pytest.raises(ValueError, match="Distortion coefficients must have at least 4"):
        CameraCalibration(synthetic_calib.camera_matrix, np.array([0.0, 0.0]), 640, 480)


def test_yaml_serialization_round_trip(synthetic_calib: CameraCalibration) -> None:
    """Scenario 6b: Save to YAML and reload preserves numerical integrity."""
    with tempfile.NamedTemporaryFile(suffix=".yaml", delete=False) as tmp:
        synthetic_calib.save_yaml(tmp.name)
        loaded = CameraCalibration.load_yaml(tmp.name)

        assert loaded.image_width == synthetic_calib.image_width
        assert loaded.image_height == synthetic_calib.image_height
        assert np.allclose(loaded.camera_matrix, synthetic_calib.camera_matrix)
        assert np.allclose(loaded.dist_coeffs, synthetic_calib.dist_coeffs)
        assert loaded.reprojection_error == synthetic_calib.reprojection_error

