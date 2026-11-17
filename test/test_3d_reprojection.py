"""
test_3d_reprojection.py.

Automated unit tests for 2D pixel to 3D Cartesian back-projection and forward projection.
Verifies camera pinhole math: X_c = (u - cx) * Z / fx, Y_c = (v - cy) * Z / fy, Z_c = Z.
"""

import math
import pytest

from surgical_sim.camera_calibration import CameraCalibration


@pytest.fixture
def calib() -> CameraCalibration:
    """Fixture providing synthetic camera calibration with known parameters."""
    return CameraCalibration.create_synthetic(width=640, height=480, hfov_deg=60.0)


def test_optical_center_back_projection_at_known_depth(calib: CameraCalibration) -> None:
    """Scenario 7: Pixel at optical center (cx, cy) projects directly onto optical Z axis."""
    u, v = calib.cx, calib.cy
    z = 0.50  # 500 mm depth

    x_c, y_c, z_c = calib.back_project(u, v, z)
    assert pytest.approx(x_c, abs=1e-5) == 0.0
    assert pytest.approx(y_c, abs=1e-5) == 0.0
    assert pytest.approx(z_c, abs=1e-5) == 0.50


def test_off_center_pixel_back_projection_math(calib: CameraCalibration) -> None:
    """Scenario 7b: Off-center pixel back-projection verifies linear angle scaling."""
    z = 1.0  # 1 metre depth
    # Displace by +100 px in u and +50 px in v
    u = calib.cx + 100.0
    v = calib.cy + 50.0

    x_c, y_c, z_c = calib.back_project(u, v, z)
    expected_x = 100.0 * z / calib.fx
    expected_y = 50.0 * z / calib.fy

    assert pytest.approx(x_c, rel=1e-5) == expected_x
    assert pytest.approx(y_c, rel=1e-5) == expected_y
    assert pytest.approx(z_c, rel=1e-5) == 1.0


def test_projection_round_trip(calib: CameraCalibration) -> None:
    """Scenario 7c: Forward projection matches original back-projected pixel."""
    orig_u, orig_v = 450.0, 310.0
    depth = 0.75

    x_c, y_c, z_c = calib.back_project(orig_u, orig_v, depth)
    proj_u, proj_v = calib.project(x_c, y_c, z_c)

    assert pytest.approx(proj_u, abs=1e-4) == orig_u
    assert pytest.approx(proj_v, abs=1e-4) == orig_v


def test_invalid_depth_rejection(calib: CameraCalibration) -> None:
    """Scenario 8: Non-positive, NaN, and infinite depth values must be rejected."""
    # Zero depth
    with pytest.raises(ValueError, match="Invalid depth"):
        calib.back_project(320.0, 240.0, 0.0)

    # Negative depth (behind camera)
    with pytest.raises(ValueError, match="Invalid depth"):
        calib.back_project(320.0, 240.0, -0.5)

    # NaN depth
    with pytest.raises(ValueError, match="Invalid depth"):
        calib.back_project(320.0, 240.0, float("nan"))

    # Infinite depth
    with pytest.raises(ValueError, match="Invalid depth"):
        calib.back_project(320.0, 240.0, math.inf)

