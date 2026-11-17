"""
test_vision_detection.py.

Automated unit tests for OpenCV-based surgical target and marker detection.
Verifies ArUco marker detection, sub-pixel centroid accuracy, missing target handling,
color fiducial fallback, and malformed input rejection.
"""

import cv2
import numpy as np
import pytest

from surgical_sim.vision_detector import DetectionResult, VisionTargetDetector


@pytest.fixture
def detector() -> VisionTargetDetector:
    """Fixture providing default VisionTargetDetector."""
    return VisionTargetDetector(target_marker_id=0)


def test_valid_aruco_marker_detection(detector: VisionTargetDetector) -> None:
    """Scenario 1: Synthetic ArUco marker image is accurately detected."""
    img = VisionTargetDetector.generate_synthetic_aruco_image(
        width=640, height=480, marker_id=0, center_x=320, center_y=240, marker_size_px=100
    )
    res = detector.detect(img)

    assert res.detected is True
    assert res.marker_id == 0
    assert pytest.approx(res.pixel_x, abs=2.0) == 320.0
    assert pytest.approx(res.pixel_y, abs=2.0) == 240.0
    assert res.confidence > 0.0
    assert res.annotated_image is not None


def test_missing_target_returns_no_detection(detector: VisionTargetDetector) -> None:
    """Scenario 2: Blank or noise image without markers returns detected=False."""
    blank_img = np.ones((480, 640, 3), dtype=np.uint8) * 128
    res = detector.detect(blank_img)

    assert res.detected is False
    assert "No markers detected" in res.status_message or "No colored fiducial" in res.status_message
    assert res.annotated_image is not None


def test_invalid_image_inputs_rejected(detector: VisionTargetDetector) -> None:
    """Scenario 3: None, empty array, or zero-dimension images fail closed without crash."""
    # None input
    res_none = detector.detect(None)
    assert res_none.detected is False
    assert "INVALID_INPUT" in res_none.status_message

    # Empty array
    res_empty = detector.detect(np.array([], dtype=np.uint8))
    assert res_empty.detected is False
    assert "INVALID_INPUT" in res_empty.status_message

    # Tiny resolution (<10x10)
    res_tiny = detector.detect(np.zeros((5, 5, 3), dtype=np.uint8))
    assert res_tiny.detected is False
    assert "INVALID_INPUT" in res_tiny.status_message


def test_pixel_centroid_accuracy_at_different_locations(detector: VisionTargetDetector) -> None:
    """Scenario 4: Verify centroid accuracy at various spatial pixel coordinates."""
    test_coords = [(150, 150), (450, 200), (250, 380)]
    for tx, ty in test_coords:
        img = VisionTargetDetector.generate_synthetic_aruco_image(
            width=640, height=480, marker_id=0, center_x=tx, center_y=ty, marker_size_px=80
        )
        res = detector.detect(img)
        assert res.detected is True
        assert pytest.approx(res.pixel_x, abs=2.5) == float(tx)
        assert pytest.approx(res.pixel_y, abs=2.5) == float(ty)


def test_color_fiducial_fallback_detection() -> None:
    """Verify fallback detection of green circular surgical fiducial when ArUco absent."""
    det = VisionTargetDetector(target_marker_id=99)  # Non-existent ArUco ID
    img = np.ones((480, 640, 3), dtype=np.uint8) * 200  # Gray background

    # Draw green circle (HSV around H=60, S=200, V=200 -> BGR [0, 200, 0])
    cv2.circle(img, (300, 200), radius=25, color=(35, 200, 35), thickness=-1)

    res = det.detect(img)
    assert res.detected is True
    assert pytest.approx(res.pixel_x, abs=3.0) == 300.0
    assert pytest.approx(res.pixel_y, abs=3.0) == 200.0
    assert "COLOR_FIDUCIAL_DETECTED" in res.status_message

