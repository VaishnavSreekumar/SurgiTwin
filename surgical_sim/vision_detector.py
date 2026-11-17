"""
vision_detector.py: OpenCV-based visual target detection and tracking module.

Detects surgical fiducial markers (ArUco DICT_4X4_50) and high-contrast color markers,
computes sub-pixel centroids, tracks across frames, and renders visual overlay diagnostics.
"""

from dataclasses import dataclass
import math
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np


@dataclass
class DetectionResult:
    """Structure encapsulating target detection outcome."""

    detected: bool
    pixel_x: float = 0.0
    pixel_y: float = 0.0
    confidence: float = 0.0
    marker_id: Optional[int] = None
    corners: Optional[np.ndarray] = None
    annotated_image: Optional[np.ndarray] = None
    status_message: str = "NO_DETECTION"


class VisionTargetDetector:
    """OpenCV-based surgical target detector supporting ArUco and color fiducials."""

    def __init__(
        self,
        dictionary_id: int = cv2.aruco.DICT_4X4_50,
        target_marker_id: Optional[int] = None,
        min_contour_area: float = 100.0,
    ) -> None:
        """Initialize detector with ArUco dictionary and tracking state."""
        self.dictionary_id = dictionary_id
        self.target_marker_id = target_marker_id
        self.min_contour_area = min_contour_area

        # ArUco parameters
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.parameters = cv2.aruco.DetectorParameters_create()

        # Frame-to-frame tracking state
        self._last_centroid: Optional[Tuple[float, float]] = None
        self._frame_count: int = 0
        self._consecutive_misses: int = 0

    def detect(self, image: Optional[np.ndarray]) -> DetectionResult:
        """
        Process single input image frame and detect target.

        Fails closed on None, empty image, or malformed dimensions.
        """
        if image is None or not isinstance(image, np.ndarray):
            return DetectionResult(False, status_message="INVALID_INPUT: Image is None or not an ndarray")

        if image.size == 0 or len(image.shape) < 2:
            return DetectionResult(False, status_message="INVALID_INPUT: Empty image dimensions")

        h, w = image.shape[:2]
        if h < 10 or w < 10:
            return DetectionResult(False, status_message="INVALID_INPUT: Image resolution too small (<10x10)")

        self._frame_count += 1

        # Attempt ArUco detection first
        res = self._detect_aruco(image)
        if not res.detected:
            # Fall back to color/contrast circular fiducial detection
            res = self._detect_color_fiducial(image)

        # Annotate image
        annotated = self._render_annotation(image, res)
        res.annotated_image = annotated

        # Update tracking telemetry
        if res.detected:
            self._last_centroid = (res.pixel_x, res.pixel_y)
            self._consecutive_misses = 0
        else:
            self._consecutive_misses += 1

        return res

    def _detect_aruco(self, image: np.ndarray) -> DetectionResult:
        """Perform ArUco marker detection."""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if len(image.shape) == 3 else image
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.dictionary, parameters=self.parameters
        )

        if ids is None or len(ids) == 0:
            return DetectionResult(False, status_message="ARUCO: No markers detected")

        # Search for specific target_marker_id or use first detected
        target_idx = 0
        if self.target_marker_id is not None:
            matches = np.where(ids.flatten() == self.target_marker_id)[0]
            if len(matches) == 0:
                return DetectionResult(
                    False,
                    status_message=f"ARUCO: Marker ID {self.target_marker_id} not found in {ids.flatten().tolist()}",
                )
            target_idx = matches[0]

        marker_id = int(ids[target_idx][0])
        marker_corners = corners[target_idx][0]  # Shape: (4, 2)

        # Centroid calculation: arithmetic mean of 4 corner coordinates
        cx = float(np.mean(marker_corners[:, 0]))
        cy = float(np.mean(marker_corners[:, 1]))

        # Geometric confidence estimation based on marker area and squareness
        area = float(cv2.contourArea(marker_corners))
        confidence = min(1.0, max(0.2, area / 2500.0))  # Scale with pixel coverage

        return DetectionResult(
            detected=True,
            pixel_x=cx,
            pixel_y=cy,
            confidence=confidence,
            marker_id=marker_id,
            corners=marker_corners,
            status_message=f"ARUCO_DETECTED: ID {marker_id} at ({cx:.1f}, {cy:.1f})",
        )

    def _detect_color_fiducial(self, image: np.ndarray) -> DetectionResult:
        """Fallback: Detect high-contrast surgical color fiducial (e.g. green marker)."""
        if len(image.shape) != 3:
            return DetectionResult(False, status_message="COLOR_FALLBACK: Requires 3-channel color image")

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # Green surgical marker threshold
        lower_green = np.array([35, 80, 80])
        upper_green = np.array([85, 255, 255])
        mask = cv2.inRange(hsv, lower_green, upper_green)

        # Morphological filtering to reduce noise
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return DetectionResult(False, status_message="COLOR_FALLBACK: No colored fiducial detected")

        # Select largest contour
        largest = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest)
        if area < self.min_contour_area:
            return DetectionResult(False, status_message="COLOR_FALLBACK: Contour area below threshold")

        m = cv2.moments(largest)
        if m["m00"] == 0:
            return DetectionResult(False, status_message="COLOR_FALLBACK: Zero spatial moment")

        cx = float(m["m10"] / m["m00"])
        cy = float(m["m01"] / m["m00"])

        return DetectionResult(
            detected=True,
            pixel_x=cx,
            pixel_y=cy,
            confidence=0.85,
            marker_id=None,
            corners=largest.reshape(-1, 2),
            status_message=f"COLOR_FIDUCIAL_DETECTED at ({cx:.1f}, {cy:.1f})",
        )

    def _render_annotation(self, image: np.ndarray, res: DetectionResult) -> np.ndarray:
        """Draw bounding overlay, centroid marker, and coordinates text."""
        annotated = image.copy()
        if len(annotated.shape) == 2:
            annotated = cv2.cvtColor(annotated, cv2.COLOR_GRAY2BGR)

        h, w = annotated.shape[:2]

        if res.detected:
            cx, cy = int(round(res.pixel_x)), int(round(res.pixel_y))

            # Draw polygon boundary
            if res.corners is not None and len(res.corners) >= 4:
                pts = res.corners.astype(np.int32).reshape((-1, 1, 2))
                cv2.polylines(annotated, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Draw centroid crosshair and circle
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(annotated, (cx - 15, cy), (cx + 15, cy), (0, 255, 255), 1)
            cv2.line(annotated, (cx, cy - 15), (cx, cy + 15), (0, 255, 255), 1)

            # Text label
            label = f"Target ({cx}, {cy}) Conf:{res.confidence:.2f}"
            cv2.putText(
                annotated,
                label,
                (max(10, cx - 60), max(20, cy - 15)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )
            status_color = (0, 255, 0)
        else:
            status_color = (0, 0, 255)

        # Status banner at top of frame
        cv2.putText(
            annotated,
            f"Status: {res.status_message}",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            status_color,
            2,
        )
        return annotated

    @staticmethod
    def generate_synthetic_aruco_image(
        width: int = 640,
        height: int = 480,
        marker_id: int = 0,
        marker_size_px: int = 120,
        center_x: int = 320,
        center_y: int = 240,
        dictionary_id: int = cv2.aruco.DICT_4X4_50,
    ) -> np.ndarray:
        """
        Generate a synthetic grayscale/BGR image with an ArUco marker at exact pixel coordinates.

        Useful for deterministic testing without webcam hardware.
        """
        img = np.ones((height, width, 3), dtype=np.uint8) * 230  # Light gray background

        # Generate marker bitmap
        dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        marker_img = cv2.aruco.drawMarker(dictionary, marker_id, marker_size_px)
        marker_bgr = cv2.cvtColor(marker_img, cv2.COLOR_GRAY2BGR)

        half = marker_size_px // 2
        x1 = max(0, center_x - half)
        y1 = max(0, center_y - half)
        x2 = min(width, x1 + marker_size_px)
        y2 = min(height, y1 + marker_size_px)

        img[y1:y2, x1:x2] = marker_bgr[: y2 - y1, : x2 - x1]
        return img

