# SurgiTwin — Phase 2: Computer Vision & Coordinate Registration Implementation Report

## 1. Executive Summary

Phase 2 of the SurgiTwin Vision-Guided Surgical Robotics Digital Twin delivers a complete, mathematically rigorous visual perception pipeline. It maps raw 2D image coordinates to 3D Cartesian coordinates in the camera optical frame using calibrated camera intrinsics, and subsequently registers those points into the robot's `base_link` frame using homogeneous rigid-body transformation matrices.

The pipeline integrates directly with the Phase 1 Safety Supervisor, publishing targets to `/target_pose` and ensuring that only geometrically valid, in-bounds, safe targets ever reach the robot's downstream trajectory interface (`/safe_target_pose`).

---

## 2. End-to-End Pipeline Data Flow

```
[Camera Input / Synthetic Feed]
              │
              ▼  (sensor_msgs/msg/Image)
  [VisionTargetDetector] ────────────> Detects ArUco / Color Fiducials
              │                         Extracts Sub-Pixel Centroid (u, v)
              ▼
  [CameraCalibration] ───────────────> Pinhole Back-Projection with Depth Z:
              │                         X_c = (u - cx) * Z / fx
              │                         Y_c = (v - cy) * Z / fy
              │                         Z_c = Z
              ▼
 [CoordinateRegistration] ───────────> Rigid-Body Homogeneous Transform (4x4):
              │                         P_base = T_base_camera * P_camera
              │                         R_base = R_base_camera * R_camera
              ▼
       [/target_pose]  (base_link frame)
              │
              ▼
 [Phase 1 Safety Supervisor] ────────> Workspace Bounds, Velocity & E-Stop Checks
              │
              ▼  (If Valid)
    [/safe_target_pose]
```

---

## 3. Implemented Components

### 3.1. Camera Calibration (`surgical_sim/camera_calibration.py`)
- **Pinhole Intrinsic Matrix $K$**:
  $$K = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix}$$
- **Mathematical Back-Projection**: Given calibrated intrinsics and estimated/known working depth $Z$:
  $$X_c = \frac{(u - c_x) \cdot Z}{f_x}, \quad Y_c = \frac{(v - c_y) \cdot Z}{f_y}, \quad Z_c = Z$$
- **Validation Rules**: Strict checking of $f_x, f_y > 0$, principal point within image boundaries, $Z > 0$, and non-nan/non-infinite coordinates.
- **Checkerboard Calibration Engine**: Uses OpenCV's `cv2.calibrateCamera` with sub-pixel corner refinement (`cv2.cornerSubPix`).
- **YAML Serialization**: Standard OpenCV/ROS calibration format (`save_yaml`, `load_yaml`).

### 3.2. Coordinate Registration (`surgical_sim/coordinate_registration.py`)
- **Homogeneous Transformation Matrix ($4 \times 4$)**:
  $$T_{base}^{cam} = \begin{bmatrix} R & t \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$
- **Rigid-Body Validation**:
  - Orthogonality constraint: $R \cdot R^T = I$ within $10^{-3}$ tolerance.
  - Proper rotation determinant: $\det(R) = +1.0$ (rejects reflection matrices).
  - Bottom row invariance: $[0, 0, 0, 1]$.
- **Overhead Mounting Geometry**: Configured for an operating theater camera positioned at $(0.0, 0.35, 0.80)\text{ m}$ pointing down at the surgical table. Target at optical center at depth $Z=0.60\text{ m}$ maps to robot home position $(0.0, 0.35, 0.20)\text{ m}$.

### 3.3. Vision Target Detector (`surgical_sim/vision_detector.py`)
- **Fiducial Detection**: Detects ArUco markers (`DICT_4X4_50`) with sub-pixel corner extraction.
- **Centroid Calculation**: $\bar{u} = \frac{1}{4}\sum_{i=1}^4 u_i, \quad \bar{v} = \frac{1}{4}\sum_{i=1}^4 v_i$.
- **Color Fiducial Fallback**: Green surgical contrast marker detection via HSV color thresholding and spatial contour moments ($m_{10}/m_{00}, m_{01}/m_{00}$).
- **Fail-Closed Malformed Input Protection**: Explicit rejection of None, empty, or sub-resolution image matrices.
- **Diagnostic Annotation**: Renders bounding polygons, centroid crosshairs, coordinates, and status banners.

### 3.4. ROS 2 Vision Target Node (`surgical_sim/vision_target_node.py`)
- **Subscribed Topics**: `/camera/image_raw` (`sensor_msgs/msg/Image`).
- **Published Topics**:
  - `/vision/detection_image` (`sensor_msgs/msg/Image`) — visual overlay stream.
  - `/vision/target_pixel` (`geometry_msgs/msg/PointStamped`) — pixel coordinates.
  - `/vision/target_pose_camera` (`geometry_msgs/msg/PoseStamped`) — 3D camera pose.
  - `/vision/target_pose_base` (`geometry_msgs/msg/PoseStamped`) — 3D base pose.
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`) — forwarded to Phase 1 Safety Supervisor.

---

## 4. Verification and Automated Test Results

52 automated tests executed via `colcon test` and `pytest`:
- **Phase 1 Safety Tests**: 28 passed.
- **`test_vision_detection.py`** (5 tests): ArUco marker detection, missing target rejection, malformed image rejection, centroid accuracy, color fiducial fallback.
- **`test_camera_calibration.py`** (4 tests): Intrinsic matrix validation, invalid parameters rejection, distortion coefficients, YAML round-trip.
- **`test_3d_reprojection.py`** (4 tests): Optical center back-projection, linear off-center scaling, projection round-trip, invalid depth rejection.
- **`test_coordinate_registration.py`** (7 tests): Identity, translation, rotation, invalid matrix rejection, overhead mounting, quaternion conversion, YAML serialization.
- **`test_vision_integration.py`** (1 test): Complete end-to-end vision-to-safety test with in-bounds target forwarding, blank image fail-closed rejection, and out-of-bounds safety interception.
- **Linters**: `test_flake8.py` and `test_pep257.py` passed with 0 errors.
- **Overall Result**: **51 Passed, 1 Skipped (upstream linter skip), 0 Failed.**

---

## 5. Live Demonstration Evidence

Executed via `demo_phase2.sh`:
- Case A (In-Bounds Target): Marker at pixel $(319.5, 239.5)$ $\rightarrow$ 3D Camera $(-0.0005, -0.0005, 0.6000)\text{ m}$ $\rightarrow$ 3D Base $(-0.0005, 0.3505, 0.2000)\text{ m}$ $\rightarrow$ **ACCEPTED** by Safety Supervisor and forwarded to `/safe_target_pose`.
- Case B (Out-of-Bounds Target): Marker displaced to $(569.5, 239.5)$ at depth $0.80\text{ m}$ $\rightarrow$ 3D Base $X=0.3601\text{ m} > 0.30\text{ m}$ limit $\rightarrow$ **REJECTED** by Safety Supervisor.
- Diagnostic overlays exported to:
  - `docs/images/phase2_detection.png`
  - `docs/images/phase2_oob_rejection.png`

