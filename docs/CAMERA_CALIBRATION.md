# SurgiTwin — Camera Calibration & 3D Reprojection Specification

## 1. Mathematical Formulation

SurgiTwin models the optical sensor using the standard pinhole camera model.

### 1.1. Pinhole Projection Matrix
The relationship between a 3D point in camera coordinates $\mathbf{P}_c = [X_c, Y_c, Z_c]^T$ and its 2D image pixel coordinates $\mathbf{p} = [u, v]^T$ is governed by:
$$s \begin{bmatrix} u \\ v \\ 1 \end{bmatrix} = K \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix} = \begin{bmatrix} f_x & 0 & c_x \\ 0 & f_y & c_y \\ 0 & 0 & 1 \end{bmatrix} \begin{bmatrix} X_c \\ Y_c \\ Z_c \end{bmatrix}$$
where:
- $f_x, f_y$ are the focal lengths in pixel units.
- $c_x, c_y$ is the principal point (optical center).
- $s = Z_c$ is the projective scale factor (depth along the optical axis).

---

### 1.2. 3D Back-Projection (Pixel-to-Camera)
Because a single 2D pixel $[u, v]^T$ represents a ray in 3D space rather than a unique point, depth $Z_c$ must be provided:
$$\mathbf{P}_c = Z_c \cdot K^{-1} \begin{bmatrix} u \\ v \\ 1 \end{bmatrix}$$
Expanded into Cartesian coordinates:
$$X_c = \frac{(u - c_x) \cdot Z_c}{f_x}$$
$$Y_c = \frac{(v - c_y) \cdot Z_c}{f_y}$$
$$Z_c = Z_c$$

### Depth Determination Methods in SurgiTwin:
1. **Constrained Surgical Table Plane**: For instruments operating on an anatomical plane at a known elevation above the operating table, $Z_c$ is constrained by planar intersection ($Z_c = 0.60\text{ m}$ nominal working distance).
2. **Known-Size Fiducial Marker Pose Estimation**: Solves the Perspective-n-Point (PnP) problem using calibrated intrinsics and the known metric edge size of the ArUco marker ($L = 0.05\text{ m}$).

---

## 2. Calibration File Format (`config/camera_calibration.yaml`)

```yaml
image_width: 640
image_height: 480
camera_matrix:
  rows: 3
  cols: 3
  data: [554.256, 0.0, 320.0, 0.0, 554.256, 240.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [0.0, 0.0, 0.0, 0.0, 0.0]
reprojection_error: 0.0
```

---

## 3. Synthetic Assumptions & Physical Verification

> [!NOTE]
> **Synthetic Calibration Baseline**: In accordance with the educational simulation scope, SurgiTwin provides a mathematically verified synthetic calibration model based on a 60° horizontal field-of-view sensor with zero radial distortion. For physical cameras, the integrated `calibrate_checkerboard()` routine processes standard $9 \times 6$ checkerboard datasets to solve real-world distortion coefficients $[k_1, k_2, p_1, p_2, k_3]$.

