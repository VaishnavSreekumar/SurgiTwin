# SurgiTwin — Coordinate Registration & Extrinsics Specification

## 1. Mathematical Formulation

Coordinate registration maps 3D points from the sensor coordinate frame (`camera_optical_frame`) into the robot reference frame (`base_link`).

### 1.1. Homogeneous Rigid-Body Transformation Matrix
A rigid-body spatial transform in $\mathbb{SE}(3)$ is represented by a $4 \times 4$ homogeneous matrix:
$$T_{base}^{cam} = \begin{bmatrix} R_{base}^{cam} & \mathbf{t}_{base}^{cam} \\ \mathbf{0}_{1 \times 3} & 1 \end{bmatrix}$$
where:
- $R_{base}^{cam} \in \mathbb{SO}(3)$ is a $3 \times 3$ proper rotation matrix.
- $\mathbf{t}_{base}^{cam} = [t_x, t_y, t_z]^T$ is the 3D translation vector representing the camera origin in robot coordinates.

### Point Transformation:
$$\begin{bmatrix} X_{base} \\ Y_{base} \\ Z_{base} \\ 1 \end{bmatrix} = T_{base}^{cam} \begin{bmatrix} X_{cam} \\ Y_{cam} \\ Z_{cam} \\ 1 \end{bmatrix}$$

### Orientation Transformation:
$$R_{base} = R_{base}^{cam} \cdot R_{cam}$$

---

## 2. Invariant Physical Validation Rules

To prevent mathematical or algorithmic errors from corrupting downstream robot motion planning, the `CoordinateRegistration` class enforces four strict invariants:
1. **Dimensional Invariance**: The matrix must have shape $(4, 4)$.
2. **Affine Bottom Row**: The fourth row must equal $[0, 0, 0, 1]$.
3. **Orthogonality of Rotation**: $R \cdot R^T = I_{3 \times 3}$ within $10^{-3}$ tolerance.
4. **Proper Rotation Determinant**: $\det(R) = +1.0$ (rejects non-physical reflection matrices with $\det(R) = -1$).

---

## 3. Overhead Surgical Camera Mounting Geometry

In the SurgiTwin operating room configuration:
- The camera is mounted directly overhead at $(0.0, 0.35, 0.80)\text{ m}$ in `base_link` coordinates.
- The optical axis ($Z_{cam}$) points vertically downward towards the operating table (along $-Z_{base}$).
- The camera $X_{cam}$ axis (image horizontal right) aligns with the robot $+X_{base}$ axis (forward reach).
- The camera $Y_{cam}$ axis (image vertical down) aligns with the robot $-Y_{base}$ axis (right).

```
T_base_cam =
[  1.0,   0.0,   0.0,  0.00 ]   <- X_base = X_cam + 0.00
[  0.0,  -1.0,   0.0,  0.35 ]   <- Y_base = -Y_cam + 0.35
[  0.0,   0.0,  -1.0,  0.80 ]   <- Z_base = -Z_cam + 0.80
[  0.0,   0.0,   0.0,  1.00 ]
```

### Verification Proof:
When the camera detects an instrument at its optical center $(u=320, v=240)$ at depth $Z_{cam} = 0.60\text{ m}$:
- $X_{cam} = 0.0\text{ m}$, $Y_{cam} = 0.0\text{ m}$, $Z_{cam} = 0.60\text{ m}$.
- $X_{base} = 0.0 + 0.0 = 0.00\text{ m}$.
- $Y_{base} = 0.35 - 0.0 = 0.35\text{ m}$.
- $Z_{base} = 0.80 - 0.60 = 0.20\text{ m}$.
- The mapped point $(0.00, 0.35, 0.20)\text{ m}$ is the exact **UR5e home pose** inside the surgical operating volume!

