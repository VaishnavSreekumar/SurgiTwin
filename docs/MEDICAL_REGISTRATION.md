# SurgiTwin — Medical-Image-to-Robot Coordinate Registration

## 1. Frame Definitions & Units

Coordinate transformation maps points from the medical imaging patient frame into the robotic manipulator's base reference frame:

1. **Source Frame**: `patient_lps` (DICOM LPS: Left, Posterior, Superior), expressed in **millimeters** ($\text{mm}$).
2. **Target Frame**: `base_link` (SurgiTwin robot mounting base), expressed in **meters** ($\text{m}$).

> [!IMPORTANT]
> **Unit Conversion Principle**:
> Physical patient coordinates must be scaled by $10^{-3}$ before multiplication with the $SE(3)$ transformation matrix:
> $$\mathbf{p}_{base} = T_{base}^{patient} \begin{bmatrix} \mathbf{p}_{patient} \times 10^{-3} \\ 1 \end{bmatrix}$$

## 2. Simulated Operating Table Registration Model

In clinical image-guided surgery (e.g. neurosurgery or orthopedic robotics), registration is established using optical tracking fiducials or surface matching. In this simulation prototype, registration is explicitly modeled using a calibrated synthetic operating table frame:

- **Patient Position**: Patient is placed supine (lying on back) on the operating table centered directly in front of the robot manipulator.
- **Table Coordinates**: Table center at $(0.0, 0.35, 0.18)$ m in `base_link`.
- **Orientation Mapping**:
  - Patient Left ($+X_p$) aligns with Robot $+X_b$
  - Patient Posterior ($+Y_p$, back) aligns with Robot $-Z_b$ (pointing downward into table)
  - Patient Superior ($+Z_p$, head) aligns with Robot $+Y_b$ (pointing away along table)

$$
T_{base}^{patient} = \begin{bmatrix}
1.0 &  0.0 & 0.0 & 0.00 \\
0.0 &  0.0 & 1.0 & 0.35 \\
0.0 & -1.0 & 0.0 & 0.18 \\
0.0 &  0.0 & 0.0 & 1.00
\end{bmatrix}
$$

## 3. SE(3) Rigid Transformation Validation

Every registration matrix must satisfy the following algebraic invariants before application:
1. **Orthogonality**: $R R^T = I_{3 \times 3} \pm 10^{-4}$
2. **Proper Rotation (No Reflection)**: $\det(R) = +1.0 \pm 10^{-4}$
3. **Homogeneous Integrity**: Bottom row is strictly $[0, 0, 0, 1]$.

Matrices violating these invariants are immediately rejected with an exception to prevent misleading spatial transformations.

