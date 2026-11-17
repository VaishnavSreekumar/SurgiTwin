# SurgiTwin — 6-DOF Manipulator Forward Kinematics

## 1. Kinematic Description & Denavit-Hartenberg Parameters

The SurgiTwin robotic manipulator is an anthropomorphic 6-degree-of-freedom serial arm modeled on standard industrial and surgical collaborative manipulator architecture (UR5e kinematic geometry with a rigid surgical instrument attachment).

### Standard Denavit-Hartenberg Parameter Table

The kinematic chain consists of six revolute joints ($i = 1, \dots, 6$) and a surgical tool extension along the tool flange z-axis ($L_{tool} = 0.150$ m):

| Link $i$ | Joint Name | $\theta_i$ (rad) | $d_i$ (m) | $a_i$ (m) | $\alpha_i$ (rad) | Motion Range |
|---|---|---|---|---|---|---|
| 1 | `shoulder_pan_joint` | $q_1$ | $0.1625$ | $0$ | $\pi/2$ | $[-\pi, \pi]$ |
| 2 | `shoulder_lift_joint` | $q_2$ | $0$ | $-0.4250$ | $0$ | $[-\pi, \pi]$ |
| 3 | `elbow_joint` | $q_3$ | $0$ | $-0.3922$ | $0$ | $[-\pi, \pi]$ |
| 4 | `wrist_1_joint` | $q_4$ | $0.1333$ | $0$ | $\pi/2$ | $[-\pi, \pi]$ |
| 5 | `wrist_2_joint` | $q_5$ | $0.0997$ | $0$ | $-\pi/2$ | $[-\pi, \pi]$ |
| 6 | `wrist_3_joint` | $q_6$ | $0.0996 + 0.150 = 0.2496$ | $0$ | $0$ | $[-2\pi, 2\pi]$ |

## 2. Mathematical Transformation Matrix

For each link $i$, the homogeneous transformation matrix $A_i$ from frame $i-1$ to frame $i$ is formulated under the standard DH convention:

$$
A_i = \text{Rot}_z(\theta_i) \text{Trans}_z(d_i) \text{Trans}_x(a_i) \text{Rot}_x(\alpha_i)
$$

$$
A_i = \begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i \cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i \sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
$$

The overall forward kinematics transformation $T_0^6$ from the base link to the surgical end-effector tip is given by the cumulative product:

$$
T_0^{tool}(\mathbf{q}) = A_1(q_1) A_2(q_2) A_3(q_3) A_4(q_4) A_5(q_5) A_6(q_6) = \begin{bmatrix} R(\mathbf{q}) & \mathbf{p}(\mathbf{q}) \\ \mathbf{0}^T & 1 \end{bmatrix}
$$

## 3. Geometric Jacobian Matrix

The 6x6 geometric Jacobian $J(\mathbf{q})$ maps joint velocities $\dot{\mathbf{q}}$ to the spatial twist $\mathbf{v} = [\mathbf{v}_e^T, \boldsymbol{\omega}_e^T]^T$:

$$
J(\mathbf{q}) = \begin{bmatrix} J_{v,1} & J_{v,2} & \cdots & J_{v,6} \\ J_{\omega,1} & J_{\omega,2} & \cdots & J_{\omega,6} \end{bmatrix}
$$

For revolute joints:
$$
J_{v,i} = \hat{\mathbf{z}}_{i-1} \times (\mathbf{p}_{tool} - \mathbf{p}_{i-1})
$$
$$
J_{\omega,i} = \hat{\mathbf{z}}_{i-1}
$$

where $\hat{\mathbf{z}}_{i-1}$ is the unit vector along the z-axis of frame $i-1$, and $\mathbf{p}_{i-1}$ is the origin of frame $i-1$ expressed in the base frame.

