# SurgiTwin — Inverse Kinematics Architecture

## 1. Mathematical Formulation

Inverse kinematics determines the joint configurations $\mathbf{q}^* \in \mathbb{R}^6$ required to place the surgical instrument tip at target Cartesian pose $\mathbf{x}_d = [\mathbf{p}_d^T, \boldsymbol{\phi}_d^T]^T$.

Due to kinematic non-linearities and potential kinematic singularities, SurgiTwin employs a numerical **Damped Least Squares (DLS / Levenberg-Marquardt)** Jacobian solver enhanced with **nullspace joint centering** and multi-start seeding.

### Damped Least Squares (DLS)

The basic update step solves:
$$
\Delta \mathbf{q}_{task} = J^T (J J^T + \lambda^2 I)^{-1} \mathbf{e}
$$
where:
- $J$ is the Jacobian matrix ($3 \times 6$ for position-only, $6 \times 6$ for full 6D pose).
- $\lambda = 0.02$ is the damping factor preventing velocity explosion near kinematic singularities.
- $\mathbf{e} = \mathbf{x}_d - \mathbf{x}(q)$ is the Cartesian task space error vector.

### Nullspace Projection for Joint Centering

To avoid joint limits and maintain comfortable surgical arm postures, the redundant degrees of freedom are projected into the nullspace of the task Jacobian:

$$
\Delta \mathbf{q}_{null} = (I - J^\# J) k_{null} (\mathbf{q}_{home} - \mathbf{q})
$$

$$
\Delta \mathbf{q}_{total} = \Delta \mathbf{q}_{task} + \Delta \mathbf{q}_{null}
$$

This ensures that while the primary positioning task is strictly achieved, the secondary objective keeps joints safely away from mechanical end-stops.

## 2. Multi-Start Seeding Strategy

If the initial guess does not converge within tolerance or encounters joint limits, the solver automatically seeds from candidate surgical configurations:
1. Current robot joint state
2. Default surgical home pose $[0, -\pi/2, \pi/2, -\pi/2, -\pi/2, 0]$
3. Elbow-up / elbow-down postures
4. Positive and negative shoulder pan quadrants

## 3. Reachability & Validation Gates

Before iterative solving, targets are checked against workspace reachability:
- $R_{target} = \|\mathbf{p}_d\| \le 1.05$ m (maximum kinematic reach)
- $R_{target} \ge 0.10$ m (base collision exclusion radius)

After convergence:
- Joint configurations are clamped to mechanical limits $\mathbf{q} \in [\mathbf{q}_{min}, \mathbf{q}_{max}]$.
- Forward kinematics round-trip verifies residual positioning error $\|FK(\mathbf{q}^*) - \mathbf{p}_d\| \le 0.5$ mm.

