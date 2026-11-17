# SurgiTwin — Trajectory Planning & Closed-Loop Joint Control

## 1. Quintic Polynomial Minimum-Jerk Trajectory

To prevent sudden accelerations and jerks that could cause tissue damage or mechanical resonance during robotic surgery, trajectories are planned using $C^2$-continuous quintic polynomials.

### Mathematical Formulation

Normalized time $\tau = \frac{t}{T} \in [0, 1]$, where $T$ is the total planned duration.

$$
s(\tau) = 10\tau^3 - 15\tau^4 + 6\tau^5
$$
$$
\dot{s}(\tau) = \frac{1}{T}(30\tau^2 - 60\tau^3 + 30\tau^4) = \frac{30}{T}\tau^2(1 - \tau)^2
$$
$$
\ddot{s}(\tau) = \frac{1}{T^2}(60\tau - 180\tau^2 + 120\tau^3)
$$

Boundary conditions:
$$
s(0) = 0, \quad s(1) = 1
$$
$$
\dot{s}(0) = \dot{s}(1) = 0, \quad \ddot{s}(0) = \ddot{s}(1) = 0
$$

Joint positions, velocities, and accelerations:
$$
\mathbf{q}(t) = \mathbf{q}_0 + s(t/T) (\mathbf{q}_f - \mathbf{q}_0)
$$
$$
\dot{\mathbf{q}}(t) = \dot{s}(t/T) (\mathbf{q}_f - \mathbf{q}_0)
$$
$$
\ddot{\mathbf{q}}(t) = \ddot{s}(t/T) (\mathbf{q}_f - \mathbf{q}_0)
$$

## 2. Dynamic Time Scaling & Safety Constraints

The trajectory duration $T$ is dynamically computed to strictly satisfy all physical safety limits:

1. **Cartesian Tip Velocity Limit**:
   $v_{cart}(t) \le 100$ mm/s ($0.10$ m/s), matching the Phase 1 safety supervisor threshold.
2. **Joint Velocity Limit**:
   $|\dot{q}_i(t)| \le 1.0$ rad/s.
3. **Joint Acceleration Limit**:
   $|\ddot{q}_i(t)| \le 2.0$ rad/s$^2$.

An iterative verification pass inspects actual end-effector Cartesian velocities along the 3D space curve and dynamically scales duration if curvature introduces arc speeds exceeding $100$ mm/s.

## 3. Closed-Loop PID Joint Control

Each manipulator joint is actuated by an independent PID controller with anti-windup clamping:

$$
e_i(t) = q_{desired, i}(t) - q_{actual, i}(t)
$$
$$
\dot{e}_i(t) = \dot{q}_{desired, i}(t) - \dot{q}_{actual, i}(t)
$$
$$
\tau_i(t) = K_{p,i} e_i(t) + K_{i,i} \int_0^t e_i(\xi) d\xi + K_{d,i} \dot{e}_i(t)
$$

## 4. Mechanical Plant Simulation & Emergency Braking

The physical system is modeled as a 6-DOF dynamic plant:

$$
M \ddot{\mathbf{q}} + B \dot{\mathbf{q}} = \boldsymbol{\tau}
$$

where:
- $M = \text{diag}([2.5, 2.5, 1.8, 0.6, 0.4, 0.25])$ kg$\cdot$m$^2$ (effective link inertia).
- $B = \text{diag}([4.0, 4.0, 3.0, 1.2, 0.8, 0.5])$ N$\cdot$m$\cdot$s/rad (joint viscous friction).

### Active Emergency Stop Brake Dynamics
When the safety supervisor asserts `/safety/emergency_stop`:
$$
\boldsymbol{\tau}_{brake} = -B_{estop} \dot{\mathbf{q}}
$$
where $B_{estop} = [60.0, 60.0, 50.0, 20.0, 15.0, 10.0]$ N$\cdot$m$\cdot$s/rad. This active electromagnetic braking arrests joint velocity to zero within $< 50$ ms.

