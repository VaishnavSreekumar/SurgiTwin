# SurgiTwin — Safety Architecture & Risk Control Specification

## 1. Safety Principles & Architectural Isolation

In safety-critical robotics systems (such as medical manipulators), safety supervision must be structurally independent of high-level task planning and perception. Upstream vision models and trajectory planners may encounter unhandled edge cases, optical occlusion, or numerical singularities. The safety subsystem acts as an unconditional, fail-closed gatekeeper.

```
       [Unsafe World / Vision / Trajectory Planner]
                            │
                            ▼
               +──────────────────────────+
               |  SurgiTwin Gatekeeper    |
               | (IEC 62304 Architecture) |
               +──────────────────────────+
                            │
              Is command 100% compliant?
                            │
              ┌─────────────┴─────────────┐
              │ YES                       │ NO / E-STOP
              ▼                           ▼
      [/safe_target_pose]          [Discard Command]
      To Robot Controllers         Safe State: HOLD_POSITION
```

---

## 2. Risk Mitigation Matrix (ISO 14971 Aligned)

| Hazard ID | Hazard Description | Hazardous Situation | Software Risk Mitigation | Verification Method |
|---|---|---|---|---|
| **HAZ-01** | Tool collision with patient/table | Coordinate outside operating volume | 3D Cartesian boundary limits ($X \in [-0.3, 0.3]$, $Y \in [0.1, 0.6]$, $Z \in [0.0, 0.4]$ m) | `test_safety_bounds.py` (Tests 1–8) |
| **HAZ-02** | Excessive tool velocity | Sudden jump causing tissue laceration | Cartesian velocity cap ($v \le 100\text{ mm/s}$) calculated as $\frac{\|\Delta P\|}{\Delta t}$ | `test_velocity_limits.py` (Tests 9–10) |
| **HAZ-03** | Corrupted / Malformed input | NaN or $\pm\infty$ causing solver explosion | Numerical floating-point check (`isnan`, `isinf`) prior to any geometric calculation | `test_invalid_inputs.py` (Tests 11–12) |
| **HAZ-04** | Stale / Delayed commands | Communication lag causing out-of-date execution | Temporal staleness check ($t_{now} - t_{stamp} \le 2.0\text{ s}$) and future drift rejection | `test_invalid_inputs.py` (Test 14) |
| **HAZ-05** | Loss of operator control | Unintended movement during emergency | Latching E-Stop state machine requiring explicit reset; prevents command replay | `test_emergency_stop.py` (Tests 15–16) |

---

## 3. Emergency Stop State Machine

```
                 [System Startup]
                        │
                        ▼
                 ┌──────────────┐
                 │    NORMAL    │ ◄─────────────────────────┐
                 │ (Accepts &   │                           │
                 │  Validates)  │                           │
                 └──────┬───────┘                           │
                        │                                   │
             Operator / Topic E-Stop                     Explicit
                        │                             Service Reset
                        ▼                             (/safety/reset)
                 ┌──────────────┐                           │
                 │   STOPPED    │                           │
                 │ (Drop All    │                           │
                 │  Commands)   │ ──────────────────────────┘
                 └──────────────┘
```

### State Machine Transition Rules:
1. **Startup**: State initializes to `NORMAL`.
2. **Transition: NORMAL $\rightarrow$ STOPPED**:
   - Triggered by `/safety/emergency_stop` topic (`data: true`) OR `/safety/estop` service.
   - Immediately sets state to `STOPPED`, increments `estop_activations`, and records audit event.
   - All subsequent incoming target commands are rejected with reason code `ESTOP_ACTIVE`.
3. **Transition: STOPPED $\rightarrow$ NORMAL**:
   - Requires explicit service call to `/safety/reset`.
   - Clears kinematic baseline (`_last_accepted_pose = None`) so that future motion velocity is evaluated from a clean baseline rather than risking a jump error.
   - Rejected commands queued during `STOPPED` state are never replayed.

---

## 4. Cartesian Velocity Calculation Method

The velocity validator calculates instantaneous tip speed between consecutive accepted poses $P_{k-1}$ and $P_k$:
$$\Delta P = \sqrt{(x_k - x_{k-1})^2 + (y_k - y_{k-1})^2 + (z_k - z_{k-1})^2}$$
$$\Delta t = t_k - t_{k-1}$$
$$v = \frac{\Delta P}{\Delta t}$$

### Edge Case Handling:
- If pose is the first pose received after startup or after E-stop reset: $\Delta t$ is undefined; the pose is recorded as the spatial baseline without velocity penalty.
- If $\Delta t \le 0$: rejected with `VELOCITY_VIOLATION: Non-positive time delta`.
- If $v > 0.100\text{ m/s}$: rejected with `VELOCITY_VIOLATION: Computed velocity exceeds limit`.

