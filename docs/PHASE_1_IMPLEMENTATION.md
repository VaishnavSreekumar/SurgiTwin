# SurgiTwin — Phase 1: Core Robotics & Safety Subsystem Implementation Report

## 1. Executive Summary

Phase 1 of the SurgiTwin Vision-Guided Surgical Robotics Digital Twin establishes the core ROS 2 communication infrastructure and an independent, fail-closed Safety Supervisor. 

Prior to Phase 1, the upstream repository relied on MoveIt 2 action clients that crashed immediately on ROS 2 Jazzy / Python 3.12 due to uninstalled MoveIt bindings, and lacked any automated safety validation tests. Phase 1 decouples the safety interlock mechanism into a dedicated, deterministic algorithmic validator (`safety_validator.py`) and a production-grade ROS 2 node (`surgitwin_safety_controller.py`).

---

## 2. Architecture & Data Flow

```
[Upstream Nodes / Vision Pipeline / CLI Publisher]
                         │
                         ▼  (/target_pose: geometry_msgs/PoseStamped)
        +─────────────────────────────────────────+
        |     SurgiTwin Safety Supervisor         |
        |      (surgitwin_safety_controller)      |
        |                                         |
        |  1. E-Stop Interlock Check              |
        |     (Blocks immediately if STOPPED)     |
        |                                         |
        |  2. Numerical & Frame Integrity         |
        |     (Rejects NaN, Inf, bad frame, stale)|
        |                                         |
        |  3. 3D Cartesian Workspace Validation   |
        |     (X:[-0.3, 0.3], Y:[0.1, 0.6],       |
        |      Z:[0.0, 0.4] metres)               |
        |                                         |
        |  4. Velocity Threshold Enforcement      |
        |     (v = ||ΔP|| / Δt <= 100 mm/s)       |
        +─────────────────────────────────────────+
                         │
         ┌───────────────┴───────────────┐
         │ (If VALID)                    │ (If REJECTED / E-STOP)
         ▼                               ▼
[/safe_target_pose]               [Drop Command & Fail Closed]
(Downstream Kinematics)            (Publish /safety/events & log audit)
```

---

## 3. Implemented Components

### 3.1. `surgical_sim/safety_validator.py`
Pure algorithmic safety verification module with zero ROS 2 runtime dependencies for fast unit testing:
- **`WorkspaceBounds`**: Enforces configurable minimum and maximum Cartesian boundaries (inclusive). Default surgical table envelope:
  - $X \in [-0.30, 0.30]\text{ m}$ (left/right reach)
  - $Y \in [0.10, 0.60]\text{ m}$ (forward reach)
  - $Z \in [0.00, 0.40]\text{ m}$ (height above table)
- **`SafetyValidator`**:
  - **Velocity Validation**: Enforces maximum tip speed ($v \le 100\text{ mm/s}$). Handles zero and negative $\Delta t$ safely.
  - **Input Integrity**: Detects and rejects NaN, $\pm\infty$, frame mismatches, negative timestamps, and stale commands ($>2.0\text{ s}$).
  - **E-Stop State Machine**: Implements `NORMAL` and `STOPPED` states with explicit reset requirement.
  - **Audit Logging**: Maintains chronological event history and exports to CSV (`export_events_csv`).

### 3.2. `surgical_sim/surgitwin_safety_controller.py`
ROS 2 node wrapping `SafetyValidator`:
- **Subscribed Topics**:
  - `/target_pose` (`geometry_msgs/msg/PoseStamped`)
  - `/safety/emergency_stop` (`std_msgs/msg/Bool`)
- **Published Topics**:
  - `/safe_target_pose` (`geometry_msgs/msg/PoseStamped`, Reliable QoS)
  - `/safety/status` (`std_msgs/msg/String`, 5 Hz heartbeat telemetry)
  - `/safety/events` (`std_msgs/msg/String`, discrete event stream)
- **Services**:
  - `/safety/estop` (`std_srvs/srv/Trigger`)
  - `/safety/reset` (`std_srvs/srv/Trigger`)

### 3.3. Launch Configuration
- **`launch/safety_supervisor.launch.py`**: Configurable launch file exposing all workspace bounds, velocity thresholds, and frame parameters.

---

## 4. Verification and Automated Test Results

31 automated tests executed via `colcon test` and `pytest`:
- **`test/test_safety_bounds.py`** (9 tests): Valid targets accepted, boundary values inclusive, single/multi-axis violations rejected.
- **`test/test_velocity_limits.py`** (6 tests): Baseline establishment, legal speed forwarding, excessive velocity rejection, zero $\Delta t$ and negative $\Delta t$ protection, numerical velocity precision.
- **`test/test_invalid_inputs.py`** (6 tests): NaN rejection across all axes, $\pm\infty$ rejection, frame validation, staleness limit ($>2.0\text{ s}$) rejection, future clock skew rejection.
- **`test/test_emergency_stop.py`** (6 tests): State transition to `STOPPED`, universal blocking of valid and invalid targets, explicit reset behavior, audit trail.
- **`test/test_safety_integration.py`** (1 comprehensive test): Multi-threaded live ROS 2 node integration verifying topic forwarding, rejection, E-stop, reset, and rapid invalid input burst resilience.
- **`test/test_flake8.py` & `test/test_pep257.py`** (2 tests): Code style and docstring conformance passed.
- **Result: 30 Passed, 1 Skipped (upstream linter skip), 0 Failed.**

---

## 5. Live Demonstration Evidence

Executed via `demo_phase1.sh`:
1. Started `surgitwin_safety_controller` ROS 2 node.
2. Published valid target `(0.00, 0.35, 0.20)` m $\rightarrow$ **ACCEPTED** and published to `/safe_target_pose`.
3. Published out-of-bounds target `(0.80, 0.35, 0.20)` m $\rightarrow$ **REJECTED** (`WORKSPACE_VIOLATION: X above maximum`).
4. Called `/safety/estop` $\rightarrow$ State transitioned to `STOPPED`.
5. Published target during E-Stop $\rightarrow$ **BLOCKED** (`ESTOP_ACTIVE`).
6. Called `/safety/reset` $\rightarrow$ Resumed `NORMAL` state.
7. Published target after reset $\rightarrow$ **ACCEPTED** and published to `/safe_target_pose`.
8. Shutdown node $\rightarrow$ Verified 100% of event entries written to `~/surgitwin_safety_events.csv`.

---

## 6. Regulatory & Standards Context

> [!NOTE]
> **Educational & Simulation Scope**: SurgiTwin is an educational research prototype. This software architecture demonstrates safety design concepts inspired by **IEC 62304 Class B** software lifecycle principles and **ISO 14971** risk mitigation techniques. It is **not** a clinically certified medical device and must never be used for real patient treatment.

