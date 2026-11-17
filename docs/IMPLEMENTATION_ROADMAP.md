# SurgiTwin — Technical Implementation Roadmap

## 1. Roadmap Strategy: Same-Day MVP to Full Digital Twin

To guarantee a demonstrable, mathematically rigorous engineering project delivered on day one, SurgiTwin uses a staged milestone architecture:

- **Track A (Same-Day MVP — P0)**: Self-contained, robust simulation running on native ROS 2 Jazzy without external Gazebo Classic dependencies. It features active OpenCV perception, pinhole camera projection, rigid spatial transformation, 6-DOF kinematics, closed-loop trajectory simulation, real-time safety boundary monitoring, and CSV metrics logging.
- **Track B (Extended Simulation — P1 & P2)**: Gazebo Harmonic (`ros_gz`), MoveIt 2 Jazzy motion planning, 3D Slicer / SimpleITK DICOM registration, and physical microcontroller hardware-in-the-loop (HIL).

---

## 2. Milestone Breakdown

```
[Phase 0: Repository Audit & Roadmap]  <-- CURRENT MILESTONE
                 │
                 ▼
[Phase 1: Same-Day MVP Architecture & Safety Subsystem]
                 │
                 ▼
[Phase 2: Vision Perception Pipeline (OpenCV & Transform Engine)]
                 │
                 ▼
[Phase 3: Robot Kinematics & Closed-Loop Trajectory Simulation]
                 │
                 ▼
[Phase 4: End-to-End System Integration & Live Verification]
                 │
                 ▼
[Phase 5: Extended Domains (DICOM Imaging, Gazebo Harmonic, Hardware)]
```

---

## 3. Phase Details & Acceptance Criteria

### Phase 0: Repository Audit, Compatibility Analysis & Technical Roadmap (Done)
- **Goal**: Audit legacy repository, catalog broken components, verify ROS 2 Jazzy environment, and establish architectural roadmap.
- **Acceptance Criteria**:
  - `docs/INITIAL_AUDIT.md`, `docs/ARCHITECTURE.md`, `docs/IMPLEMENTATION_ROADMAP.md`, `docs/DEPENDENCY_COMPATIBILITY.md`, `docs/ENGINEERING_DECISIONS.md` completed.
  - Zero unapproved packages installed.

### Phase 1: Core Safety Controller & Interactive Test Harness (P0 — Same-Day MVP)
- **Goal**: Resolve the broken `surgical_controller.py` by providing a robust, standalone safety supervision layer that decouples MoveIt action dependency from safety logic.
- **Deliverables**:
  - `surgitwin_safety_controller.py`: Standalone safety node implementing:
    - 3D Cartesian workspace limits ($X \in [-0.3, 0.3]\text{ m}$, $Y \in [0.1, 0.6]\text{ m}$, $Z \in [0.0, 0.4]\text{ m}$).
    - Velocity ceiling enforcement ($\le 100\text{ mm/s}$).
    - Target rejection and error reporting via `/controller_status`.
    - Emergency stop handling (`/emergency_stop`).
  - Unit tests verifying workspace boundary checks and target acceptance/rejection.
- **Acceptance Criteria**: `pytest` passes 100% of safety limit test cases.

### Phase 2: Computer Vision & Coordinate Registration Engine (P0 — Same-Day MVP)
- **Goal**: Replace manual CLI coordinate entry with an automated OpenCV visual perception pipeline.
- **Deliverables**:
  - `vision_target_detector.py`: Detects target or surgical marker from camera frame/video using OpenCV (HSV color thresholding, contour extraction, centroid determination, and noise filtering).
  - `coordinate_registration.py`: Transforms 2D image coordinates $(u, v)$ with depth $Z$ to 3D camera coordinates $(X_c, Y_c, Z_c)$ using pinhole camera model, then transforms camera frame to robot `base_link` frame using homogeneous transform matrix $T_{base}^{cam}$.
  - Publishes target to `/target_pose`.
- **Acceptance Criteria**: Synthetic or sample image with test marker correctly detected, transformed into `base_link` coordinates, and verified against ground truth within $1.0\text{ mm}$.

### Phase 3: Kinematics Engine & Simulated Robotic Movement (P0 — Same-Day MVP)
- **Goal**: Provide forward and inverse kinematics and trajectory execution for the 6-DOF robotic arm without requiring heavy external simulator overhead.
- **Deliverables**:
  - `kinematics_engine.py`:
    - Forward Kinematics (FK) based on UR5e standard Denavit-Hartenberg (DH) parameters.
    - Numerical / analytical Inverse Kinematics (IK) with convergence guarantees and joint limit enforcement.
  - `simulated_robot.py`:
    - Simulates 6-DOF joint motion responding to commanded trajectories.
    - Publishes `/joint_states` and `/tool_pose` at 50 Hz.
  - Trajectory interpolator generating smooth motion profiles under $100\text{ mm/s}$ tip velocity.
- **Acceptance Criteria**: Robot arm reaches commanded target poses with sub-millimeter position accuracy and publishes valid `/tool_pose`.

### Phase 4: End-to-End Integration, Metrics & Live Demonstration (P0 — Same-Day MVP)
- **Goal**: Wire all components into a single launch workflow and produce verifiable execution data.
- **Deliverables**:
  - `surgitwin_demo.launch.py`: One-command launch of perception, registration, safety, kinematics, robot simulator, and metrics logger.
  - Verification run generating `~/surgical_sim_metrics.csv`.
  - Offline visualization script (`plot_metrics.py`) rendering setpoint vs. actual trajectory, tracking error, and velocity profiles.
- **Acceptance Criteria**:
  - End-to-end execution: Visual marker $\rightarrow$ Perception $\rightarrow$ Registration $\rightarrow$ Safety Check $\rightarrow$ IK $\rightarrow$ Simulated Motion $\rightarrow$ CSV Log.
  - Complete demo log and performance plots showing tip error $< 1\text{ mm}$ and tip velocity $< 100\text{ mm/s}$.

### Phase 5: Extended Engineering Domains (P1 & P2 — Next Stages)
- **Domain 6 (Medical Imaging)**: SimpleITK / 3D Slicer pipeline reading anonymized DICOM series, extracting 3D anatomical volume, and registering fiducial landmarks to robot coordinates.
- **Domain 8 (Gazebo Harmonic & MoveIt 2)**: Port SDF models to Gazebo Harmonic (`ros_gz`), configure MoveIt 2 Jazzy OMPL planning pipeline with graphical RViz2.
- **Domain 5 (Embedded Hardware)**: Microcontroller serial bridge (ESP32 / Arduino) streaming encoder values or actuating miniature servo to mimic surgical tool articulation.
- **Domain 7 (Medical Device Documentation)**: Complete Software Requirements Specification (SRS), Software Architecture Document (SAD), Traceability Matrix (RTM), and ISO 14971 Risk Assessment.

---

## 4. Complexity & Resource Estimates

| Milestone | Complexity | Primary Tools | Execution Time |
|---|---|---|---|
| Phase 0: Audit & Roadmap | Low | WSL2, Git, Python | Completed |
| Phase 1: Safety & Test Harness | Low | rclpy, pytest | 1 hour |
| Phase 2: Perception & Transform | Medium | OpenCV, NumPy, TF2 | 1.5 hours |
| Phase 3: Kinematics & Simulation | Medium | NumPy, SciPy, kinematics math | 1.5 hours |
| Phase 4: Integration & Metrics | Medium | ROS 2 Launch, Matplotlib | 1 hour |
| Phase 5: Extended Gazebo & MoveIt | High | Gazebo Harmonic, MoveIt2 | Subsequent phase |
| Phase 5: DICOM / Medical Imaging | Medium | SimpleITK, pydicom | Subsequent phase |
| Phase 5: Embedded Hardware | Medium | pyserial, Arduino C++ | Subsequent phase |

