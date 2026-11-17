# SurgiTwin — System Architecture Specification

## 1. System Overview

SurgiTwin is a Vision-Guided Surgical Robotics Digital Twin that models minimally invasive robotic surgery. The digital twin simulates end-to-end perception, spatial coordinate registration, inverse kinematics, trajectory control, and real-time safety supervision.

```
+-----------------------------------------------------------------------------------+
|                                 SurgiTwin Digital Twin                            |
|                                                                                   |
|  [DICOM CT / Medical Imaging]                                                     |
|             │                                                                     |
|             ▼                                                                     |
|  [Medical Volume / Segmentation] ──(Anatomical Centroid)──> [Medical Registration]|
|             │                                                      │              |
|  [Camera / Vision Perception]   ──(Fiducial Centroid)───> [Vision Registration]   |
|                                                                    │              |
|                                                                    ▼              |
|                                                       /target_pose (base_link)    |
|                                                                    │              |
|                                                                    ▼              |
|  [Phase 1 Safety Supervisor] <─────────────────────────────────────┤              |
|             │ (If Valid)                                                          |
|             ▼                                                                     |
|       /safe_target_pose                                                           |
|             │                                                                     |
|             ▼                                                                     |
|  [Phase 3 Kinematics Engine] (DLS Inverse Kinematics + Nullspace Joint Centering) |
|             │                                                                     |
|             ▼                                                                     |
|  [Quintic Trajectory Planner] (Cartesian speed <= 100 mm/s)                       |
|             │                                                                     |
|             ▼                                                                     |
|  [50 Hz Closed-Loop PID Simulation] ──> /joint_states & /tool_pose                |
|             │                                                                     |
|             ▼                                                                     |
|  [Phase 4 Hardware Interface] (Binary protocol with CRC16-CCITT)                  |
|             │                                                                     |
|             ▼                                                                     |
|  [Microcontroller Simulator / Hardware] ──> /hardware/joint_feedback              |
+-----------------------------------------------------------------------------------+
```

---

## 2. ROS 2 Package Responsibilities

1. **`surgitwin_safety_controller` (Phase 1)**:
   - Fail-closed workspace boundary check: $X \in [-0.3, 0.3]$, $Y \in [0.1, 0.6]$, $Z \in [0.0, 0.4]$ m.
   - Cartesian velocity limit check ($\le 100$ mm/s).
   - E-stop state machine (`NORMAL`, `STOPPED`).
   - Audit logging to CSV.
2. **`vision_target_node` (Phase 2)**:
   - OpenCV ArUco (`DICT_4X4_50`) and color fiducial detection.
   - Pinhole camera back-projection: $\mathbf{P}_c = (u - c_x) Z / f_x, (v - c_y) Z / f_y, Z$.
   - Coordinate registration from camera frame to `base_link`.
3. **`robot_sim_node` (Phase 3)**:
   - Standard DH forward kinematics model.
   - Numerical Damped Least Squares (DLS) inverse kinematics with nullspace joint centering.
   - Minimum-jerk quintic polynomial trajectory planner.
   - 6-axis PID controllers with dynamic plant simulation and active E-stop brake.
4. **`surgitwin_medical_node` (Phase 4)**:
   - DICOM Part 10 series ingestion and physical slice normal sorting.
   - 3D medical volume reconstruction in Hounsfield Units.
   - Anatomical segmentation and target centroid extraction.
   - Patient-to-robot coordinate registration.
   - Embedded Hardware Abstraction Layer (HAL) with binary packet framing and CRC16-CCITT.
   - Microcontroller simulation with encoder feedback and E-stop propagation.

---

## 3. Coordinate Systems & Registrations

```
[DICOM Patient LPS (mm)]
          │  T_base_patient (Medical Registration, mm -> m)
          ▼
     [base_link] <─────── T_base_camera (Vision Registration) ─────── [Camera Frame (m)]
          │
          ▼  FK (DH Parameters)
     [tool0] (End-Effector Tip)
```

---

## 4. Communication Topology (Topics, Services)

| Interface Name | Type | Publisher / Server | Subscriber / Client | Purpose |
|---|---|---|---|---|
| `/target_pose` | `geometry_msgs/PoseStamped` | `vision_target_node`, `surgitwin_medical_node` | `surgitwin_safety_controller` | Raw target pose |
| `/safe_target_pose` | `geometry_msgs/PoseStamped` | `surgitwin_safety_controller` | `robot_sim_node` | Validated safe target |
| `/tool_pose` | `geometry_msgs/PoseStamped` | `robot_sim_node` | Evaluation / UI | Actual end-effector tool pose |
| `/joint_states` | `sensor_msgs/JointState` | `robot_sim_node` | `surgitwin_medical_node`, `robot_state_publisher` | Simulated joint telemetry |
| `/safety/emergency_stop` | `std_msgs/Bool` | Safety supervisor / UI | `robot_sim_node`, `surgitwin_medical_node` | Emergency stop override |
| `/hardware/joint_feedback` | `sensor_msgs/JointState` | `surgitwin_medical_node` | Evaluation / UI | Hardware encoder telemetry |
| `/hardware/status` | `std_msgs/String` | `surgitwin_medical_node` | Evaluation / UI | Hardware firmware state & health |
