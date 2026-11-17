# SurgiTwin — Software Requirements Specification (SRS)

**Document ID:** SRS-ST-001  
**Standard Compliance:** Inspired by IEC 62304:2006/Amd 1:2015 (Medical device software — Software life cycle processes, Software Safety Class B)  
**System:** SurgiTwin: Vision-Guided Surgical Robotics Digital Twin  
**Status:** Baseline Approved  

---

## 1. Scope & System Overview

SurgiTwin is a digital twin simulation platform modeling a 6-DOF surgical manipulator guided by real-time computer vision fiducial tracking and pre-operative 3D DICOM medical imaging. In accordance with IEC 62304 principles for Class B medical device software (software whose failure cannot result in death or serious injury, but could result in non-serious injury before risk mitigations), this Software Requirements Specification details all safety, functional, interface, performance, and lifecycle requirements.

---

## 2. Requirement Structure & Classification

Every requirement is uniquely identified using the scheme `SR-[SUBSYSTEM]-[INDEX]`:
- **SAF**: Safety Supervisor & Risk Mitigation
- **VIS**: Vision Perception & Calibration
- **KIN**: Robot Kinematics & Motion Planning
- **MED**: Medical Imaging & Anatomical Registration
- **EMB**: Embedded Hardware Abstraction & Telemetry
- **SYS**: Full-System Integration & Communications

---

## 3. Subsystem Requirements

### 3.1 Safety Subsystem (SAF)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-SAF-001** | The safety supervisor shall enforce a 3D Cartesian workspace envelope: $X \in [-0.30, 0.30]$ m, $Y \in [0.10, 0.60]$ m, $Z \in [0.00, 0.40]$ m. | Prevents mechanical collisions with surgical table or patient boundaries. | Automated Integration Test (`test_safety_bounds.py`) | Rejects targets outside bounds; forwards targets within bounds. Zero false admissions. |
| **SR-SAF-002** | The system shall limit Cartesian tip velocity to $\le 100.0$ mm/s ($0.100$ m/s). | Prevents rapid uncontrolled motion near sensitive anatomical structures. | Unit & Integration Tests (`test_velocity_limits.py`) | Commands with $\Delta P / \Delta t > 0.100$ m/s rejected; trajectory planner clamps peak Cartesian tip speed. |
| **SR-SAF-003** | The system shall maintain an Emergency Stop (E-Stop) state machine with `NORMAL` and `STOPPED` states. | Immediate arresting of all robotic motion upon hardware or operator trigger. | Integration Test (`test_emergency_stop.py`, `test_full_system_fault_injection.py`) | State transitions to `STOPPED` in $< 100\ \mu\text{s}$; drops all incoming trajectory commands while stopped. |
| **SR-SAF-004** | Recovery from `STOPPED` state shall require an explicit, unblocked `/safety/reset` command. | Prevents automatic restart or unexpected jerk following fault clearance. | State Machine Verification (`test_safety_integration.py`) | Normal operations resume only after explicit reset service call. |
| **SR-SAF-005** | The safety node shall reject targets containing `NaN`, `+Inf`, `-Inf`, unsupported coordinate frames, or negative timestamps. | Fails closed on numerical divergence or data corruption. | Fault Injection Test (`test_invalid_inputs.py`, `test_full_system_fault_injection.py`) | $100\%$ detection and rejection of malformed or non-finite inputs. |
| **SR-SAF-006** | All safety events, rejections, and state changes shall be recorded to an audit trail CSV log. | Ensures non-repudiation and post-procedure forensic review. | Verification Audit (`test_safety_validator.py`) | Timestamped log written with rejection reasons and exact target coordinates. |

---

### 3.2 Vision Perception & Coordinate Registration (VIS)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-VIS-001** | The vision pipeline shall detect ArUco fiducial markers (dictionary `DICT_4X4_50`) with sub-pixel centroid accuracy. | Accurate instrument tracking from surgical camera feed. | Synthetic Benchmark (`test_vision_detection.py`, `evaluate_surgitwin.py`) | Detection rate $\ge 95.0\%$; sub-pixel centroid error $< 0.8$ px. |
| **SR-VIS-002** | The camera calibration module shall implement a pinhole model ($K$) and OpenCV distortion parameters. | Maps 2D pixel space to 3D optical ray vectors. | Geometric Reprojection Test (`test_camera_calibration.py`, `test_3d_reprojection.py`) | Reprojection error $< 0.001$ px; strictly positive focal lengths and depths. |
| **SR-VIS-003** | The coordinate registration engine shall compute a rigid SE(3) homogeneous transformation ($4\times 4$) from camera to robot base. | Aligns optical tracking coordinates with robot kinematic origin. | Group Orthogonality Test (`test_coordinate_registration.py`) | $R R^T = I$, $\det(R) = +1.0$, translation in metres; transformation error $< 0.01$ mm. |
| **SR-VIS-004** | The vision node shall subscribe to `/camera/image_raw` and publish candidate targets to `/target_pose`. | Standardized ROS 2 perception integration. | ROS 2 Integration Test (`test_vision_integration.py`) | Publishes valid `geometry_msgs/PoseStamped` messages at $\ge 10$ Hz. |

---

### 3.3 Robot Kinematics & Simulation (KIN)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-KIN-001** | Manipulator kinematics shall be defined via standard Denavit-Hartenberg (DH) parameters modeling a 6-DOF arm with surgical scalpel. | Provides deterministic mathematical foundation for forward and differential kinematics. | Analytical Forward Kinematics Test (`test_forward_kinematics.py`) | End-effector tool pose matches URDF chain to within $10^{-6}$ m. |
| **SR-KIN-002** | Inverse kinematics shall implement Damped Least Squares (DLS) Jacobian pseudoinverse with nullspace joint centering. | Solves multi-DOF reachability while avoiding kinematic singularities and joint limits. | Multi-target Grid Test (`test_inverse_kinematics.py`, `evaluate_surgitwin.py`) | Solvability $100\%$ across surgical envelope; position residual error $< 0.5$ mm. |
| **SR-KIN-003** | The trajectory planner shall generate minimum-jerk quintic polynomials with $v(0)=v(T)=0$ and $a(0)=a(T)=0$. | Eliminates acceleration discontinuities and mechanical vibrations. | Trajectory Verification (`test_trajectory_generation.py`) | Zero jerk discontinuities; peak Cartesian tip velocity $\le 100$ mm/s. |
| **SR-KIN-004** | The simulation engine shall execute a 6-axis second-order dynamic plant with 50 Hz PID joint control. | Simulates realistic actuator lag, inertia, and closed-loop control. | Simulation Tracking Test (`test_pid_controller.py`, `test_robot_simulation.py`) | Closed-loop Cartesian tracking RMSE $< 1.0$ mm. |

---

### 3.4 Medical Imaging & Registration (MED)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-MED-001** | The medical imaging module shall ingest DICOM Part 10 series and verify metadata completeness. | Ingestion of standard pre-operative CT/MR scan data. | Parser Test (`test_dicom_loader.py`) | Validates `PixelSpacing`, `ImagePositionPatient`, and rescale intercept/slope. |
| **SR-MED-002** | The volume engine shall reconstruct 3D volumes in Hounsfield Units (HU) with continuous LPS affine mapping. | Accurate 3D volumetric representation in patient coordinates. | Round-trip Affine Test (`test_medical_volume.py`) | Voxel-to-patient round-trip error $< 10^{-6}$ mm. |
| **SR-MED-003** | The segmentation engine shall isolate anatomical targets via intensity windowing and 3D connected-component labeling. | Delineates surgical target lesions from surrounding healthy tissue. | Segmentation Test (`test_medical_segmentation.py`, `evaluate_surgitwin.py`) | Volume estimation error $< 10\%$; binary STL surface mesh export. |
| **SR-MED-004** | The target extractor shall compute sub-voxel continuous centroids and patient physical bounding boxes. | Extracts exact geometric target coordinate for robotic guidance. | Extraction Test (`test_anatomical_target.py`) | Accurate centroid localization; fails closed on empty segmentation masks. |
| **SR-MED-005** | The medical registration module shall map patient DICOM coordinates (mm) to robot base coordinates (m). | Transforms anatomical target into robot workspace. | Registration Test (`test_medical_registration.py`) | Rigid SE(3) transformation; produces target within robot safety envelope. |

---

### 3.5 Embedded Hardware Abstraction & Telemetry (EMB)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-EMB-001** | The embedded protocol shall use framed binary packets with header `0xAA 0x55`, footer `0x0D 0x0A`, length byte, and sequence number. | Robust byte framing over lossy or noisy serial links. | Protocol Codec Test (`test_hardware_protocol.py`) | Correct encoding/decoding; framing errors detected and rejected. |
| **SR-EMB-002** | Every packet shall include a 16-bit CRC-CCITT checksum calculated over header, type, sequence, length, and payload. | Detects single-bit and multi-bit communication corruption. | Corrupted Packet Test (`test_hardware_protocol.py`, `evaluate_surgitwin.py`) | $100\%$ detection of bit corruptions; corrupt packets discarded with NACK. |
| **SR-EMB-003** | The embedded HAL shall support virtual in-memory SIMULATION mode and physical SERIAL port communication. | Hardware-in-the-loop and software-in-the-loop flexibility. | HAL Mode Test (`test_hardware_protocol.py`) | Seamless transport switching without altering upper-layer application logic. |
| **SR-EMB-004** | The microcontroller simulator shall model optical encoder feedback with 4096 counts/rev and actuator lag. | Realistic firmware telemetry feedback emulation. | MCU Simulator Test (`test_microcontroller_sim.py`) | Quantization step $\approx 0.00153$ rad; returns valid status and telemetry packets. |

---

### 3.6 Full-System Integration (SYS)

| Requirement ID | Description | Rationale | Verification Method | Acceptance Criteria |
| :--- | :--- | :--- | :--- | :--- |
| **SR-SYS-001** | The master launch file `launch/surgitwin_full_system.launch.py` shall orchestrate all nodes with configurable target sources. | Reproducible one-command system startup. | Launch Test & Master Demo (`demo_full_system.py`) | Clean startup and graceful termination across `VISION`, `MEDICAL`, and `BOTH` modes. |
| **SR-SYS-002** | Total end-to-end planning cycle latency shall not exceed 50.0 ms (supporting $> 20$ Hz dynamic updates). | Real-time responsiveness for surgical guidance. | Performance Benchmark (`evaluate_surgitwin.py`) | Measured total planning latency $< 20.0$ ms ($67$ Hz achieved). |
| **SR-SYS-003** | DDS domain isolation shall prevent inter-node crosstalk during test execution. | Eliminates network non-determinism during automated validation. | Integration Suite Configuration | Explicit domain ID assignment across integration tests and demos. |

---

## 4. Software Safety Classification Summary

In compliance with IEC 62304 Section 4.3:
- **Software Safety Class:** Class B
- **Failure Mode Impact:** Without risk mitigation, software errors (such as out-of-bounds trajectory generation) could cause unintended motion.
- **Risk Mitigation:** Independent Phase 1 Safety Supervisor, strict envelope bounding, velocity limits, fail-closed CRC framing, and active electromagnetic E-stop braking reduce residual risk to acceptable levels (broadly acceptable under ISO 14971).

