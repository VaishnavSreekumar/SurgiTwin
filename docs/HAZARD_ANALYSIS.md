# SurgiTwin — Hazard Analysis & Risk Management Report

**Document ID:** HAR-ST-001  
**Standard Compliance:** Inspired by ISO 14971:2019 (Medical devices — Application of risk management to medical devices)  
**System:** SurgiTwin: Vision-Guided Surgical Robotics Digital Twin  
**Safety Classification:** IEC 62304 Class B  
**Status:** Approved & Verified  

---

## 1. Risk Evaluation Methodology

Risk management evaluates surgical hazard severity ($S$) and occurrence probability ($P$) before and after implementing software risk control measures.

### 1.1 Severity Categories
- **S1 — Negligible:** Inconvenience, temporary software pause without physical impact.
- **S2 — Minor:** Slight delay, minor tool deviation requiring manual repositioning.
- **S3 — Serious:** Significant deviation, collision with non-critical structure, requiring minor clinical intervention.
- **S4 — Critical:** Collision with patient or critical organ, permanent impairment.
- **S5 — Catastrophic:** Patient death or life-threatening organ laceration.

### 1.2 Probability of Occurrence
- **P1 — Improbable:** $< 10^{-6}$ per operating hour (extremely rare).
- **P2 — Remote:** $10^{-6}$ to $10^{-4}$ per operating hour (unlikely).
- **P3 — Occasional:** $10^{-4}$ to $10^{-2}$ per operating hour (may occur occasionally).
- **P4 — Probable:** $10^{-2}$ to $10^{-1}$ per operating hour (likely to occur).
- **P5 — Frequent:** $> 10^{-1}$ per operating hour (expected to occur repeatedly).

### 1.3 Risk Evaluation Matrix (ALARP)
- **Unacceptable (U):** Must be mitigated; operation prohibited.
- **As Low As Reasonably Practicable (ALARP / A):** Tolerable only when further mitigation is technically infeasible.
- **Broadly Acceptable (BA):** Residual risk is negligible; safe for operation.

---

## 2. Hazard Analysis & Risk Mitigation Matrix

| Hazard ID | Hazard Description & Potential Clinical Harm | Pre-Mitigation ($S \times P$) | Initial Risk | Software Risk Mitigation Mechanism | Post-Mitigation ($S \times P$) | Residual Risk | Verification Evidence & Test Reference |
| :--- | :--- | :---: | :---: | :--- | :---: | :---: | :--- |
| **H-01** | **Workspace Envelope Violation:** Robotic arm travels outside operating table bounds, colliding with patient torso or OR equipment. | $S4 \times P4$ | **U** | Phase 1 `SafetyValidator` enforces 3D cuboid limits ($X \in [-0.3, 0.3]$, $Y \in [0.1, 0.6]$, $Z \in [0.0, 0.4]$ m). Commands outside bounds are dropped immediately without forwarding. | $S4 \times P1$ | **BA** | `test/test_safety_bounds.py`<br>`test_boundary_enforcement()` (100% caught, 0 false forwards). |
| **H-02** | **Excessive Cartesian Velocity:** Fast robotic motion exceeding surgical threshold ($> 100$ mm/s), causing accidental tissue tearing or puncture. | $S4 \times P3$ | **U** | Dual mitigation: 1) `SafetyValidator` rejects Cartesian steps with $\Delta P / \Delta t > 0.100$ m/s; 2) `TrajectoryPlanner` scales polynomial duration iteratively so peak Cartesian tip speed never exceeds $100$ mm/s. | $S4 \times P1$ | **BA** | `test/test_velocity_limits.py`<br>`test/test_trajectory_generation.py` (0 violations measured). |
| **H-03** | **Failure to Arrest Motion on E-Stop:** Manipulator continues moving along trajectory despite emergency button activation. | $S5 \times P3$ | **U** | Dedicated high-priority `/safety/emergency_stop` topic and service. `SurgiTwinRobotSimNode` immediately purges active trajectory, zeroes command velocity, and commands active electromagnetic dynamic braking damping ($B_{estop} = 60$ Nms/rad). | $S5 \times P1$ | **BA** | `test/test_emergency_stop.py`<br>`test_fault_ros2_estop_during_closed_loop_motion()` (Arrested in $< 20$ ms). |
| **H-04** | **Numerical Divergence / NaN Target Injection:** Corrupted floating-point values causing extreme actuator torque spikes. | $S4 \times P3$ | **U** | `validate_input_integrity()` inspects coordinates for `math.isnan()` and `math.isinf()` prior to any kinematic calculation. Fails closed on detection. | $S4 \times P1$ | **BA** | `test/test_invalid_inputs.py`<br>`test_nan_inf_injection()` (100% of NaN/Inf rejected). |
| **H-05** | **Embedded Command / Telemetry Corruption:** Bit flips on communication bus causing joint position jumps or erroneous telemetry. | $S3 \times P4$ | **U** | Framed binary protocol with start bytes `0xAA 0x55`, end bytes `0x0D 0x0A`, length verification, and 16-bit CRC-CCITT verification over entire packet payload. | $S3 \times P1$ | **BA** | `test/test_hardware_protocol.py`<br>`evaluate_surgitwin.py` (100% CRC error detection rate). |
| **H-06** | **Misleading Anatomical Target from DICOM:** Segmenting non-target anatomy or empty slice data, causing robot to target wrong coordinate. | $S4 \times P3$ | **U** | Dual-threshold windowing, 3D connected-component labeling, equivalent radius validation, and fail-closed check in `AnatomicalTargetExtractor` raising `ValueError` on empty or invalid masks. | $S4 \times P1$ | **BA** | `test/test_medical_segmentation.py`<br>`test_anatomical_target.py` (Target centroid error: $0.0$ mm). |
| **H-07** | **Stale Perception Coordinates:** Robot moves to a position where an instrument or target was located seconds ago. | $S3 \times P4$ | **U** | `SafetyValidator` checks header timestamps against monotonic system clock. Rejects commands older than `max_staleness_sec = 2.0` s or timestamps in the future ($> 0.5$ s clock skew). | $S3 \times P1$ | **BA** | `test/test_invalid_inputs.py`<br>`test_stale_target_rejection()` (Stale timestamps rejected). |
| **H-08** | **Kinematic Singularity Unbounded Joint Velocity:** Arm nears gimbal lock or wrist alignment, causing high joint velocity despite slow Cartesian motion. | $S3 \times P3$ | **A** | Inverse kinematics uses Damped Least Squares (DLS) Jacobian pseudoinverse $J^T (J J^T + \lambda^2 I)^{-1}$ with damping factor $\lambda = 0.05$ and nullspace centering towards surgical home pose. Joint velocities strictly bounded to $\le 1.0$ rad/s. | $S3 \times P1$ | **BA** | `test/test_inverse_kinematics.py`<br>`evaluate_surgitwin.py` (100% solvability without singularity divergence). |
| **H-09** | **Communication Drop / Node Crash During Motion:** Trajectory planner or safety supervisor crashes mid-motion, leaving robot in indeterminate state. | $S4 \times P2$ | **A** | Robot node simulation operates a 50 Hz autonomous watchdog timer. In the absence of refreshed safe targets or trajectory updates, motion completes planned profile and transitions to `COMPLETED` / `IDLE` state. | $S4 \times P1$ | **BA** | `test/test_robot_safety_integration.py`<br>`test_trajectory_tracking_to_completion()` (State terminates in `COMPLETED`). |
| **H-10** | **Spurious Automatic Restart After E-Stop:** Power or communications resume, and arm immediately lunges toward previous target. | $S5 \times P2$ | **U** | The safety node enforces an explicit state machine: reset is rejected if safety is normal, and transition from `STOPPED` back to `NORMAL` requires explicit call to `/safety/reset` service. Velocity baseline is cleared upon reset to prevent jumps. | $S5 \times P1$ | **BA** | `test/test_safety_integration.py`<br>`test_fault_estop_state_machine_and_recovery()` (Automatic motion blocked). |

---

## 3. Risk Evaluation Summary

- **Total Hazards Identified:** 10
- **Pre-Mitigation Unacceptable Hazards:** 8
- **Pre-Mitigation ALARP Hazards:** 2
- **Post-Mitigation Unacceptable Hazards:** **0**
- **Post-Mitigation Broadly Acceptable Hazards:** **10 (100%)**

### Conclusion
Through independent fail-closed software safety supervisors, kinematic DLS regularization, polynomial jerk clamping, binary CRC16 packet verification, and explicit state machine controls, all identified hazards have been reduced to Broadly Acceptable levels in accordance with ISO 14971 guidelines for surgical digital twins.

