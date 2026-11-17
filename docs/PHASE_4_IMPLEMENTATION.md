# SurgiTwin — Phase 4 Implementation Report

## 1. Executive Summary

Phase 4 introduces medical imaging ingestion and embedded hardware communication to the SurgiTwin Vision-Guided Surgical Robotics Digital Twin.

The medical imaging pipeline ingests cross-sectional DICOM series (CT/MRI), reconstructs physically consistent 3D medical volumes in Hounsfield Units, isolates anatomical structures or surgical targets via threshold segmentation and connected-component analysis, extracts geometric target centroids, and transforms them into robot base coordinates through rigid $SE(3)$ coordinate registration.

The embedded systems pipeline provides a Hardware Abstraction Layer (HAL) communicating with embedded microcontrollers via a framed binary protocol with CRC16-CCITT error detection, supporting both simulated actuator/encoder models and physical serial transport.

The complete data pipeline:
$$\text{DICOM CT} \longrightarrow \text{Volume} \longrightarrow \text{Segmentation} \longrightarrow \text{Target Centroid} \longrightarrow \text{Registration} \longrightarrow \text{/target\_pose} \longrightarrow \text{Phase 1 Safety} \longrightarrow \text{Phase 3 Robot Controller} \longrightarrow \text{HAL} \longrightarrow \text{Microcontroller}$$
has been executed and verified.

## 2. Implemented Subsystems & Modules

1. **DICOM Ingestion Engine** (`surgical_sim/dicom_loader.py`):
   - Native DICOM Part 10 parser (compatible with and without `pydicom`).
   - Physical slice normal sorting based on `ImagePositionPatient` and `ImageOrientationPatient`.
   - Modality value conversion to Hounsfield Units via `RescaleSlope` and `RescaleIntercept`.
   - Deterministic synthetic DICOM phantom generator (`generate_synthetic_series`).

2. **3D Medical Volume Engine** (`surgical_sim/medical_volume.py`):
   - 3D volumetric array $(N_z, N_y, N_x)$ in Hounsfield Units.
   - Voxel-to-patient and patient-to-voxel bidirectional affine mappings (LPS coordinate system in mm).
   - Multi-Planar Reconstruction (MPR) orthogonal slice extractors (Axial, Coronal, Sagittal).

3. **Medical Segmentation Engine** (`surgical_sim/medical_segmentation.py`):
   - Dual-threshold intensity windowing.
   - 3D morphological opening and closing noise filters.
   - 3D connected-component labeling and physical volume filtering.
   - 3D surface mesh export in binary STL format (`docs/models/segmented_target.stl`).

4. **Anatomical Target Extractor** (`surgical_sim/anatomical_target.py`):
   - Sub-voxel continuous centroid calculation.
   - Equivalent spherical radius and physical 3D bounding box computation.
   - Fail-closed empty/invalid mask detection.

5. **Medical Coordinate Registration** (`surgical_sim/medical_registration.py`):
   - 4x4 homogeneous transformation from patient space (mm) to robot base (m).
   - Rotation matrix orthogonality and determinant verification ($\det(R) = +1.0$).
   - Calibrated synthetic supine operating table model.
   - YAML configuration import and export.

6. **Embedded Hardware HAL** (`surgical_sim/hardware_interface.py`):
   - Binary packet encoder and decoder with CRC16-CCITT error detection.
   - Framing: Start markers `0xAA 0x55`, End markers `0x0D 0x0A`.
   - Transport abstraction for virtual SIMULATION loopback and physical SERIAL ports.

7. **Microcontroller Firmware Simulator** (`surgical_sim/microcontroller_sim.py`):
   - Simulates 6 joint channels with optical encoder quantization (4096 counts/rev).
   - Closed-loop actuator response lag and optical encoder telemetry generation.
   - Firmware state machine: `READY`, `MOVING`, `ESTOPPED`, `FAULT`.
   - Fault injection: packet drop, CRC corruption, and hardware disconnection.

8. **ROS 2 Integration Node** (`surgical_sim/surgitwin_medical_node.py`):
   - Integrates medical image loading, segmentation, and target publishing to `/target_pose`.
   - Forwards executed joint states to embedded microcontroller.
   - Publishes `/hardware/joint_feedback` and `/hardware/status` at 50 Hz.
   - Propagates emergency stop events.

9. **Launch File** (`launch/medical_robot_system.launch.py`):
   - Launches safety controller, robot sim node, medical/hardware node, and `robot_state_publisher`.

10. **Live Demonstration** (`demo_phase4.py` & `demo_phase4.sh`):
    - Full end-to-end demonstration generating diagnostic images (`docs/images/phase4_mpr_segmentation.png`), STL models (`docs/models/segmented_target.stl`), coordinate reports (`docs/reports/phase4_coordinate_report.json`), and telemetry logs (`docs/reports/phase4_hardware_telemetry.csv`).

