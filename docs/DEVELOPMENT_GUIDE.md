# SurgiTwin — Developer Onboarding & Engineering Guide

**Target Environment:** Ubuntu 24.04 LTS (Noble Numbat) / ROS 2 Jazzy Jalisco  
**Python Version:** 3.12.3  
**Package Name:** `surgical_sim`  

---

## 1. Environment Setup

### 1.1 ROS 2 Jazzy Installation
Ensure ROS 2 Jazzy desktop is installed on Ubuntu 24.04:
```bash
# Source ROS 2 base environment
source /opt/ros/jazzy/setup.bash
echo $ROS_DISTRO  # Must output: jazzy
```

### 1.2 Python Dependencies
SurgiTwin relies on standard open-source scientific and robotics libraries:
```bash
python3 -c "import cv2, numpy, scipy, matplotlib, yaml, serial; print('All core libraries available!')"
```

---

## 2. Workspace Build Workflow

Always build the workspace from the repository root:

```bash
cd /home/vaish123/robotics/SurgiTwin

# 1. Source ROS 2 underlay
source /opt/ros/jazzy/setup.bash

# 2. Build the surgical_sim package with symlink install
colcon build --packages-select surgical_sim --symlink-install

# 3. Source the local workspace overlay
source install/setup.bash
```

> [!TIP]
> Using `--symlink-install` allows you to edit Python files in `surgical_sim/` without having to rebuild the package every time. However, modifications to launch files or package metadata (`setup.py`, `package.xml`) require a re-run of `colcon build`.

---

## 3. Automated Testing Workflows

### 3.1 Running the Full Regression Suite
Execute the entire test suite via colcon:
```bash
colcon test --packages-select surgical_sim && colcon test-result --verbose
```
Current expected output: **Summary: 138 tests, 0 errors, 0 failures, 1 skipped (`test_copyright.py`)**.

### 3.2 Running via Pytest
For faster interactive development, run pytest directly with verbose test discovery:
```bash
# Run all tests
pytest test/ -v

# Run a specific subsystem test suite
pytest test/test_inverse_kinematics.py -v
pytest test/test_full_system_fault_injection.py -v

# Filter tests by expression
pytest test/ -k "estop or velocity" -v
```

### 3.3 Code Style & Static Linting
Ensure full compliance with PEP 8 and PEP 257 docstring conventions:
```bash
pytest test/test_flake8.py test/test_pep257.py -v
```

---

## 4. Running the Demonstrations & Benchmarks

### 4.1 Master 17-Step Full-System Demonstration
Executes end-to-end verification across DICOM reconstruction, vision fiducial tracking, safety bounds gating, DLS inverse kinematics, quintic trajectory planning, 50 Hz PID simulation, embedded HAL communication, and active E-stop dynamic braking:
```bash
./demo_full_system.sh
# Alternatively:
python3 demo_full_system.py
```
Output report is generated at: `docs/reports/surgitwin_master_demo_report.json`.

### 4.2 Quantitative Performance Benchmark
Measures sub-pixel accuracy, round-trip registration errors, IK solvability, trajectory compliance, and pipeline latencies:
```bash
python3 scripts/evaluate_surgitwin.py
```
Outputs generated:
- `docs/reports/surgitwin_validation_results.json`
- `docs/reports/surgitwin_validation_summary.md`
- `docs/images/evaluation_tracking_rmse.png`
- `docs/images/evaluation_latency_breakdown.png`

---

## 5. Launch File Catalog

Launch nodes individually or as coordinated subsystems:

```bash
# Master Phase 5 Full-System Launch
ros2 launch surgical_sim surgitwin_full_system.launch.py target_source:=BOTH

# Phase 1 Safety Supervisor Node Only
ros2 launch surgical_sim safety_supervisor.launch.py

# Phase 2 Vision Perception Pipeline Only
ros2 launch surgical_sim vision_pipeline.launch.py

# Phase 3 Robot Kinematics & Simulation Only
ros2 launch surgical_sim robot_simulation.launch.py

# Phase 4 Medical Imaging & Embedded Hardware Only
ros2 launch surgical_sim medical_robot_system.launch.py
```

---

## 6. Development Conventions & Safety Rules

1. **Fail-Closed Safety Gate:** Never bypass the safety supervisor. Any command originating from vision or medical planning must publish to `/target_pose`. Only the `surgitwin_safety_controller` is authorized to publish to `/safe_target_pose`.
2. **DDS Domain Isolation:** When authoring new ROS 2 integration tests, instantiate nodes under an isolated domain ID (`domain_id=47, 50, 51`) to avoid multi-threaded cross-talk across parallel test runs.
3. **No Sudo or System Modifications:** Never run `sudo` commands or modify system libraries outside the workspace.
4. **MIT Attribution:** Maintain the original MIT copyright headers and attribution to Zara Shariff in all created and modified source files.

