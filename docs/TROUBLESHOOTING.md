# SurgiTwin — Troubleshooting & Diagnostic Guide

This document lists common development issues, failure signatures, root causes, and verified solutions when developing and running SurgiTwin under Ubuntu 24.04 and ROS 2 Jazzy.

---

## 1. ROS 2 Middleware & DDS Communication Issues

### Issue 1.1: Multi-Node Background Crosstalk or Stray Target Messages
- **Symptom:** Integration tests fail with assertions like `assert 3 == 2` or receive unexpected messages published from previous runs.
- **Root Cause:** Background ROS 2 daemon or stray node processes from previous test runs are still publishing or discovery multicasting on default DDS domain `0`.
- **Solution:**
  1. Kill all stray ROS 2 node processes:
     ```bash
     pkill -9 -f surgitwin || true
     pkill -9 -f ros2 || true
     ```
  2. Use isolated ROS 2 domain IDs in test environments:
     ```bash
     export ROS_DOMAIN_ID=55
     pytest test/ -v
     ```
  3. Reset the ROS 2 daemon:
     ```bash
     ros2 daemon stop && ros2 daemon start
     ```

---

## 2. Python Environment & Import Errors

### Issue 2.1: `ModuleNotFoundError: No module named 'surgical_sim'`
- **Symptom:** Running `pytest` or standalone scripts fails with an import error for `surgical_sim`.
- **Root Cause:** The workspace overlay environment has not been sourced into the current shell session.
- **Solution:**
  Ensure you source the built workspace overlay before executing Python commands:
  ```bash
  source /opt/ros/jazzy/setup.bash
  source /home/vaish123/robotics/SurgiTwin/install/setup.bash
  ```
  Alternatively, prepend `PYTHONPATH`:
  ```bash
  export PYTHONPATH="/home/vaish123/robotics/SurgiTwin:$PYTHONPATH"
  ```

---

## 3. Motion Planning & Kinematic Issues

### Issue 3.1: Inverse Kinematics Non-Convergence or Target Rejection
- **Symptom:** Target inside bounds fails IK solve, and robot transitions to `STATE_ERROR`.
- **Root Cause:** The target position falls outside the dexterous workspace radius ($R < 0.10$ m or $R > 1.05$ m from the robot base) or lies in a kinematic singularity where joint limits prohibit reaching the target.
- **Solution:**
  Check target coordinates relative to the base frame. Verify that:
  $$\sqrt{X^2 + Y^2 + Z^2} \le 0.85\ \text{m}$$
  If targeting a boundary coordinate, use candidate seed switching in `InverseKinematicsSolver.solve_with_seeds()`.

### Issue 3.2: Velocity Violation Rejection on Rapid Successive Targets
- **Symptom:** Target rejected with reason `VELOCITY_VIOLATION: Computed velocity ... exceeds limit 100.0 mm/s`.
- **Root Cause:** A new target was commanded too quickly after a previous target, resulting in $\Delta P / \Delta t > 0.100$ m/s.
- **Solution:**
  Allow adequate execution time for each trajectory before sending a new target, or clear the velocity baseline by invoking `/safety/reset` if resetting after an operational pause.

---

## 4. Hardware & Serial Port Permissions

### Issue 4.1: `SerialException: [Errno 13] Permission denied: '/dev/ttyACM0'`
- **Symptom:** Setting `hardware_mode:=SERIAL` fails to open the physical serial port.
- **Root Cause:** The current Linux user lacks permissions for the `dialout` group.
- **Solution:**
  1. Add your user to the `dialout` group:
     ```bash
     sudo usermod -a -G dialout $USER
     ```
  2. For virtual testing without physical hardware connected, ensure the default `SIMULATION` mode is selected:
     ```bash
     ros2 launch surgical_sim surgitwin_full_system.launch.py hardware_mode:=SIMULATION
     ```

---

## 5. GUI & WSL2 Display Forwarding (RViz2 / OpenCV)

### Issue 5.1: `Could not connect to display` or RViz2 Black Screen
- **Symptom:** Launching RViz2 or OpenCV GUI windows in WSL2 crashes or displays an X11 connection error.
- **Root Cause:** Wayland / WSLg display server socket mismatch or headless execution environment.
- **Solution:**
  1. Set the display environment variable:
     ```bash
     export DISPLAY=:0
     ```
  2. For automated verification and headless CI execution, run with `headless:=true`:
     ```bash
     ros2 launch surgical_sim surgitwin_full_system.launch.py headless:=true
     ```

