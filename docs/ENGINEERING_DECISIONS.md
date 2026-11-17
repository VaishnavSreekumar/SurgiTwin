# SurgiTwin — Engineering Decisions & Architectural Trade-offs

## 1. Decision Records

### ADR 01: Dual-Track Simulation Strategy (Kinematic Simulator vs. Heavy Simulators)
- **Context**: The original repository relied entirely on Gazebo Classic 11 and MoveIt 2 Humble. In Ubuntu 24.04 and ROS 2 Jazzy, Gazebo Classic is obsolete, while installing and configuring Gazebo Harmonic and MoveIt 2 within a WSL2 environment can consume hours in dependency resolution, GPU passthrough issues, and RAM constraints (~7.6 GB available).
- **Decision**: Adopt a dual-track strategy:
  - **Track A (Core MVP)**: Build an embedded analytical/numerical kinematics engine and kinematic robot simulator directly in ROS 2 Jazzy. It subscribes to joint commands, solves forward kinematics via DH matrices, and publishes `/joint_states` and `/tool_pose` at 50 Hz.
  - **Track B (Extended Simulation)**: Gazebo Harmonic and MoveIt 2 Jazzy will be developed as a drop-in execution backend once the core perception, registration, safety, and kinematics pipeline is verified.
- **Alternatives Considered**:
  - *Option 1: Block development until Gazebo Harmonic and MoveIt 2 are installed and compiled.* Rejected because it risks zero working software on Day 1.
  - *Option 2: Mock all ROS topics with static playback.* Rejected because it violates the mandate for authentic engineering and genuine closed-loop math.
- **Trade-offs**: Track A lacks full multi-body dynamic physics (inertia, contact friction), but provides 100% deterministic, instant startup, sub-millimeter kinematic accuracy, and headless testing without graphical WSL dependencies.

---

### ADR 02: Computer Vision Technique Selection (OpenCV vs. Deep Learning)
- **Context**: Domain 2 requires camera acquisition, image preprocessing, instrument/target detection, and coordinate extraction.
- **Decision**: Implement the MVP perception pipeline using deterministic OpenCV computer vision techniques (HSV color space thresholding, morphological noise filtering, contour hierarchy, and spatial centroid moments) paired with pinhole camera back-projection.
- **Alternatives Considered**:
  - *Option 1: Train a YOLOv8 / Mask R-CNN network on EndoVis.* Considered for Phase 5. For the same-day MVP, training or hosting deep learning weights introduces massive compute overhead, non-deterministic latency, and potential GPU inference issues on WSL2.
- **Trade-offs**: Classical CV requires controlled illumination and distinct visual markers or tool coloration, but runs at $>60\text{ FPS}$ with zero GPU dependency, sub-pixel centroid accuracy, and complete mathematical interpretability.

---

### ADR 03: Safety Monitoring Placement (Isolated Node vs. Kinematics Decorator)
- **Context**: Medical robotics standards (IEC 62304 / ISO 14971) require that safety-critical interlocking and workspace limit checks cannot be bypassed by upstream algorithmic failures.
- **Decision**: Decouple safety verification into a dedicated supervisory ROS 2 node (`surgitwin_safety_controller`). Target coordinates from the perception pipeline must be validated by the safety layer before being forwarded to the motion planner.
- **Trade-offs**: Introduces an extra ROS 2 topic hop ($\approx 0.5\text{ ms}$ latency), but guarantees fail-safe isolation and strict conformance to safety architecture requirements.

---

### ADR 04: Coordinate Registration & Pinhole Projection Formulation
- **Context**: The system must map 2D image coordinates to 3D Cartesian coordinates in the robot's base frame.
- **Decision**: Implement a classical pinhole projection model using calibrated camera intrinsic parameters $(f_x, f_y, c_x, c_y)$ and rigid homogeneous transformation matrices:
  $$\mathbf{P}_{cam} = Z \begin{bmatrix} (u - c_x)/f_x \\ (v - c_y)/f_y \\ 1 \end{bmatrix}$$
  $$\mathbf{P}_{base} = T_{base}^{cam} \begin{bmatrix} \mathbf{P}_{cam} \\ 1 \end{bmatrix}$$
- **Trade-offs**: Assumes a known or estimated working distance / depth plane $Z$ for monocular vision, or stereo disparity. For the surgical operating field, working depth is constrained by the surgical table plane ($Z_{plane} = 0.15\text{ m}$ above table).

---

### ADR 05: Medical Device Risk Management & Regulatory Positioning
- **Context**: The project references IEC 62304, ISO 13485, and ISO 14971.
- **Decision**: Include formal software lifecycle documentation (requirements, traceability, risk register), but explicitly state in all code headers and documentation that **SurgiTwin is a research digital twin prototype and NOT a clinically approved medical device**.
- **Rationale**: Demonstrates rigorous engineering competence while upholding ethical and regulatory integrity.

