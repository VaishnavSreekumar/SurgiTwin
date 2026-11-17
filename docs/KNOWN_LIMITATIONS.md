# SurgiTwin — System Boundaries & Known Limitations

**Document ID:** LIM-ST-001  
**Project:** SurgiTwin: Vision-Guided Surgical Robotics Digital Twin  
**Scope:** Transparent enumeration of engineering, physical, perceptual, and regulatory limitations  
**Status:** Open Engineering Appraisal  

---

## 1. Regulatory & Clinical Disclaimer

> [!CAUTION]
> **EDUCATIONAL AND RESEARCH PROTOTYPE ONLY**  
> SurgiTwin is an academic digital twin simulation platform developed for educational and experimental robotics research. It is **NOT** a clinically validated medical device, has **NOT** undergone FDA 510(k) clearance or CE mark conformity assessment, and is strictly prohibited from being used for clinical diagnosis, surgical planning, or real human patient treatment.

---

## 2. Detailed Technical Limitations

### 2.1 Simulation vs. Physical Manipulator Reality
1. **Rigid Body Dynamics Assumption:** The URDF model and physics simulation assume rigid links and ideal revolute joints. In real surgical manipulators (such as the intuitive da Vinci or Medtronic Hugo), link compliance, structural flexure under payload, harmonic drive elasticity, and joint backlash introduce sub-millimetre kinematic deviations that are not modeled.
2. **Simplified Friction Model:** The dynamic plant models joint friction as simple linear viscous damping ($B \dot{q}$). Physical robot joints exhibit non-linear friction, including Stribeck effects, Coulomb static friction, and temperature-dependent lubricant viscosity.
3. **Actuator Torque Saturation:** While the software specifies effort limits, physical motor thermal limits, back-EMF saturation, and amplifier current limits under sustained holding torques are simplified.

### 2.2 Medical Imaging & Anatomical Modeling
1. **Synthetic Phantom vs. Clinical CT Artifacts:** The DICOM series used in validation is a synthetic geometric phantom with uniform attenuation shells. Physical clinical CT scans exhibit patient motion artifacts, beam hardening, metal scattering (e.g. from dental fillings or surgical clips), and Poisson noise.
2. **Monolithic Intensity Segmentation:** Dual-threshold intensity windowing relies on distinct Hounsfield Unit intervals. In actual human soft tissues, tumor margins frequently have overlapping radiodensity with healthy parenchyma, requiring advanced deep-learning or contrast-enhanced multi-spectral segmentation.
3. **Rigid Anatomical Assumption:** The volume reconstruction assumes a static, rigid patient coordinate frame ($X_{LPS}, Y_{LPS}, Z_{LPS}$). In live surgical environments, respiratory motion, cardiac pulsation, bowel peristalsis, and tool-induced tissue displacement create dynamic tissue deformations (brain shift, liver displacement) that rigid transformations cannot correct.

### 2.3 Computer Vision & Perception
1. **Pinhole Depth Assumption:** The single-camera vision pipeline reconstructs 3D Cartesian coordinates by assuming a known target depth $Z_c$ or fixed table height. Real surgical navigation systems require stereoscopic optical tracking (e.g., NDI Polaris) or RGB-D active structured light sensors to measure continuous depth disparity independently.
2. **Ideal Visual Lighting:** The ArUco and color fiducial detectors were validated on synthetic images with controlled contrast. In an actual operating theater, surgical lighting produces specular glare from polished metal scalpels, shadows from the surgical team, and obscuration from blood, saline irrigation, or electrocautery smoke.
3. **Marker Dependency:** The current vision pipeline requires cooperative fiducial markers. Markerless surgical instrument tracking (e.g., semantic segmentation of laparoscopic tool shafts) is not implemented in this baseline.

### 2.4 Embedded Hardware & Real-Time Communications
1. **Linux OS vs. Hard Real-Time Fieldbus:** The simulation nodes and embedded HAL run on standard Linux (WSL2 / Ubuntu 24.04). While achieving 50 Hz control with $14.89$ ms latency, standard Linux lacks hard real-time guarantees (`PREEMPT_RT`). Occasional thread scheduling jitter may cause millisecond timing spikes that would be unacceptable on a physical real-time EtherCAT or CANopen fieldbus.
2. **Virtual vs. Physical Serial Port:** The embedded HAL was validated primarily in virtual in-memory `SIMULATION` mode. Operating in physical `SERIAL` mode over USB UART (`/dev/ttyACM0`) introduces physical packet transmission delays (baud rate bandwidth limits at 115200 bps: ~1.5 ms per 17-byte packet) and potential OS buffer overruns under high bus contention.
3. **Lack of Cryptographic Authentication:** While CRC16-CCITT guarantees detection of transmission bit flips, it does not provide cryptographic message authentication (MAC) or encryption against malicious man-in-the-middle packet spoofing.

### 2.5 Motion Planning & Kinematics
1. **Point-to-Point Cartesian Linear Trajectories:** The trajectory generator calculates minimum-jerk profiles between start and goal configurations. It does not perform active dynamic collision avoidance with surgical staff or dynamic operating table obstacles during motion.
2. **Numerical Inverse Kinematics Initial Seed Sensitivity:** The numerical DLS IK solver converges reliably (100% on the surgical grid), but solver iteration count and trajectory continuity depend on proximity to the starting seed. Near kinematic singular configurations, nullspace damping prioritizes numerical stability over strict orientation tracking.

---

## 3. Summary of Limitations & Future Research Vectors

| Boundary Dimension | Current Digital Twin Baseline | Required for Clinical Product |
| :--- | :--- | :--- |
| **Anatomical Physics** | Rigid DICOM volume ($4\times 4$ affine) | Finite Element Method (FEM) soft-tissue deformation |
| **Depth Perception** | Pinhole back-projection with known $Z_c$ | Binocular stereo vision or time-of-flight depth sensor |
| **Dynamic Collision** | Static 3D envelope bounds | Real-time volumetric depth-map obstacle avoidance |
| **Communications** | Standard ROS 2 on Linux + CRC16 UART | Safe hard-real-time fieldbus (EtherCAT / IEC 61508 SIL3) |
| **Instrument Tracking** | Cooperative ArUco / color fiducials | Markerless deep neural network tool pose estimation |
| **Regulatory Status** | Educational / Research prototype | Formal verification under IEC 62304 / ISO 14971 / 510(k) |

