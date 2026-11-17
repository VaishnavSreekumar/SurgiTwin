# SurgiTwin — Initial Repository Audit (Phase 0)

## 1. Executive Summary

- **Repository**: [SurgiTwin](https://github.com/zshariff02/Surgical_Sim) (originally `surgical_sim` by Zara Shariff)
- **Local Location**: `/home/vaish123/robotics/SurgiTwin`
- **License**: MIT License (Preserved with attribution)
- **Target Platform (Original)**: Ubuntu 22.04 LTS (Jammy), ROS 2 Humble, Gazebo Classic 11, MoveIt 2 Humble
- **Target Platform (Current)**: Windows 11 WSL2, Ubuntu 24.04.4 LTS (Noble), ROS 2 Jazzy, Gazebo Harmonic (modern `ros_gz`), MoveIt 2 Jazzy, Python 3.12.3

The repository is a lightweight starter prototype containing three Python nodes, two launch files, three config files, and an SDF world file. While the README describes an end-to-end surgical simulator with Gazebo Classic and MoveIt2, **none of the simulation launch files or MoveIt action calls currently execute on Ubuntu 24.04 / ROS 2 Jazzy**. Additionally, multiple features and automated tests claimed in the README are completely absent from the codebase.

---

## 2. Existing Architecture & Code Inventory

```
SurgiTwin/
├── config/
│   ├── moveit_controllers.yaml       # UR5e trajectory controller interface
│   ├── surgical_sim_moveit.yaml      # OMPL planner config & workspace constraints
│   └── ur_sim_controllers.yaml       # ros2_control JointTrajectoryController config
├── launch/
│   ├── surgical_sim.launch.py        # Spawns UR5e in Gazebo Classic via gazebo_ros (BROKEN on Jazzy)
│   └── ur_moveit_sim.launch.py       # PickNik MoveIt launch script template (BROKEN on Jazzy)
├── resource/
│   └── surgical_sim                  # Ament resource marker
├── surgical_sim/
│   ├── __init__.py
│   ├── metrics_logger.py             # CSV logging node (10 Hz) for tip error & velocity
│   ├── pose_publisher.py             # CLI node for publishing geometry_msgs/PoseStamped targets
│   └── surgical_controller.py        # Central node: bounds check & MoveGroup action client
├── test/
│   ├── test_copyright.py             # Skipped linter test
│   ├── test_flake8.py                # Style linter
│   └── test_pep257.py                # Docstring linter
├── worlds/
│   └── surgical_table.world          # SDF 1.6 world with surgical table, floor, and lighting
├── package.xml                       # Package manifest (ament_python)
├── setup.py                          # Setup script with entrypoints & data_files
└── README.md                         # Documentation
```

---

## 3. Detailed Component Analysis

### 3.1. Nodes (`surgical_sim/`)

1. **`surgical_controller.py`**
   - **Intended Role**: Subscribes to `/target_pose`, validates Cartesian boundaries (`x: [-0.3, 0.3]`, `y: [0.1, 0.6]`, `z: [0.0, 0.4]`), dispatches goals to MoveIt2 `/move_action` (`MoveGroup`), monitors `/tool_pose` for tip velocity violation (> 100 mm/s), and publishes `/controller_status` and `/position_error`.
   - **Current Status**: **BROKEN / INOPERABLE**. Fails immediately on import with `ModuleNotFoundError: No module named 'moveit_msgs'` because `moveit_msgs` is not installed in the ROS 2 Jazzy environment.
   - **Architectural Limitation**: Hardcoded to MoveGroup action client with no fallback for direct inverse kinematics, analytical forward kinematics, or headless simulation.

2. **`pose_publisher.py`**
   - **Intended Role**: Interactive CLI terminal prompt accepting `x y z` coordinates or `home`, publishing `geometry_msgs/PoseStamped` to `/target_pose` at frame `base_link` with a fixed downward orientation quaternion `[0, 1, 0, 0]`.
   - **Current Status**: **FUNCTIONAL**. Imports cleanly and runs as an independent ROS 2 node.

3. **`metrics_logger.py`**
   - **Intended Role**: Subscribes to `/target_pose`, `/tool_pose`, and `/controller_status`. Logs timestamps, Cartesian setpoints, actual positions, Euclidean errors (mm), and velocities (mm/s) to `~/surgical_sim_metrics.csv` at 10 Hz.
   - **Current Status**: **FUNCTIONAL**. Starts properly, manages file lifecycle, writes CSV headers, and flushes output cleanly.

### 3.2. Launch Files & Configs (`launch/`, `config/`)

1. **`launch/surgical_sim.launch.py`**
   - **Current Status**: **BROKEN**. Directly invokes `gazebo_ros` package (`gazebo.launch.py` and `spawn_entity.py`). In Ubuntu 24.04 and ROS 2 Jazzy, `gazebo_ros` is obsolete and replaced by `ros_gz_sim`. It fails immediately with `package 'gazebo_ros' not found`.
   - Missing local URDF: References `ur_description/urdf/ur.urdf.xacro` with `sim_gazebo:=true`. In ROS 2 Jazzy / Gazebo Harmonic, the xacro parameter is `sim_ignition:=true` or `simulation_controllers`.

2. **`launch/ur_moveit_sim.launch.py`**
   - **Current Status**: **BROKEN**. Directly depends on `ur_moveit_config.launch_common`, which is uninstalled and Humble-specific in its configuration syntax.

3. **`worlds/surgical_table.world`**
   - **Current Status**: Valid SDF 1.6 world containing static box geometry for an operating table, ground plane, and point lights. Can be rendered in Gazebo Harmonic (`gz sim`) after minor SDF version migration.

---

## 4. Discrepancies Between Documentation and Implementation

| Item / Claim in README | Stated in README | Actual Implementation in Repository |
|---|---|---|
| **Automated Tests** | "10 automated tests covering safety rules and accuracy in `test/test_surgical_sim.py`" | **Does not exist**. Only 3 standard ament linters exist (`test_copyright.py`, `test_flake8.py`, `test_pep257.py`). |
| **URDF Directory** | `setup.py` lists `(os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf.xacro'))` | The `urdf/` directory does not exist in the repository. |
| **Computer Vision** | Planned next step | No vision code, camera drivers, OpenCV nodes, or target detection modules exist. |
| **Kinematics Engine** | Delegated to MoveIt2 MoveGroup | No native kinematics solvers (FK/IK) or analytical kinematics exist in the package. |
| **Medical Imaging** | None | No DICOM loading, SimpleITK, or coordinate registration modules exist. |
| **Control System** | JointTrajectoryController via Gazebo Classic | No feedback control loop running within ROS node logic; relies entirely on external Gazebo plugin. |

---

## 5. Dependency & Platform Risks

1. **Ubuntu 24.04 / Python 3.12 Compatibility**:
   - Python 3.12 enforces strict module loading and deprecates `pkg_resources` and `distutils`.
   - ROS 2 Jazzy uses new ament and Python 3.12 bindings.
2. **Gazebo Classic End-of-Life**:
   - Gazebo Classic 11 reached EOL in 2025 and is unavailable in Ubuntu 24.04 package repos.
   - Attempting to force Gazebo Classic onto Noble would break system libraries.
   - All Gazebo simulation must target **Gazebo Harmonic (`ros_gz`)**.
3. **MoveIt 2 Jazzy vs. Headless Kinematics**:
   - Running full MoveIt 2 inside WSL2 with graphical RViz requires substantial RAM, GPU acceleration, and package installation.
   - For an achievable Same-Day MVP, a dual-track approach is necessary:
     - **Track A (Guaranteed MVP)**: Direct numerical/analytical IK solver + TF2 transform broadcaster + robot state publisher + headless/2D visualizer.
     - **Track B (Full Stack)**: MoveIt 2 Jazzy and Gazebo Harmonic integration once base packages are confirmed.

---

## 6. Git Status & Remote Architecture

- Current branch: `main`
- Origin remote: `https://github.com/zshariff02/Surgical_Sim.git`
- Clean working directory with no untracked files or uncommitted changes.
- In accordance with project instructions, original commit history (`025cb25`) and MIT attribution to Zara Shariff will remain intact. Remote re-pointing to user's independent repository will occur only upon human authorization.

