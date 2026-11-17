# SurgiTwin — Dependency Compatibility Analysis

## 1. Platform Matrix Comparison

| Attribute | Upstream Repository (`surgical_sim`) | SurgiTwin Target Environment | Status & Impact |
|---|---|---|---|
| **Host OS** | Ubuntu 22.04 LTS (Native) | Windows 11 + WSL2 | Supported; GUI visualization requires WSLg or X-Server |
| **Linux Distribution** | Ubuntu 22.04 (Jammy Jellyfish) | Ubuntu 24.04.4 LTS (Noble Numbat) | System libraries (glibc 2.39, Python 3.12) are newer |
| **ROS Distribution** | ROS 2 Humble Hawksbill | ROS 2 Jazzy Jalisco | API differences in launch, tf2, and message vendors |
| **Physics Simulator** | Gazebo Classic 11 (`gazebo_ros`) | Gazebo Harmonic (`ros_gz`) | **INCOMPATIBLE**: Gazebo Classic is discontinued on Noble |
| **Motion Planning** | MoveIt 2 Humble | MoveIt 2 Jazzy (`ros-jazzy-moveit`) | Available via apt; requires Jazzy configuration schema |
| **Python Version** | Python 3.10 | Python 3.12.3 | Strict packaging; `setuptools` deprecated build helpers removed |

---

## 2. Dependency Audit & Replacement Strategy

### 2.1. Gazebo Classic (`gazebo_ros`) $\rightarrow$ Gazebo Harmonic (`ros_gz`)
- **Root Cause of Incompatibility**: Gazebo Classic 11 reached end-of-life in early 2025 and was removed from Ubuntu 24.04 repositories. The ROS 2 Humble bridge `gazebo_ros_pkgs` does not compile against ROS 2 Jazzy.
- **Replacement**: Gazebo Harmonic (`ros-jazzy-ros-gz`).
- **Code Changes Required**:
  - Launch files must replace `FindPackageShare('gazebo_ros')` with `FindPackageShare('ros_gz_sim')`.
  - Spawning entities uses `ros_gz_sim create` rather than `spawn_entity.py`.
  - SDF files need update from version 1.6 to 1.9/1.10.
- **MVP Strategy**: Implement an internal kinematic simulator node in Python for the day-one demonstration, eliminating Gazebo launch instability while maintaining exact ROS 2 topic/joint interfaces.

### 2.2. MoveIt 2 Humble $\rightarrow$ MoveIt 2 Jazzy
- **Root Cause of Incompatibility**: MoveIt 2 in Jazzy underwent significant restructuring in its Python bindings, launch parameter utilities (`moveit_configs_utils`), and kinematics solver plugins.
- **Replacement**: `ros-jazzy-moveit` and `ros-jazzy-moveit-msgs`.
- **Code Changes Required**:
  - `surgical_controller.py` currently crashes because `moveit_msgs` is not installed.
  - Sourcing and configuring MoveIt 2 in Jazzy requires installing `ros-jazzy-moveit-msgs` or the complete `ros-jazzy-moveit` metapackage.
- **MVP Strategy**: To guarantee a working demonstration today without blocking on a 1.5 GB compilation/installation cycle, create an integrated analytical/numerical IK solver within SurgiTwin. MoveIt 2 will be layered on in Phase 5.

### 2.3. Universal Robots Description (`ur_description`, `ur_moveit_config`)
- **Status in Upstream**: Relied on `ros-humble-ur-description` and PickNik's `ros-humble-ur-moveit-config`.
- **Status in Jazzy**: Available in apt as `ros-jazzy-ur-description` and `ros-jazzy-ur-moveit-config`.
- **SurgiTwin Approach**: Embed a verified UR5e kinematic parameter set (DH parameters) directly in the SurgiTwin package so kinematics, coordinate mapping, and tool positioning can run standalone with 100% test reproducibility.

### 2.4. Python Dependencies
- **`numpy`**: Installed (v1.26.4 / v2.x compatible).
- **`opencv-python` (`cv2`)**: Installed and verified in WSL2.
- **`scipy`**: Installed.
- **`matplotlib`**: Installed.
- **`rclpy`**: Installed and verified in `/opt/ros/jazzy`.

---

## 3. Dependency Compatibility Table

| Upstream Dependency | Upstream Role | Jazzy Equivalent | Action Plan |
|---|---|---|---|
| `rclpy` | ROS 2 Python client | `rclpy` (Jazzy) | Retain; verified working. |
| `geometry_msgs` | Coordinate message types | `geometry_msgs` (Jazzy) | Retain; verified working. |
| `sensor_msgs` | Images & joint feedback | `sensor_msgs` (Jazzy) | Retain; verified working. |
| `std_msgs` | Status and string signals | `std_msgs` (Jazzy) | Retain; verified working. |
| `trajectory_msgs` | Trajectory commands | `trajectory_msgs` (Jazzy) | Retain; verified working. |
| `tf2_ros` | Frame transformations | `tf2_ros` (Jazzy) | Retain; verified working. |
| `moveit_msgs` | MoveGroup action definitions | `ros-jazzy-moveit-msgs` | Standalone kinematic solver for MVP; install when MoveIt is configured. |
| `gazebo_ros` | Gazebo Classic launcher | `ros-jazzy-ros-gz-sim` | Deprecated; migrate to `ros_gz` in Phase 5. |
| `robot_state_publisher` | TF broadcaster | `ros-jazzy-robot-state-publisher` | Retain; verified installed. |
| `joint_state_publisher` | Joint GUI/feeder | `ros-jazzy-joint-state-publisher` | Available via apt. |

