"""
surgitwin_full_system.launch.py
SurgiTwin Master Full-System Launch File.

Brings up the unified surgical robotics digital twin:
1. surgitwin_safety_controller (Phase 1 Safety Supervisor)
2. robot_sim_node (Phase 3 6-DOF Kinematics, Trajectory & Dynamic Plant)
3. vision_target_node (Phase 2 Vision Perception & Registration) [Conditional]
4. surgitwin_medical_node (Phase 4 Medical Ingestion & Embedded HAL) [Conditional]
5. robot_state_publisher (URDF TF broadcaster)

Supports fully headless simulation without physical cameras, scanners, or hardware.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("surgical_sim")
    urdf_path = os.path.join(pkg_dir, "urdf", "surgitwin_manipulator.urdf")

    robot_desc = ""
    if os.path.exists(urdf_path):
        with open(urdf_path, "r", encoding="utf-8") as f:
            robot_desc = f.read()

    # Launch configuration variables
    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate_hz",
        default_value="50.0",
        description="Dynamic simulation rate in Hz",
    )

    target_source_arg = DeclareLaunchArgument(
        "target_source",
        default_value="BOTH",
        description="Active target generation source: 'VISION', 'MEDICAL', or 'BOTH'",
    )

    hw_mode_arg = DeclareLaunchArgument(
        "hardware_mode",
        default_value="SIMULATION",
        description="Hardware transport mode: 'SIMULATION' or 'SERIAL'",
    )

    serial_port_arg = DeclareLaunchArgument(
        "serial_port",
        default_value="",
        description="Serial device path when hardware_mode is SERIAL",
    )

    # 1. Safety Supervisor Node (Core Gatekeeper)
    safety_node = Node(
        package="surgical_sim",
        executable="surgitwin_safety_controller",
        name="surgitwin_safety_controller",
        output="screen",
    )

    # 2. Robot Simulation Node (Kinematics, Trajectory, PID & Plant Dynamics)
    robot_sim_node = Node(
        package="surgical_sim",
        executable="robot_sim_node",
        name="surgitwin_robot_sim_node",
        output="screen",
        parameters=[{
            "sim_rate_hz": LaunchConfiguration("sim_rate_hz"),
            "max_cartesian_velocity": 0.10,
            "max_joint_velocity": 1.0,
            "tool_length": 0.150,
            "ik_tolerance_mm": 0.5,
        }],
    )

    # 3. Vision Perception Node (Conditional on target_source)
    run_vision = PythonExpression([
        "'", LaunchConfiguration("target_source"), "' in ['VISION', 'BOTH']"
    ])
    vision_node = Node(
        package="surgical_sim",
        executable="vision_target_node",
        name="vision_target_node",
        output="screen",
        condition=IfCondition(run_vision),
    )

    # 4. Medical Imaging & Embedded Hardware Node (Conditional on target_source)
    run_medical = PythonExpression([
        "'", LaunchConfiguration("target_source"), "' in ['MEDICAL', 'BOTH']"
    ])
    medical_node = Node(
        package="surgical_sim",
        executable="surgitwin_medical_node",
        name="surgitwin_medical_node",
        output="screen",
        parameters=[{
            "hardware_mode": LaunchConfiguration("hardware_mode"),
            "serial_port": LaunchConfiguration("serial_port"),
            "telemetry_rate_hz": LaunchConfiguration("sim_rate_hz"),
        }],
        condition=IfCondition(run_medical),
    )

    # 5. Robot State Publisher (TF Tree from URDF)
    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    return LaunchDescription([
        sim_rate_arg,
        target_source_arg,
        hw_mode_arg,
        serial_port_arg,
        safety_node,
        robot_sim_node,
        vision_node,
        medical_node,
        robot_state_pub,
    ])

