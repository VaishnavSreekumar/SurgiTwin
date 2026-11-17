"""
medical_robot_system.launch.py
SurgiTwin Full Medical Robotics & Hardware System Launch File.

Launches:
1. surgitwin_safety_controller (Phase 1 Safety Supervisor)
2. robot_sim_node (Phase 3 6-DOF Kinematics & Simulation)
3. surgitwin_medical_node (Phase 4 Medical Ingestion & Embedded Hardware Bridge)
4. robot_state_publisher (URDF TF broadcaster)
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory("surgical_sim")
    urdf_path = os.path.join(pkg_dir, "urdf", "surgitwin_manipulator.urdf")

    robot_desc = ""
    if os.path.exists(urdf_path):
        with open(urdf_path, "r") as f:
            robot_desc = f.read()

    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate_hz",
        default_value="50.0",
        description="Dynamic simulation rate in Hz",
    )

    safety_node = Node(
        package="surgical_sim",
        executable="surgitwin_safety_controller",
        name="surgitwin_safety_controller",
        output="screen",
    )

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

    medical_node = Node(
        package="surgical_sim",
        executable="surgitwin_medical_node",
        name="surgitwin_medical_node",
        output="screen",
        parameters=[{
            "hardware_mode": "SIMULATION",
            "telemetry_rate_hz": LaunchConfiguration("sim_rate_hz"),
        }],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    return LaunchDescription([
        sim_rate_arg,
        safety_node,
        robot_sim_node,
        medical_node,
        robot_state_pub,
    ])

