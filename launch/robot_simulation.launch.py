"""
robot_simulation.launch.py
SurgiTwin Robotic Arm Simulation Launch File.

Launches:
1. surgitwin_safety_controller (Phase 1 safety supervisor)
2. robot_sim_node (Phase 3 6-DOF kinematics, trajectory, PID controller & plant simulator)
3. robot_state_publisher (TF broadcast from URDF model)
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

    # Read URDF for robot_state_publisher
    robot_desc = ""
    if os.path.exists(urdf_path):
        with open(urdf_path, "r") as f:
            robot_desc = f.read()

    sim_rate_arg = DeclareLaunchArgument(
        "sim_rate_hz",
        default_value="50.0",
        description="Robot dynamic simulation rate in Hz",
    )

    use_safety_arg = DeclareLaunchArgument(
        "use_safety",
        default_value="true",
        description="Whether to run the Phase 1 safety supervisor node",
    )

    safety_node = Node(
        package="surgical_sim",
        executable="surgitwin_safety_controller",
        name="surgitwin_safety_controller",
        output="screen",
        condition=IfCondition(LaunchConfiguration("use_safety")),
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

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_desc}],
    )

    return LaunchDescription([
        sim_rate_arg,
        use_safety_arg,
        safety_node,
        robot_sim_node,
        robot_state_pub,
    ])

