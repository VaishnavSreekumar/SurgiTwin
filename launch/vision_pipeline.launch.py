"""
vision_pipeline.launch.py.

Launch file starting the SurgiTwin Vision Perception Pipeline together with
the Phase 1 Safety Supervisor.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description with vision node and safety supervisor."""
    working_depth_arg = DeclareLaunchArgument("working_depth_m", default_value="0.60")
    target_marker_arg = DeclareLaunchArgument("target_marker_id", default_value="0")
    synthetic_stream_arg = DeclareLaunchArgument("synthetic_stream_hz", default_value="5.0")

    vision_node = Node(
        package="surgical_sim",
        executable="vision_target_node",
        name="vision_target_node",
        output="screen",
        parameters=[
            {
                "working_depth_m": LaunchConfiguration("working_depth_m"),
                "target_marker_id": LaunchConfiguration("target_marker_id"),
                "synthetic_stream_hz": LaunchConfiguration("synthetic_stream_hz"),
            }
        ],
    )

    safety_node = Node(
        package="surgical_sim",
        executable="surgitwin_safety_controller",
        name="surgitwin_safety_controller",
        output="screen",
    )

    return LaunchDescription(
        [
            working_depth_arg,
            target_marker_arg,
            synthetic_stream_arg,
            vision_node,
            safety_node,
        ]
    )

