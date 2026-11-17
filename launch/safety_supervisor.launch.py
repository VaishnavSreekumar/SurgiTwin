"""
safety_supervisor.launch.py.

Launch file for starting the SurgiTwin Safety Supervisor node with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Generate launch description with safety supervisor node and arguments."""
    workspace_min_x_arg = DeclareLaunchArgument("workspace_min_x", default_value="-0.30")
    workspace_max_x_arg = DeclareLaunchArgument("workspace_max_x", default_value="0.30")
    workspace_min_y_arg = DeclareLaunchArgument("workspace_min_y", default_value="0.10")
    workspace_max_y_arg = DeclareLaunchArgument("workspace_max_y", default_value="0.60")
    workspace_min_z_arg = DeclareLaunchArgument("workspace_min_z", default_value="0.00")
    workspace_max_z_arg = DeclareLaunchArgument("workspace_max_z", default_value="0.40")
    max_velocity_arg = DeclareLaunchArgument("max_velocity_mps", default_value="0.100")
    max_staleness_arg = DeclareLaunchArgument("max_staleness_sec", default_value="2.0")
    allowed_frame_id_arg = DeclareLaunchArgument("allowed_frame_id", default_value="base_link")

    safety_node = Node(
        package="surgical_sim",
        executable="surgitwin_safety_controller",
        name="surgitwin_safety_controller",
        output="screen",
        parameters=[
            {
                "workspace_min_x": LaunchConfiguration("workspace_min_x"),
                "workspace_max_x": LaunchConfiguration("workspace_max_x"),
                "workspace_min_y": LaunchConfiguration("workspace_min_y"),
                "workspace_max_y": LaunchConfiguration("workspace_max_y"),
                "workspace_min_z": LaunchConfiguration("workspace_min_z"),
                "workspace_max_z": LaunchConfiguration("workspace_max_z"),
                "max_velocity_mps": LaunchConfiguration("max_velocity_mps"),
                "max_staleness_sec": LaunchConfiguration("max_staleness_sec"),
                "allowed_frame_id": LaunchConfiguration("allowed_frame_id"),
            }
        ],
    )

    return LaunchDescription(
        [
            workspace_min_x_arg,
            workspace_max_x_arg,
            workspace_min_y_arg,
            workspace_max_y_arg,
            workspace_min_z_arg,
            workspace_max_z_arg,
            max_velocity_arg,
            max_staleness_arg,
            allowed_frame_id_arg,
            safety_node,
        ]
    )
