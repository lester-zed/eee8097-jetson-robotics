from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    description_share = get_package_share_directory("roarm_description")
    urdf_path = os.path.join(description_share, "urdf", "roarm_description.urdf")
    rviz_config = os.path.join(description_share, "config", "roarm_description.rviz")

    with open(urdf_path, "r", encoding="utf-8") as handle:
        robot_description = handle.read()

    serial_port = LaunchConfiguration("serial_port")
    baud_rate = LaunchConfiguration("baud_rate")
    publish_rate_hz = LaunchConfiguration("publish_rate_hz")
    response_timeout_s = LaunchConfiguration("response_timeout_s")
    project_src = LaunchConfiguration("project_src")
    allow_motion = LaunchConfiguration("allow_motion")
    home_config = LaunchConfiguration("home_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyROARM"),
            DeclareLaunchArgument("baud_rate", default_value="115200"),
            DeclareLaunchArgument("publish_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("response_timeout_s", default_value="2.0"),
            DeclareLaunchArgument("project_src", default_value=""),
            DeclareLaunchArgument("allow_motion", default_value="false"),
            DeclareLaunchArgument("home_config", default_value=""),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="eee8097_roarm_driver",
                executable="roarm_state_node",
                name="eee8097_roarm_state_publisher",
                output="screen",
                parameters=[
                    {
                        "serial_port": serial_port,
                        "baud_rate": baud_rate,
                        "publish_rate_hz": publish_rate_hz,
                        "response_timeout_s": response_timeout_s,
                        "project_src": project_src,
                        "allow_motion": allow_motion,
                        "home_config": home_config,
                    }
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="screen",
                arguments=["-d", rviz_config],
            ),
        ]
    )
