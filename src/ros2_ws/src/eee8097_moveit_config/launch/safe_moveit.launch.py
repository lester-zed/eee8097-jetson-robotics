from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    use_real_hardware = LaunchConfiguration("use_real_hardware")
    allow_motion = LaunchConfiguration("allow_motion")
    allow_execution = LaunchConfiguration("allow_execution")
    use_rviz = LaunchConfiguration("use_rviz")
    serial_port = LaunchConfiguration("serial_port")
    project_src = LaunchConfiguration("project_src")
    gripper_collision_position = LaunchConfiguration(
        "gripper_collision_position_rad"
    )

    moveit_config = (
        MoveItConfigsBuilder("roarm_m2", package_name="eee8097_moveit_config")
        .robot_description(file_path="config/roarm_m2_safe.urdf.xacro")
        .robot_description_semantic(file_path="config/roarm_m2_safe.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    planning_scene_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
        "publish_robot_description": True,
        "publish_robot_description_semantic": True,
        "allow_trajectory_execution": ParameterValue(
            allow_execution, value_type=bool
        ),
        "monitor_dynamics": False,
    }

    vendor_moveit_share = get_package_share_directory("roarm_moveit")
    rviz_config = os.path.join(vendor_moveit_share, "rviz", "interact.rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_real_hardware", default_value="false"),
            DeclareLaunchArgument("allow_motion", default_value="false"),
            DeclareLaunchArgument("allow_execution", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyROARM"),
            DeclareLaunchArgument("project_src", default_value="/workspace/src"),
            DeclareLaunchArgument(
                "gripper_collision_position_rad", default_value="0.0"
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[moveit_config.robot_description],
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="fake_joint_state_publisher",
                output="screen",
                condition=UnlessCondition(use_real_hardware),
                parameters=[moveit_config.robot_description, {"rate": 10}],
            ),
            Node(
                package="eee8097_roarm_driver",
                executable="roarm_driver_node",
                name="eee8097_roarm_driver",
                output="screen",
                condition=IfCondition(use_real_hardware),
                parameters=[
                    {
                        "serial_port": ParameterValue(serial_port, value_type=str),
                        "project_src": ParameterValue(project_src, value_type=str),
                        "allow_motion": ParameterValue(allow_motion, value_type=bool),
                        "gripper_collision_position_rad": ParameterValue(
                            gripper_collision_position, value_type=float
                        ),
                    }
                ],
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                name="move_group",
                output="screen",
                parameters=[moveit_config.to_dict(), planning_scene_parameters],
            ),
            Node(
                package="eee8097_moveit_bridge",
                executable="moveit_bridge_node",
                name="eee8097_moveit_bridge",
                output="screen",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    {
                        "allow_execution": ParameterValue(
                            allow_execution, value_type=bool
                        )
                    },
                ],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                output="log",
                condition=IfCondition(use_rviz),
                arguments=["-d", rviz_config],
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                    moveit_config.planning_pipelines,
                    moveit_config.joint_limits,
                ],
            ),
        ]
    )
