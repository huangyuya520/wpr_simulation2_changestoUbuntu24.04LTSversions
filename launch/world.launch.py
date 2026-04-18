#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    world = LaunchConfiguration("world")

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"]
            )
        ),
        launch_arguments=[("gz_args", ["-r -v 1 ", world])],
    )

    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        output="screen",
    )

    set_pose_bridge = RosGzBridge(
        bridge_name="gazebo_service_bridge",
        extra_bridge_params={
            "bridges": {
                "set_pose_bridge": {
                    "service_name": "/world/default/set_pose",
                    "ros_type_name": "ros_gz_interfaces/srv/SetEntityPose",
                    "gz_req_type_name": "gz.msgs.Pose",
                    "gz_rep_type_name": "gz.msgs.Boolean",
                }
            },
            "bridge_names": ["set_pose_bridge"],
        },
        log_level="info",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "world",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("wpr_simulation2"), "worlds", "robocup_home.world"]
                ),
                description="Absolute path to the Gazebo Sim world file",
            ),
            gz_sim,
            clock_bridge,
            set_pose_bridge,
        ]
    )
