from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    return LaunchDescription([
        DeclareLaunchArgument(
            "params_file",
            default_value="/home/likunwei/humanoid_ws/src/min_snap/config/min_snap.yaml",
            description="YAML parameter file for min_snap_node.",
        ),
        Node(
            package="min_snap",
            executable="min_snap_node",
            name="min_snap_node",
            output="screen",
            parameters=[params_file],
        ),
    ])
