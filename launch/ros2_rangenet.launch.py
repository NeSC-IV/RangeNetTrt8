from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    current_pkg = FindPackageShare("rangenet_pp")
    return LaunchDescription(
        [
            Node(
                package="rangenet_pp",
                executable="ros2_demo",
                name="rangenet_pp",
                output="screen",
            ),
        ]
    )