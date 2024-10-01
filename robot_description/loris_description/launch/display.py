"""
Simple launch file to display a robot URDF file.

Optional Launch Arguments:
    urdf: Robot description file in urdf/ directory
    rviz: Rviz configuration file in rviz/ directory
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Package definitions
PACKAGE = "loris_description"
URDF = "loris.urdf.xacro"
RVIZ = "display.rviz"

# Find resource paths
pkg = FindPackageShare(PACKAGE)
urdf_path = PathJoinSubstitution([pkg, "urdf", LaunchConfiguration("urdf")])
rviz_path = PathJoinSubstitution([pkg, "rviz", LaunchConfiguration("rviz")])

# Generate URDF from XACRO
description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)


def generate_launch_description():
    # Declare launch arguments
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value=URDF,
        description="Robot description file"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=RVIZ,
        description="Rviz configuration file"
    )

    # Declare ROS nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": description}]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
        output="screen"
    )

    return LaunchDescription([
        # Launch arguments
        urdf_arg,
        rviz_arg,

        # ROS nodes
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
