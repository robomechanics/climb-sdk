"""
Simple launch file to display a robot URDF file.

Optional Launch Arguments:
    urdf: Robot description file in urdf/ directory
    rviz: Rviz configuration file in rviz/ directory
    xacro_args: Additional arguments for xacro (e.g. 'param:=value')
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

# Name of the robot
ROBOT = 'loris'

# Find resource paths
pkg = FindPackageShare(ROBOT + '_description')
urdf_path = PathJoinSubstitution([pkg, 'urdf', LaunchConfiguration('urdf')])
rviz_path = PathJoinSubstitution([pkg, 'rviz', LaunchConfiguration('rviz')])

# Generate URDF from XACRO
description = ParameterValue(
    Command(['xacro ', urdf_path, ' ', LaunchConfiguration('xacro_args')]),
    value_type=str)


def generate_launch_description():
    # Declare launch arguments
    urdf_arg = DeclareLaunchArgument(
        'urdf',
        default_value=ROBOT + '.urdf.xacro',
        description='Robot description file'
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='display.rviz',
        description='Rviz configuration file'
    )
    xacro_args_arg = DeclareLaunchArgument(
        'xacro_args',
        default_value='',
        description="Xacro arguments (e.g. 'param:=value')"
    )

    # Declare ROS nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': description}]
    )
    joint_state_publisher = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_path],
        output='screen'
    )

    return LaunchDescription([
        # Launch arguments
        urdf_arg,
        rviz_arg,
        xacro_args_arg,

        # ROS nodes
        robot_state_publisher,
        joint_state_publisher,
        rviz2
    ])
