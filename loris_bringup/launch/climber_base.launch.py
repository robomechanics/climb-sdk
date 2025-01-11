"""
Launch file to start up the robot hardware stack and user interface.

Optional Launch Arguments:
    robot: Name of the robot model
    namespace: Namespace of the individual robot
    global_config: Global parameters file in {robot}_bringup/config/
    robot_config: Robot parameters file in {robot}_description/config/
    urdf: Robot description file in {robot}_description/urdf/
    rviz: Rviz configuration file in {robot}_description/rviz/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution
)
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue

# Find launch arguments
robot = LaunchConfiguration("robot")
namespace = LaunchConfiguration("namespace")
global_config = LaunchConfiguration("global_config")
robot_config = LaunchConfiguration("robot_config")
urdf = LaunchConfiguration("urdf")
rviz = LaunchConfiguration("rviz")
xacro_args = LaunchConfiguration("xacro_args")
static_map_transform = LaunchConfiguration("static_map_transform")
static_odom_transform = LaunchConfiguration("static_odom_transform")

# Find ROS packages
main_pkg = FindPackageShare("loris_bringup")
robot_pkg = FindPackageShare([robot, "_description"])

# Find resource paths
global_config_path = PathJoinSubstitution([main_pkg, "config", global_config])
robot_config_path = PathJoinSubstitution([robot_pkg, "config", robot_config])
urdf_path = PathJoinSubstitution([robot_pkg, "urdf", urdf])
rviz_path = PathJoinSubstitution([robot_pkg, "rviz", rviz])

# Generate URDF from XACRO
description = ParameterValue(
    Command(["xacro ", urdf_path, " ", xacro_args]),
    value_type=str)


def generate_launch_description():
    # Launch arguments
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="loris",
        description="Name of the robot model"
    )
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=robot,
        description="Namespace of the individual robot"
    )
    global_config_arg = DeclareLaunchArgument(
        "global_config",
        default_value="params.yaml",
        description="Global parameters file in {robot}_bringup/config/"
    )
    robot_config_arg = DeclareLaunchArgument(
        "robot_config",
        default_value=[robot, ".yaml"],
        description="Robot parameters file in {robot}_description/config/"
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value=[robot, ".urdf.xacro"],
        description="Robot description file in {robot}_description/urdf/"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=[robot, ".rviz"],
        description="Rviz configuration file in {robot}_description/rviz/"
    )
    xacro_args_arg = DeclareLaunchArgument(
        "xacro_args",
        default_value="wrist_joint:=fixed",
        description="Xacro arguments of the form 'param:=value')"
    )
    static_odom_transform_arg = DeclareLaunchArgument(
        "static_odom_transform",
        default_value="true",
        description="Provide a static transform to the odom frame"
    )
    static_map_transform_arg = DeclareLaunchArgument(
        "static_map_transform",
        default_value="true",
        description="Provide a static transform to the map frame"
    )

    # Robot hardware stack
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
        parameters=[{
            "robot_description": description,
            "frame_prefix": [namespace, "/"]
        }, global_config_path]
    )
    robot_driver = Node(
        package="climb_robot_driver",
        namespace=namespace,
        executable="robot_driver_node",
        name="robot_driver",
        output="screen",
        parameters=[global_config_path, robot_config_path]
    )

    # User interface
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path, "--ros-args", "--log-level", "error"],
        output="screen"
    )
    teleop = Node(
        package="climb_teleop",
        namespace=namespace,
        executable="teleop_node",
        name="teleop",
        output="screen",
        parameters=[global_config_path, robot_config_path],
        remappings=[(["/", namespace, "/key_input"], '/key_input'),
                    (["/", namespace, "/key_key_output"], '/key_output')]
    )
    key_input = Node(
        package="climb_teleop",
        executable="key_input_node",
        name="key_input",
        output="screen",
        parameters=[global_config_path, robot_config_path],
        prefix="xterm -T 'Climb-SDK' -fa 'Monospace' -fs 14 -e"
    )

    # Static transforms
    map_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom",
                   "--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(static_map_transform)
    )
    odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "odom", "--child-frame-id", [namespace, "/base_link"],
                   "--ros-args", "--log-level", "ERROR"],
        condition=IfCondition(static_odom_transform)
    )

    return LaunchDescription([
        robot_arg,
        namespace_arg,
        global_config_arg,
        robot_config_arg,
        urdf_arg,
        rviz_arg,
        xacro_args_arg,
        static_odom_transform_arg,
        static_map_transform_arg,
        robot_state_publisher,
        robot_driver,
        rviz2,
        teleop,
        key_input,
        map_transform,
        odom_transform
    ])
