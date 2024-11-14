"""
Primary launch file to start up the robot, controller, and planners.

Optional Launch Arguments:
    robot: Name of the robot model
    namespace: Namespace of the individual robot
    global_config: Global parameters file in climb_main/config/
    robot_config: Robot parameters file in {robot}_description/config/
    urdf: Robot description file in {robot}_description/urdf/
    rviz: Rviz configuration file in {robot}_description/rviz/
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import (
    LaunchConfiguration,
    Command,
    PathJoinSubstitution,
    TextSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

# Find launch arguments
robot = LaunchConfiguration("robot")
namespace = LaunchConfiguration("namespace")
global_config = LaunchConfiguration("global_config")
robot_config = LaunchConfiguration("robot_config")
urdf = LaunchConfiguration("urdf")
rviz = LaunchConfiguration("rviz")

# Find ROS packages
main_pkg = FindPackageShare("climb_main")
robot_pkg = FindPackageShare([robot, TextSubstitution(text="_description")])

# Find resource paths
global_config_path = PathJoinSubstitution([main_pkg, "config", global_config])
robot_config_path = PathJoinSubstitution([robot_pkg, "config", robot_config])
urdf_path = PathJoinSubstitution([robot_pkg, "urdf", urdf])
rviz_path = PathJoinSubstitution([robot_pkg, "rviz", rviz])
camera_launch_path = PathJoinSubstitution([main_pkg, "launch", "camera.xml"])

# Generate URDF from XACRO
description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)


def generate_launch_description():
    # Declare launch arguments
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
        description="Global parameters file in climb_main/config/"
    )
    robot_config_arg = DeclareLaunchArgument(
        "robot_config",
        default_value=[robot, TextSubstitution(text=".yaml")],
        description="Robot parameters file in {robot}_description/config/"
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value=[robot, TextSubstitution(text=".urdf.xacro")],
        description="Robot description file in {robot}_description/urdf/"
    )
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value=[robot, TextSubstitution(text=".rviz")],
        description="Rviz configuration file in {robot}_description/rviz/"
    )

    # Declare ROS nodes
    robot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=namespace,
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        arguments=["--ros-args", "--log-level", "error"],
        parameters=[{
            "robot_description": description,
            "frame_prefix": [namespace, TextSubstitution(text="/")]
        }, global_config_path, global_config_path]
    )
    hardware = Node(
        package="climb_main",
        namespace=namespace,
        executable="hardware_node",
        name="hardware",
        output="screen",
        parameters=[global_config_path, robot_config_path]
    )
    controller = Node(
        package="climb_main",
        namespace=namespace,
        executable="controller_node",
        name="controller",
        output="screen",
        parameters=[{"tf_prefix": namespace},
                    global_config_path, robot_config_path]
    )
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path, "--ros-args", "--log-level", "error"],
        output="screen"
    )

    # Camera setup
    camera = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(camera_launch_path),
        launch_arguments={
            'rviz': 'False'
        }.items()
    )
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['--frame-id', 'camera_link', '--child-frame-id', 'loris/camera_link']
    )

    return LaunchDescription([
        # Launch arguments
        robot_arg,
        namespace_arg,
        global_config_arg,
        robot_config_arg,
        urdf_arg,
        rviz_arg,

        # ROS nodes
        robot_state_publisher,
        hardware,
        controller,
        rviz2,

        # Camera
        camera,
        camera_transform
    ])
