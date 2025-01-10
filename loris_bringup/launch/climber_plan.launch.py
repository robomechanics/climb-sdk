"""
Launch file to start up the full planning stack.

Optional Launch Arguments:
    robot: Name of the robot model
    namespace: Namespace of the individual robot
    global_config: Global parameters file in climb_main/config/
    robot_config: Robot parameters file in {robot}_description/config/
    urdf: Robot description file in {robot}_description/urdf/
    rviz: Rviz configuration file in {robot}_description/rviz/
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    TextSubstitution
)
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Find launch arguments
robot = LaunchConfiguration("robot")
namespace = LaunchConfiguration("namespace")
global_config = LaunchConfiguration("global_config")
robot_config = LaunchConfiguration("robot_config")
camera = LaunchConfiguration("camera")

# Find ROS packages
main_pkg = FindPackageShare("loris_bringup")
robot_pkg = FindPackageShare([robot, TextSubstitution(text="_description")])

# Find resource paths
global_config_path = PathJoinSubstitution([main_pkg, "config", global_config])
robot_config_path = PathJoinSubstitution([robot_pkg, "config", robot_config])
control_launch_path = PathJoinSubstitution(
    [main_pkg, "launch", "climber_control.launch.py"])
camera_launch_path = PathJoinSubstitution([main_pkg, "launch", "camera.xml"])


def generate_launch_description():
    # Launch arguments
    camera_arg = DeclareLaunchArgument(
        "camera",
        default_value="true",
        description="Launch the Realsense camera"
    )

    # Low-level controller launch file
    controller_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(control_launch_path)
    )

    # Footstep planner
    footstep_planner = Node(
        package='climb_footstep_planner',
        executable='footstep_planner_node',
        name='footstep_planner',
        namespace=namespace,
        output='screen',
        parameters=[{"tf_prefix": namespace},
                    global_config_path, robot_config_path],
        remappings=[('/loris/map_cloud', '/rtabmap/cloud_map')]
    )

    # Camera setup
    camera_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(camera_launch_path),
        condition=IfCondition(camera),
        launch_arguments={
            'rviz': 'False'
        }.items()
    )
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        condition=IfCondition(camera),
        arguments=['--frame-id', 'camera_link', '--child-frame-id', 'loris/camera_link',
                   "--ros-args", "--log-level", "ERROR"]
    )
    map_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        condition=UnlessCondition(camera),
        arguments=['--frame-id', "loris/map", '--child-frame-id', "map",
                   "--ros-args", "--log-level", "ERROR"])

    return LaunchDescription([
        camera_arg,
        controller_launch,
        footstep_planner,
        camera_launch,
        camera_transform,
        map_transform
    ])
