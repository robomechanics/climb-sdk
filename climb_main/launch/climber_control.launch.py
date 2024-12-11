"""
Launch file to start up the low-level control stack.

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
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Find launch arguments
robot = LaunchConfiguration("robot")
namespace = LaunchConfiguration("namespace")
global_config = LaunchConfiguration("global_config")
robot_config = LaunchConfiguration("robot_config")
xacro_args = LaunchConfiguration("xacro_args")

# Find ROS packages
main_pkg = FindPackageShare("climb_main")
robot_pkg = FindPackageShare([robot, TextSubstitution(text="_description")])

# Find resource paths
global_config_path = PathJoinSubstitution([main_pkg, "config", global_config])
robot_config_path = PathJoinSubstitution([robot_pkg, "config", robot_config])
base_launch_path = PathJoinSubstitution(
    [main_pkg, "launch", "climber_base.launch.py"])


def generate_launch_description():
    # Override default xacro arguments in base launch file
    xacro_args_arg = DeclareLaunchArgument(
        "xacro_args",
        default_value="wrist_joint:=floating",
        description="Xacro arguments of the form 'param:=value')"
    )

    # Base launch file
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={'xacro_args': xacro_args}.items()
    )

    # Low-level controller
    controller = Node(
        package="climb_main",
        namespace=namespace,
        executable="controller_node",
        name="controller",
        output="screen",
        parameters=[{"tf_prefix": namespace},
                    global_config_path, robot_config_path]
    )

    # Step planner
    step_planner = Node(
        package="climb_main",
        namespace=namespace,
        executable="step_planner_node",
        name="step_planner",
        output="screen",
        parameters=[{"tf_prefix": namespace},
                    global_config_path, robot_config_path]
    )

    return LaunchDescription([
        xacro_args_arg,
        base,
        controller,
        step_planner
    ])
