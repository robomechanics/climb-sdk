"""
Launch file to start up the low-level control stack.

Optional Launch Arguments:
    robot: Name of the robot model
    namespace: Namespace of the individual robot
    global_config: Global parameters file in {robot}_bringup/config/
    robot_config: Robot parameters file in {robot}_description/config/
    urdf: Robot description file in {robot}_description/urdf/
    rviz: Rviz configuration file in {robot}_description/rviz/
    xacro_args: Xacro arguments of the form 'param:=value')
    odometry: Estimate odometry with dead reckoning (True/False)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# Find launch arguments
robot = LaunchConfiguration('robot')
namespace = LaunchConfiguration('namespace')
global_config = LaunchConfiguration('global_config')
robot_config = LaunchConfiguration('robot_config')
xacro_args = LaunchConfiguration('xacro_args')
odometry = LaunchConfiguration('odometry')

# Find ROS packages
main_pkg = FindPackageShare('loris_bringup')
robot_pkg = FindPackageShare([robot, '_description'])

# Find resource paths
global_config_path = PathJoinSubstitution([main_pkg, 'config', global_config])
robot_config_path = PathJoinSubstitution([robot_pkg, 'config', robot_config])
base_launch_path = PathJoinSubstitution([main_pkg, 'launch', 'climber_base.launch.py'])


def generate_launch_description():
    # Launch arguments
    odometry_arg = DeclareLaunchArgument(
        'odometry',
        default_value='True',
        description='Estimate odometry with dead reckoning (True/False)'
    )

    # Override default xacro arguments in base launch file
    xacro_args_arg = DeclareLaunchArgument(
        'xacro_args',
        default_value='wrist_joint:=floating',
        description="Xacro arguments of the form 'param:=value')"
    )

    # Base launch file
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={'xacro_args': xacro_args,
                          'publish_map': 'False'}.items()
    )

    # Low-level controller
    controller = Node(
        package='climb_control',
        namespace=namespace,
        executable='controller_node',
        name='controller',
        output='screen',
        parameters=[global_config_path, robot_config_path,
                    {'compute_odometry': odometry}],
        remappings=[(['/', namespace, '/imu'], '/imu/data')]
    )

    # Gait planner
    gait_planner = Node(
        package='climb_gait_planner',
        namespace=namespace,
        executable='gait_planner_node',
        name='gait_planner',
        output='screen',
        parameters=[global_config_path, robot_config_path]
    )

    return LaunchDescription([
        odometry_arg,
        xacro_args_arg,
        base,
        controller,
        gait_planner
    ])
