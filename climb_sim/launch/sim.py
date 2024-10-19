"""
Launch the robot control stack in a simulated Gazebo environment.

Optional Launch Arguments:
    robot: Name of the robot model
    urdf: Robot description file in {robot}_description/urdf/
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    PathJoinSubstitution,
    LaunchConfiguration,
    TextSubstitution,
    Command
)
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# Find launch arguments
robot = LaunchConfiguration("robot")
urdf = LaunchConfiguration("urdf")

# Find ROS packages
gz_pkg = FindPackageShare('ros_gz_sim')
sim_pkg = FindPackageShare('climb_sim')
main_pkg = FindPackageShare('climb_main')
robot_pkg = FindPackageShare([robot, TextSubstitution(text="_description")])

# Find resource paths
urdf_path = PathJoinSubstitution([robot_pkg, "urdf", urdf])
params_path = PathJoinSubstitution([robot_pkg, "config", "gz_params.yaml"])

# Generate URDF from XACRO
description = Command(["xacro ", urdf_path])
description_param = ParameterValue(description, value_type=str)


def generate_launch_description():
    # Declare launch arguments
    robot_arg = DeclareLaunchArgument(
        "robot",
        default_value="loris",
        description="Name of the robot model"
    )
    urdf_arg = DeclareLaunchArgument(
        "urdf",
        default_value=[robot, TextSubstitution(text=".urdf.xacro")],
        description="Robot description file in {robot}_description/urdf/"
    )

    # Launch Gazebo simulator
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gz_pkg, "launch", "gz_sim.launch.py"])),
        launch_arguments={'gz_args': PathJoinSubstitution([
            sim_pkg,
            'worlds',
            'test.sdf -r'
        ])}.items(),
    )

    # Spawn robot in Gazebo
    spawn_node = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-string', description,
            '-package', robot_pkg,
            '-name', robot,
            '-x', '0',
            '-y', '0',
            '-z', '0.5'
        ]
    )

    # Declare state publisher node
    state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=["--ros-args", "--log-level", "error"],
        output='screen',
        parameters=[{'robot_description': description_param}]
    )

    # Spawn joint controller
    joint_controller = Node(
        package='controller_manager',
        executable='spawner',
        arguments=[
            'climber_controller',
            '--param-file', params_path
        ],
    )
    joint_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )
    fts_broadcaster_1 = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['fts_broadcaster_1'],
    )

    return LaunchDescription([
        # Launch arguments
        robot_arg,
        urdf_arg,
        gz_launch,
        # ROS nodes
        state_publisher,
        # Spawn robot, joint controller, and joint broadcaster one-by-one
        spawn_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[joint_broadcaster],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_node,
                on_exit=[fts_broadcaster_1],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_broadcaster,
                on_exit=[joint_controller],
            )
        )
    ])
