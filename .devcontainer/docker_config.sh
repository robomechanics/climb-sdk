# Source ROS2 environment
source /ros_entrypoint.sh

# Source ROS2 workspace
if [ -f /workspace/install/local_setup.bash ]; then
    source /workspace/install/local_setup.bash;
fi

# Handle errors
trap 'exec /bin/bash' ERR

# Alias for Gazebo commands
alias gz='ign gazebo'

# Aliases for frequently used ROS2 launch files
alias climber_base='ros2 launch loris_bringup climber_base.launch.py'
alias climber='ros2 launch loris_bringup climber_control.launch.py'
alias climber_plan='ros2 launch loris_bringup climber_plan.launch.py'
alias sim='ros2 launch climb_sim sim.launch.py'
display() {
    ros2 launch "$1_description" display.launch.py
}

# Shortcut to build ROS2 packages
rosbuild() {
    if [ $# -eq 0 ]; then
        colcon build --symlink-install --continue-on-error --cmake-args "-DCMAKE_BUILD_TYPE=Debug"
    else
        colcon build --symlink-install --packages-select "$1" --cmake-args "-DCMAKE_BUILD_TYPE=Debug"
    fi
}

# Shortcut to test ROS2 packages
rostest() {
    if [ $# -eq 0 ]; then
        colcon test
    else
        colcon test --packages-select "$1"
    fi
    colcon test-result --verbose
}

# Shortcut to clean ROS2 packages
rosclean() {
    if [ $# -eq 0 ]; then
        rm -r /workspace/build/* /workspace/install/* /workspace/log/*
    else
        rm -r /workspace/build/"$1" /workspace/install/"$1"
    fi
}

# Shortcut to kill ROS2 nodes
roskill() {
    if [ $# -eq 0 ]; then
        key="ros-args"
    else
        key="$1.*ros-args"
    fi
    nodes=$(pgrep -fl "$key" || true)
    if [ -z "$nodes" ]; then
        echo 'No ROS nodes found'
    else
        echo -e "Killing ROS nodes\n$nodes"
        pkill -f "$key"
    fi
}

# Shortcut to set USB latency timer
alias latency='echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'