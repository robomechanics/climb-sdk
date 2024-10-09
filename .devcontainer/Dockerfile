ARG ROS_DISTRO=iron

FROM osrf/ros:${ROS_DISTRO}-desktop

RUN apt-get -y update \
 && apt-get -y install \
    vim \
    python3-pip \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-dynamixel-sdk \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-ros-gz \
    ros-${ROS_DISTRO}-gz-ros2-control \
    ros-${ROS_DISTRO}-ros2-controllers \
 && rm -rf /var/lib/apt/lists/* \
 && echo "source /ros_entrypoint.sh" >> ~/.bashrc \
 && echo "if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi" >> ~/.bashrc \
 && echo "trap 'exec /bin/bash' ERR" >> ~/.bashrc \
 && echo "alias gz='ign gazebo'" >> ~/.bashrc