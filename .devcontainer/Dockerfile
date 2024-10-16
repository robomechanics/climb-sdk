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
 && rm -rf /var/lib/apt/lists/*

COPY docker_config.sh /tmp/docker_config.sh

RUN cat /tmp/docker_config.sh >> /root/.bashrc \
 && rm /tmp/docker_config.sh