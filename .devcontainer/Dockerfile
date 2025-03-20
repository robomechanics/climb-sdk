FROM osrf/ros:jazzy-desktop

RUN apt-get -y update \
 && apt-get -y install \
    vim \
    python3-pip \
    usbutils \
    xterm \
    gdb \
    ros-$ROS_DISTRO-joint-state-publisher-gui \
    ros-$ROS_DISTRO-xacro \
    ros-$ROS_DISTRO-dynamixel-sdk \
    ros-$ROS_DISTRO-kdl-parser \
    ros-$ROS_DISTRO-ros-gz \
    ros-$ROS_DISTRO-gz-ros2-control \
    ros-$ROS_DISTRO-ros2-controllers \
    ros-$ROS_DISTRO-osqp-vendor \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-realsense2-camera \
    ros-$ROS_DISTRO-imu-filter-madgwick \
    ros-$ROS_DISTRO-rtabmap-ros \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-ompl \
 && rm -rf /var/lib/apt/lists/*

COPY docker_config.sh /tmp/docker_config.sh

RUN cat /tmp/docker_config.sh >> /root/.bashrc \
 && rm /tmp/docker_config.sh