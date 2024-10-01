FROM osrf/ros:iron-desktop

RUN apt-get -y update \
 && apt-get -y install \
    vim \
    python3-pip \
    ros-iron-joint-state-publisher-gui \
    ros-iron-xacro \
    ros-iron-dynamixel-sdk \
    ros-iron-kdl-parser \
 && rm -rf /var/lib/apt/lists/* \
 && echo "source /ros_entrypoint.sh" >> ~/.bashrc \
 && echo "if [ -f install/local_setup.bash ]; then source install/local_setup.bash; fi" >> ~/.bashrc \
 && echo "trap 'exec /bin/bash' ERR" >> ~/.bashrc