# Lightweight Observation Robot for Irregular Slopes (LORIS)

## Overview

LORIS is a 3.6 kg quadruped robot that can ascend unstructured steep and vertical terrain using passive microspine grippers.

For further details, please refer to the following publication:

> P. Nadan, S. Backus and A. M. Johnson, "LORIS: A Lightweight Free-Climbing Robot for Extreme Terrain Exploration," in IEEE International Conference on Robotics and Automation (ICRA), 2024, pp. 18480-18486, doi: 10.1109/ICRA57147.2024.10611653.

## Usage

The `loris_description` package contains a `display.py` launch file that will display the robot geometry in `rviz` with an interface for changing joint angles.

```
ros2 launch loris_description display.py
```

Optional Xacro arguments can be included as follows:

```
ros2 launch loris_description display.py xacro_args:='param1:=value1 param2:=value2'
```

The following Xacro arguments are supported:

| Parameter        | Description                                                                  |
| :--------------- | :--------------------------------------------------------------------------- |
| floating_wrist   | Represent each wrist as 1 floating joint instead of 3 passive joints if true |
| camera_height    | Camera mounting height (m from body origin to pitch axis)                    |
| camera_pitch     | Camera mounting pitch (deg downward from horizontal)                         |
| gripper_offset   | Gripper angle offset (deg outward from centerline)                           |
| gripper_offset_1 | Override front left gripper angle offset                                     |
| gripper_offset_2 | Override front right gripper angle offset                                    |
| gripper_offset_3 | Override rear left gripper angle offset                                      |
| gripper_offset_4 | Override rear right gripper angle offset                                     |
| effort           | Actuator torque limit (Nm)                                                   |
| velocity         | Actuator velocity limit (rad/s)                                              |