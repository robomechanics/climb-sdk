# Robot Description

## Overview

This folder contains a description package for each robot supported by climb-sdk.

## Compatibility Requirements

1. The package name must take the form `your_robot_description`.
2. The robot's description URDF or XACRO files must be stored in a `urdf/` subfolder.
3. The robot's rviz configurations must be stored in a `rviz/` subfolder.
4. Robot-specific parameters files must be stored in a `config/` subfolder.

## Viewing the Robot

The `loris_description` package contains a `display.launch.py` launch file that will display the robot geometry in `rviz` with an interface for changing joint angles.

```
ros2 launch loris_description display.launch.py
```

## Adding a New Robot

1. To quickly set up a new robot, start by copying the `loris_description` package.
2. Rename the package to `your_robot_description`.
3. Update the `name`, `version`, `description`, `maintainer`, and `license` fields in `package.xml`.
4. Update the `project` field in `CMakeLists.txt`.
5. Replace the contents of the `urdf/` and `meshes/` folders with your robot's geometry.
	- If needed, update the paths to the meshes in your robot's URDF file.
6. Rename the primary URDF or XACRO file to `your_robot.urdf.xacro`.
7. Set the `ROBOT` field in `launch/display.launch` to `your_robot`.
8. Rename `config/loris.yaml` to `config/your_robot.yaml` and set the ROS parameters.
9. Rename `rviz/loris.rviz` to `rviz/your_robot.rviz` and modify the layout and topics as desired.
10. Update `README.md` with a description of your robot.

Once all the above changes have been made, build the package with colcon.

```
colcon build --symlink-install --packages-select your_robot_description
```

View your robot with the `display.launch.py` launch file. You may first need to source `install/local_setup.bash` in a new terminal.

```
ros2 launch your_robot_description display.launch.py
```

Now you can specify which robot model to use when launching climb_sdk.

```
ros2 launch climb_main climber_base.launch.py robot:=your_robot
```