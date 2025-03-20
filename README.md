# Climb-SDK

A general-purpose codebase for robotic climbing, originally developed for the LORIS robot ([paper](https://www.ri.cmu.edu/publications/loris-a-lightweight-free-climbing-robot-for-extreme-terrain-exploration/), [video](https://youtu.be/GjRrLqlI0yM)).

![The LORIS robot following a planned path across a horizontal tube](doc/Tube.gif)

## Installation

### Ubuntu

Climb-SDK has been tested with ROS2 Jazzy on Ubuntu 24.04. The following setup steps are required prior to installation.

1. Install Ubuntu 24.04 ([Desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop), [Dual Boot](https://help.ubuntu.com/community/WindowsDualBoot), or [WSL2](https://documentation.ubuntu.com/wsl/en/latest/guides/install-ubuntu-wsl2/))
2. Install ROS2 Jazzy ([tutorial](https://docs.ros.org/en/jazzy/Installation.html))
3. Create a colcon workspace ([tutorial](https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html))

Once the required setup is complete, run the following commands to clone the repository, install dependencies, and build packages.

```
cd ~/ros2_ws/src
git clone https://github.com/robomechanics/climb-sdk.git
cd climb-sdk
chmod +x setup.sh && ./setup.sh
cd ~/ros2_ws
colcon build --symlink-install
source install/local_setup.bash
```

### Docker Container

For consistent setup, Climb-SDK can be deployed with [Docker](https://www.docker.com/products/docker-desktop/). Once Docker is installed, the following commands will clone the repository and launch Climb-SDK in a container.

```
git clone https://github.com/robomechanics/climb-sdk.git
cd climb-sdk/.devcontainer
docker-compose up -d
docker exec -it Climb-SDK bash
```

Within the container, run the following commands to install dependencies and build packages.

```
cd /workspace/src/climb-sdk
chmod +x setup.sh && ./setup.sh
cd /workspace
colcon build --symlink-install
source install/local_setup.bash
```

Finally, run these commands to exit and shutdown the container when finished. Note that most data is not preserved between runs except for the source folder `src` and the build artifacts in `build`, `install`, and `log`.

```
exit
docker-compose down
```

### VS Code Integration

To streamline development, Climb-SDK can be integrated with [Visual Studio Code](https://code.visualstudio.com/) as a [dev container](https://code.visualstudio.com/docs/devcontainers/containers) using Docker. Install both Docker and VS Code, and from within VS Code install the Dev Containers extension. Then simply clone this repository, open the `climb-sdk` folder in VS Code, and select "Reopen in Container" when prompted. The commands to install dependencies and build packages are the same as in the Docker Container section above.

## Usage

### Quick Start

To quickly get the project running offline, run the following command from a terminal.

```
ros2 launch loris_bringup climber_plan.launch.py camera:=false
```

In the `xterm` window that opens, enter the following commands:

1. `simulate tube` (after the RViz window opens)
2. `plan`
3. `control on`
4. `execute` (after the planned path appears)

### Launch Files

`climber_base.launch.py` launches the hardware stack and user interface. This enables joint-level control of the robot. If no physical robot is detected then the robot will be simulated instead.

```
ros2 launch loris_bringup climber_base.launch.py
```

`climber_control.launch.py` additionally launches the low-level control stack. This enables a climbing gait with teleoperated or blind foothold selection.

```
ros2 launch loris_bringup climber_control.launch.py
```

`climber_plan.launch.py` launches the full planning stack. This enables fully autonomous climbing using the depth camera.

```
ros2 launch loris_bringup climber_plan.launch.py
```

The `camera` argument can be set to `false` to disable the camera (e.g. for offline simulation).

```
ros2 launch loris_bringup climber_plan.launch.py camera:=false
```

`sim.launch.py` in the `climb_sim` package launches a Gazebo simulation of the robot.

```
ros2 launch climb_sim sim.launch.py
```

### Command Interface

The base launch file opens an `xterm` window for sending commands to the robot. The same command interface can be opened in an existing terminal by launching a new `KeyInputNode`.

```
ros2 run climb_teleop key_input_node
```

A list of available commands can be found by entering `help`. Use the right arrow key for autocompletion and the up and down arrows to reuse previous commands. The following commands are currently supported.

| Command Syntax              | Description                                         | Example Usage                 |
| --------------------------- | --------------------------------------------------- | ----------------------------- |
| `help`                      | Display a list of available commands                | `help`                        |
| `enable`                    | Enable all motors and clear error status            | `enable`                      |
| `disable`                   | Disable all motors                                  | `disable`                     |
| `set JOINT position VALUE`  | Set a given joint angle in radians                  | `set tail_joint position 0.1` |
| `set JOINT velocity VALUE`  | Set a given joint velocity limit in radians/second  | `set tail_joint velocity 0.2` |
| `set JOINT effort VALUE`    | Set a given joint torque limit in Newton meters     | `set tail_joint effort 1.0`   |
| `move JOINT`                | Manually control a joint with the keyboard          | `move tail_joint`             |
| `control on`                | Turn on force control                               | `control on`                  |
| `control off      `         | Turn off force control                              | `control on`                  |
| `goto CONFIGURATION`        | Move the robot to a preset configuration            | `goto stand`                  |
| `twist CONTACT`             | Manually control an end effector with the keyboard  | `twist gripper_1`             |
| `twist BODY`                | Manually control the body frame with the keyboard   | `twist base_link`             |
| `step CONTACT`              | Take a step forward with the given end effector     | `step gripper_1`              |
| `simulate ENVIRONMENT`      | Simulate a predefined environment                   | `simulate tube`               |
| `plan`                      | Plan a route to the current goal pose               | `plan`                        |
| `execute`                   | Execute the most recently planned route             | `execute`                     |
| `align`                     | Move the robot state estimate relative to the world | `align`                       |

## Troubleshooting

### Serial Port Pass-Through

To directly control the robot over USB from a Windows host computer, the serial port must be passed through to the WSL2 distribution or Docker container. This can be done using [USB/IP](https://learn.microsoft.com/en-us/windows/wsl/connect-usb).

Note that this procedure requires a [WSL2](https://documentation.ubuntu.com/wsl/en/latest/guides/install-ubuntu-wsl2/) distribution to be installed and running in parallel even if using a Docker container.

```
wsl -d Ubuntu
```

Run the following in an administrator terminal to install the `usbipd` tool and identify the bus ID of the serial port.

```
winget install usbipd
usbipd list
```

Finally, bind the port and attach it to the running WSL2 distribution. The binding step (which requires administrator permissions) is only required once per port, while the attaching step must be done every time the robot is connected.

```
usbipd bind --busid <busid>
usbipd attach --wsl --busid <busid>
```

### Reducing Dynamixel Motor Latency

Set the USB port latency to 1 ms.

```
echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
```

Set the `Return Delay Time` parameter of each Dynamixel motor to zero.
