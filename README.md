# Climb-SDK

A general-purpose codebase for robotic climbing, originally developed for the LORIS robot ([paper](https://www.ri.cmu.edu/publications/loris-a-lightweight-free-climbing-robot-for-extreme-terrain-exploration/), [video](https://youtu.be/GjRrLqlI0yM)).

## Installation

### Ubuntu

Climb-SDK has been tested with ROS2 Iron on Ubuntu 22.04. The following setup steps are required prior to installation.

1. Install Ubuntu 22.04 ([Desktop](https://ubuntu.com/tutorials/install-ubuntu-desktop), [Dual Boot](https://help.ubuntu.com/community/WindowsDualBoot), or [WSL2](https://documentation.ubuntu.com/wsl/en/latest/guides/install-ubuntu-wsl2/))
2. Install ROS2 Iron ([tutorial](https://docs.ros.org/en/iron/Installation.html))
3. Create a colcon workspace ([tutorial](https://docs.ros.org/en/iron/Tutorials/Beginner-Client-Libraries/Creating-A-Workspace/Creating-A-Workspace.html))

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

`climber_base.launch.py` launches the hardware stack and user interface. This enables joint-level control of the robot.

```
ros2 launch climb_main climber_base.launch.py
```

`climber_control.launch.py` additionally launches the low-level control stack. This enables a climbing gait with teleoperated or blind foothold selection.

```
ros2 launch climb_main climber_control.launch.py
```

`climber_plan.launch.py` launches the full planning stack. This enables fully autonomous climbing using the depth camera.

```
ros2 launch climb_main climber_plan.launch.py
```

`sim.launch.py` in the `climb_sim` package launches a Gazebo simulation of the robot.

```
ros2 launch climb_sim sim.launch.py
```

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