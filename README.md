# naoqi_driver2

This repo defines the __naoqi_driver__ package for ROS2. The driver is in charge of providing bridging capabilities between ROS2 and NAOqiOS.

## Dependencies
To run, the driver requires the `naoqi_libqi`, `naoqi_libqicore` and `naoqi_bridge_msgs` packages. Additionally, `pepper_meshes` and/or `nao_meshes` can be useful if you try to display the robot in RViz.

### Using the binaries
To install the binaries, use the following commands:
```sh
sudo apt-get install ros-<your_distro>-naoqi-libqi
sudo apt-get install ros-<your_distro>-naoqi-libqicore
sudo apt-get install ros-<your_distro>-naoqi-bridge-msgs

# Optional, if you want to install the Pepper or NAO meshes
sudo apt-get install ros-<your_distro>-pepper-meshes
sudo apt-get install ros-<your_distro>-nao-meshes
```

Please not that if the driver has been released, you can also install it using apt-get
```sh
sudo apt-get install ros-<your_distro>-naoqi-driver
```

### Installing from source

Create a workspace, and clone the required repositories:

```sh
cd <your_workspace>/src
vcs import < naoqi_driver2/dependencies.repos
```

Then, build the workspace:

```sh
cd <your_workspace>
colcon build --symlink-install
```

## Launch
The driver can be launched using the following command:
```sh
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<ip> nao_port:=<port> network_interface:=<interface> username:=<name> password:=<passwd>
```
Note that the username and password arguments are only required for robots running naoqi 2.9 or greater.

## Documentation

For further information, you can consult the documentation (__OUTDATED__) [here](http://ros-naoqi.github.io/naoqi_driver2/) or build it:

```sh
cd doc
doxygen Doxyfile
sphinx-build -b html ./source/ ./build/
```

## Development
Check how to [install the driver from source](#installing-from-source),
or use the [`Dockerfile`](Dockerfile) to get setup a reproducible dev environment:

```sh
docker build -t ros2-naoqi-driver .
```

## Build status

ROS Distro| Binary Status | Source Status | Github Build |
|-------------------|-------------------|-------------------|-------------------|
Humble | | | [![ros2-humble-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml/badge.svg)](https://github.com/naoqi_driver/actions/workflows/humble_jammy.yml)
Galactic | [![Build Status](https://build.ros2.org/job/Gbin_uF64__naoqi_driver__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros2.org/job/Gbin_uF64__naoqi_driver__ubuntu_focal_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Gsrc_uF__naoqi_driver__ubuntu_focal__source/badge/icon)](https://build.ros2.org/job/Gsrc_uF__naoqi_driver__ubuntu_focal__source/) | [![ros-galactic-focal](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/galactic_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/galactic_focal.yml)
Foxy | [![Build Status](https://build.ros2.org/job/Fbin_uF64__naoqi_driver__ubuntu_focal_amd64__binary/badge/icon)](https://build.ros2.org/job/Fbin_uF64__naoqi_driver__ubuntu_focal_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Fsrc_uF__naoqi_driver__ubuntu_focal__source/badge/icon)](https://build.ros2.org/job/Fsrc_uF__naoqi_driver__ubuntu_focal__source/) | [![ros-foxy-focal](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/foxy_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/foxy_focal.yml) |