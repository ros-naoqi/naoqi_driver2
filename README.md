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
Create a workspace, and clone the required repositories. Then, checkout a relevant tag / branch for the cloned repos, and build the packages. This is a cloning example for the ROS2 foxy distro:
```sh
git clone --branch v2.5.0-foxy https://github.com/ros-naoqi/libqi.git
git clone --branch v2.5.0-foxy https://github.com/ros-naoqi/libqicore.git
git clone https://github.com/ros-naoqi/naoqi_bridge_msgs2.git
git clone https://github.com/ros-naoqi/naoqi_driver2.git

# Eventually, if you didn't install the robot meshes.
git clone --branch ros2 https://github.com/ros-naoqi/pepper_meshes.git
git clone --branch ros2 https://github.com/ros-naoqi/nao_meshes.git
```

## Launch
The driver can be launched using the following command:
```sh
roslaunch naoqi_driver naoqi_driver.launch nao_ip:=<ip> nao_port:=<port> network_interface:=<interface> username:=<name> password:=<passwd>
```
Note that the username and password arguments are only required for robots running naoqi 2.9 or greater.

## Documentation
For further information, you can consult the documentation (__OUTDATED__) [here](http://ros-naoqi.github.io/naoqi_driver2/) or build it:

```sh
cd doc
doxygen Doxyfile
sphinx-build -b html ./source/ ./build/
```

## Build status

ROS Distro| Binary Status | Source Status | Github Build |
|-------------------|-------------------|-------------------|-------------------|
Humble | | | [![ros2-humble-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml/badge.svg)](https://github.com/naoqi_driver2/actions/workflows/humble_jammy.yml)
Galactic | | | [![ros-galactic-focal](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/galactic_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/galactic_focal.yml)
Foxy | | | [![ros-foxy-focal](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/foxy_focal.yml/badge.svg)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/foxy_focal.yml) |