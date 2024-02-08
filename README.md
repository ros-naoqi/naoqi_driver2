# naoqi_driver2

This repo defines the __naoqi_driver__ package for ROS2. The driver is in charge of providing bridging capabilities between ROS2 and NAOqiOS.


## How it works

The __naoqi_driver__ is a ROS node.
It connects to a robot running NAOqi using libQi.
To support the audio features, __naoqi_driver__ opens a public endpoint, on a random port by default.

> To avoid opening a public endpoint,
> you would need to disable the audio features
> by setting the `audio` argument to `false` in [boot_config.json](share/boot_config.json),
> and then set  `qi_listen_url` to listen to a local endpoint,
> *e.g.* `qi_listen_url:=tcp://127.0.0.1:12345`.


## Installation

### Dependencies

To run, the driver requires the [`naoqi_libqi`](https://github.com/ros-naoqi/libqi),
[`naoqi_libqicore`](https://github.com/ros-naoqi/libqicore)
and [`naoqi_bridge_msgs`](https://github.com/ros-naoqi/naoqi_bridge_msgs2) packages.
Those can be installed using apt-get (if they have been released for your ROS distro) or from source.
Additionally, [`pepper_meshes`](https://github.com/ros-naoqi/pepper_meshes2)
and/or [`nao_meshes`](https://github.com/ros-naoqi/nao_meshes2) can be useful to display the robot in RViz.

On Ubuntu, install them using:

```sh
sudo apt-get install ros-<distro>-naoqi-libqi ros-<distro>-naoqi-libqicore ros-<distro>-naoqi-bridge-msgs ros-<distro>-pepper-meshes ros-<distro>-nao-meshes
```

### Installing from source

In your ROS2 workspace, clone the repo and its dependencies:

```sh
cd <ws>/src
git clone https://github.com/ros-naoqi/naoqi_driver2.git
vcs import < naoqi_driver2/dependencies.repos
cd <ws>
rosdep install --from-paths src --ignore-src --rosdistro <distro> -y
```

> To install vcs: `sudo apt-get install python3-vcstool`

Then build the workspace:

```sh
cd <ws>
colcon build --symlink-install
```

> Meshes can only be built on x86(_64) platforms.
> You can skip them by building with these arguments:
>
> ```sh
> colcon build --packages-skip nao_meshes pepper_meshes
> ```

#### License of the meshes

The source repositories include
[`pepper_meshes2`](https://github.com/ros-naoqi/pepper_meshes2)
and [`nao_meshes2`](https://github.com/ros-naoqi/nao_meshes2),
which require an interactive agreement to be provided.
If you agree to their license terms (
[CC BY-NC-ND 4.0](https://creativecommons.org/licenses/by-nc-nd/4.0/legalcode.en):
[`pepper_meshes2` LICENSE](https://github.com/ros-naoqi/pepper_meshes/blob/master/LICENSE),
[`nao_meshes2` LICENSE](https://github.com/ros-naoqi/nao_meshes/blob/master/LICENSE)).
you may skip the interactive prompt by setting
the `AGREE_TO_NAO_MESHES_LICENSE` and `I_AGREE_TO_PEPPER_MESHES_LICENSE` environment variables:

```sh
I_AGREE_TO_NAO_MESHES_LICENSE=1 I_AGREE_TO_PEPPER_MESHES_LICENSE=1 colcon build --symlink-install
```

Automated jobs on non-interactive environments
(`DEBIAN_FRONTEND=noninteractive`)
defaults to agreeing to the licenses,
assuming the author of the job has agreed to the license terms.


## Launch

### Avoid interference with autonomous life

To have full control of the robot with ROS,
you may want to disable autonomous behaviors first:

```sh
ssh nao@<robot_host>
qicli call ALAutonomousLife.setState disabled
qicli call ALMotion.wakeUp
```


### NAOqi 2.8 and lower

The driver can be launched from a remote machine this way:

```sh
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> qi_listen_url:=tcp://0.0.0.0:0
```


### NAOqi 2.9 and higher

Username and password arguments are required
for robots running NAOqi 2.9 or greater:

```sh
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> nao_username:=nao nao_password:=<robot_password> qi_listen_url:=tcp://0.0.0.0:0
```

Warning: If you have a `connection refused error` such as [this issue](https://github.com/ros-naoqi/naoqi_driver/issues/162)
(from ROS1 driver version) when using robots running NAOqi 2.8 and greater, please try to give `nao_port:=9503` explicitly.

### From a Docker container

If you run __naoqi_driver__ from a Docker container with audio features enabled,
you must specify the libQi endpoint with, *e.g.* for port 56000:

```sh
source /opt/ros/<distro>/setup.bash # or source <ws>/install/setup.bash if built from source
ros2 launch naoqi_driver naoqi_driver.launch.py nao_ip:=<robot_host> qi_listen_url:=tcp://0.0.0.0:56000
```

Then you must [expose](https://docs.docker.com/engine/reference/commandline/run/#publish) it from the container.


### On the robot or on the same machine

If you run the driver directly from the robot (or your machine running a virtual robot),
you can omit the options:

```sh
ros2 launch naoqi_driver naoqi_driver.launch.py
```

## Check that the node is running correctly

Check that the driver is connected:

```sh
ros2 node info /naoqi_driver
```

### Hello, world

Make the robot say hello:

```sh
ros2 topic pub --once /speech std_msgs/String "data: hello"
```

### Listen to words

You can setup speech recognition and get one result.

```sh
ros2 action send_goal listen naoqi_bridge_msgs/action/Listen "expected: [\"hello\"]
language: \"en\""
```

### Move the head

Check that you can move the head by publishing on `/joint_angles`:

```sh
ros2 topic pub --once /joint_angles naoqi_bridge_msgs/JointAnglesWithSpeed "{header: {stamp: now, frame_id: ''}, joint_names: ['HeadYaw', 'HeadPitch'], joint_angles: [0.5,0.1], speed: 0.1, relative: 0}"
```

You can see the published message with `ros2 topic echo /joint_angles`

### Move around

Check that you can move the robot by publishing on `cmd_vel` to make the robot move:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.5
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.785"
```

> Make some room around the robot!

To stop the robot, you must send a new message with linear and angular velocities set to 0:

```sh
ros2 topic pub --once /cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```


## Development

Check how to [install the driver from source](#installing-from-source),
or use `docker compose` to get a more reproducible dev environment:

```sh
ROS_DISTRO=iron docker compose up dev --build
```

> To get all logs in live and build debuggable binaries,
> you may build with this command:
>
> ```sh
> colcon build --event-handlers console_direct+ --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Debug
> ```
>
> It works also when running tests:
>
> ```sh
> colcon test --event-handlers console_direct+ --ctest-args tests
> ```


## Build Status

ROS Distro | Binary Status | Source Status | GitHub Status |
|-------------------|-------------------|-------------------|--------------------|
Humble | [![ros2-humble-jammy-bin-status-badge](https://build.ros2.org/job/Hbin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary) | [![ros2-humble-jammy-src-status-badge](https://build.ros2.org/job/Hsrc_uJ__naoqi_driver__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Hsrc_uJ__naoqi_driver__ubuntu_jammy__source) | [![ros2-humble-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml/badge.svg?branch=main)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/humble_jammy.yml)
Iron | [![ros2-iron-jammy-bin-status-badge](https://build.ros2.org/job/Ibin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Ibin_uJ64__naoqi_driver__ubuntu_jammy_amd64__binary) | [![ros2-iron-jammy-src-status-badge](https://build.ros2.org/job/Isrc_uJ__naoqi_driver__ubuntu_jammy__source/badge/icon)](https://build.ros2.org/job/Isrc_uJ__naoqi_driver__ubuntu_jammy__source) | [![ros2-iron-jammy](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/iron_jammy.yml/badge.svg?branch=main)](https://github.com/ros-naoqi/naoqi_driver2/actions/workflows/iron_jammy.yml)
