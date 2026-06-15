# Copyright 2025 Aldebaran
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Bring up ros2_control for a NAOqi robot.

Loads the controller_manager with a NAOqi hardware backend (ALMotion by
default), then spawns the joint_state_broadcaster and a joint trajectory
controller. Works against a real robot or, with emulation_mode:=true, the
in-process fake NAOqi (no robot needed).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("naoqi_driver")

    args = [
        DeclareLaunchArgument("emulation_mode", default_value="false",
                              description="Run against the in-process fake NAOqi (no robot)."),
        DeclareLaunchArgument("plugin", default_value="naoqi_driver/AlMotionSystem",
                              description="ros2_control hardware plugin to load."),
        DeclareLaunchArgument("nao_ip", default_value="127.0.0.1"),
        DeclareLaunchArgument("nao_port", default_value="9559"),
        DeclareLaunchArgument("user", default_value="nao"),
        DeclareLaunchArgument("password", default_value="no_password"),
        DeclareLaunchArgument("max_velocity", default_value="0.1",
                              description="Fraction of each joint's max speed (ALMotion)."),
    ]

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([
                FindPackageShare("naoqi_driver"), "share", "ros2_control", "nao.urdf.xacro"]),
            " plugin:=", LaunchConfiguration("plugin"),
            " emulation_mode:=", LaunchConfiguration("emulation_mode"),
            " nao_ip:=", LaunchConfiguration("nao_ip"),
            " nao_port:=", LaunchConfiguration("nao_port"),
            " user:=", LaunchConfiguration("user"),
            " password:=", LaunchConfiguration("password"),
            " max_velocity:=", LaunchConfiguration("max_velocity"),
        ])
    }

    controllers = os.path.join(pkg_share, "config", "nao_controllers.yaml")

    # robot_state_publisher publishes /robot_description, which controller_manager
    # consumes on Iron and later. On Humble, controller_manager reads the
    # robot_description parameter instead, so it is also passed there.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controllers],
    )

    spawn_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )

    spawn_trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["nao_joint_trajectory_controller", "-c", "/controller_manager"],
    )

    return LaunchDescription(args + [
        robot_state_publisher,
        control_node,
        spawn_broadcaster,
        spawn_trajectory_controller,
    ])
