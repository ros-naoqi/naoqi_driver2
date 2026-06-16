/*
 * Copyright 2025 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

// Drives the DCM ros2_control system through its lifecycle against the fake
// NAOqi (fake DCM + fake ALMemory). Verifies a position command flows through
// write() -> DCM.setAlias -> ALMemory sensor key -> read().

#include <cmath>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "naoqi_driver/hardware/dcm_system.hpp"

namespace
{
hardware_interface::HardwareInfo makeInfo(const std::vector<std::string>& joints)
{
  hardware_interface::HardwareInfo info;
  info.name = "NaoDcmSystem";
  info.type = "system";
  info.hardware_parameters["emulation_mode"] = "true";
  info.hardware_parameters["robot_type"] = "nao";

  for (const auto& name : joints)
  {
    hardware_interface::ComponentInfo joint;
    joint.name = name;

    hardware_interface::InterfaceInfo position_cmd;
    position_cmd.name = "position";
    joint.command_interfaces.push_back(position_cmd);

    for (const std::string& state : {"position", "velocity", "effort"})
    {
      hardware_interface::InterfaceInfo s;
      s.name = state;
      joint.state_interfaces.push_back(s);
    }
    info.joints.push_back(joint);
  }
  return info;
}
}  // namespace

TEST(naoqi_driver_dcm, dcm_position_round_trip)
{
  const std::vector<std::string> joints = {"HeadYaw", "HeadPitch"};
  naoqi::hardware::DcmSystem system;

  ASSERT_EQ(system.on_init(makeInfo(joints)), hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = system.export_state_interfaces();
  auto command_interfaces = system.export_command_interfaces();
  ASSERT_EQ(state_interfaces.size(), 3u * joints.size());
  ASSERT_EQ(command_interfaces.size(), joints.size());

  ASSERT_EQ(system.on_activate(state), hardware_interface::CallbackReturn::SUCCESS);

  bool commanded = false;
  for (auto& cmd : command_interfaces)
  {
    if (cmd.get_name() == "HeadYaw/position")
    {
      (void)cmd.set_value(0.5);
      commanded = true;
    }
  }
  ASSERT_TRUE(commanded);

  const rclcpp::Time time(0, 0, RCL_ROS_TIME);
  const rclcpp::Duration period(0, 20000000);
  ASSERT_EQ(system.write(time, period), hardware_interface::return_type::OK);

  double head_yaw = 0.0;
  ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
  for (auto& s : state_interfaces)
  {
    if (s.get_name() == "HeadYaw/position")
      head_yaw = s.get_value();
  }
  EXPECT_NEAR(head_yaw, 0.5, 1e-3);

  EXPECT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
