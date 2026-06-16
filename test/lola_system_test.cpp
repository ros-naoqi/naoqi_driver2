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

// Drives the LoLA ros2_control system against a fake LoLA Unix-socket server,
// exercising the MessagePack codec, the IO thread and the URDF<->LoLA joint
// mapping. Verifies a position command is read back through the socket.

#include <chrono>
#include <cmath>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "fake_lola_server.hpp"
#include "naoqi_driver/hardware/lola_protocol.hpp"
#include "naoqi_driver/hardware/lola_system.hpp"

namespace
{
hardware_interface::HardwareInfo makeInfo(const std::vector<std::string>& joints,
                                          const std::string& socket_path)
{
  hardware_interface::HardwareInfo info;
  info.name = "NaoLolaSystem";
  info.type = "system";
  info.hardware_parameters["lola_socket"] = socket_path;
  info.hardware_parameters["stiffness"] = "1.0";

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

TEST(naoqi_driver_lola, lola_position_round_trip)
{
  const std::string socket_path = "/tmp/naoqi_lola_test_" + std::to_string(::getpid()) + ".sock";
  naoqi::fake::FakeLolaServer server(socket_path);

  const std::vector<std::string> joints = {"HeadYaw", "HeadPitch"};
  naoqi::hardware::LolaSystem system;

  ASSERT_EQ(system.on_init(makeInfo(joints, socket_path)),
            hardware_interface::CallbackReturn::SUCCESS);

  const rclcpp_lifecycle::State state;
  ASSERT_EQ(system.on_configure(state), hardware_interface::CallbackReturn::SUCCESS);

  auto state_interfaces = system.export_state_interfaces();
  auto command_interfaces = system.export_command_interfaces();
  ASSERT_EQ(state_interfaces.size(), 3u * joints.size());
  ASSERT_EQ(command_interfaces.size(), joints.size());

  // Let the IO thread exchange a few frames so the first sensor frame arrives.
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
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

  double head_yaw = 0.0;
  for (int i = 0; i < 200; ++i)
  {
    ASSERT_EQ(system.write(time, period), hardware_interface::return_type::OK);
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    ASSERT_EQ(system.read(time, period), hardware_interface::return_type::OK);
    for (auto& s : state_interfaces)
    {
      if (s.get_name() == "HeadYaw/position")
        head_yaw = s.get_value();
    }
    if (std::abs(head_yaw - 0.5) < 1e-3)
      break;
  }
  EXPECT_NEAR(head_yaw, 0.5, 1e-2);

  EXPECT_EQ(system.on_deactivate(state), hardware_interface::CallbackReturn::SUCCESS);
  EXPECT_EQ(system.on_cleanup(state), hardware_interface::CallbackReturn::SUCCESS);
}

TEST(naoqi_driver_lola, codec_round_trip)
{
  const size_t n = naoqi::hardware::lola::jointOrder().size();
  std::vector<float> position(n), stiffness(n, 1.0f);
  for (size_t i = 0; i < n; ++i)
    position[i] = 0.01f * static_cast<float>(i);

  const auto frame = naoqi::hardware::lola::encodeActuators(position, stiffness);
  std::map<std::string, std::vector<float>> decoded;
  ASSERT_TRUE(naoqi::hardware::lola::decodeSensors(frame.data(), frame.size(), decoded));
  ASSERT_EQ(decoded["Position"].size(), n);
  for (size_t i = 0; i < n; ++i)
    EXPECT_NEAR(decoded["Position"][i], position[i], 1e-6);
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
