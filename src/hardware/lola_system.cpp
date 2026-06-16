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

#include "naoqi_driver/hardware/lola_system.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>
#include <map>
#include <unordered_map>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "naoqi_driver/hardware/lola_protocol.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/logging.hpp"

namespace naoqi
{
namespace hardware
{

namespace
{
rclcpp::Logger logger()
{
  return rclcpp::get_logger("LolaSystem");
}

std::string getParam(const std::unordered_map<std::string, std::string>& params,
                     const std::string& key,
                     const std::string& fallback)
{
  const auto it = params.find(key);
  return it != params.end() ? it->second : fallback;
}
}  // namespace

LolaSystem::~LolaSystem()
{
  stopIo();
}

hardware_interface::CallbackReturn LolaSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  socket_path_ = getParam(info_.hardware_parameters, "lola_socket", "/tmp/robocup");
  stiffness_ = std::stod(getParam(info_.hardware_parameters, "stiffness", "1.0"));

  // Map each URDF joint to an index in LoLA's 25-joint order. RHipYawPitch is
  // coupled to LHipYawPitch and shares its slot.
  const auto& order = lola::jointOrder();
  std::unordered_map<std::string, int> index_of;
  for (size_t i = 0; i < order.size(); ++i)
    index_of[order[i]] = static_cast<int>(i);

  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' must expose exactly one 'position' command interface.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    std::string name = joint.name == "RHipYawPitch" ? "LHipYawPitch" : joint.name;
    const auto it = index_of.find(name);
    if (it == index_of.end())
    {
      RCLCPP_ERROR(logger(), "Joint '%s' is not a LoLA joint.", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
    lola_index_.push_back(it->second);
  }

  const size_t n = joint_names_.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  positions_.assign(n, nan);
  velocities_.assign(n, 0.0);
  efforts_.assign(n, 0.0);
  position_commands_.assign(n, nan);

  const size_t lola_n = order.size();
  sensor_position_.assign(lola_n, 0.0f);
  command_position_.assign(lola_n, 0.0f);
  command_stiffness_.assign(lola_n, 0.0f);  // limp until activated

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LolaSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
  if (fd_ < 0)
  {
    RCLCPP_ERROR(logger(), "Could not create LoLA socket: %s", std::strerror(errno));
    return hardware_interface::CallbackReturn::ERROR;
  }

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  std::strncpy(addr.sun_path, socket_path_.c_str(), sizeof(addr.sun_path) - 1);
  if (::connect(fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
  {
    RCLCPP_ERROR(logger(),
                 "Could not connect to LoLA socket '%s': %s",
                 socket_path_.c_str(),
                 std::strerror(errno));
    ::close(fd_);
    fd_ = -1;
    return hardware_interface::CallbackReturn::ERROR;
  }

  running_ = true;
  io_ok_ = true;
  io_thread_ = std::thread(&LolaSystem::ioLoop, this);

  RCLCPP_INFO(logger(),
              "Connected to LoLA at '%s' (%zu joints).",
              socket_path_.c_str(),
              joint_names_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LolaSystem::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  stopIo();
  return hardware_interface::CallbackReturn::SUCCESS;
}

void LolaSystem::stopIo()
{
  running_ = false;
  if (io_thread_.joinable())
    io_thread_.join();
  if (fd_ >= 0)
  {
    ::close(fd_);
    fd_ = -1;
  }
}

void LolaSystem::ioLoop()
{
  std::vector<uint8_t> buffer(4096);
  while (running_)
  {
    const ssize_t received = ::recv(fd_, buffer.data(), buffer.size(), 0);
    if (received <= 0)
    {
      io_ok_ = false;
      break;
    }

    std::map<std::string, std::vector<float>> sensors;
    if (lola::decodeSensors(buffer.data(), static_cast<size_t>(received), sensors))
    {
      const auto it = sensors.find("Position");
      if (it != sensors.end() && it->second.size() == sensor_position_.size())
      {
        std::lock_guard<std::mutex> lock(mutex_);
        sensor_position_ = it->second;
        sensor_received_ = true;
      }
    }

    std::vector<float> position, stiffness;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      position = command_position_;
      stiffness = command_stiffness_;
    }
    const std::vector<uint8_t> frame = lola::encodeActuators(position, stiffness);
    if (::send(fd_, frame.data(), frame.size(), 0) < 0)
    {
      io_ok_ = false;
      break;
    }
  }
}

std::vector<hardware_interface::StateInterface> LolaSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_POSITION, &positions_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
    interfaces.emplace_back(joint_names_[i], hardware_interface::HW_IF_EFFORT, &efforts_[i]);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> LolaSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
    interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  return interfaces;
}

hardware_interface::CallbackReturn
LolaSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Seed commands from the latest sensed position so activation does not jump
  // the robot, then raise stiffness so commands take effect.
  std::lock_guard<std::mutex> lock(mutex_);
  command_position_ = sensor_position_;
  for (size_t i = 0; i < joint_names_.size(); ++i)
    position_commands_[i] = sensor_position_[lola_index_[i]];
  std::fill(command_stiffness_.begin(), command_stiffness_.end(), static_cast<float>(stiffness_));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
LolaSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::fill(command_stiffness_.begin(), command_stiffness_.end(), 0.0f);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type LolaSystem::read(const rclcpp::Time& /*time*/,
                                                 const rclcpp::Duration& /*period*/)
{
  if (!io_ok_)
  {
    RCLCPP_ERROR(logger(), "LoLA connection lost.");
    return hardware_interface::return_type::ERROR;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < joint_names_.size(); ++i)
    positions_[i] = sensor_position_[lola_index_[i]];
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type LolaSystem::write(const rclcpp::Time& /*time*/,
                                                  const rclcpp::Duration& /*period*/)
{
  if (!io_ok_)
    return hardware_interface::return_type::ERROR;
  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (!std::isnan(position_commands_[i]))
      command_position_[lola_index_[i]] = static_cast<float>(position_commands_[i]);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardware
}  // namespace naoqi

PLUGINLIB_EXPORT_CLASS(naoqi::hardware::LolaSystem, hardware_interface::SystemInterface)
