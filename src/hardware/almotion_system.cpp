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

#include "naoqi_driver/hardware/almotion_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <unordered_map>

#include <qi/anyvalue.hpp>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "naoqi_driver/hardware/joint_state_keys.hpp"
#include "naoqi_driver/hardware/session_factory.hpp"
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
  return rclcpp::get_logger("AlMotionSystem");
}

std::string getParam(const std::unordered_map<std::string, std::string>& params,
                     const std::string& key,
                     const std::string& fallback)
{
  const auto it = params.find(key);
  return it != params.end() ? it->second : fallback;
}
}  // namespace

hardware_interface::CallbackReturn
AlMotionSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  max_velocity_fraction_ = std::stod(getParam(info_.hardware_parameters, "max_velocity", "0.1"));
  if (max_velocity_fraction_ <= 0.0 || max_velocity_fraction_ > 1.0)
  {
    RCLCPP_ERROR(logger(), "max_velocity must be in (0, 1], got %f", max_velocity_fraction_);
    return hardware_interface::CallbackReturn::ERROR;
  }

  joint_names_.reserve(info_.joints.size());
  for (const auto& joint : info_.joints)
  {
    // ALMotion commands position only.
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' must expose exactly one command interface '%s'.",
                   joint.name.c_str(),
                   hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_names_.push_back(joint.name);
  }

  const size_t n = joint_names_.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  positions_.assign(n, nan);
  velocities_.assign(n, 0.0);
  efforts_.assign(n, 0.0);
  position_commands_.assign(n, nan);

  memory_keys_ = jointStateKeys(joint_names_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AlMotionSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  ConnectionOptions options;
  const auto& p = info_.hardware_parameters;
  options.emulation = getParam(p, "emulation_mode", "false") == "true";
  options.robot_type = getParam(p, "robot_type", "nao");
  options.ip = getParam(p, "nao_ip", "127.0.0.1");
  options.port = std::stoi(getParam(p, "nao_port", "9559"));
  options.user = getParam(p, "user", "nao");
  options.password = getParam(p, "password", kNoPassword);

  try
  {
    session_ = makeSession(options);
    motion_ = session_->service("ALMotion").value();
    memory_ = session_->service("ALMemory").value();
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "Failed to configure ALMotion system: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(),
              "Connected to ALMotion (%zu joints, %s mode).",
              joint_names_.size(),
              options.emulation ? "emulation" : "robot");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AlMotionSystem::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  motion_ = qi::AnyObject();
  memory_ = qi::AnyObject();
  if (session_)
  {
    session_->close();
    session_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> AlMotionSystem::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> AlMotionSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    interfaces.emplace_back(
        joint_names_[i], hardware_interface::HW_IF_POSITION, &position_commands_[i]);
  }
  return interfaces;
}

hardware_interface::CallbackReturn
AlMotionSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  // Seed the command with the current state so controllers do not jerk the
  // robot to an uninitialized target on the first write().
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_commands_ = positions_;

  // Stiffen the joints so commands take effect. Optional: the fake ALMotion
  // does not implement setStiffnesses, and some setups manage stiffness
  // elsewhere, so a failure here is only a warning.
  try
  {
    motion_.call<void>("setStiffnesses", std::string("Body"), 1.0f);
  }
  catch (const std::exception& e)
  {
    RCLCPP_WARN(logger(), "Could not set stiffness on activation: %s", e.what());
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
AlMotionSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type AlMotionSystem::read(const rclcpp::Time& /*time*/,
                                                     const rclcpp::Duration& /*period*/)
{
  if (!readJointStates(memory_, memory_keys_, positions_, velocities_, efforts_))
    return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type AlMotionSystem::write(const rclcpp::Time& /*time*/,
                                                      const rclcpp::Duration& /*period*/)
{
  std::vector<float> angles(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (std::isnan(position_commands_[i]))
    {
      // No controller has written a command yet; do not move.
      return hardware_interface::return_type::OK;
    }
    angles[i] = static_cast<float>(position_commands_[i]);
  }

  try
  {
    motion_.call<void>(
        "setAngles", joint_names_, angles, static_cast<float>(max_velocity_fraction_));
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "ALMotion setAngles failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardware
}  // namespace naoqi

PLUGINLIB_EXPORT_CLASS(naoqi::hardware::AlMotionSystem, hardware_interface::SystemInterface)
