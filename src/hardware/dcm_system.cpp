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

#include "naoqi_driver/hardware/dcm_system.hpp"

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
  return rclcpp::get_logger("DcmSystem");
}

std::string getParam(const std::unordered_map<std::string, std::string>& params,
                     const std::string& key,
                     const std::string& fallback)
{
  const auto it = params.find(key);
  return it != params.end() ? it->second : fallback;
}

/// Substitutes the joint name into a key template, replacing the first "{}".
std::string fillTemplate(const std::string& tmpl, const std::string& joint)
{
  const auto pos = tmpl.find("{}");
  if (pos == std::string::npos)
    return tmpl;
  std::string out = tmpl;
  out.replace(pos, 2, joint);
  return out;
}
}  // namespace

hardware_interface::CallbackReturn DcmSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  stiffness_ = std::stod(getParam(info_.hardware_parameters, "stiffness", "1.0"));
  const std::string velocity_key_tmpl =
      getParam(info_.hardware_parameters, "velocity_actuator_key", "");

  // Determine the command mode from the URDF: every joint must declare exactly
  // one command interface, and they must all be the same ("position" or
  // "velocity"). Mixing modes would need command-mode switching, which the DCM
  // system does not implement yet.
  bool mode_set = false;
  for (const auto& joint : info_.joints)
  {
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' must declare exactly one command interface (position or velocity).",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    const std::string& cmd = joint.command_interfaces[0].name;
    CommandMode mode;
    if (cmd == hardware_interface::HW_IF_POSITION)
      mode = CommandMode::POSITION;
    else if (cmd == hardware_interface::HW_IF_VELOCITY)
      mode = CommandMode::VELOCITY;
    else
    {
      RCLCPP_ERROR(logger(),
                   "Joint '%s' has unsupported command interface '%s'.",
                   joint.name.c_str(),
                   cmd.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    if (mode_set && mode != command_mode_)
    {
      RCLCPP_ERROR(logger(),
                   "All joints must share the same command interface (position XOR "
                   "velocity); joint '%s' differs.",
                   joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    command_mode_ = mode;
    mode_set = true;
    joint_names_.push_back(joint.name);
  }

  if (command_mode_ == CommandMode::VELOCITY && velocity_key_tmpl.empty())
  {
    RCLCPP_ERROR(logger(),
                 "Velocity command requested but 'velocity_actuator_key' is not set. The "
                 "per-joint DCM speed key must be provided (see ASSUMPTIONS.md).");
    return hardware_interface::CallbackReturn::ERROR;
  }

  const size_t n = joint_names_.size();
  const double nan = std::numeric_limits<double>::quiet_NaN();
  positions_.assign(n, nan);
  velocities_.assign(n, 0.0);
  efforts_.assign(n, 0.0);
  commands_.assign(n, nan);

  memory_keys_ = jointStateKeys(joint_names_);
  for (const auto& name : joint_names_)
  {
    position_keys_.push_back("Device/SubDeviceList/" + name + "/Position/Actuator/Value");
    stiffness_keys_.push_back("Device/SubDeviceList/" + name + "/Hardness/Actuator/Value");
    if (command_mode_ == CommandMode::VELOCITY)
      velocity_keys_.push_back(fillTemplate(velocity_key_tmpl, name));
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DcmSystem::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
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
    memory_ = session_->service("ALMemory").value();
    dcm_ = session_->service("DCM").value();

    createAlias(kStiffnessAlias, stiffness_keys_);
    if (command_mode_ == CommandMode::POSITION)
      createAlias(kPositionAlias, position_keys_);
    else
      createAlias(kVelocityAlias, velocity_keys_);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "Failed to configure DCM system: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(logger(),
              "Connected to DCM (%zu joints, %s command, %s mode).",
              joint_names_.size(),
              command_mode_ == CommandMode::POSITION ? "position" : "velocity",
              options.emulation ? "emulation" : "robot");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DcmSystem::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  dcm_ = qi::AnyObject();
  memory_ = qi::AnyObject();
  if (session_)
  {
    session_->close();
    session_.reset();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DcmSystem::export_state_interfaces()
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

std::vector<hardware_interface::CommandInterface> DcmSystem::export_command_interfaces()
{
  const char* interface = command_mode_ == CommandMode::POSITION
                              ? hardware_interface::HW_IF_POSITION
                              : hardware_interface::HW_IF_VELOCITY;
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < joint_names_.size(); ++i)
    interfaces.emplace_back(joint_names_[i], interface, &commands_[i]);
  return interfaces;
}

hardware_interface::CallbackReturn
DcmSystem::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  if (read(rclcpp::Time(0), rclcpp::Duration(0, 0)) != hardware_interface::return_type::OK)
    return hardware_interface::CallbackReturn::ERROR;

  // Seed the command so the first write() holds position / commands zero velocity.
  if (command_mode_ == CommandMode::POSITION)
    commands_ = positions_;
  else
    commands_.assign(joint_names_.size(), 0.0);

  try
  {
    setAlias(kStiffnessAlias,
             std::vector<float>(joint_names_.size(), static_cast<float>(stiffness_)));
    stiffness_written_ = true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "Failed to set stiffness via DCM: %s", e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
DcmSystem::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DcmSystem::read(const rclcpp::Time& /*time*/,
                                                const rclcpp::Duration& /*period*/)
{
  if (!readJointStates(memory_, memory_keys_, positions_, velocities_, efforts_))
    return hardware_interface::return_type::ERROR;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DcmSystem::write(const rclcpp::Time& /*time*/,
                                                 const rclcpp::Duration& /*period*/)
{
  std::vector<float> values(joint_names_.size());
  for (size_t i = 0; i < joint_names_.size(); ++i)
  {
    if (std::isnan(commands_[i]))
      return hardware_interface::return_type::OK;  // not commanded yet
    values[i] = static_cast<float>(commands_[i]);
  }

  try
  {
    if (!stiffness_written_)
    {
      setAlias(kStiffnessAlias,
               std::vector<float>(joint_names_.size(), static_cast<float>(stiffness_)));
      stiffness_written_ = true;
    }
    setAlias(command_mode_ == CommandMode::POSITION ? kPositionAlias : kVelocityAlias, values);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(logger(), "DCM setAlias failed: %s", e.what());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

void DcmSystem::createAlias(const std::string& name, const std::vector<std::string>& keys)
{
  std::vector<qi::AnyValue> alias;
  alias.push_back(qi::AnyValue::from(name));
  alias.push_back(qi::AnyValue::from(keys));
  dcm_.call<void>("createAlias", qi::AnyValue::from(alias));
}

void DcmSystem::setAlias(const std::string& name, const std::vector<float>& values)
{
  const int t = dcm_.call<int>("getTime", 0);
  const std::vector<int> times{t};
  std::vector<std::vector<float>> timed_values;
  timed_values.reserve(values.size());
  for (const float v : values)
    timed_values.push_back({v});

  // [name, updateType, "time-separate", 0, [times], [[v0],[v1],...]]
  std::vector<qi::AnyValue> command;
  command.push_back(qi::AnyValue::from(name));
  command.push_back(qi::AnyValue::from(std::string("ClearAll")));
  command.push_back(qi::AnyValue::from(std::string("time-separate")));
  command.push_back(qi::AnyValue::from(0));
  command.push_back(qi::AnyValue::from(times));
  command.push_back(qi::AnyValue::from(timed_values));
  dcm_.call<void>("setAlias", qi::AnyValue::from(command));
}

}  // namespace hardware
}  // namespace naoqi

PLUGINLIB_EXPORT_CLASS(naoqi::hardware::DcmSystem, hardware_interface::SystemInterface)
