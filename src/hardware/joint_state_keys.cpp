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

#include "naoqi_driver/hardware/joint_state_keys.hpp"

#include <qi/anyvalue.hpp>

#include "rclcpp/logging.hpp"

namespace naoqi
{
namespace hardware
{

namespace
{
double toDouble(const qi::AnyValue& value)
{
  // Some joints (e.g. hands) have no velocity or torque key; ALMemory returns
  // a void there. Treat it as zero, like the joint_state converter.
  if (value.kind() == qi::TypeKind_Void)
  {
    return 0.0;
  }
  return value.toDouble();
}
}  // namespace

std::vector<std::string> jointStateKeys(const std::vector<std::string>& joints)
{
  std::vector<std::string> keys;
  keys.reserve(3 * joints.size());
  for (const auto& name : joints)
    keys.push_back("Device/SubDeviceList/" + name + "/Position/Sensor/Value");
  for (const auto& name : joints)
    keys.push_back("Motion/Velocity/Sensor/" + name);
  for (const auto& name : joints)
    keys.push_back("Motion/Torque/Sensor/" + name);
  return keys;
}

bool readJointStates(qi::AnyObject memory,
                     const std::vector<std::string>& keys,
                     std::vector<double>& positions,
                     std::vector<double>& velocities,
                     std::vector<double>& efforts)
{
  const size_t n = positions.size();
  std::vector<qi::AnyValue> data;
  try
  {
    data = memory.call<std::vector<qi::AnyValue>>("getListData", keys);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(rclcpp::get_logger("naoqi_hardware"), "ALMemory read failed: %s", e.what());
    return false;
  }

  if (data.size() != 3 * n)
  {
    RCLCPP_ERROR(rclcpp::get_logger("naoqi_hardware"),
                 "ALMemory returned %zu values, expected %zu.",
                 data.size(),
                 3 * n);
    return false;
  }

  for (size_t i = 0; i < n; ++i)
  {
    positions[i] = toDouble(data[i]);
    velocities[i] = toDouble(data[n + i]);
    efforts[i] = toDouble(data[2 * n + i]);
  }
  return true;
}

}  // namespace hardware
}  // namespace naoqi
