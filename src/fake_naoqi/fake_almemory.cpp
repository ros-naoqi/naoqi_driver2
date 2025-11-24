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

#include "fake_almemory.hpp"
#include <iostream>

namespace naoqi
{
namespace fake
{

FakeALMemory::FakeALMemory(const std::string& robot_type) : robot_type_(robot_type)
{
  initializeMemoryKeys();
}

std::string FakeALMemory::getNaoqiVersion() const
{
  if (robot_type_ == "nao")
  {
    return "2.8.6.23";
  }
  else if (robot_type_ == "pepper")
  {
    return "2.9.5.3";
  }
  else if (robot_type_ == "romeo")
  {
    return "2.1.4.13";
  }
  return "2.8.6.23";  // default to NAO version
}

void FakeALMemory::initializeMemoryKeys()
{
  std::lock_guard<std::mutex> lock(memory_mutex_);

  // Robot configuration keys
  if (robot_type_ == "nao")
  {
    memory_["RobotConfig/Body/Type"] = qi::AnyValue::from(std::string("nao"));
    memory_["RobotConfig/Body/BaseVersion"] = qi::AnyValue::from(std::string("6.0"));
  }
  else if (robot_type_ == "pepper")
  {
    memory_["RobotConfig/Body/Type"] = qi::AnyValue::from(std::string("pepper"));
    memory_["RobotConfig/Body/BaseVersion"] = qi::AnyValue::from(std::string("1.8a"));
  }
  else if (robot_type_ == "romeo")
  {
    memory_["RobotConfig/Body/Type"] = qi::AnyValue::from(std::string("romeo"));
    memory_["RobotConfig/Body/BaseVersion"] = qi::AnyValue::from(std::string("1.0"));
  }

  memory_["RobotConfig/Head/BaseBoard/Version"] = qi::AnyValue::from(std::string("6.0"));
  memory_["Device/DeviceList"] = qi::AnyValue::from(std::vector<std::string>());

  // Initialize some common sensor keys with default values
  // These will be updated by converters/subscribers as needed

  // IMU keys
  memory_["Device/SubDeviceList/InertialSensor/AccelerometerX/Sensor/Value"] =
      qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/AccelerometerY/Sensor/Value"] =
      qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/AccelerometerZ/Sensor/Value"] =
      qi::AnyValue::from(9.81f);
  memory_["Device/SubDeviceList/InertialSensor/GyroscopeX/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/GyroscopeY/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/GyroscopeZ/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/AngleX/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/InertialSensor/AngleY/Sensor/Value"] = qi::AnyValue::from(0.0f);

  // Sonar keys
  memory_["Device/SubDeviceList/US/Left/Sensor/Value"] = qi::AnyValue::from(1.0f);
  memory_["Device/SubDeviceList/US/Right/Sensor/Value"] = qi::AnyValue::from(1.0f);

  // Battery
  memory_["Device/SubDeviceList/Battery/Charge/Sensor/Value"] = qi::AnyValue::from(0.8f);
  memory_["Device/SubDeviceList/Battery/Current/Sensor/Value"] = qi::AnyValue::from(0.5f);

  // Temperature keys (simplified)
  memory_["Device/SubDeviceList/Head/Temperature/Sensor/Value"] = qi::AnyValue::from(40.0f);

  // Touch sensors (simplified - will add per robot type if needed)
  memory_["Device/SubDeviceList/Head/Touch/Front/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/Head/Touch/Middle/Sensor/Value"] = qi::AnyValue::from(0.0f);
  memory_["Device/SubDeviceList/Head/Touch/Rear/Sensor/Value"] = qi::AnyValue::from(0.0f);

  std::cout << "FakeALMemory: Initialized with robot type '" << robot_type_ << "' (NAOqi "
            << getNaoqiVersion() << ")" << std::endl;
}

qi::AnyValue FakeALMemory::getData(const std::string& key)
{
  std::lock_guard<std::mutex> lock(memory_mutex_);

  auto it = memory_.find(key);
  if (it != memory_.end())
  {
    return it->second;
  }

  // For joint current/torque keys that may not be initialized, return sensible defaults
  if (key.find("ElectricCurrent/Sensor/Value") != std::string::npos)
  {
    memory_[key] = qi::AnyValue::from(0.0);
    return memory_[key];
  }

  // Default return for unknown keys - create it with default value
  std::cerr << "FakeALMemory::getData: Unknown key '" << key << "', creating with default 0.0"
            << std::endl;
  memory_[key] = qi::AnyValue::from(0.0);
  return memory_[key];
}

qi::AnyValue FakeALMemory::getListData(const std::vector<std::string>& keys)
{
  std::vector<qi::AnyValue> values;

  for (const auto& key : keys)
  {
    values.push_back(getData(key));
  }

  return qi::AnyValue::from(values);
}

void FakeALMemory::insertData(const std::string& key, const qi::AnyValue& value)
{
  std::lock_guard<std::mutex> lock(memory_mutex_);
  memory_[key] = value;
}

void FakeALMemory::setData(const std::string& key, const qi::AnyValue& value)
{
  qi::AnyObject subscriber_to_trigger;

  {
    std::lock_guard<std::mutex> lock(memory_mutex_);
    memory_[key] = value;

    // Get the subscriber object if it exists
    auto it = subscriber_objects_.find(key);
    if (it != subscriber_objects_.end())
    {
      subscriber_to_trigger = it->second;
    }
  }

  // Emit signal if anyone is subscribed to this key (outside the lock)
  if (subscriber_to_trigger)
  {
    try
    {
      subscriber_to_trigger.call<void>("signal", value);
    }
    catch (...)
    {
      // Ignore errors when triggering signals
    }
  }
}

void FakeALMemory::subscribeToEvent(const std::string& event_name,
                                    const std::string& object_name,
                                    const std::string& callback_name)
{
  // In fake implementation, we just log the subscription
  // Real implementation would set up callbacks
  std::cout << "FakeALMemory: Subscribed to event '" << event_name << "' for object '"
            << object_name << "'" << std::endl;
}

void FakeALMemory::unsubscribeToEvent(const std::string& event_name, const std::string& object_name)
{
  std::cout << "FakeALMemory: Unsubscribed from event '" << event_name << "' for object '"
            << object_name << "'" << std::endl;
}

qi::AnyObject FakeALMemory::subscriber(const std::string& key)
{
  std::cout << "FakeALMemory: Created subscriber for key '" << key << "'" << std::endl;

  // Create a dynamic object with a "signal" member
  qi::DynamicObjectBuilder builder;
  builder.advertiseSignal<qi::AnyValue>("signal");

  qi::AnyObject obj = builder.object();

  // Store the subscriber object so we can trigger its signal when the key is modified
  std::lock_guard<std::mutex> lock(memory_mutex_);
  subscriber_objects_[key] = obj;

  return obj;
}

}  // namespace fake
}  // namespace naoqi

// Register the FakeALMemory class with libqi
QI_REGISTER_OBJECT(naoqi::fake::FakeALMemory, getData, getListData, setData, subscriber)
