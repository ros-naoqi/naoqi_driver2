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

#include "fake_dcm.hpp"

#include <chrono>
#include <iostream>

#include <qi/type/dynamicobjectbuilder.hpp>

namespace naoqi
{
namespace fake
{

FakeDCM::FakeDCM(qi::AnyObject memory) : memory_(memory) {}

int FakeDCM::getTime(int delay)
{
  using namespace std::chrono;
  const auto now = duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count();
  return static_cast<int>(now) + delay;
}

void FakeDCM::createAlias(const qi::AnyValue& alias)
{
  const auto items = alias.to<std::vector<qi::AnyValue>>();
  if (items.size() < 2)
  {
    std::cerr << "FakeDCM::createAlias: expected [name, [keys]]" << std::endl;
    return;
  }
  const auto name = items[0].to<std::string>();
  const auto keys = items[1].to<std::vector<std::string>>();

  std::lock_guard<std::mutex> lock(mutex_);
  aliases_[name] = keys;
}

void FakeDCM::setAlias(const qi::AnyValue& command)
{
  const auto items = command.to<std::vector<qi::AnyValue>>();
  if (items.size() < 6)
  {
    std::cerr << "FakeDCM::setAlias: expected a 6-element command" << std::endl;
    return;
  }
  const auto name = items[0].to<std::string>();
  // [5] is a list of per-actuator value lists; take the last timed value of each.
  const auto values = items[5].to<std::vector<std::vector<float>>>();

  std::vector<std::string> keys;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    const auto it = aliases_.find(name);
    if (it == aliases_.end())
    {
      std::cerr << "FakeDCM::setAlias: unknown alias '" << name << "'" << std::endl;
      return;
    }
    keys = it->second;
  }

  if (values.size() != keys.size())
  {
    std::cerr << "FakeDCM::setAlias: " << values.size() << " values for " << keys.size() << " keys"
              << std::endl;
    return;
  }

  for (size_t i = 0; i < keys.size(); ++i)
  {
    if (values[i].empty())
      continue;
    const float value = values[i].back();
    memory_.call<void>("insertData", keys[i], qi::AnyValue::from(value));

    // Close the loop: a commanded ".../Actuator/Value" becomes the matching
    // ".../Sensor/Value" the state read path consumes.
    const std::string& key = keys[i];
    const auto pos = key.find("/Actuator/");
    if (pos != std::string::npos)
    {
      std::string sensor_key = key;
      sensor_key.replace(pos, std::string("/Actuator/").size(), "/Sensor/");
      memory_.call<void>("insertData", sensor_key, qi::AnyValue::from(value));
    }
  }
}

}  // namespace fake
}  // namespace naoqi

QI_REGISTER_OBJECT(naoqi::fake::FakeDCM, getTime, createAlias, setAlias)
