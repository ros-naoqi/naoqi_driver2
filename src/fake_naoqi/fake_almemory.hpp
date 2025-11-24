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

#ifndef FAKE_ALMEMORY_HPP
#define FAKE_ALMEMORY_HPP

#include <mutex>
#include <qi/anyobject.hpp>
#include <qi/signal.hpp>
#include <qi/type/dynamicobjectbuilder.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace naoqi
{
namespace fake
{

/**
 * @brief Fake implementation of NAOqi's ALMemory service
 *
 * This class emulates the ALMemory service for testing and simulation.
 * It stores memory keys and their values, and provides event subscription.
 */
class FakeALMemory
{
  public:
  explicit FakeALMemory(const std::string& robot_type);

  // Memory access
  qi::AnyValue getData(const std::string& key);
  qi::AnyValue getListData(const std::vector<std::string>& keys);
  void insertData(const std::string& key, const qi::AnyValue& value);
  void setData(const std::string& key, const qi::AnyValue& value);

  // Event subscription - returns a fake subscriber object
  qi::AnyObject subscriber(const std::string& key);

  // Event subscription (simplified - no actual callbacks)
  void subscribeToEvent(const std::string& event_name,
                        const std::string& object_name,
                        const std::string& callback_name);
  void unsubscribeToEvent(const std::string& event_name, const std::string& object_name);

  private:
  void initializeMemoryKeys();
  std::string getNaoqiVersion() const;

  std::string robot_type_;
  std::unordered_map<std::string, qi::AnyValue> memory_;
  std::unordered_map<std::string, qi::AnyObject> subscriber_objects_;
  std::mutex memory_mutex_;
};

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_ALMEMORY_HPP
