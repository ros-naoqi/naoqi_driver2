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

#ifndef FAKE_DCM_HPP
#define FAKE_DCM_HPP

#include <map>
#include <mutex>
#include <string>
#include <vector>

#include <qi/anyobject.hpp>
#include <qi/anyvalue.hpp>

namespace naoqi
{
namespace fake
{

/// Fake NAOqi DCM service, just enough to exercise the DCM ros2_control system
/// in emulation. Implements the createAlias/getTime/setAlias flow used to write
/// actuator values, and closes the control loop by mirroring each
/// ".../Actuator/Value" it receives to the matching ".../Sensor/Value" in
/// ALMemory, so a commanded position is read back as the sensed position.
class FakeDCM
{
  public:
  explicit FakeDCM(qi::AnyObject memory);

  /// Current DCM time in ms, plus an optional future delay.
  int getTime(int delay);

  /// alias = [name, [memory_key, ...]].
  void createAlias(const qi::AnyValue& alias);

  /// command = [name, updateType, "time-separate", 0, [times], [[v0],[v1],...]].
  /// Only the alias name and the per-actuator values are used here.
  void setAlias(const qi::AnyValue& command);

  private:
  qi::AnyObject memory_;
  std::map<std::string, std::vector<std::string>> aliases_;
  std::mutex mutex_;
};

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_DCM_HPP
