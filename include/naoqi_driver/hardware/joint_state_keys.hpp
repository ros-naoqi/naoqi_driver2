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

#ifndef NAOQI_DRIVER_HARDWARE_JOINT_STATE_KEYS_HPP
#define NAOQI_DRIVER_HARDWARE_JOINT_STATE_KEYS_HPP

#include <string>
#include <vector>

#include <qi/anyobject.hpp>

namespace naoqi
{
namespace hardware
{

/// ALMemory keys for joint state, laid out [positions..., velocities...,
/// torques...] over the given joints:
///   Device/SubDeviceList/<Joint>/Position/Sensor/Value
///   Motion/Velocity/Sensor/<Joint>
///   Motion/Torque/Sensor/<Joint>
/// These are populated by the running NAOqi on every version, so both the
/// ALMotion and DCM systems read state the same way (matching the
/// joint_state converter).
std::vector<std::string> jointStateKeys(const std::vector<std::string>& joints);

/// Reads the keys (from jointStateKeys) in one batched getListData and fills
/// the three output vectors, which must already be sized to the joint count.
/// Returns false (and logs) on a qi error or unexpected reply size.
bool readJointStates(qi::AnyObject memory,
                     const std::vector<std::string>& keys,
                     std::vector<double>& positions,
                     std::vector<double>& velocities,
                     std::vector<double>& efforts);

}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_JOINT_STATE_KEYS_HPP
