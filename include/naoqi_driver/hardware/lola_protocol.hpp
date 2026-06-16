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

#ifndef NAOQI_DRIVER_HARDWARE_LOLA_PROTOCOL_HPP
#define NAOQI_DRIVER_HARDWARE_LOLA_PROTOCOL_HPP

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace naoqi
{
namespace hardware
{
namespace lola
{

/// Canonical LoLA joint order (25 joints). NAO's left/right HipYawPitch are
/// mechanically coupled, so LoLA exposes a single LHipYawPitch. Confirmed
/// against ros-sports/nao_lola, B-Human and HTWK clients (see ASSUMPTIONS.md).
const std::vector<std::string>& jointOrder();

/// Encodes a LoLA actuator frame (MessagePack map) carrying Position and
/// Stiffness, each a 25-float array in jointOrder(). Other actuator groups
/// (LEDs) are omitted.
std::vector<uint8_t> encodeActuators(const std::vector<float>& position,
                                     const std::vector<float>& stiffness);

/// Decodes a LoLA sensor frame (MessagePack map of string -> array) into a map
/// from field name (Position, Stiffness, Temperature, ...) to its float values.
/// Non-numeric fields (e.g. RobotConfig strings) are decoded to empty vectors.
/// Returns false on a malformed buffer.
bool decodeSensors(const uint8_t* data,
                   size_t size,
                   std::map<std::string, std::vector<float>>& out);

}  // namespace lola
}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_LOLA_PROTOCOL_HPP
