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

#ifndef FAKE_NAOQI_HPP
#define FAKE_NAOQI_HPP

#include <string>

#include <qi/session.hpp>

namespace naoqi
{
namespace fake
{

/**
 * @brief Creates and configures a local qi::Session with fake NAOqi services
 *
 * This function creates a local session without network connectivity and
 * registers fake implementations of all NAOqi services required by the driver.
 *
 * @param robot_type The type of robot to emulate ("nao", "pepper", or "romeo")
 * @return qi::SessionPtr A configured session with fake services registered
 */
qi::SessionPtr createFakeNaoqiSession(const std::string& robot_type = "nao");

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_NAOQI_HPP
