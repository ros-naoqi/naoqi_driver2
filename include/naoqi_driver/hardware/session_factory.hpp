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

#ifndef NAOQI_DRIVER_HARDWARE_SESSION_FACTORY_HPP
#define NAOQI_DRIVER_HARDWARE_SESSION_FACTORY_HPP

#include <string>

#include <qi/session.hpp>

namespace naoqi
{
namespace hardware
{

/// Sentinel value meaning "no password set" (kept identical to the node CLI).
inline const char* kNoPassword = "no_password";

/// Connection parameters for a NAOqi session, parsed from ros2_control
/// <hardware><param> entries. A hardware interface runs inside the
/// controller_manager process, not the naoqi_driver node, so it opens its own
/// session.
struct ConnectionOptions
{
  /// When true, no robot is contacted: an in-process fake NAOqi is created
  /// (see naoqi::fake::createFakeNaoqiSession). Used for tests and CI.
  bool emulation = false;
  /// Robot model for emulation ("nao", "pepper", "romeo"). Ignored for real
  /// connections (the model is read from the robot).
  std::string robot_type = "nao";

  std::string ip = "127.0.0.1";
  int port = 9559;
  std::string user = "nao";
  std::string password = kNoPassword;
};

/// Creates a connected qi::Session from the given options, or throws
/// std::runtime_error on failure. For real connections with a password set and
/// libqi >= 2.9, TLS is enabled and the default port is switched to 9503,
/// mirroring external_registration.cpp.
///
/// Real connections require a process-wide qi runtime. This factory lazily
/// instantiates a single qi::Application the first time it connects to a real
/// robot; the emulation path does not need it.
qi::SessionPtr makeSession(const ConnectionOptions& options);

}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_SESSION_FACTORY_HPP
