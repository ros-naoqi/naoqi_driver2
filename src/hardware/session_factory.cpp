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

#include "naoqi_driver/hardware/session_factory.hpp"

#include <mutex>
#include <stdexcept>

#include <qi/application.hpp>

#include "fake_naoqi/fake_naoqi.hpp"

#if LIBQI_VERSION >= 29
#include "driver_authenticator.hpp"
#endif

namespace naoqi
{
namespace hardware
{

namespace
{
/// libqi networking relies on a process-wide runtime that qi::Application sets
/// up (event loops, logging). The naoqi_driver node owns a qi::ApplicationSession
/// in main(); a hardware interface has no main(), so we create one qi::Application
/// for the whole process the first time we connect to a real robot.
///
/// ASSUMPTION (untested against a real robot): creating qi::Application from
/// within a pluginlib-loaded library, after the process already started, is
/// sufficient to use qi::Session networking. Verify on hardware; if it is not,
/// the controller_manager executable must create qi::Application in its main().
void ensureQiRuntime()
{
  static std::once_flag flag;
  std::call_once(flag, [] {
    // qi::Application takes argc/argv by non-const reference, so both must be
    // lvalues that outlive it (hence static).
    static int argc = 1;
    static char arg0[] = "naoqi_hardware";
    static char* argv_storage[] = {arg0, nullptr};
    static char** argv = argv_storage;
    static qi::Application app(argc, argv);
  });
}
}  // namespace

qi::SessionPtr makeSession(const ConnectionOptions& options)
{
  if (options.emulation)
  {
    return naoqi::fake::createFakeNaoqiSession(options.robot_type);
  }

  ensureQiRuntime();

  std::string protocol = "tcp://";
  int port = options.port;
  const bool has_password = options.password != kNoPassword;

#if LIBQI_VERSION >= 29
  if (has_password)
  {
    protocol = "tcps://";
    port = (port == 9559) ? 9503 : port;
  }
#endif

  qi::SessionPtr session = qi::makeSession();

#if LIBQI_VERSION >= 29
  if (has_password)
  {
    auto* factory = new naoqi::DriverAuthenticatorFactory;
    factory->user = options.user;
    factory->pass = options.password;
    session->setClientAuthenticatorFactory(qi::ClientAuthenticatorFactoryPtr(factory));
  }
#else
  if (has_password)
  {
    throw std::runtime_error("A password was set but libqi < 2.9 does not support authentication.");
  }
#endif

  const qi::Url url(protocol + options.ip + ":" + std::to_string(port));
  qi::Future<void> connection = session->connect(url);
  if (connection.hasError())
  {
    throw std::runtime_error("Failed to connect to NAOqi at " + url.str() + ": " +
                             connection.error());
  }

  return session;
}

}  // namespace hardware
}  // namespace naoqi
