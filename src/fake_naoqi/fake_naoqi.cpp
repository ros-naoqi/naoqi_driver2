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

#include "fake_naoqi.hpp"
#include "fake_almemory.hpp"
#include "fake_almotion.hpp"
#include "fake_services.hpp"

#include <iostream>
#include <memory>
#include <qi/applicationsession.hpp>

namespace naoqi
{
namespace fake
{

qi::SessionPtr createFakeNaoqiSession(const std::string& robot_type)
{
  std::cout << "Creating fake NAOqi session for robot type: " << robot_type << std::endl;

  // Create a new session
  qi::SessionPtr session = qi::makeSession();

  // Make the session listen on a local endpoint (required for service registration)
  session->listenStandalone("tcp://127.0.0.1:0");

  std::cout << "Fake NAOqi session listening on: " << session->endpoints()[0].str() << std::endl;

  // Create fake services as shared pointers
  // QI_REGISTER_OBJECT allows these to be registered as services
  auto motion = boost::make_shared<FakeALMotion>(robot_type);
  auto memory = boost::make_shared<FakeALMemory>(robot_type);
  auto tts = boost::make_shared<FakeALTextToSpeech>();
  auto video = boost::make_shared<FakeALVideoDevice>();
  auto audio = boost::make_shared<FakeALAudioDevice>();
  auto sonar = boost::make_shared<FakeALSonar>();
  auto body_temp = boost::make_shared<FakeALBodyTemperature>();
  auto robot_model = boost::make_shared<FakeALRobotModel>(robot_type);
  auto system = boost::make_shared<FakeALSystem>();
  auto dialog = boost::make_shared<FakeALDialog>();
  auto speech_rec = boost::make_shared<FakeALSpeechRecognition>();
  auto log_manager = boost::make_shared<FakeLogManager>();

  // Register services with the session
  try
  {
    session->registerService("ALMotion", motion);
    session->registerService("ALMemory", memory);
    session->registerService("ALTextToSpeech", tts);
    session->registerService("ALVideoDevice", video);
    session->registerService("ALAudioDevice", audio);
    session->registerService("ALSonar", sonar);
    session->registerService("ALBodyTemperature", body_temp);
    session->registerService("ALRobotModel", robot_model);
    session->registerService("ALSystem", system);
    session->registerService("ALDialog", dialog);
    session->registerService("ALSpeechRecognition", speech_rec);
    session->registerService("LogManager", log_manager);

    std::cout << "Fake NAOqi services registered successfully" << std::endl;
  }
  catch (const std::exception& e)
  {
    std::cerr << "Error registering fake NAOqi services: " << e.what() << std::endl;
  }

  return session;
}

}  // namespace fake
}  // namespace naoqi
