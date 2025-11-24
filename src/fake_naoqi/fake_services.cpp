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

#include "fake_services.hpp"
#include <cstring>
#include <iostream>
#include <sstream>

namespace naoqi
{
namespace fake
{

// FakeALTextToSpeech implementation
void FakeALTextToSpeech::say(const std::string& text)
{
  std::cout << "FakeALTextToSpeech: '" << text << "'" << std::endl;
}

void FakeALTextToSpeech::setLanguage(const std::string& language)
{
  std::lock_guard<std::mutex> lock(mutex_);
  language_ = language;
  std::cout << "FakeALTextToSpeech: Language set to '" << language << "'" << std::endl;
}

std::string FakeALTextToSpeech::getLanguage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return language_;
}

// FakeALVideoDevice implementation
std::string FakeALVideoDevice::subscribeCamera(
    const std::string& name, int camera_index, int resolution, int color_space, int fps)
{
  std::lock_guard<std::mutex> lock(mutex_);
  std::ostringstream oss;
  oss << "fake_camera_handle_" << next_handle_id_++;
  std::cout << "FakeALVideoDevice: Subscribed camera '" << name << "' with handle " << oss.str()
            << std::endl;
  return oss.str();
}

void FakeALVideoDevice::unsubscribe(const std::string& handle)
{
  std::cout << "FakeALVideoDevice: Unsubscribed handle '" << handle << "'" << std::endl;
}

qi::AnyValue FakeALVideoDevice::getImageRemote(const std::string& handle)
{
  // Return a fake image structure
  // Real structure: [width, height, layers, colorspace, timestamp_s, timestamp_us, image_data,
  // camera_id, left_angle, top_angle, right_angle, bottom_angle]
  std::vector<qi::AnyValue> image_data;

  // Create a fake 640x480 RGB image (all zeros for simplicity)
  int width = 640;
  int height = 480;
  int channels = 3;
  std::vector<unsigned char> pixel_data(width * height * channels, 0);

  image_data.push_back(qi::AnyValue::from(width));
  image_data.push_back(qi::AnyValue::from(height));
  image_data.push_back(qi::AnyValue::from(channels));
  image_data.push_back(qi::AnyValue::from(11));  // kRGBColorSpace = 11
  image_data.push_back(qi::AnyValue::from(0));   // timestamp_s
  image_data.push_back(qi::AnyValue::from(0));   // timestamp_us

  // Convert pixel data to qi::Buffer
  qi::Buffer buffer;
  buffer.write(pixel_data.data(), pixel_data.size());
  image_data.push_back(qi::AnyValue::from(buffer));

  image_data.push_back(qi::AnyValue::from(0));     // camera_id
  image_data.push_back(qi::AnyValue::from(0.0f));  // left_angle
  image_data.push_back(qi::AnyValue::from(0.0f));  // top_angle
  image_data.push_back(qi::AnyValue::from(0.0f));  // right_angle
  image_data.push_back(qi::AnyValue::from(0.0f));  // bottom_angle

  return qi::AnyValue::from(image_data);
}

// FakeALAudioDevice implementation
void FakeALAudioDevice::setClientPreferences(const std::string& name,
                                             int sample_rate,
                                             int channels,
                                             int deinterleaved)
{
  std::cout << "FakeALAudioDevice: Set client preferences for '" << name << "' - " << sample_rate
            << "Hz, " << channels << " channels" << std::endl;
}

void FakeALAudioDevice::subscribe(const std::string& name)
{
  std::cout << "FakeALAudioDevice: Subscribed '" << name << "'" << std::endl;
}

void FakeALAudioDevice::unsubscribe(const std::string& name)
{
  std::cout << "FakeALAudioDevice: Unsubscribed '" << name << "'" << std::endl;
}

// FakeALSonar implementation
void FakeALSonar::subscribe(const std::string& name)
{
  std::cout << "FakeALSonar: Subscribed '" << name << "'" << std::endl;
}

void FakeALSonar::unsubscribe(const std::string& name)
{
  std::cout << "FakeALSonar: Unsubscribed '" << name << "'" << std::endl;
}

// FakeALBodyTemperature implementation
void FakeALBodyTemperature::setEnableNotifications(bool enable)
{
  std::cout << "FakeALBodyTemperature: Notifications " << (enable ? "enabled" : "disabled")
            << std::endl;
}

// FakeALRobotModel implementation
FakeALRobotModel::FakeALRobotModel(const std::string& robot_type) : robot_type_(robot_type) {}

std::string FakeALRobotModel::getRobotType()
{
  return robot_type_;
}

bool FakeALRobotModel::hasLegs()
{
  // Both NAO and Pepper have legs (at least NAO does)
  return true;
}

std::string FakeALRobotModel::getConfig()
{
  // Return a minimal configuration string
  // Format is XML-like with head version information
  return "[Head]\nVersion=6\n";
}

int FakeALRobotModel::_getMicrophoneConfig()
{
  // Return a standard microphone configuration
  // NAO V5/V6: 0 = default config
  return 0;
}

std::map<std::string, std::string> FakeALRobotModel::_getConfigMap()
{
  // Return a map with configuration information
  std::map<std::string, std::string> config;
  config["MicrophoneConfig"] = "0";
  config["RobotType"] = robot_type_;
  return config;
}

// FakeALSystem implementation
std::string FakeALSystem::robotName()
{
  return "FakeRobot";
}

std::string FakeALSystem::systemVersion()
{
  // Return appropriate version for each robot type
  // This is just the system version string, actual version logic is in ALMemory
  return "2.8.6.23";  // Will be overridden by proper detection
}

// FakeALDialog implementation
void FakeALDialog::setLanguage(const std::string& language)
{
  std::lock_guard<std::mutex> lock(mutex_);
  language_ = language;
  std::cout << "FakeALDialog: Language set to '" << language << "'" << std::endl;
}

std::string FakeALDialog::getLanguage()
{
  std::lock_guard<std::mutex> lock(mutex_);
  return language_;
}

// FakeALSpeechRecognition implementation
void FakeALSpeechRecognition::_enableFreeSpeechToText()
{
  std::cout << "FakeALSpeechRecognition: Free speech to text enabled" << std::endl;
}

// FakeLogManager implementation
void FakeLogManager::log(const std::string& message)
{
  std::cout << "FakeLogManager: " << message << std::endl;
}

}  // namespace fake
}  // namespace naoqi

// Register all fake service classes with libqi
QI_REGISTER_OBJECT(naoqi::fake::FakeALTextToSpeech, say, setLanguage, getLanguage)
QI_REGISTER_OBJECT(naoqi::fake::FakeALVideoDevice, subscribeCamera, unsubscribe, getImageRemote)
QI_REGISTER_OBJECT(naoqi::fake::FakeALAudioDevice, setClientPreferences, subscribe, unsubscribe)
QI_REGISTER_OBJECT(naoqi::fake::FakeALSonar, subscribe, unsubscribe)
QI_REGISTER_OBJECT(naoqi::fake::FakeALBodyTemperature, setEnableNotifications)
QI_REGISTER_OBJECT(naoqi::fake::FakeALRobotModel,
                   getRobotType,
                   hasLegs,
                   getConfig,
                   _getMicrophoneConfig,
                   _getConfigMap)
QI_REGISTER_OBJECT(naoqi::fake::FakeALSystem, robotName, systemVersion)
QI_REGISTER_OBJECT(naoqi::fake::FakeALDialog, setLanguage, getLanguage)
QI_REGISTER_OBJECT(naoqi::fake::FakeALSpeechRecognition, _enableFreeSpeechToText)
QI_REGISTER_OBJECT(naoqi::fake::FakeLogManager, log)
