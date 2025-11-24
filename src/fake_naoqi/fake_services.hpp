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

#ifndef FAKE_SERVICES_HPP
#define FAKE_SERVICES_HPP

#include <mutex>
#include <qi/anyobject.hpp>
#include <qi/type/dynamicobjectbuilder.hpp>
#include <string>
#include <vector>

namespace naoqi
{
namespace fake
{

// Fake ALTextToSpeech
class FakeALTextToSpeech
{
  public:
  void say(const std::string& text);
  void setLanguage(const std::string& language);
  std::string getLanguage();

  private:
  std::string language_ = "en-US";
  std::mutex mutex_;
};

// Fake ALVideoDevice
class FakeALVideoDevice
{
  public:
  std::string subscribeCamera(
      const std::string& name, int camera_index, int resolution, int color_space, int fps);
  void unsubscribe(const std::string& handle);
  qi::AnyValue getImageRemote(const std::string& handle);

  private:
  int next_handle_id_ = 0;
  std::mutex mutex_;
};

// Fake ALAudioDevice
class FakeALAudioDevice
{
  public:
  void
  setClientPreferences(const std::string& name, int sample_rate, int channels, int deinterleaved);
  void subscribe(const std::string& name);
  void unsubscribe(const std::string& name);
};

// Fake ALSonar
class FakeALSonar
{
  public:
  void subscribe(const std::string& name);
  void unsubscribe(const std::string& name);
};

// Fake ALBodyTemperature
class FakeALBodyTemperature
{
  public:
  void setEnableNotifications(bool enable);
};

// Fake ALRobotModel
class FakeALRobotModel
{
  public:
  explicit FakeALRobotModel(const std::string& robot_type);

  // Add methods as needed
  std::string getRobotType();
  bool hasLegs();
  std::string getConfig();
  int _getMicrophoneConfig();
  std::map<std::string, std::string> _getConfigMap();

  private:
  std::string robot_type_;
};

// Fake ALSystem
class FakeALSystem
{
  public:
  std::string robotName();
  std::string systemVersion();
};

// Fake ALDialog
class FakeALDialog
{
  public:
  void setLanguage(const std::string& language);
  std::string getLanguage();

  private:
  std::string language_ = "en-US";
  std::mutex mutex_;
};

// Fake ALSpeechRecognition
class FakeALSpeechRecognition
{
  public:
  void _enableFreeSpeechToText();
};

// Fake LogManager (from libqicore)
class FakeLogManager
{
  public:
  // Minimal implementation for logging
  void log(const std::string& message);
};

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_SERVICES_HPP
