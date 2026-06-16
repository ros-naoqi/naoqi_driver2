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

#ifndef FAKE_LOLA_SERVER_HPP
#define FAKE_LOLA_SERVER_HPP

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <map>
#include <string>
#include <thread>
#include <vector>

#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

#include "naoqi_driver/hardware/lola_protocol.hpp"

namespace naoqi
{
namespace fake
{

/// Minimal LoLA server for tests: an AF_UNIX socket that, once a client
/// connects, repeatedly sends a sensor frame and reads back an actuator frame,
/// mirroring the commanded Position into the sensed Position so a commanded
/// angle is read back. Uses the same MessagePack codec as the LoLA system.
class FakeLolaServer
{
  public:
  explicit FakeLolaServer(const std::string& path) : path_(path)
  {
    listen_fd_ = ::socket(AF_UNIX, SOCK_STREAM, 0);
    ::unlink(path_.c_str());
    sockaddr_un addr{};
    addr.sun_family = AF_UNIX;
    std::strncpy(addr.sun_path, path_.c_str(), sizeof(addr.sun_path) - 1);
    ::bind(listen_fd_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
    ::listen(listen_fd_, 1);

    mirror_position_.assign(hardware::lola::jointOrder().size(), 0.0f);
    running_ = true;
    thread_ = std::thread(&FakeLolaServer::serve, this);
  }

  ~FakeLolaServer()
  {
    running_ = false;
    if (client_fd_ >= 0)
      ::shutdown(client_fd_, SHUT_RDWR);
    if (listen_fd_ >= 0)
      ::shutdown(listen_fd_, SHUT_RDWR);
    if (thread_.joinable())
      thread_.join();
    if (client_fd_ >= 0)
      ::close(client_fd_);
    if (listen_fd_ >= 0)
      ::close(listen_fd_);
    ::unlink(path_.c_str());
  }

  private:
  void serve()
  {
    client_fd_ = ::accept(listen_fd_, nullptr, nullptr);
    if (client_fd_ < 0)
      return;

    const std::vector<float> stiffness(mirror_position_.size(), 0.0f);
    std::vector<uint8_t> buffer(4096);
    while (running_)
    {
      const std::vector<uint8_t> frame =
          hardware::lola::encodeActuators(mirror_position_, stiffness);
      if (::send(client_fd_, frame.data(), frame.size(), 0) < 0)
        break;

      const ssize_t received = ::recv(client_fd_, buffer.data(), buffer.size(), 0);
      if (received <= 0)
        break;
      std::map<std::string, std::vector<float>> actuators;
      if (hardware::lola::decodeSensors(buffer.data(), static_cast<size_t>(received), actuators))
      {
        const auto it = actuators.find("Position");
        if (it != actuators.end() && it->second.size() == mirror_position_.size())
          mirror_position_ = it->second;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
  }

  std::string path_;
  int listen_fd_ = -1;
  int client_fd_ = -1;
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::vector<float> mirror_position_;
};

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_LOLA_SERVER_HPP
