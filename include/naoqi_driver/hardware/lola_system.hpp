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

#ifndef NAOQI_DRIVER_HARDWARE_LOLA_SYSTEM_HPP
#define NAOQI_DRIVER_HARDWARE_LOLA_SYSTEM_HPP

#include <atomic>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace naoqi
{
namespace hardware
{

/// ros2_control SystemInterface driving NAO (and, per internal sources, Pepper
/// 2.9) joints through the LoLA protocol: a local AF_UNIX socket (/tmp/robocup)
/// speaking MessagePack at ~83 Hz. This bypasses NAOqi and must run on the
/// robot. Position command + stiffness only.
///
/// A background thread owns the socket and runs the LoLA exchange (recv a
/// sensor frame, send an actuator frame) at LoLA's pace, so the
/// controller_manager update rate is decoupled from the 12 ms LoLA cycle.
/// read()/write() just copy from/to thread-shared buffers.
///
/// LoLA exposes 25 joints (one LHipYawPitch). URDF joints are mapped onto that
/// order; RHipYawPitch aliases LHipYawPitch (mechanically coupled).
class LolaSystem : public hardware_interface::SystemInterface
{
  public:
  ~LolaSystem() override;

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  hardware_interface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  hardware_interface::return_type read(const rclcpp::Time& time,
                                       const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time,
                                        const rclcpp::Duration& period) override;

  private:
  void ioLoop();
  void stopIo();

  std::vector<std::string> joint_names_;
  std::vector<int> lola_index_;  // per URDF joint: index into LoLA's 25 arrays

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> position_commands_;

  std::string socket_path_ = "/tmp/robocup";
  double stiffness_ = 1.0;
  int fd_ = -1;

  std::thread io_thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> io_ok_{true};

  // Shared with ioLoop(). Sized to LoLA's joint count (25).
  std::mutex mutex_;
  std::vector<float> sensor_position_;    // latest sensed positions
  std::vector<float> command_position_;   // positions to send
  std::vector<float> command_stiffness_;  // stiffness to send
  bool sensor_received_ = false;
};

}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_LOLA_SYSTEM_HPP
