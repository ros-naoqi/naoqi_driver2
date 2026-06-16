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

#ifndef NAOQI_DRIVER_HARDWARE_DCM_SYSTEM_HPP
#define NAOQI_DRIVER_HARDWARE_DCM_SYSTEM_HPP

#include <string>
#include <vector>

#include <qi/anyobject.hpp>
#include <qi/session.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace naoqi
{
namespace hardware
{

/// ros2_control SystemInterface driving joints through NAOqi's DCM (Device
/// Communication Manager) over libqi, the low-level path on robots where the
/// DCM is still exposed: NAO 2.1 and Pepper 2.5.
///
/// Commands are written to ALMemory actuator keys via the DCM alias mechanism:
///   createAlias([name, [keys]]) once, then per cycle getTime(0) + setAlias.
/// Position: Device/SubDeviceList/<Joint>/Position/Actuator/Value
/// Stiffness: Device/SubDeviceList/<Joint>/Hardness/Actuator/Value
///
/// Command mode is chosen from the URDF: each joint declares exactly one
/// command interface, either "position" or "velocity", and all joints must
/// agree. Velocity requires the velocity_actuator_key hardware parameter (the
/// per-joint DCM speed key is not confirmed in public sources; see
/// ASSUMPTIONS.md). State is read from ALMemory like the ALMotion system.
class DcmSystem : public hardware_interface::SystemInterface
{
  public:
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
  enum class CommandMode
  {
    POSITION,
    VELOCITY
  };

  /// Registers a DCM alias grouping the given actuator keys. Throws on qi error.
  void createAlias(const std::string& name, const std::vector<std::string>& keys);
  /// Writes one timed value per actuator of the alias at the current DCM time.
  void setAlias(const std::string& name, const std::vector<float>& values);

  std::vector<std::string> joint_names_;

  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;
  std::vector<double> commands_;  // position or velocity, per command_mode_

  std::vector<std::string> memory_keys_;     // state, see jointStateKeys()
  std::vector<std::string> position_keys_;   // Position/Actuator/Value
  std::vector<std::string> stiffness_keys_;  // Hardness/Actuator/Value
  std::vector<std::string> velocity_keys_;   // from velocity_actuator_key param

  CommandMode command_mode_ = CommandMode::POSITION;
  double stiffness_ = 1.0;
  bool stiffness_written_ = false;

  // Alias names registered with the DCM.
  static constexpr const char* kPositionAlias = "naoqi_ros2_control_position";
  static constexpr const char* kStiffnessAlias = "naoqi_ros2_control_stiffness";
  static constexpr const char* kVelocityAlias = "naoqi_ros2_control_velocity";

  qi::SessionPtr session_;
  qi::AnyObject memory_;
  qi::AnyObject dcm_;
};

}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_DCM_SYSTEM_HPP
