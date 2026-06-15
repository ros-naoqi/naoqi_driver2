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

#ifndef NAOQI_DRIVER_HARDWARE_ALMOTION_SYSTEM_HPP
#define NAOQI_DRIVER_HARDWARE_ALMOTION_SYSTEM_HPP

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

/// ros2_control SystemInterface driving joints through NAOqi's high-level
/// ALMotion service over libqi.
///
/// Works on every NAOqi version reachable over the network (2.1 through 2.9),
/// which makes it the universal fallback and the only joint-control path left
/// on Pepper 2.9. It is position-only: ALMotion exposes setAngles(names, angles,
/// speed) where speed is a fraction of the joint's max speed, not a velocity
/// command. Velocity is published as a state interface but cannot be commanded
/// here; use the DCM or LoLA systems for that.
///
/// State is read from ALMemory in one batched getListData call, using the same
/// keys as the joint_state converter:
///   Device/SubDeviceList/<Joint>/Position/Sensor/Value
///   Motion/Velocity/Sensor/<Joint>
///   Motion/Torque/Sensor/<Joint>
class AlMotionSystem : public hardware_interface::SystemInterface
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
  std::vector<std::string> joint_names_;

  // State, indexed like joint_names_.
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> efforts_;

  // Position command, indexed like joint_names_.
  std::vector<double> position_commands_;

  // ALMemory keys, laid out [positions..., velocities..., torques...].
  std::vector<std::string> memory_keys_;

  // Fraction of each joint's maximum speed used to reach the commanded
  // position (the ALMotion setAngles "fractionMaxSpeed" argument), in (0, 1].
  double max_velocity_fraction_ = 0.1;

  qi::SessionPtr session_;
  qi::AnyObject motion_;
  qi::AnyObject memory_;
};

}  // namespace hardware
}  // namespace naoqi

#endif  // NAOQI_DRIVER_HARDWARE_ALMOTION_SYSTEM_HPP
