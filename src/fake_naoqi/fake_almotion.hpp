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

#ifndef FAKE_ALMOTION_HPP
#define FAKE_ALMOTION_HPP

#include <atomic>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <qi/anyobject.hpp>
#include <qi/type/dynamicobjectbuilder.hpp>

namespace naoqi
{
namespace fake
{

/**
 * @brief Fake implementation of NAOqi's ALMotion service
 *
 * This class emulates the ALMotion service for testing and simulation.
 * It maintains internal state for joint positions, velocities, and trajectories.
 */
class FakeALMotion
{
  public:
  explicit FakeALMotion(const std::string& robot_type, qi::AnyObject memory);
  ~FakeALMotion();

  // Robot configuration
  std::vector<std::vector<qi::AnyValue>> getRobotConfig();

  // Joint information
  std::vector<std::string> getBodyNames(const std::string& part_name);
  std::vector<double> getAngles(const std::string& part_name, bool use_sensors);
  qi::AnyValue getLimits(const std::string& joint_name);
  std::vector<std::string> getSensorNames();

  // Position and velocity
  std::vector<float> getPosition(const std::string& name, int space, bool use_sensors);
  std::vector<float> getRobotVelocity();

  // Motion commands
  void setAngles(const std::vector<std::string>& joint_names,
                 const std::vector<float>& angles,
                 float speed);
  void changeAngles(const std::vector<std::string>& joint_names,
                    const std::vector<float>& angles,
                    float speed);
  void move(float x, float y, float theta);
  void moveTo(float x, float y, float theta);

  // Trajectory execution
  void angleInterpolation(const std::vector<std::string>& joint_names,
                          const std::vector<std::vector<float>>& positions,
                          const std::vector<std::vector<float>>& times,
                          bool is_absolute);
  void killTasksUsingResources(const std::vector<std::string>& resource_names);

  private:
  void initializeJoints();
  void updateTrajectories(double dt);
  void trajectoryUpdateLoop();
  /// Build an insertListData entry: a list-type AnyValue of [key, value]
  static qi::AnyValue makeEntry(const std::string& key, float value);
  /// Write joint positions/velocities/torques to ALMemory in one batch call
  void syncJointsToMemory(const std::vector<std::string>& names,
                          const std::vector<double>& positions,
                          const std::vector<double>& velocities);

  struct JointState
  {
    double position = 0.0;
    double velocity = 0.0;
    double min_limit = -2.0;
    double max_limit = 2.0;
  };

  struct TrajectoryPoint
  {
    double target_position;
    double target_time;
    double start_position;
    double start_time;
  };

  std::string robot_type_;
  qi::AnyObject memory_;

  std::map<std::string, JointState> joints_;
  std::map<std::string, std::vector<TrajectoryPoint>> active_trajectories_;
  std::mutex joints_mutex_;
  std::mutex trajectories_mutex_;

  // Base position and velocity
  std::vector<float> base_position_;  // [x, y, z, roll, pitch, yaw]
  std::vector<float> base_velocity_;  // [vx, vy, vtheta]
  std::mutex base_mutex_;

  // Trajectory update thread
  std::thread trajectory_thread_;
  std::atomic<bool> running_;
};

}  // namespace fake
}  // namespace naoqi

#endif  // FAKE_ALMOTION_HPP
