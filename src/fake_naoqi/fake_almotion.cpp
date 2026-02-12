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

#include "fake_almotion.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>

namespace naoqi
{
namespace fake
{

namespace
{
// NAO joint names based on real robot
const std::vector<std::string> NAO_JOINT_NAMES = {
    "HeadYaw",     "HeadPitch",  "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
    "LWristYaw",   "LHand",      "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
    "RWristYaw",   "RHand",      "LHipYawPitch",   "LHipRoll",      "LHipPitch", "LKneePitch",
    "LAnklePitch", "LAnkleRoll", "RHipYawPitch",   "RHipRoll",      "RHipPitch", "RKneePitch",
    "RAnklePitch", "RAnkleRoll"};

// Pepper joint names (simplified)
const std::vector<std::string> PEPPER_JOINT_NAMES = {
    "HeadYaw",    "HeadPitch",  "LShoulderPitch", "LShoulderRoll",  "LElbowYaw",
    "LElbowRoll", "LWristYaw",  "LHand",          "RShoulderPitch", "RShoulderRoll",
    "RElbowYaw",  "RElbowRoll", "RWristYaw",      "RHand",          "HipRoll",
    "HipPitch",   "KneePitch",  "WheelFL",        "WheelFR",        "WheelB"};

// Romeo joint names (simplified)
const std::vector<std::string> ROMEO_JOINT_NAMES = {
    "HeadYaw",     "HeadPitch",  "LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll",
    "LWristYaw",   "LHand",      "RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll",
    "RWristYaw",   "RHand",      "LHipYawPitch",   "LHipRoll",      "LHipPitch", "LKneePitch",
    "LAnklePitch", "LAnkleRoll", "RHipYawPitch",   "RHipRoll",      "RHipPitch", "RKneePitch",
    "RAnklePitch", "RAnkleRoll", "TrunkYaw",       "TrunkPitch"};

}  // namespace

FakeALMotion::FakeALMotion(const std::string& robot_type, qi::AnyObject memory)
    : robot_type_(robot_type), memory_(memory), base_position_(6, 0.0f), base_velocity_(3, 0.0f),
      running_(true)
{
  initializeJoints();
  trajectory_thread_ = std::thread(&FakeALMotion::trajectoryUpdateLoop, this);
}

FakeALMotion::~FakeALMotion()
{
  running_ = false;
  if (trajectory_thread_.joinable())
  {
    trajectory_thread_.join();
  }
}

qi::AnyValue FakeALMotion::makeEntry(const std::string& key, float value)
{
  std::vector<qi::AnyValue> pair;
  pair.push_back(qi::AnyValue::from(key));
  pair.push_back(qi::AnyValue::from(value));
  return qi::AnyValue::from(pair);
}

void FakeALMotion::syncJointsToMemory(const std::vector<std::string>& names,
                                      const std::vector<double>& positions,
                                      const std::vector<double>& velocities)
{
  if (!memory_)
    return;
  std::vector<qi::AnyValue> entries;
  entries.reserve(3 * names.size());
  for (size_t i = 0; i < names.size(); ++i)
  {
    entries.push_back(makeEntry("Device/SubDeviceList/" + names[i] + "/Position/Sensor/Value",
                                static_cast<float>(positions[i])));
    entries.push_back(
        makeEntry("Motion/Velocity/Sensor/" + names[i], static_cast<float>(velocities[i])));
    entries.push_back(makeEntry("Motion/Torque/Sensor/" + names[i], 0.0f));
  }
  memory_.call<void>("insertListData", entries);
}

void FakeALMotion::initializeJoints()
{
  std::vector<std::string> joint_names;

  if (robot_type_ == "nao")
  {
    joint_names = NAO_JOINT_NAMES;
  }
  else if (robot_type_ == "pepper")
  {
    joint_names = PEPPER_JOINT_NAMES;
  }
  else if (robot_type_ == "romeo")
  {
    joint_names = ROMEO_JOINT_NAMES;
  }
  else
  {
    // Default to NAO
    joint_names = NAO_JOINT_NAMES;
  }

  std::lock_guard<std::mutex> lock(joints_mutex_);
  for (const auto& name : joint_names)
  {
    JointState state;
    state.position = 0.0;
    state.velocity = 0.0;
    // Set reasonable limits (these should be more accurate per joint)
    state.min_limit = -2.0857;
    state.max_limit = 2.0857;
    joints_[name] = state;
  }
}

std::vector<std::vector<qi::AnyValue>> FakeALMotion::getRobotConfig()
{
  std::vector<std::vector<qi::AnyValue>> config(2);

  config[0].push_back(qi::AnyValue::from("Model Type"));
  config[0].push_back(qi::AnyValue::from("Head Version"));
  config[0].push_back(qi::AnyValue::from("Body Version"));
  config[0].push_back(qi::AnyValue::from("Arm Version"));
  config[0].push_back(qi::AnyValue::from("Laser"));
  config[0].push_back(qi::AnyValue::from("Extended Arms"));

  if (robot_type_ == "nao")
  {
    config[1].push_back(qi::AnyValue::from(std::string("V6")));
    config[1].push_back(qi::AnyValue::from(std::string("6.0")));
    config[1].push_back(qi::AnyValue::from(std::string("6.0")));
    config[1].push_back(qi::AnyValue::from(std::string("6.0")));
    config[1].push_back(qi::AnyValue::from(false));
    config[1].push_back(qi::AnyValue::from(false));
  }
  else if (robot_type_ == "pepper")
  {
    config[1].push_back(qi::AnyValue::from(std::string("Pepper")));
    config[1].push_back(qi::AnyValue::from(std::string("1.8a")));
    config[1].push_back(qi::AnyValue::from(std::string("1.8a")));
    config[1].push_back(qi::AnyValue::from(std::string("1.8a")));
    config[1].push_back(qi::AnyValue::from(true));
    config[1].push_back(qi::AnyValue::from(false));
  }
  else  // romeo
  {
    config[1].push_back(qi::AnyValue::from(std::string("Romeo")));
    config[1].push_back(qi::AnyValue::from(std::string("1.0")));
    config[1].push_back(qi::AnyValue::from(std::string("1.0")));
    config[1].push_back(qi::AnyValue::from(std::string("1.0")));
    config[1].push_back(qi::AnyValue::from(false));
    config[1].push_back(qi::AnyValue::from(true));
  }

  return config;
}

std::vector<std::string> FakeALMotion::getBodyNames(const std::string& part_name)
{
  std::lock_guard<std::mutex> lock(joints_mutex_);

  if (part_name == "Body")
  {
    std::vector<std::string> names;
    for (const auto& pair : joints_)
    {
      names.push_back(pair.first);
    }
    return names;
  }
  else if (part_name == "JointActuators")
  {
    // Same as Body for simplicity
    std::vector<std::string> names;
    for (const auto& pair : joints_)
    {
      names.push_back(pair.first);
    }
    return names;
  }
  else if (part_name == "Head")
  {
    return {"HeadYaw", "HeadPitch"};
  }
  else if (part_name == "LArm")
  {
    return {"LShoulderPitch", "LShoulderRoll", "LElbowYaw", "LElbowRoll", "LWristYaw", "LHand"};
  }
  else if (part_name == "RArm")
  {
    return {"RShoulderPitch", "RShoulderRoll", "RElbowYaw", "RElbowRoll", "RWristYaw", "RHand"};
  }

  return {};
}

std::vector<double> FakeALMotion::getAngles(const std::string& part_name, bool use_sensors)
{
  // Get joint names first (which locks mutex internally)
  auto joint_names = getBodyNames(part_name);

  // Then lock and get angles
  std::lock_guard<std::mutex> lock(joints_mutex_);
  std::vector<double> angles;

  for (const auto& name : joint_names)
  {
    auto it = joints_.find(name);
    if (it != joints_.end())
    {
      angles.push_back(it->second.position);
    }
    else
    {
      angles.push_back(0.0);
    }
  }

  return angles;
}

qi::AnyValue FakeALMotion::getLimits(const std::string& joint_name)
{
  std::lock_guard<std::mutex> lock(joints_mutex_);

  auto it = joints_.find(joint_name);
  if (it != joints_.end())
  {
    std::vector<qi::AnyValue> limits;
    limits.push_back(qi::AnyValue::from(it->second.min_limit));
    limits.push_back(qi::AnyValue::from(it->second.max_limit));
    return qi::AnyValue::from(limits);
  }

  std::vector<qi::AnyValue> default_limits;
  default_limits.push_back(qi::AnyValue::from(-2.0857));
  default_limits.push_back(qi::AnyValue::from(2.0857));
  return qi::AnyValue::from(default_limits);
}

std::vector<std::string> FakeALMotion::getSensorNames()
{
  // Return a list of sensor names that would be available on a real robot
  // These are used by the driver to read sensor data
  std::vector<std::string> sensors = {"AccX",
                                      "AccY",
                                      "AccZ",
                                      "GyrX",
                                      "GyrY",
                                      "GyrRef",
                                      "AngleX",
                                      "AngleY",
                                      "LFsrFL",
                                      "LFsrFR",
                                      "LFsrRL",
                                      "LFsrRR",
                                      "RFsrFL",
                                      "RFsrFR",
                                      "RFsrRL",
                                      "RFsrRR",
                                      "CenterOfPressure/X",
                                      "CenterOfPressure/Y"};
  return sensors;
}

std::vector<float> FakeALMotion::getPosition(const std::string& name, int space, bool use_sensors)
{
  std::lock_guard<std::mutex> lock(base_mutex_);
  return base_position_;
}

std::vector<float> FakeALMotion::getRobotVelocity()
{
  std::lock_guard<std::mutex> lock(base_mutex_);
  return base_velocity_;
}

void FakeALMotion::setAngles(const std::vector<std::string>& joint_names,
                             const std::vector<float>& angles,
                             float speed)
{
  if (joint_names.size() != angles.size())
  {
    std::cerr << "FakeALMotion::setAngles: joint_names and angles size mismatch" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(joints_mutex_);
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    auto it = joints_.find(joint_names[i]);
    if (it != joints_.end())
    {
      // Clamp to limits
      float target = std::max(static_cast<float>(it->second.min_limit),
                              std::min(static_cast<float>(it->second.max_limit), angles[i]));

      // For simplicity, set immediately (in real robot this would be gradual)
      // TODO: Add trajectory for smooth motion
      it->second.position = target;
      it->second.velocity = 0.0;
    }
  }
  // Batch-sync all changed joints to ALMemory
  {
    std::vector<double> positions(joint_names.size(), 0.0);
    std::vector<double> velocities(joint_names.size(), 0.0);
    for (size_t i = 0; i < joint_names.size(); ++i)
    {
      auto it = joints_.find(joint_names[i]);
      if (it != joints_.end())
      {
        positions[i] = it->second.position;
        velocities[i] = it->second.velocity;
      }
    }
    syncJointsToMemory(joint_names, positions, velocities);
  }
}

void FakeALMotion::changeAngles(const std::vector<std::string>& joint_names,
                                const std::vector<float>& angles,
                                float speed)
{
  if (joint_names.size() != angles.size())
  {
    std::cerr << "FakeALMotion::changeAngles: joint_names and angles size mismatch" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(joints_mutex_);
  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    auto it = joints_.find(joint_names[i]);
    if (it != joints_.end())
    {
      float target = it->second.position + angles[i];
      // Clamp to limits
      target = std::max(static_cast<float>(it->second.min_limit),
                        std::min(static_cast<float>(it->second.max_limit), target));
      it->second.position = target;
      it->second.velocity = 0.0;
    }
  }
  // Batch-sync all changed joints to ALMemory
  {
    std::vector<double> positions, velocities;
    positions.reserve(joint_names.size());
    velocities.reserve(joint_names.size());
    for (const auto& name : joint_names)
    {
      auto it = joints_.find(name);
      if (it != joints_.end())
      {
        positions.push_back(it->second.position);
        velocities.push_back(it->second.velocity);
      }
    }
    syncJointsToMemory(joint_names, positions, velocities);
  }
}

void FakeALMotion::move(float x, float y, float theta)
{
  std::lock_guard<std::mutex> lock(base_mutex_);
  base_velocity_[0] = x;
  base_velocity_[1] = y;
  base_velocity_[2] = theta;
}

void FakeALMotion::moveTo(float x, float y, float theta)
{
  // For simplicity, just set the target position
  // In reality, this would plan and execute a trajectory
  std::lock_guard<std::mutex> lock(base_mutex_);
  base_position_[0] = x;
  base_position_[1] = y;
  base_position_[5] = theta;  // yaw
  base_velocity_[0] = 0.0f;
  base_velocity_[1] = 0.0f;
  base_velocity_[2] = 0.0f;
}

void FakeALMotion::angleInterpolation(const std::vector<std::string>& joint_names,
                                      const std::vector<std::vector<float>>& positions,
                                      const std::vector<std::vector<float>>& times,
                                      bool is_absolute)
{
  if (joint_names.size() != positions.size() || joint_names.size() != times.size())
  {
    std::cerr << "FakeALMotion::angleInterpolation: size mismatch" << std::endl;
    return;
  }

  auto now = std::chrono::steady_clock::now();
  double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();

  std::lock_guard<std::mutex> traj_lock(trajectories_mutex_);
  std::lock_guard<std::mutex> joint_lock(joints_mutex_);

  for (size_t i = 0; i < joint_names.size(); ++i)
  {
    const auto& joint_name = joint_names[i];
    auto joint_it = joints_.find(joint_name);
    if (joint_it == joints_.end())
    {
      continue;
    }

    std::vector<TrajectoryPoint>& traj = active_trajectories_[joint_name];
    traj.clear();

    double start_position = joint_it->second.position;
    double start_time = current_time;

    for (size_t j = 0; j < positions[i].size(); ++j)
    {
      TrajectoryPoint point;
      point.target_position = positions[i][j];
      point.target_time = current_time + times[i][j];
      point.start_position = (j == 0) ? start_position : positions[i][j - 1];
      point.start_time = (j == 0) ? start_time : (current_time + times[i][j - 1]);
      traj.push_back(point);
    }
  }
}

void FakeALMotion::killTasksUsingResources(const std::vector<std::string>& resource_names)
{
  std::lock_guard<std::mutex> lock(trajectories_mutex_);

  for (const auto& resource : resource_names)
  {
    active_trajectories_.erase(resource);
  }
}

void FakeALMotion::updateTrajectories(double dt)
{
  auto now = std::chrono::steady_clock::now();
  double current_time = std::chrono::duration<double>(now.time_since_epoch()).count();

  std::lock_guard<std::mutex> traj_lock(trajectories_mutex_);
  std::lock_guard<std::mutex> joint_lock(joints_mutex_);

  // Update base position based on velocity
  {
    std::lock_guard<std::mutex> base_lock(base_mutex_);
    base_position_[0] += base_velocity_[0] * dt;
    base_position_[1] += base_velocity_[1] * dt;
    base_position_[5] += base_velocity_[2] * dt;
  }

  // Update joint trajectories
  for (auto& traj_pair : active_trajectories_)
  {
    const auto& joint_name = traj_pair.first;
    auto& trajectory = traj_pair.second;

    auto joint_it = joints_.find(joint_name);
    if (joint_it == joints_.end() || trajectory.empty())
    {
      continue;
    }

    // Find current trajectory segment
    size_t current_segment = 0;
    while (current_segment < trajectory.size() &&
           current_time > trajectory[current_segment].target_time)
    {
      current_segment++;
    }

    if (current_segment < trajectory.size())
    {
      const auto& point = trajectory[current_segment];
      double segment_duration = point.target_time - point.start_time;
      double elapsed = current_time - point.start_time;

      if (segment_duration > 0.0)
      {
        double progress = std::min(1.0, elapsed / segment_duration);
        // Linear interpolation (could be improved with better interpolation)
        double new_position =
            point.start_position + (point.target_position - point.start_position) * progress;
        joint_it->second.position = new_position;

        if (progress < 1.0)
        {
          joint_it->second.velocity =
              (point.target_position - point.start_position) / segment_duration;
        }
        else
        {
          joint_it->second.velocity = 0.0;
        }
      }
    }
    else
    {
      // Trajectory completed
      if (!trajectory.empty())
      {
        joint_it->second.position = trajectory.back().target_position;
        joint_it->second.velocity = 0.0;
      }
    }
  }

  // Batch-sync all trajectory joints to ALMemory
  if (!active_trajectories_.empty())
  {
    std::vector<std::string> names;
    std::vector<double> positions;
    std::vector<double> velocities;
    for (const auto& traj_pair : active_trajectories_)
    {
      auto joint_it = joints_.find(traj_pair.first);
      if (joint_it != joints_.end())
      {
        names.push_back(traj_pair.first);
        positions.push_back(joint_it->second.position);
        velocities.push_back(joint_it->second.velocity);
      }
    }
    if (!names.empty())
    {
      syncJointsToMemory(names, positions, velocities);
    }
  }

  // Clean up completed trajectories
  for (auto it = active_trajectories_.begin(); it != active_trajectories_.end();)
  {
    bool all_completed = true;
    for (const auto& point : it->second)
    {
      if (current_time < point.target_time)
      {
        all_completed = false;
        break;
      }
    }

    if (all_completed && !it->second.empty())
    {
      it = active_trajectories_.erase(it);
    }
    else
    {
      ++it;
    }
  }
}

void FakeALMotion::trajectoryUpdateLoop()
{
  const double update_rate = 100.0;  // 100 Hz update
  const auto sleep_duration = std::chrono::milliseconds(static_cast<int>(1000.0 / update_rate));

  while (running_)
  {
    updateTrajectories(1.0 / update_rate);
    std::this_thread::sleep_for(sleep_duration);
  }
}

}  // namespace fake
}  // namespace naoqi

// Register the FakeALMotion class with libqi
QI_REGISTER_OBJECT(naoqi::fake::FakeALMotion,
                   getRobotConfig,
                   getBodyNames,
                   getAngles,
                   getLimits,
                   getSensorNames,
                   getPosition,
                   getRobotVelocity,
                   setAngles,
                   changeAngles,
                   move,
                   moveTo,
                   angleInterpolation,
                   killTasksUsingResources)
