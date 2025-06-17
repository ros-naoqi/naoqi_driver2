/*
 * Copyright 2015 Aldebaran
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

/*
 * LOCAL includes
 */
#include "teleop.hpp"

#include <boost/thread/synchronized_value.hpp>

namespace naoqi
{
namespace subscriber
{

TeleopSubscriber::TeleopSubscriber(const std::string& name,
                                   const std::string& cmd_vel_topic,
                                   const std::string& joint_angles_topic,
                                   const std::string& joint_trajectory_topic,
                                   const qi::SessionPtr& session)
    : BaseSubscriber(name, cmd_vel_topic, session), cmd_vel_topic_(cmd_vel_topic),
      joint_angles_topic_(joint_angles_topic), joint_trajectory_topic_(joint_trajectory_topic),
      p_motion_(session->service("ALMotion").value())
{
}

namespace
{
void on_cmd_vel_msg(qi::AnyObject motion, const geometry_msgs::msg::Twist::SharedPtr twist)
{
  // no need to check for max velocity since motion clamps the velocities
  // internally
  const float& vel_x = twist->linear.x;
  const float& vel_y = twist->linear.y;
  const float& vel_th = twist->angular.z;

  std::cout << "going to move x: " << vel_x << " y: " << vel_y << " th: " << vel_th << std::endl;
  motion.async<void>("move", vel_x, vel_y, vel_th);
}

void on_joint_angles_msg(qi::AnyObject motion,
                         const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr ja)
{
  if (ja->relative == 0)
  {
    motion.async<void>("setAngles", ja->joint_names, ja->joint_angles, ja->speed);
  }
  else
  {
    motion.async<void>("changeAngles", ja->joint_names, ja->joint_angles, ja->speed);
  }
}

void on_joint_trajectory_msg(
    const qi::AnyObject motion,
    boost::synchronized_value<std::vector<std::string>>& previous_joint_names,
    qi::Future<void>& ongoing,
    const trajectory_msgs::msg::JointTrajectory::SharedPtr traj)
{
  // Kill previous trajectory, leading to the completion of `ongoing`.
  // If `ongoing` had completed already, `previous_joint_names` will be empty.
  auto previous_joint_names_value = previous_joint_names.synchronize();
  if (!previous_joint_names_value->empty())
  {
    motion.async<void>("killTasksUsingResources", previous_joint_names);
  }

  // Chain previous trajectory with the new one.
  ongoing =
      ongoing
          .then([motion, traj, &previous_joint_names](qi::Future<void>) mutable {
            size_t nof_joints = traj->joint_names.size();
            size_t nof_points = traj->points.size();
            std::vector<std::vector<float>> positions(nof_joints, std::vector<float>(nof_points));
            std::vector<std::vector<float>> times(nof_joints, std::vector<float>(nof_points));
            for (size_t i = 0; i < nof_joints; ++i)
            {
              for (size_t j = 0; j < nof_points; ++j)
              {
                positions[i][j] = traj->points[j].positions[i];
                times[i][j] = static_cast<float>(traj->points[j].time_from_start.sec) +
                              traj->points[j].time_from_start.nanosec * 1e-9;
              }
            }

            // Task is about to start, register the joint names to be able to cancel it later.
            *previous_joint_names.synchronize() = traj->joint_names;
            return motion
                .async<void>("angleInterpolation", traj->joint_names, positions, times, true)
                .then([&previous_joint_names](qi::Future<void> f) mutable {
                  if (f.hasError())
                  {
                    std::cerr << "Error executing joint trajectory: " << f.error() << std::endl;
                  }
                  // Motion task has completed (by itself or by cancellation),
                  // clear the previous joint names to avoid future cancellations.
                  // (We don't want to interrupt other moves than these trajectories, by accident.)
                  previous_joint_names.synchronize()->clear();
                });
          })
          .unwrap();
}
}  // namespace

void TeleopSubscriber::reset(rclcpp::Node* node)
{
  auto motion = p_motion_;
  sub_cmd_vel_ = node->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, 10, [=](const geometry_msgs::msg::Twist::SharedPtr twist_msg) {
        on_cmd_vel_msg(motion, twist_msg);
      });

  sub_joint_angles_ = node->create_subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>(
      joint_angles_topic_,
      10,
      [=](const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg) {
        on_joint_angles_msg(motion, js_msg);
      });

  // Joint trajectory callback has some internal state.
  // It tracks the previous calls to cancel them and wait for them to finish.
  // By using a shared pointer, we bypass the constraint that the callback can't be mutable.
  struct JointTrajectoryState
  {
    boost::synchronized_value<std::vector<std::string>> previous_joint_names;
    qi::Future<void> ongoing{nullptr};  // No need to be thread-safe for this one.
  };

  sub_joint_trajectory_ = node->create_subscription<trajectory_msgs::msg::JointTrajectory>(
      joint_trajectory_topic_,
      10,
      [motion, state = std::make_shared<JointTrajectoryState>()](
          const trajectory_msgs::msg::JointTrajectory::SharedPtr traj_msg) {
        on_joint_trajectory_msg(motion, state->previous_joint_names, state->ongoing, traj_msg);
      });

  is_initialized_ = true;
}

}  // namespace subscriber
}  // namespace naoqi
