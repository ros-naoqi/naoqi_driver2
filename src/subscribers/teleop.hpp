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


#ifndef TELEOP_SUBSCRIBER_HPP
#define TELEOP_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <naoqi_bridge_msgs/msg/joint_angles_with_speed.hpp>

namespace naoqi
{
namespace subscriber
{

class TeleopSubscriber: public BaseSubscriber<TeleopSubscriber>
{
public:
  TeleopSubscriber( const std::string& name, const std::string& cmd_vel_topic, const std::string& joint_angles_topic, const qi::SessionPtr& session );
  ~TeleopSubscriber(){}

  void reset( rclcpp::Node* node );
  void cmd_vel_callback( const geometry_msgs::msg::Twist::SharedPtr twist_msg );
  void joint_angles_callback( const naoqi_bridge_msgs::msg::JointAnglesWithSpeed::SharedPtr js_msg );

private:

  std::string cmd_vel_topic_;
  std::string joint_angles_topic_;

  qi::AnyObject p_motion_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::Subscription<naoqi_bridge_msgs::msg::JointAnglesWithSpeed>::SharedPtr sub_joint_angles_;



}; // class Teleop

} // subscriber
}// naoqi
#endif
