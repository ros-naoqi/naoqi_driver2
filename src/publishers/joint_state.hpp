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

#ifndef JOINT_STATES_PUBLISHER_HPP
#define JOINT_STATES_PUBLISHER_HPP

/*
* ROS includes
*/
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>

namespace naoqi
{
namespace publisher
{

class JointStatePublisher
{

public:
  JointStatePublisher( const std::string& topic = "/joint_states" );

  inline std::string topic() const
  {
  return topic_;
  }

  inline bool isInitialized() const
  {
  return is_initialized_;
  }

  virtual void publish( const sensor_msgs::msg::JointState& js_msg,
                        const std::vector<geometry_msgs::msg::TransformStamped>& tf_transforms );

  virtual void reset( rclcpp::Node* node );

  virtual bool isSubscribed() const;

private:
  boost::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcasterPtr_;

  /** initialize separate publishers for js and odom */
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_joint_states_;

  std::string topic_;

  bool is_initialized_;

}; // class

} //publisher
} // naoqi

#endif
