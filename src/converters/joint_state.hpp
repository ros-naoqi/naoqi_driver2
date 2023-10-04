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

#ifndef JOINT_STATES_CONVERTER_HPP
#define JOINT_STATES_CONVERTER_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include "../tools/robot_description.hpp"
#include <naoqi_driver/message_actions.h>
#include <naoqi_driver/ros_helpers.hpp>

/*
* ROS includes
*/
#include <urdf/model.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2_ros/buffer.h>
#include <robot_state_publisher/robot_state_publisher.hpp>

namespace naoqi
{
namespace converter
{

class JointStateConverter : public BaseConverter<JointStateConverter>
{

  typedef boost::function<void(sensor_msgs::msg::JointState&, std::vector<geometry_msgs::msg::TransformStamped>&) > Callback_t;

  typedef boost::shared_ptr<tf2_ros::Buffer> BufferPtr;

  typedef std::map<std::string, urdf::JointMimicSharedPtr> MimicMap;

public:
  JointStateConverter( const std::string& name, const float& frequency, const BufferPtr& tf2_buffer, const qi::SessionPtr& session );

  ~JointStateConverter();

  virtual void reset( );

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

private:

  /** blatently copied from robot state publisher */
  void addChildren(const KDL::SegmentMap::const_iterator segment);
  std::map<std::string, robot_state_publisher::SegmentPair> segments_, segments_fixed_;
  void setTransforms(const std::map<std::string, double>& joint_positions, const rclcpp::Time& time, const std::string& tf_prefix);
  void setFixedTransforms(const std::string& tf_prefix, const rclcpp::Time& time);

  /** Global Shared tf2 buffer **/
  BufferPtr tf2_buffer_;

  /** Motion Proxy **/
  qi::AnyObject p_motion_;
  qi::AnyObject p_memory_;

  /** Registered Callbacks **/
  std::map<message_actions::MessageAction, Callback_t> callbacks_;

  /** MimicJoint List **/
  MimicMap mimic_;

  /** JointState Message **/
  sensor_msgs::msg::JointState msg_joint_states_;

  /** Transform Messages **/
  std::vector<geometry_msgs::msg::TransformStamped> tf_transforms_;

}; // class

} //publisher
} // naoqi

#endif
