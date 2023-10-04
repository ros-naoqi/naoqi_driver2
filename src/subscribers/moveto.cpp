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
#include "moveto.hpp"

/*
 * ROS includes
 */
//#include <tf/transform_datatypes.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include "../helpers/transform_helpers.hpp"

namespace naoqi
{
namespace subscriber
{

MovetoSubscriber::MovetoSubscriber( const std::string& name, const std::string& topic, const qi::SessionPtr& session,
                                    const boost::shared_ptr<tf2_ros::Buffer>& tf2_buffer):
  BaseSubscriber( name, topic, session ),
  p_motion_( session->service("ALMotion").value() ),
  tf2_buffer_( tf2_buffer )
{}

void MovetoSubscriber::reset( rclcpp::Node* node )
{
  sub_moveto_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    topic_,
    10,
    std::bind(&MovetoSubscriber::callback, this, std::placeholders::_1));

  is_initialized_ = true;
}

void MovetoSubscriber::callback( const geometry_msgs::msg::PoseStamped::SharedPtr pose_msg )
{
  if (pose_msg->header.frame_id == "odom") {
    geometry_msgs::msg::PoseStamped pose_msg_bf;

    bool canTransform = tf2_buffer_->canTransform(
      "base_footprint",
      "odom",
      tf2::get_now(),
      tf2::Duration(2));

    if (!canTransform) {
      std::cout << "Cannot transform from "
                << "odom"
                << " to base_footprint"
                << std::endl;
      return;
    }

    try {
      tf2_buffer_->transform(
        *pose_msg,
        pose_msg_bf,
        "base_footprint",
        tf2::get_now(),
        "odom");

      double yaw = helpers::transform::getYaw(pose_msg_bf.pose);

      std::cout << "odom to move x: " <<  pose_msg_bf.pose.position.x
                << " y: " << pose_msg_bf.pose.position.y
                << " yaw: " << yaw << std::endl;

      if (std::isnan(yaw)) {
        yaw = 0.0;
        std::cout << "Yaw is nan, changed to 0.0" << std::endl;
      }

      p_motion_.async<void>(
        "moveTo",
        pose_msg_bf.pose.position.x,
        pose_msg_bf.pose.position.y,
        yaw);

    } catch( const tf2::LookupException& e) {
      std::cout << e.what() << std::endl;
      std::cout << "moveto position in frame_id "
                << pose_msg->header.frame_id
                << "is not supported in any other base frame than "
                   "basefootprint"
                << std::endl;
    } catch( const tf2::ExtrapolationException& e) {
      std::cout << "received an error on the time lookup" << std::endl;
    }
  }
  else if (pose_msg->header.frame_id == "base_footprint"){
    double yaw = helpers::transform::getYaw(pose_msg->pose);
    std::cout << "going to move x: "
              <<  pose_msg->pose.position.x
              << " y: " << pose_msg->pose.position.y
              << " yaw: " << yaw << std::endl;

    if (std::isnan(yaw)) {
      yaw = 0.0;
      std::cout << "Yaw is nan, changed to 0.0" << std::endl;
    }

    p_motion_.async<void>(
      "moveTo",
      pose_msg->pose.position.x,
      pose_msg->pose.position.y,
      yaw);
  }

  else
    std::cout << "Cannot reach position expressed in the "
              << pose_msg->header.frame_id
              << " frame, enter a valid frame id in the pose's header"
                 " (base_footprint or odom)"
              << std::endl;
}

} //publisher
} // naoqi
