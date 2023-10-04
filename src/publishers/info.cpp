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
#include "info.hpp"
#include <std_msgs/msg/string.hpp>
#include "../tools/robot_description.hpp"

namespace naoqi
{
namespace publisher
{

InfoPublisher::InfoPublisher(const std::string& topic , const robot::Robot& robot_type)
  : BasicPublisher( topic ),
    robot_(robot_type)
{
}

void InfoPublisher::reset( rclcpp::Node* node )
{
  auto description_pub = node->create_publisher<std_msgs::msg::String>(
    "robot_description",
    // Transient local is similar to latching in ROS 1.
    rclcpp::QoS(1).transient_local()
  );

  std::string robot_desc = naoqi::tools::getRobotDescription(robot_);
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = robot_desc;

  // Publish the robot description
  description_pub->publish(std::move(msg));
  std::cout << "published robot description" << std::endl;

  is_initialized_ = true;
}

} // publisher
} //naoqi
