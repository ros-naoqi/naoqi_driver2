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

#ifndef ROBOT_DESCRIPTION_HPP
#define ROBOT_DESCRIPTION_HPP

/*
* STANDARD includes
*/
#include <string>
#include <fstream>

// ROS includes
#include <std_msgs/msg/string.hpp>
#include <rclcpp/publisher.hpp>

/*
* LOCAL includes
*/
#include <naoqi_driver/tools.hpp>
namespace rclcpp {
  class Node;
}

namespace naoqi {

namespace tools {

std::string getRobotDescription( const robot::Robot& robot);

/**
 * Publishes the robot description on the conventional topic /robot_description.
 * Returns the publisher that must be kept alive to keep the robot description published.
 */
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
publishRobotDescription(rclcpp::Node *node, const robot::Robot &robot_type);
}
}

#endif // ROBOT_DESCRIPTION_HPP
