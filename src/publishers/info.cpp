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
  // We latch as we only publish once
  pub_ = node->create_publisher<naoqi_bridge_msgs::msg::StringStamped>( topic_, 1);

  std::string robot_desc = naoqi::tools::getRobotDescription(robot_);
  std::string parameter_name = "/robot_description";
  rclcpp::ParameterValue value = node->declare_parameter(parameter_name);
  node->set_parameter(rclcpp::Parameter(parameter_name, robot_desc));
  std::cout << "load robot description from file" << std::endl;

  is_initialized_ = true;
}

} // publisher
} //naoqi
