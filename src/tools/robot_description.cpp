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

#include <rclcpp/node.hpp>

/*
 * LOCAL includes
 */
#include "../helpers/filesystem_helpers.hpp"
#include "robot_description.hpp"

namespace naoqi
{

namespace tools
{

std::string getRobotDescription(const robot::Robot& robot)
{
  std::string urdf_path;
  static std::string robot_desc;
  if (!robot_desc.empty())
    return robot_desc;

  if (robot == robot::PEPPER)
  {
    urdf_path = helpers::filesystem::getURDF("pepper.urdf");
  }
  else if (robot == robot::NAO)
  {
    urdf_path = helpers::filesystem::getURDF("nao.urdf");
  }
  else if (robot == robot::ROMEO)
  {
    urdf_path = helpers::filesystem::getURDF("romeo.urdf");
  }
  else
  {
    std::cerr << " could not load urdf file from disk " << std::endl;
    return std::string();
  }

  std::ifstream stream((urdf_path).c_str());
  if (!stream)
  {
    std::cerr << "failed to load robot description in joint_state_publisher: " << urdf_path
              << std::endl;
    return std::string();
  }
  robot_desc =
      std::string((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
  return robot_desc;
}

rclcpp::Publisher<std_msgs::msg::String>::SharedPtr
publishRobotDescription(rclcpp::Node* node, const robot::Robot& robot_type)
{
  static const auto topic = "robot_description";
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr description_pub =
      node->create_publisher<std_msgs::msg::String>(
          topic,
          // Transient local is similar to latching in ROS 1.
          rclcpp::QoS(1).transient_local());

  std::string robot_desc = getRobotDescription(robot_type);
  auto msg = std::make_unique<std_msgs::msg::String>();
  msg->data = robot_desc;

  // Publish the robot description
  description_pub->publish(std::move(msg));
  std::cout << "published robot description to /" << topic << std::endl;
  return description_pub;
}

}  // namespace tools

}  // namespace naoqi
