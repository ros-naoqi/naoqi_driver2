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

#ifndef PUBLISHER_SONAR_HPP
#define PUBLISHER_SONAR_HPP

/*
* ROS includes
*/
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <naoqi_driver/ros_helpers.hpp>


namespace naoqi
{
namespace publisher
{

class SonarPublisher
{
public:
  SonarPublisher( const std::vector<std::string>& topics );

  inline std::string topic() const
  {
    return "sonar";
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  void publish( const std::vector<sensor_msgs::msg::Range>& sonar_msgs );

  void reset( rclcpp::Node* node );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) return false;
    for(std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr>::const_iterator it = pubs_.begin(); it != pubs_.end(); ++it)
      if (helpers::Node::count_subscribers((*it)->get_topic_name()))
        return true;
    return false;
  }

private:
  std::vector<std::string> topics_;
  std::vector<rclcpp::Publisher<sensor_msgs::msg::Range>::SharedPtr> pubs_;
  bool is_initialized_;

};

} //publisher
} //naoqi

#endif
