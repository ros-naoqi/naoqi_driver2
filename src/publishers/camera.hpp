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

#ifndef PUBLISHER_CAMERA_HPP
#define PUBLISHER_CAMERA_HPP

/*
* ROS includes
*/
#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

#include <naoqi_driver/ros_helpers.hpp>

namespace naoqi
{
namespace publisher
{

class CameraPublisher
{
public:
  CameraPublisher( const std::string& topic, int camera_source );

  ~CameraPublisher();

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  void publish( const sensor_msgs::msg::Image::SharedPtr& img, const sensor_msgs::msg::CameraInfo& camera_info );

  void reset( rclcpp::Node* node );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false){
      return false;
    } else{
      return helpers::Node::count_subscribers(topic_) > 0;
    }
  }

private:
  std::string topic_;

  bool is_initialized_;

  //image_transport::ImageTransport it_;
  image_transport::CameraPublisher pub_;

  int camera_source_;
};

} //publisher
} //naoqi


#endif
