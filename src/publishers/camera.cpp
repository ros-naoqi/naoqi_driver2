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
#include "camera.hpp"

/*
* ALDEBARAN includes
*/
#include "../tools/alvisiondefinitions.h" // for kTop...

namespace naoqi
{
namespace publisher
{

CameraPublisher::CameraPublisher( const std::string& topic, int camera_source ):
  topic_( topic ),
  is_initialized_(false),
  camera_source_( camera_source )
{
}

CameraPublisher::~CameraPublisher()
{
}

void CameraPublisher::publish( const sensor_msgs::msg::Image::SharedPtr& img, const sensor_msgs::msg::CameraInfo& camera_info )
{
  pub_.publish( *img, camera_info );
}

void CameraPublisher::reset( rclcpp::Node* node )
{
  pub_ = image_transport::create_camera_publisher(node, topic_);
  /* TODO */
  /* Specify the QoS or remove unwanted image_transports topics by disabling plugin
  in the lanchfile. */

  // Unregister compressedDepth topics for non depth cameras
  // if (camera_source_!=AL::kDepthCamera)
  // {
  //   // Get our URI as a caller
  //   std::string node_name = node->get_name();
  //   XmlRpc::XmlRpcValue args, result, payload;
  //   args[0] = node_name;
  //   args[1] = node_name;
  //   ros::master::execute("lookupNode", args, result, payload, false);
  //   args[2] = result[2];

  //   // List the topics to remove
  //   std::vector<std::string> topic_list;
  //   topic_list.push_back(std::string("/") + node_name + "/" + topic_ + std::string("/compressedDepth"));
  //   topic_list.push_back(std::string("/") + node_name + "/" + topic_ + std::string("/compressedDepth/parameter_updates"));
  //   topic_list.push_back(std::string("/") + node_name + "/" + topic_ + std::string("/compressedDepth/parameter_descriptions"));

  //   // Remove undesirable topics
  //   for(std::vector<std::string>::const_iterator topic = topic_list.begin(); topic != topic_list.end(); ++topic)
  //   {
  //     args[1] = *topic;
  //     ros::master::execute("unregisterPublisher", args, result, payload, false);
  //   }
  // }

  is_initialized_ = true;
}

} // publisher
} //naoqi
