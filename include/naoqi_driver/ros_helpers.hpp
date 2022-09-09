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


#ifndef HELPERS_HPP
#define HELPERS_HPP

#include <rclcpp/rclcpp.hpp>
#include <boost/shared_ptr.hpp>

namespace naoqi {
namespace helpers {

/**
 * @brief Node helper class, holds a pointer towards the driver rclcpp::Node 
 * 
 */
class Node {
public:
  /**
   * @brief Set the Node object
   * 
   * @param node_ptr_ 
   */
  static void setNode(const boost::shared_ptr<rclcpp::Node>& node_ptr) {
    Node::node_ptr_ = node_ptr;
  }

  /**
   * @brief Get the logger object for the driver node
   * 
   * @return rclcpp::Logger 
   */
  static rclcpp::Logger get_logger() {
    return Node::node_ptr_->get_logger();
  }

  /**
   * @brief Get the number of subscribers for a publisher on a specific topic
   * 
   * @return int 
   */
  static size_t count_subscribers(const std::string& topic_name) {
    return Node::node_ptr_->count_subscribers(topic_name);
  }

protected:
  static boost::shared_ptr<rclcpp::Node> node_ptr_;
};

/**
 * @brief Time helper class, used to access to time related functionalities 
 * throughout the project
 * 
 */
class Time : public Node {
public:
  /**
   * @brief Calls the method now of the node instance
   * 
   * @return rclcpp::Time 
   */
  static rclcpp::Time now() {
    return Time::node_ptr_->now();
  }
};

} // naoqi
} // helpers

#endif // HELPERS_HPP