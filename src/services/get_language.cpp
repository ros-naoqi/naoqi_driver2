/*
 * Copyright 2017 Aldebaran
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

#include "get_language.hpp"
#include "../helpers/driver_helpers.hpp"

namespace naoqi
{
namespace service
{

GetLanguageService::GetLanguageService( const std::string& name, const std::string& topic, const qi::SessionPtr& session )
  : name_(name),
  topic_(topic),
  session_(session)
{}

void GetLanguageService::reset( rclcpp::Node* node )
{
  service_ = node->create_service<naoqi_bridge_msgs::srv::GetString>(
    topic_,
    std::bind(&GetLanguageService::callback, this, std::placeholders::_1, std::placeholders::_2));
}

void GetLanguageService::callback( const std::shared_ptr<naoqi_bridge_msgs::srv::GetString::Request> req, std::shared_ptr<naoqi_bridge_msgs::srv::GetString::Response> resp )
{
  std::cout << "Receiving service call of getting language" << std::endl;
  resp->data = helpers::driver::getLanguage(session_);
}


}
}
