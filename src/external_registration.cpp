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

#include <rclcpp/rclcpp.hpp>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>
#include <qi/anymodule.hpp>

#if LIBQI_VERSION >= 29
#include "driver_authenticator.hpp"
#endif

#include <naoqi_driver/naoqi_driver.hpp>

#include <boost/program_options.hpp>
#include "naoqi_driver/tools.hpp"
#include "naoqi_driver/ros_helpers.hpp"

int main(int argc, char** argv) {
  const std::string no_password = "no_password";
  std::string protocol = "tcp://";
  
  std::string nao_ip;
  int nao_port;
  std::string user;
  std::string password;
  std::string network_interface;

  // Initialize ROS and create the driver Node
  rclcpp::init(argc, argv);
  boost::shared_ptr<naoqi::Driver> bs = boost::make_shared<naoqi::Driver>();

  // Setup the time helper
  naoqi::helpers::Node::setNode(bs);
  
  // Retrieve the parameters
  bs->declare_parameter<std::string>("nao_ip", "127.0.0.1");
  bs->declare_parameter<int>("nao_port", 9559);
  bs->declare_parameter<std::string>("user", "nao");
  bs->declare_parameter<std::string>("password", no_password);
  bs->declare_parameter<std::string>("network_interface", "eth0");
  
  bs->get_parameter("nao_ip", nao_ip);
  bs->get_parameter("nao_port", nao_port);
  bs->get_parameter("user", user);
  bs->get_parameter("password", password);
  bs->get_parameter("network_interface", network_interface);

  if (password.compare(no_password) != 0) {
#if LIBQI_VERSION >= 29
    protocol = "tcps://";
#else
    std::cout << BOLDRED 
              << "No need to set a password" 
              << RESETCOLOR 
              << std::endl;
#endif
  }

  // Build the q::ApplicationSession, and connect the associated session
  qi::Url url(protocol + nao_ip + ":" + std::to_string(nao_port));
  qi::ApplicationSession app(argc, argv);

#if LIBQI_VERSION >= 29
  if (password.compare(no_password) != 0) {
    naoqi::DriverAuthenticatorFactory *factory = new naoqi::DriverAuthenticatorFactory;
    factory->user = user;
    factory->pass = password;

    app.session()->setClientAuthenticatorFactory(
      qi::ClientAuthenticatorFactoryPtr(factory));
  }
#endif

  qi::Future<void> connection = app.session()->connect(url);
  
  if (connection.hasError()) {
    std::cout << BOLDRED << connection.error() <<  RESETCOLOR << std::endl;
    return 0;
  }
  
  // Register the ROS-Driver service and init the driver node
  app.session()->registerService("ROS-Driver", bs);

  // Pass the create session to the driver node, and init the node
  bs->setQiSession(app.session());
  bs->init();
  std::cout << BOLDYELLOW 
            << "naoqi_driver initialized" 
            << RESETCOLOR 
            << std::endl;
  
  // Run the qi::ApplicationSession
  app.run();
  
  // Stop the driver service, close the qi::Session and stop ROS
  bs->stopService();
  app.session()->close();
  rclcpp::shutdown();
  return 0;
}
