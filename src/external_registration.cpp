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

#include <signal.h>
#include <memory>

#include <qi/anymodule.hpp>
#include <qi/applicationsession.hpp>
#include <qi/session.hpp>
#include <rclcpp/rclcpp.hpp>

#if LIBQI_VERSION >= 29
#include "driver_authenticator.hpp"
#endif

#include <naoqi_driver/naoqi_driver.hpp>

#include "fake_naoqi/fake_naoqi.hpp"
#include "naoqi_driver/ros_helpers.hpp"
#include "naoqi_driver/tools.hpp"

boost::weak_ptr<naoqi::Driver> driver_weak;

void sigint_handler(int sig)
{
  if (auto driver = driver_weak.lock())
  {
    driver->stop();
  }
}

int main(int argc, char** argv)
{
  const std::string no_password = "no_password";
  std::string protocol = "tcp://";

  std::string nao_ip;
  int nao_port;
  std::string user;
  std::string password;
  std::string network_interface;
  std::string listen_url;
  bool emulation_mode;
  std::string robot_type;

  // Initialize ROS and create the driver Node
  rclcpp::init(argc, argv);
  auto bs = boost::make_shared<naoqi::Driver>();

  // Setup the time helper
  naoqi::helpers::Node::setNode(bs);

  // Retrieve the parameters
  bs->declare_parameter<std::string>("nao_ip", "127.0.0.1");
  bs->declare_parameter<int>("nao_port", 9559);
  bs->declare_parameter<std::string>("user", "nao");
  bs->declare_parameter<std::string>("password", no_password);
  bs->declare_parameter<std::string>("network_interface", "eth0");
  bs->declare_parameter<std::string>("qi_listen_url", "tcp://0.0.0.0:0");
  bs->declare_parameter<bool>("emulation_mode", false);
  bs->declare_parameter<std::string>("robot_type", "nao");

  bs->get_parameter("nao_ip", nao_ip);
  bs->get_parameter("nao_port", nao_port);
  bs->get_parameter("user", user);
  bs->get_parameter("password", password);
  bs->get_parameter("network_interface", network_interface);
  bs->get_parameter("qi_listen_url", listen_url);
  bs->get_parameter("emulation_mode", emulation_mode);
  bs->get_parameter("robot_type", robot_type);

  qi::SessionPtr session;
  std::unique_ptr<qi::ApplicationSession> app;

  if (emulation_mode)
  {
    // Emulation mode: create fake NAOqi session
    std::cout << BOLDCYAN << "Starting in emulation mode (fake NAOqi)" << RESETCOLOR << std::endl;
    session = naoqi::fake::createFakeNaoqiSession(robot_type);
  }
  else
  {
    // Normal mode: connect to real robot
    if (password.compare(no_password) != 0)
    {
#if LIBQI_VERSION >= 29
      protocol = "tcps://";
      nao_port = (nao_port == 9559) ? 9503 : nao_port;
#else
      std::cout << BOLDRED << "No need to set a password, ignored." << RESETCOLOR << std::endl;
#endif
    }

    // Build the q::ApplicationSession, and connect the associated session
    qi::Url url(protocol + nao_ip + ":" + std::to_string(nao_port));
    app = std::make_unique<qi::ApplicationSession>(argc, argv);

#if LIBQI_VERSION >= 29
    if (password.compare(no_password) != 0)
    {
      naoqi::DriverAuthenticatorFactory* factory = new naoqi::DriverAuthenticatorFactory;
      factory->user = user;
      factory->pass = password;

      app->session()->setClientAuthenticatorFactory(qi::ClientAuthenticatorFactoryPtr(factory));
    }
#endif

    session = app->session();
    qi::Future<void> connection = session->connect(url);

    if (connection.hasError())
    {
      std::cout << BOLDRED << connection.error() << RESETCOLOR << std::endl;
      return EXIT_FAILURE;
    }
    // The session needs to listen on the network to process audio callbacks.
    if (!listen_url.empty())
      session->listen(listen_url);
  }

  // Pass the created session to the driver node, and init the node
  bs->setQiSession(session);

  // Run the driver node. Stop it on SIGINT.
  driver_weak = bs;
  signal(SIGINT, sigint_handler);
  bs->run();
  driver_weak.reset();

  // Close the qi::Session and shutdown ROS.
  if (app)
  {
    app->session()->close();
  }
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
