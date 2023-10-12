#include <rclcpp_action/rclcpp_action.hpp>
#include <qi/session.hpp>
#include <naoqi_bridge_msgs/action/listen.hpp>

namespace naoqi
{
namespace action {

rclcpp_action::Server<naoqi_bridge_msgs::action::Listen>::SharedPtr
createListenServer(rclcpp::Node* node, qi::SessionPtr sesssion);

} // ends namespace action
} // ends namespace naoqi