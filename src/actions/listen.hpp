#include <naoqi_bridge_msgs/action/listen.hpp>
#include <qi/session.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

namespace naoqi
{
namespace action
{

rclcpp_action::Server<naoqi_bridge_msgs::action::Listen>::SharedPtr
createListenServer(rclcpp::Node* node, qi::SessionPtr sesssion);

}  // namespace action
}  // namespace naoqi