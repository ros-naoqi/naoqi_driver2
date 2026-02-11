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
#include "joint_state.hpp"
#include "nao_footprint.hpp"

/*
* ROS includes
*/
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace naoqi
{

namespace converter
{

JointStateConverter::JointStateConverter( const std::string& name, const float& frequency, const BufferPtr& tf2_buffer, const qi::SessionPtr& session ):
  BaseConverter( name, frequency, session ),
  p_motion_( session->service("ALMotion").value() ),
  p_memory_( session->service("ALMemory").value() ),
  tf2_buffer_(tf2_buffer)
{
}

JointStateConverter::~JointStateConverter()
{
  // clear all sessions
}

void JointStateConverter::reset()
{
  std::string robot_desc = naoqi::tools::getRobotDescription(robot_);
  if ( robot_desc.empty() )
  {
    std::cout << "error in loading robot description" << std::endl;
    return;
  }

  urdf::Model model;
  model.initString(robot_desc);
  KDL::Tree tree;
  kdl_parser::treeFromUrdfModel( model, tree );

  addChildren( tree.getRootSegment() );

  // set mimic joint list
  mimic_.clear();
  for(std::map< std::string, urdf::JointSharedPtr >::iterator i = model.joints_.begin(); i != model.joints_.end(); i++){
    if(i->second->mimic){
      mimic_.insert(make_pair(i->first, i->second->mimic));
    }
  }
  // pre-fill joint states message

  // precompute ALMemory keys: [positions..., velocities..., torques...]
  const size_t n = msg_joint_states_.name.size();
  memory_keys_.clear();
  memory_keys_.reserve(3 * n);
  for (const auto& name : msg_joint_states_.name)
  {
    memory_keys_.push_back("Device/SubDeviceList/" + name + "/Position/Sensor/Value");
  }
  for (const auto& name : msg_joint_states_.name)
  {
    memory_keys_.push_back("Motion/Velocity/Sensor/" + name);
  }
  for (const auto& name : msg_joint_states_.name)
  {
    memory_keys_.push_back("Motion/Torque/Sensor/" + name);
  }
}

void JointStateConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

namespace
{
constexpr int FRAME_TORSO = 0;
constexpr int FRAME_WORLD = 1;
constexpr int FRAME_ROBOT = 2;
}  // namespace

void JointStateConverter::callAll(const std::vector<message_actions::MessageAction>& actions)
{
  auto step_time = helpers::Time::now();
  auto lap = [&step_time](const char* step) {
    auto now = helpers::Time::now();
    RCLCPP_DEBUG(helpers::Node::get_logger(), "JointState::%s took %ld ns",
                step, now.nanoseconds() - step_time.nanoseconds());
    step_time = now;
  };

  /*
   * get torso position for odometry at the same time as joint states
   * can be called via getRobotPosture
   * but this would require a proper URDF
   * with a base_link and base_footprint in the base
   */
  auto getting_odometry_data =
      p_motion_.async<std::vector<float>>("getPosition", "Torso", FRAME_WORLD, true);

  // Batch-fetch positions, velocities and torques in one call
  auto getting_memory_data =
      p_memory_.async<std::vector<qi::AnyValue>>("getListData", memory_keys_);

  // Take the timestamp right after launching the async calls
  const rclcpp::Time& stamp = helpers::Time::now();

  std::vector<float> al_odometry_data = getting_odometry_data.value();
  const size_t n_joints = msg_joint_states_.name.size();
  std::vector<qi::AnyValue> memory_data = getting_memory_data.value();
  lap("getData(pos,vel,torque)");

  // Conversion to double. For some joints like hands, the velocity and torque values do not exist, so a void is returned.
  auto to_double = [](const qi::AnyValue& v) {
    if (v.kind() == qi::TypeKind_Void) {
      return 0.0;
    }
    return v.toDouble();
  };

  // Layout: [0..N) = positions, [N..2N) = velocities, [2N..3N) = torques
  msg_joint_states_.position.resize(n_joints);
  std::transform(memory_data.begin(), memory_data.begin() + n_joints,
                 msg_joint_states_.position.begin(), to_double);

  msg_joint_states_.velocity.resize(n_joints);
  std::transform(memory_data.begin() + n_joints, memory_data.begin() + 2 * n_joints,
                 msg_joint_states_.velocity.begin(), to_double);

  msg_joint_states_.effort.resize(n_joints);
  std::transform(memory_data.begin() + 2 * n_joints, memory_data.begin() + 3 * n_joints,
                 msg_joint_states_.effort.begin(), to_double);

  /**
   * JOINT STATE PUBLISHER
   */
  msg_joint_states_.header.stamp = stamp;

  /**
   * ROBOT STATE PUBLISHER
   */
  // put joint states in tf broadcaster
  std::map<std::string, double> joint_state_map;
  for (size_t i = 0; i < n_joints; ++i)
  {
    joint_state_map[msg_joint_states_.name[i]] = msg_joint_states_.position[i];
  }
  lap("parseData(pos,vel,torque)");

  // for mimic map
  for(MimicMap::iterator i = mimic_.begin(); i != mimic_.end(); i++){
    if(joint_state_map.find(i->second->joint_name) != joint_state_map.end()){
      double pos = joint_state_map[i->second->joint_name] * i->second->multiplier + i->second->offset;
      joint_state_map[i->first] = pos;
    }
  }

  // reset the transforms we want to use at this time
  tf_transforms_.clear();
  static const std::string& jt_tf_prefix = "";
  setTransforms(joint_state_map, stamp, jt_tf_prefix);
  setFixedTransforms(jt_tf_prefix, stamp);

  /**
   * ODOMETRY
   */
  const rclcpp::Time &odom_stamp = stamp;
  const float &odomX = al_odometry_data[0];
  const float& odomY  =  al_odometry_data[1];
  const float& odomZ  =  al_odometry_data[2];
  const float& odomWX =  al_odometry_data[3];
  const float& odomWY =  al_odometry_data[4];
  const float& odomWZ =  al_odometry_data[5];
  //since all odometry is 6DOF we'll need a quaternion created from yaw
  tf2::Quaternion tf_quat;
  tf_quat.setRPY( odomWX, odomWY, odomWZ );
  geometry_msgs::msg::Quaternion odom_quat = tf2::toMsg( tf_quat );

  static geometry_msgs::msg::TransformStamped msg_tf_odom;
  msg_tf_odom.header.frame_id = "odom";
  msg_tf_odom.child_frame_id = "base_link";
  msg_tf_odom.header.stamp = odom_stamp;

  msg_tf_odom.transform.translation.x = odomX;
  msg_tf_odom.transform.translation.y = odomY;
  msg_tf_odom.transform.translation.z = odomZ;
  msg_tf_odom.transform.rotation = odom_quat;

  tf_transforms_.push_back( msg_tf_odom );
  tf2_buffer_->setTransform( msg_tf_odom, "naoqiconverter", false);

  if (robot_ == robot::NAO )
  {
    nao::addBaseFootprint( tf2_buffer_, tf_transforms_, odom_stamp - tf2::durationFromSec(0.1) );
  }

  // If nobody uses that buffer, do not fill it next time
  if (( tf2_buffer_ ) && ( tf2_buffer_.use_count() == 1 ))
  {
    tf2_buffer_.reset();
  }
  lap("odometry");

  for( message_actions::MessageAction action: actions )
  {
    callbacks_[action]( msg_joint_states_, tf_transforms_ );
  }
  lap("callbacks");
}


// Copied from robot state publisher
void JointStateConverter::setTransforms(const std::map<std::string, double>& joint_positions, const rclcpp::Time& time, const std::string& tf_prefix)
{
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;

  // loop over all joints
  for (std::map<std::string, double>::const_iterator jnt=joint_positions.begin(); jnt != joint_positions.end(); jnt++){
    std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg = segments_.find(jnt->first);
    if (seg != segments_.end()){
      seg->second.segment.pose(jnt->second).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                            tf_transform.transform.rotation.y,
                                                            tf_transform.transform.rotation.z,
                                                            tf_transform.transform.rotation.w);
      tf_transform.transform.translation.x = seg->second.segment.pose(jnt->second).p.x();
      tf_transform.transform.translation.y = seg->second.segment.pose(jnt->second).p.y();
      tf_transform.transform.translation.z = seg->second.segment.pose(jnt->second).p.z();

      //tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
      //tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
      tf_transform.header.frame_id = seg->second.root; // tf2 does not suppport tf_prefixing
      tf_transform.child_frame_id = seg->second.tip;

      tf_transforms_.push_back(tf_transform);

      if (tf2_buffer_)
          tf2_buffer_->setTransform(tf_transform, "naoqiconverter", false);
    }
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

// Copied from robot state publisher
void JointStateConverter::setFixedTransforms(const std::string& tf_prefix, const rclcpp::Time& time)
{
  geometry_msgs::msg::TransformStamped tf_transform;
  tf_transform.header.stamp = time;

  // loop over all fixed segments
  for (std::map<std::string, robot_state_publisher::SegmentPair>::const_iterator seg=segments_fixed_.begin(); seg != segments_fixed_.end(); seg++){
    seg->second.segment.pose(0).M.GetQuaternion(tf_transform.transform.rotation.x,
                                                tf_transform.transform.rotation.y,
                                                tf_transform.transform.rotation.z,
                                                tf_transform.transform.rotation.w);
    tf_transform.transform.translation.x = seg->second.segment.pose(0).p.x();
    tf_transform.transform.translation.y = seg->second.segment.pose(0).p.y();
    tf_transform.transform.translation.z = seg->second.segment.pose(0).p.z();

    //tf_transform.header.frame_id = tf::resolve(tf_prefix, seg->second.root);
    //tf_transform.child_frame_id = tf::resolve(tf_prefix, seg->second.tip);
    tf_transform.header.frame_id = seg->second.root;
    tf_transform.child_frame_id = seg->second.tip;

    tf_transforms_.push_back(tf_transform);

    if (tf2_buffer_)
      tf2_buffer_->setTransform(tf_transform, "naoqiconverter", true);
  }
  //tf_broadcaster_.sendTransform(tf_transforms);
}

void JointStateConverter::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();

  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (unsigned int i=0; i<children.size(); i++){
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    robot_state_publisher::SegmentPair s(GetTreeElementSegment(children[i]->second), root, child.getName());
    if (child.getJoint().getType() == KDL::Joint::None){
      segments_fixed_.insert(std::make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(helpers::Node::get_logger(), "Adding fixed segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    else{
      segments_.insert(std::make_pair(child.getJoint().getName(), s));
      RCLCPP_DEBUG(helpers::Node::get_logger(), "Adding moving segment from %s to %s", root.c_str(), child.getName().c_str());
    }
    addChildren(children[i]);
  }
}

} //publisher
} // naoqi
