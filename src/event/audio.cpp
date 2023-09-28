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

#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>

#include <rclcpp/rclcpp.hpp>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

#include "audio.hpp"

namespace naoqi
{

AudioEventRegister::AudioEventRegister( const std::string& name, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_audio_( session->service("ALAudioDevice")),
    p_robot_model_(session->service("ALRobotModel")),
    session_(session),
    naoqi_version_(helpers::driver::getNaoqiVersion(session)),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  // _getMicrophoneConfig is used for NAOqi < 2.9, _getConfigMap for NAOqi > 2.9
  int micConfig;

  if (helpers::driver::isNaoqiVersionLesser(naoqi_version_, 2, 8))
  {
    micConfig = p_robot_model_.call<int>("_getMicrophoneConfig");
  }
  else
  {
    std::map<std::string, std::string> config_map =\
      p_robot_model_.call<std::map<std::string, std::string> >("_getConfigMap");

    micConfig = std::atoi(
      config_map["RobotConfig/Head/Device/Micro/Version"].c_str());
  }

  if(micConfig){
    channelMap.push_back(3);
    channelMap.push_back(5);
    channelMap.push_back(0);
    channelMap.push_back(2);
  }
  else{
    channelMap.push_back(0);
    channelMap.push_back(2);
    channelMap.push_back(1);
    channelMap.push_back(4);
  }
  publisher_ = boost::make_shared<publisher::BasicPublisher<naoqi_bridge_msgs::msg::AudioBuffer> >( name );
  recorder_ = boost::make_shared<recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::AudioBuffer> >( name );
  converter_ = boost::make_shared<converter::AudioEventConverter>( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::msg::AudioBuffer>::publish, publisher_, _1) );
  converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::AudioBuffer>::write, recorder_, _1) );
  converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<naoqi_bridge_msgs::msg::AudioBuffer>::bufferize, recorder_, _1) );

}

AudioEventRegister::~AudioEventRegister()
{
  stopProcess();
}

void AudioEventRegister::resetPublisher(rclcpp::Node* node)
{
  publisher_->reset(node);
}

void AudioEventRegister::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  recorder_->reset(gr, converter_->frequency());
}

void AudioEventRegister::startProcess()
{
  boost::mutex::scoped_lock start_lock(subscription_mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      serviceId = session_->registerService("ROS-Driver-Audio", shared_from_this()).value();
      p_audio_.call<void>(
              "setClientPreferences",
              "ROS-Driver-Audio",
              48000,
              0,
              0
              );
      p_audio_.call<void>("subscribe","ROS-Driver-Audio");
      std::cout << "Audio Extractor: Start" << std::endl;
    }
    isStarted_ = true;
  }
}

void AudioEventRegister::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(subscription_mutex_);
  if (isStarted_)
  {
    if(serviceId){
      p_audio_.call<void>("unsubscribe", "ROS-Driver-Audio");
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << "Audio Extractor: Stop" << std::endl;
    isStarted_ = false;
  }
}

void AudioEventRegister::writeDump(const rclcpp::Time& time)
{
  if (isStarted_)
  {
    recorder_->writeDump(time);
  }
}

void AudioEventRegister::setBufferDuration(float duration)
{
  recorder_->setBufferDuration(duration);
}

void AudioEventRegister::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(processing_mutex_);
  isRecording_ = state;
}

void AudioEventRegister::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(processing_mutex_);
  isPublishing_ = state;
}

void AudioEventRegister::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(processing_mutex_);
  isDumping_ = state;
}

void AudioEventRegister::registerCallback()
{
}

void AudioEventRegister::unregisterCallback()
{
}

void AudioEventRegister::processRemote(int nbOfChannels, int samplesByChannel, qi::AnyValue altimestamp, qi::AnyValue buffer)
{
  naoqi_bridge_msgs::msg::AudioBuffer msg = naoqi_bridge_msgs::msg::AudioBuffer();
  msg.header.stamp = helpers::Time::now();
  msg.frequency = 48000;
  msg.channel_map = channelMap;

  std::pair<char*, size_t> buffer_pointer = buffer.asRaw();

  int16_t* remoteBuffer = (int16_t*)buffer_pointer.first;
  int bufferSize = nbOfChannels * samplesByChannel;
  msg.data = std::vector<int16_t>(remoteBuffer, remoteBuffer+bufferSize);

  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(processing_mutex_);
  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      actions.push_back(message_actions::RECORD);
    }
    if ( !isDumping_ )
    {
      actions.push_back(message_actions::LOG);
    }
    if (actions.size() >0)
    {
      converter_->callAll( actions, msg );
    }
  }

}

}//namespace
