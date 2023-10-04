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

#include "touch.hpp"

namespace naoqi
{

template<class T>
TouchEventRegister<T>::TouchEventRegister()
{
}

template<class T>
TouchEventRegister<T>::TouchEventRegister( const std::string& name, const std::vector<std::string> keys, const float& frequency, const qi::SessionPtr& session )
  : p_memory_( session->service("ALMemory").value()),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<T> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<T> >( name );
  converter_ = boost::make_shared<converter::TouchEventConverter<T> >( name, frequency, session );

  converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<T>::publish, publisher_, _1) );
  //converter_->registerCallback( message_actions::RECORD, boost::bind(&recorder::BasicEventRecorder<T>::write, recorder_, _1) );
  //converter_->registerCallback( message_actions::LOG, boost::bind(&recorder::BasicEventRecorder<T>::bufferize, recorder_, _1) );

  keys_.resize(keys.size());
  size_t i = 0;
  for(std::vector<std::string>::const_iterator it = keys.begin(); it != keys.end(); ++it, ++i)
    keys_[i] = *it;

  name_ = name;
}

template<class T>
TouchEventRegister<T>::~TouchEventRegister()
{
  stopProcess();
}

template<class T>
void TouchEventRegister<T>::resetPublisher(rclcpp::Node* node)
{
  publisher_->reset(node);
}

template<class T>
void TouchEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

template<class T>
void TouchEventRegister<T>::startProcess()
{
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if (subscriptions_.empty())
    {
      for (const auto& key : keys_) {
        auto subscriber = p_memory_.call<qi::AnyObject>("subscriber", key);
        qi::SignalLink subscription = subscriber.connect("signal", [=](const qi::AnyValue& v){ touchCallback(key, v); }).value();
        subscriptions_.emplace_back(std::move(subscriber), std::move(subscription));
      }
    }
    isStarted_ = true;
  }
}

template<class T>
void TouchEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    if (!subscriptions_.empty()){
      for (const auto& subscription : subscriptions_) {
        try {
          subscription.subscriber.disconnect(subscription.link).value();
        } catch (const std::exception& e) {
          std::cerr << "Error attempting to clean-up ALMemory subscription: " << e.what() << std::endl;
        }
      }
      subscriptions_.clear();
    }
    isStarted_ = false;
  }
}

template<class T>
void TouchEventRegister<T>::writeDump(const rclcpp::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

template<class T>
void TouchEventRegister<T>::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

template<class T>
void TouchEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void TouchEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void TouchEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void TouchEventRegister<T>::registerCallback()
{
}

template<class T>
void TouchEventRegister<T>::unregisterCallback()
{
}

template<class T>
void TouchEventRegister<T>::touchCallback(const std::string &key, const qi::AnyValue& value)
{
  T msg = T();
  bool state =  value.toFloat() > 0.5f;

  touchCallbackMessage(key, state, msg);

  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);
  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      //actions.push_back(message_actions::RECORD);
    }
    if ( !isDumping_ )
    {
      //actions.push_back(message_actions::LOG);
    }
    if (actions.size() >0)
    {
      converter_->callAll( actions, msg );
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(const std::string &key, bool &state, naoqi_bridge_msgs::msg::Bumper &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.bumper = i;
      msg.state = state?(naoqi_bridge_msgs::msg::Bumper::STATE_PRESSED):(naoqi_bridge_msgs::msg::Bumper::STATE_RELEASED);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(const std::string &key, bool &state, naoqi_bridge_msgs::msg::HandTouch &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.hand = i;
      msg.state = state?(naoqi_bridge_msgs::msg::HandTouch::STATE_PRESSED):(naoqi_bridge_msgs::msg::HandTouch::STATE_RELEASED);
    }
  }
}

template<class T>
void TouchEventRegister<T>::touchCallbackMessage(const std::string &key, bool &state, naoqi_bridge_msgs::msg::HeadTouch &msg)
{
  int i = 0;
  for(std::vector<std::string>::const_iterator it = keys_.begin(); it != keys_.end(); ++it, ++i)
  {
    if ( key == it->c_str() ) {
      msg.button = i;
      msg.state = state?(naoqi_bridge_msgs::msg::HeadTouch::STATE_PRESSED):(naoqi_bridge_msgs::msg::HeadTouch::STATE_RELEASED);
    }
  }
}

// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class TouchEventRegister<naoqi_bridge_msgs::msg::Bumper>;
template class TouchEventRegister<naoqi_bridge_msgs::msg::HandTouch>;
template class TouchEventRegister<naoqi_bridge_msgs::msg::HeadTouch>;

}//namespace
