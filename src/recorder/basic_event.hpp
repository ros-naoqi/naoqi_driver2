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

#ifndef BASIC_EVENT_RECORDER_HPP
#define BASIC_EVENT_RECORDER_HPP

/*
* LOCAL includes
*/
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/ros_helpers.hpp>
#include "../helpers/recorder_helpers.hpp"

/*
* STANDARD includes
*/
#include <string>

namespace naoqi
{
namespace recorder
{

template<class T>
class BasicEventRecorder
{

public:
  BasicEventRecorder( const std::string& topic ):
    topic_( topic ),
    buffer_duration_( helpers::recorder::bufferDefaultDuration ),
    is_initialized_( false ),
    is_subscribed_( false )
  {}

  virtual ~BasicEventRecorder() {}

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline void subscribe( bool state)
  {
    is_subscribed_ = state;
  }

  inline bool isSubscribed() const
  {
    return is_subscribed_;
  }

  virtual void write(const T& msg)
  {
    if (!helpers::recorder::isZero(msg.header.stamp)) {
      gr_->write(topic_, msg, msg.header.stamp);
    }
    else {
      gr_->write(topic_, msg);
    }
  }

  virtual void writeDump(const rclcpp::Time& time)
  {
    boost::mutex::scoped_lock lock_write_buffer( mutex_ );
    removeOlderThan(time);
    typename std::list<T>::iterator it;
    for (it = buffer_.begin(); it != buffer_.end(); it++)
    {
      if (!helpers::recorder::isZero(it->header.stamp)) {
        gr_->write(topic_, *it, it->header.stamp);
      }
      else {
        gr_->write(topic_, *it);
      }
    }
  }

  virtual void bufferize(const T& msg)
  {
    boost::mutex::scoped_lock lock_bufferize( mutex_ );
    typename std::list<T>::iterator it;
    removeOld();
    buffer_.push_back(msg);

  }

  virtual void reset(boost::shared_ptr<GlobalRecorder> gr, float conv_frequency)
  {
    gr_ = gr;
    is_initialized_ = true;
  }

  virtual void setBufferDuration(float duration)
  {
    boost::mutex::scoped_lock lock_bufferize( mutex_ );
    buffer_duration_ = duration;
  }

protected:
  bool isTooOld(const T& msg)
  {
    rclcpp::Duration d((helpers::Time::now() - msg.header.stamp).nanoseconds());
    if (static_cast<float>(d.seconds()) > buffer_duration_)
    {
      return true;
    }
    return false;
  }

  bool isOlderThan(const T& msg, const rclcpp::Time& time)
  {
    rclcpp::Duration d((time - msg.header.stamp).nanoseconds());
    if (static_cast<float>(d.seconds()) > buffer_duration_)
    {
      return true;
    }
    return false;
  }

  void removeOld()
  {
    while (buffer_.size() > 0 && isTooOld(buffer_.front()))
    {
      buffer_.pop_front();
    }
  }

  void removeOlderThan(const rclcpp::Time& time)
  {
    while (buffer_.size() > 0 && isOlderThan(buffer_.front(), time))
    {
      buffer_.pop_front();
    }
  }

  std::string topic_;

  std::list<T> buffer_;
  float buffer_duration_;
  boost::mutex mutex_;

  bool is_initialized_;
  bool is_subscribed_;

  boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr_;
}; // class

} // recorder
} // naoqi

#endif
