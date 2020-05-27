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

#ifndef RECORDER_HPP
#define RECORDER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
# include <boost/thread/mutex.hpp>

#include <rclcpp/rclcpp.hpp>
// #include <rosbag/bag.h>
// #include <rosbag/view.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "naoqi_driver/recorder/globalrecorder.hpp"

namespace naoqi
{
namespace recorder
{

/**
* @brief Recorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible recorder instance has to implement the virtual functions mentioned in the concept
*/
class Recorder
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  template<typename T>
  Recorder( T rec ):
    recPtr_( boost::make_shared<RecorderModel<T> >(rec) )
  {}

  /**
  * @brief checks if the recorder is correctly initialized
  @ @return bool value indicating true for success
  */
  bool isInitialized() const
  {
    return recPtr_->isInitialized();
  }


  void subscribe( bool state )
  {
    recPtr_->subscribe(state);
  }

  /**
  * @brief checks if the recorder has a subscription and is hence allowed to record
  * @return bool value indicating true for number of sub > 0
  */
  bool isSubscribed() const
  {
    return recPtr_->isSubscribed();
  }

  std::string topic() const
  {
    return recPtr_->topic();
  }

  /**
   * @brief initializes/resets the recorder into ROS with a given 
   * GlobalRecorder pointer and a frequency, this will be called at first for
   * initialization
   * 
   * @param gr GlobalRecorder pointer
   * @param frequency the frequency of the recoder
   */
  void reset( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr, float frequency)
  {
    recPtr_->reset( gr, frequency );
  }

  void writeDump(const rclcpp::Time& time)
  {
    recPtr_->writeDump(time);
  }

  void setBufferDuration(float duration)
  {
    recPtr_->setBufferDuration(duration);
  }

  friend bool operator==( const Recorder& lhs, const Recorder& rhs )
  {
    // decision made for OR-comparison since we want to be more restrictive
    if ( lhs.topic() == rhs.topic() )
      return true;
    return false;
  }

private:

  /**
  * BASE concept struct
  */
  struct RecorderConcept
  {
    virtual ~RecorderConcept(){}
    virtual bool isInitialized() const = 0;
    virtual void subscribe(bool state) = 0;
    virtual bool isSubscribed() const = 0;
    virtual std::string topic() const = 0;
    virtual void writeDump(const rclcpp::Time& time) = 0;
    virtual void setBufferDuration(float duration) = 0;
    virtual void reset( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr, float frequency ) = 0;
  };


  /**
  * templated instances of base concept
  */
  template<typename T>
  struct RecorderModel : public RecorderConcept
  {
    RecorderModel( const T& other ):
      recorder_( other )
    {}

    void reset( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr, float frequency )
    {
      recorder_->reset( gr, frequency );
    }

    bool isInitialized() const
    {
      return recorder_->isInitialized();
    }

    void subscribe(bool state)
    {
      recorder_->subscribe( state );
    }

    bool isSubscribed() const
    {
      return recorder_->isSubscribed();
    }

    std::string topic() const
    {
      return recorder_->topic();
    }

    void writeDump(const rclcpp::Time& time)
    {
      recorder_->writeDump(time);
    }

    void setBufferDuration(float duration)
    {
      recorder_->setBufferDuration(duration);
    }

    T recorder_;
  };

  boost::shared_ptr<RecorderConcept> recPtr_;

}; // class recorder

} // recorder
} //naoqi

#endif
