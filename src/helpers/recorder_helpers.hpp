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


#ifndef RECORDER_HELPERS_HPP
#define RECORDER_HELPERS_HPP

#include <builtin_interfaces/msg/time.hpp>

namespace naoqi
{
namespace helpers
{
namespace recorder
{

static const float bufferDefaultDuration = 10.f;

/**
 * @brief Checks if the time value contained in the builtin_interfaces time
 * message is set to zero or not
 * 
 * @param msg a builtin_interfaces time message
 * @return true if the time value of the message is 0 (0 seconds and 0
 * nanoseconds)
 * @return false if the time value of the message is not 0 
 */
static bool isZero(const builtin_interfaces::msg::Time& msg) {
  return msg.sec == 0 && msg.nanosec == 0;
}

} // recorder
} // helpers
} // naoqi

#endif
