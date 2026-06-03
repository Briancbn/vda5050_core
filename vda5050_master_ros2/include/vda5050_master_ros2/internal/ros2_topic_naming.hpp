/*
 * Copyright (C) 2026 ROS-Industrial Consortium Asia Pacific
 * Advanced Remanufacturing and Technology Centre
 * A*STAR Research Entities (Co. Registration No. 199702110H)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_
#define VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_

#include <cctype>
#include <string>

namespace vda5050_master_ros2 {
namespace internal {

// ROS 2 topic name segments must match ^[A-Za-z_][A-Za-z0-9_]*$. Vendor-
// supplied manufacturer / serial_number strings (e.g. "001", "3M") may start
// with a digit and break that rule when spliced into a topic path.
//
// to_ros2_topic_segment() leaves a segment unchanged when its first character
// is already valid, and prepends '_' when it would be invalid. The
// transformation is intentionally minimal so that operators correlating ROS 2
// topics with VDA5050 wire identities can still recognise the original string.
inline std::string to_ros2_topic_segment(const std::string& s)
{
  if (s.empty())
  {
    return "_";
  }
  if (std::isdigit(static_cast<unsigned char>(s[0])))
  {
    return "_" + s;
  }
  return s;
}

// True when to_ros2_topic_segment(s) would alter s. Useful for one-shot
// logging without paying for the std::string allocation on the happy path.
inline bool needs_topic_sanitization(const std::string& s)
{
  return s.empty() || std::isdigit(static_cast<unsigned char>(s[0]));
}

}  // namespace internal
}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__INTERNAL__ROS2_TOPIC_NAMING_HPP_
