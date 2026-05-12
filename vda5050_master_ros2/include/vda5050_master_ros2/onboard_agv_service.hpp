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

#ifndef VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_

#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/onboard_agv.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OnboardAGVService (Task #53).
// =============================================================================
//
// Synchronous ROS 2 service that lets an FMS register an AGV with the
// master. Wraps `VDA5050Master::onboard_agv(mfg, serial, max_queue_size,
// drop_oldest)` plus an `is_agv_onboarded()` pre-check to derive the
// idempotent ALREADY_ONBOARDED response status (the C++ method returns
// void).
//
// Service name: /<topic_namespace>/onboard_agv
//   Default <topic_namespace> = "vda5050_master".
//
// **Idempotent**: re-onboarding the same {manufacturer, serial_number}
// returns ALREADY_ONBOARDED without disturbing the existing AGV
// instance. FMS clients can call this safely on every startup.
//
// **OnboardHandler callable**: takes `std::function` (not raw
// `Master*`) to keep the service unit-testable without a full Master
// instance. VDA5050MasterROS2 wraps `is_agv_onboarded` +
// `onboard_agv` via a single lambda that returns the OnboardOutcome
// decision based on the pre-check result.

class OnboardAGVService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "onboard_agv";

  /// Decision the handler returned. The handler does the
  /// is_agv_onboarded pre-check internally and reports back which
  /// branch it took.
  struct OnboardOutcome
  {
    enum Decision
    {
      ONBOARDED,  ///< Handler successfully onboarded the vda5050_core::master::AGV.
      ALREADY_ONBOARDED  ///< vda5050_core::master::AGV was already in the master's onboard list.
    };
    Decision decision;
  };

  /// Onboard handler callable. Typically wraps
  /// VDA5050Master::is_agv_onboarded + onboard_agv.
  using OnboardHandler = std::function<OnboardOutcome(
    const std::string& manufacturer, const std::string& serial_number,
    std::size_t max_queue_size, bool drop_oldest)>;

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the service. Must outlive
  ///                        this object.
  /// \param handler         Onboard handler callable.
  /// \param topic_namespace Prefix for the service name. Defaults to
  ///                        "vda5050_master".
  OnboardAGVService(
    rclcpp::Node::SharedPtr node, OnboardHandler handler,
    const std::string& topic_namespace = "vda5050_master");

  ~OnboardAGVService() = default;
  OnboardAGVService(const OnboardAGVService&) = delete;
  OnboardAGVService& operator=(const OnboardAGVService&) = delete;
  OnboardAGVService(OnboardAGVService&&) = delete;
  OnboardAGVService& operator=(OnboardAGVService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OnboardAGV = vda5050_master_ros2::srv::OnboardAGV;

  void handle_request(
    const std::shared_ptr<OnboardAGV::Request> request,
    std::shared_ptr<OnboardAGV::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OnboardHandler handler_;
  std::string service_name_;
  rclcpp::Service<OnboardAGV>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__ONBOARD_AGV_SERVICE_HPP_
