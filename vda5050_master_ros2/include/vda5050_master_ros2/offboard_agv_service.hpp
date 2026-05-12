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

#ifndef VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_
#define VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_

#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "vda5050_master_ros2/srv/offboard_agv.hpp"

namespace vda5050_master_ros2 {
// =============================================================================
// OffboardAGVService (Task #54).
// =============================================================================
//
// Synchronous ROS 2 service that lets an FMS deregister an AGV from
// the master. Wraps `VDA5050Master::offboard_agv(mfg, serial)` plus an
// `is_agv_onboarded()` pre-check to derive the idempotent
// NOT_ONBOARDED response status (the C++ method returns void).
//
// Service name: /<topic_namespace>/offboard_agv
//   Default <topic_namespace> = "vda5050_master".
//
// **Idempotent**: offboarding a non-onboarded AGV returns NOT_ONBOARDED
// without raising. FMS clients can call this safely without first
// checking onboarded state.
//
// **OffboardHandler callable**: takes `std::function` (not raw
// `Master*`) to keep the service unit-testable without a full Master
// instance. VDA5050MasterROS2 wraps `is_agv_onboarded` +
// `offboard_agv` via a single lambda that returns the OffboardOutcome
// decision based on the pre-check result.

class OffboardAGVService
{
public:
  /// Service name leaf — appended after the namespace prefix.
  static constexpr const char* kServiceLeaf = "offboard_agv";

  /// Decision the handler returned. The handler does the
  /// is_agv_onboarded pre-check internally and reports back which
  /// branch it took.
  struct OffboardOutcome
  {
    enum Decision
    {
      OFFBOARDED,  ///< Handler successfully offboarded the vda5050_core::master::AGV.
      NOT_ONBOARDED  ///< vda5050_core::master::AGV was not in the master's onboard list.
    };
    Decision decision;
  };

  /// Offboard handler callable. Typically wraps
  /// VDA5050Master::is_agv_onboarded + offboard_agv.
  using OffboardHandler = std::function<OffboardOutcome(
    const std::string& manufacturer, const std::string& serial_number)>;

  /// \brief Construct.
  /// \param node            ROS 2 node hosting the service. Must outlive
  ///                        this object.
  /// \param handler         Offboard handler callable.
  /// \param topic_namespace Prefix for the service name. Defaults to
  ///                        "vda5050_master".
  OffboardAGVService(
    rclcpp::Node::SharedPtr node, OffboardHandler handler,
    const std::string& topic_namespace = "vda5050_master");

  ~OffboardAGVService() = default;
  OffboardAGVService(const OffboardAGVService&) = delete;
  OffboardAGVService& operator=(const OffboardAGVService&) = delete;
  OffboardAGVService(OffboardAGVService&&) = delete;
  OffboardAGVService& operator=(OffboardAGVService&&) = delete;

  /// \brief The fully-qualified ROS 2 service name.
  const std::string& service_name() const
  {
    return service_name_;
  }

private:
  using OffboardAGV = vda5050_master_ros2::srv::OffboardAGV;

  void handle_request(
    const std::shared_ptr<OffboardAGV::Request> request,
    std::shared_ptr<OffboardAGV::Response> response);

  static std::string make_service_name(const std::string& topic_namespace);

  rclcpp::Node::SharedPtr node_;
  OffboardHandler handler_;
  std::string service_name_;
  rclcpp::Service<OffboardAGV>::SharedPtr service_;
};

}  // namespace vda5050_master_ros2

#endif  // VDA5050_MASTER_ROS2__OFFBOARD_AGV_SERVICE_HPP_
