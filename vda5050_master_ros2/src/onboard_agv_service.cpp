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

#include "vda5050_master_ros2/onboard_agv_service.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"

namespace vda5050_master_ros2 {
namespace {

// Master-side default queue size for an onboarded AGV. Keep in sync
// with VDA5050Master::onboard_agv default in master.hpp.
constexpr std::size_t kDefaultMaxQueueSize = 10;

}  // namespace

std::string OnboardAGVService::make_service_name(
  const std::string& topic_namespace)
{
  std::string name = "/";
  if (!topic_namespace.empty())
  {
    name += topic_namespace;
    name += "/";
  }
  name += kServiceLeaf;
  return name;
}

OnboardAGVService::OnboardAGVService(
  rclcpp::Node::SharedPtr node, OnboardHandler handler,
  const std::string& topic_namespace)
: node_(std::move(node)),
  handler_(std::move(handler)),
  service_name_(make_service_name(topic_namespace))
{
  service_ = node_->create_service<OnboardAGV>(
    service_name_, [this](
                     const std::shared_ptr<OnboardAGV::Request> request,
                     std::shared_ptr<OnboardAGV::Response> response) {
      this->handle_request(request, response);
    });

  VDA5050_INFO("[OnboardAGVService] advertised service on {}", service_name_);
}

void OnboardAGVService::handle_request(
  const std::shared_ptr<OnboardAGV::Request> request,
  std::shared_ptr<OnboardAGV::Response> response)
{
  // Echo identity for FMS-side correlation.
  response->manufacturer = request->manufacturer;
  response->serial_number = request->serial_number;

  if (request->manufacturer.empty() || request->serial_number.empty())
  {
    response->status = OnboardAGV::Response::INVALID_REQUEST;
    return;
  }

  // Coerce 0 sentinel to the C++ default to avoid the lambda having
  // to know about the master-side default (keeps the service-master
  // boundary clean).
  const std::size_t qs = request->max_queue_size == 0
                           ? kDefaultMaxQueueSize
                           : static_cast<std::size_t>(request->max_queue_size);

  OnboardOutcome outcome = handler_(
    request->manufacturer, request->serial_number, qs, request->drop_oldest);

  switch (outcome.decision)
  {
    case OnboardOutcome::ONBOARDED:
      response->status = OnboardAGV::Response::SUCCESS;
      break;
    case OnboardOutcome::ALREADY_ONBOARDED:
      response->status = OnboardAGV::Response::ALREADY_ONBOARDED;
      break;
  }
}

}  // namespace vda5050_master_ros2
