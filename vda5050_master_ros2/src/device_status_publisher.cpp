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

#include "vda5050_master_ros2/device_status_publisher.hpp"

#include <string>
#include <utility>

#include "vda5050_core/logger/logger.hpp"
#include "vda5050_master_ros2/internal/to_msg.hpp"

namespace vda5050_master_ros2 {
using internal::to_msg;

DeviceStatusPublisher::DeviceStatusPublisher(
  rclcpp::Node::SharedPtr node, const std::string& topic_namespace)
: node_(std::move(node)), namespace_(topic_namespace)
{
}

std::string DeviceStatusPublisher::agv_id(
  const std::string& manufacturer, const std::string& serial_number)
{
  return manufacturer + "/" + serial_number;
}

std::string DeviceStatusPublisher::make_topic(
  const std::string& manufacturer, const std::string& serial_number,
  const std::string& leaf) const
{
  std::string topic = "/";
  if (!namespace_.empty())
  {
    topic += namespace_;
    topic += "/";
  }
  topic += manufacturer;
  topic += "/";
  topic += serial_number;
  topic += "/";
  topic += leaf;
  return topic;
}

std::string DeviceStatusPublisher::state_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "state");
}

std::string DeviceStatusPublisher::connection_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "connection");
}

std::string DeviceStatusPublisher::factsheet_topic(
  const std::string& manufacturer, const std::string& serial_number) const
{
  return make_topic(manufacturer, serial_number, "factsheet");
}

bool DeviceStatusPublisher::has_publishers_for(
  const std::string& manufacturer, const std::string& serial_number) const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return publishers_.count(agv_id(manufacturer, serial_number)) != 0;
}

DeviceStatusPublisher::PerAgvPublishers&
DeviceStatusPublisher::ensure_publishers_locked(
  const std::string& manufacturer, const std::string& serial_number)
{
  const std::string id = agv_id(manufacturer, serial_number);
  auto it = publishers_.find(id);
  if (it != publishers_.end()) return it->second;

  PerAgvPublishers pubs;
  pubs.state = node_->create_publisher<vda5050_interfaces::msg::State>(
    state_topic(manufacturer, serial_number), kQosDepth);
  pubs.connection =
    node_->create_publisher<vda5050_interfaces::msg::Connection>(
      connection_topic(manufacturer, serial_number), kQosDepth);
  pubs.factsheet = node_->create_publisher<vda5050_interfaces::msg::Factsheet>(
    factsheet_topic(manufacturer, serial_number), kQosDepth);

  VDA5050_INFO(
    "[DeviceStatusPublisher] created publishers for {} on topics {} / {} / "
    "{}",
    id, state_topic(manufacturer, serial_number),
    connection_topic(manufacturer, serial_number),
    factsheet_topic(manufacturer, serial_number));

  return publishers_.emplace(id, std::move(pubs)).first->second;
}

void DeviceStatusPublisher::publish_state(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::State& state)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto& pubs = ensure_publishers_locked(manufacturer, serial_number);
  auto msg =
    to_msg<vda5050_core::types::State, vda5050_interfaces::msg::State>(state);
  pubs.state->publish(std::move(msg));
}

void DeviceStatusPublisher::publish_connection(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Connection& connection)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto& pubs = ensure_publishers_locked(manufacturer, serial_number);
  auto msg = to_msg<
    vda5050_core::types::Connection, vda5050_interfaces::msg::Connection>(
    connection);
  pubs.connection->publish(std::move(msg));
}

void DeviceStatusPublisher::publish_factsheet(
  const std::string& manufacturer, const std::string& serial_number,
  const vda5050_core::types::Factsheet& factsheet)
{
  std::lock_guard<std::mutex> lock(mutex_);
  auto& pubs = ensure_publishers_locked(manufacturer, serial_number);
  auto msg =
    to_msg<vda5050_core::types::Factsheet, vda5050_interfaces::msg::Factsheet>(
      factsheet);
  pubs.factsheet->publish(std::move(msg));
}

void DeviceStatusPublisher::remove_agv(
  const std::string& manufacturer, const std::string& serial_number)
{
  std::lock_guard<std::mutex> lock(mutex_);
  publishers_.erase(agv_id(manufacturer, serial_number));
}

}  // namespace vda5050_master_ros2
