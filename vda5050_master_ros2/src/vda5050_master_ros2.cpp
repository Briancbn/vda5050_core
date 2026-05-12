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

#include "vda5050_master_ros2/vda5050_master_ros2.hpp"

#include <memory>
#include <string>
#include <utility>

#include "vda5050_master_ros2/order_status_builder.hpp"

namespace vda5050_master_ros2 {
VDA5050MasterROS2::VDA5050MasterROS2(
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client,
  rclcpp::Node::SharedPtr ros2_node, const std::string& topic_namespace)
: vda5050_core::master::VDA5050Master(std::move(mqtt_client)),
  device_status_(
    std::make_unique<DeviceStatusPublisher>(ros2_node, topic_namespace)),
  device_status_service_(std::make_unique<DeviceStatusService>(
    ros2_node,
    [this](const std::string& mfg, const std::string& serial) {
      return this->get_agv(mfg, serial);
    },
    topic_namespace)),
  order_status_publisher_(
    std::make_unique<OrderStatusPublisher>(ros2_node, topic_namespace)),
  order_status_service_(std::make_unique<OrderStatusService>(
    ros2_node,
    [this](const std::string& mfg, const std::string& serial) {
      return this->get_agv(mfg, serial);
    },
    topic_namespace)),
  order_send_service_(std::make_unique<OrderSendService>(
    ros2_node,
    [this](
      const std::string& mfg, const std::string& serial,
      const vda5050_core::types::Order& order) {
      return this->assign_order(mfg, serial, order);
    },
    topic_namespace)),
  instant_actions_send_service_(std::make_unique<InstantActionsSendService>(
    ros2_node,
    [this](
      const std::string& mfg, const std::string& serial,
      const vda5050_core::types::InstantActions& actions) {
      return this->assign_instant_actions(mfg, serial, actions);
    },
    topic_namespace)),
  onboard_agv_service_(std::make_unique<OnboardAGVService>(
    ros2_node,
    [this](
      const std::string& mfg, const std::string& serial, std::size_t qs,
      bool drop) -> OnboardAGVService::OnboardOutcome {
      if (this->is_agv_onboarded(mfg, serial))
      {
        return {OnboardAGVService::OnboardOutcome::ALREADY_ONBOARDED};
      }
      this->onboard_agv(mfg, serial, qs, drop);
      return {OnboardAGVService::OnboardOutcome::ONBOARDED};
    },
    topic_namespace)),
  offboard_agv_service_(std::make_unique<OffboardAGVService>(
    ros2_node,
    [this](const std::string& mfg, const std::string& serial)
      -> OffboardAGVService::OffboardOutcome {
      if (!this->is_agv_onboarded(mfg, serial))
      {
        return {OffboardAGVService::OffboardOutcome::NOT_ONBOARDED};
      }
      this->offboard_agv(mfg, serial);
      return {OffboardAGVService::OffboardOutcome::OFFBOARDED};
    },
    topic_namespace)),
  get_loaded_map_service_(std::make_unique<GetLoadedMapService>(
    ros2_node, [this]() { return this->get_loaded_map(); },
    [this]() { return this->get_alignment_cache_snapshot(); },
    topic_namespace)),
  resume_mode_cancelled_queue_service_(
    std::make_unique<ResumeModeCancelledQueueService>(
      ros2_node,
      [this](const std::string& mfg, const std::string& serial)
        -> std::optional<std::pair<std::size_t, std::size_t>> {
        auto agv = this->get_agv(mfg, serial);
        if (!agv) return std::nullopt;
        return agv->resume_mode_cancelled_queue();
      },
      topic_namespace)),
  discard_mode_cancelled_queue_service_(
    std::make_unique<DiscardModeCancelledQueueService>(
      ros2_node,
      [this](const std::string& mfg, const std::string& serial)
        -> std::optional<std::pair<std::size_t, std::size_t>> {
        auto agv = this->get_agv(mfg, serial);
        if (!agv) return std::nullopt;
        return agv->discard_mode_cancelled_queue();
      },
      topic_namespace)),
  get_master_broker_status_service_(
    std::make_unique<GetMasterBrokerStatusService>(
      std::move(ros2_node),
      [this]() {
        const auto snap = this->get_broker_status();
        GetMasterBrokerStatusService::StatusSnapshot out;
        out.connected = snap.connected;
        out.last_disconnect_at = snap.last_disconnect_at;
        out.reconnect_count = snap.reconnect_count;
        return out;
      },
      topic_namespace))
{
}

std::pair<std::string, std::string> VDA5050MasterROS2::split_agv_id(
  const std::string& agv_id)
{
  // Format per master.cpp:228 is "{manufacturer}/{serial_number}".
  const auto slash = agv_id.find('/');
  if (slash == std::string::npos)
  {
    // Defensive: degraded form. Treat whole id as manufacturer with
    // empty serial. Will produce a malformed topic but won't crash.
    return {agv_id, std::string{}};
  }
  return {agv_id.substr(0, slash), agv_id.substr(slash + 1)};
}

void VDA5050MasterROS2::on_state(
  const std::string& agv_id, const vda5050_core::types::State& state)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_state(mfg, serial, state);

  // Publish OrderStatus alongside the device-status State stream. The
  // bundle's State is the one just cached by handle_state (which runs
  // before this on_state callback), so the bundle is fresh by
  // construction.
  if (auto agv = get_agv(mfg, serial))
  {
    auto bundle = agv->get_order_status_bundle();
    auto msg = build_order_status_msg(bundle, mfg, serial);
    order_status_publisher_->publish_order_status(mfg, serial, msg);
  }

  vda5050_core::master::VDA5050Master::on_state(agv_id, state);
}

void VDA5050MasterROS2::on_connection(
  const std::string& agv_id, const vda5050_core::types::Connection& connection)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_connection(mfg, serial, connection);
  vda5050_core::master::VDA5050Master::on_connection(agv_id, connection);
}

void VDA5050MasterROS2::on_factsheet(
  const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_factsheet(mfg, serial, factsheet);
  vda5050_core::master::VDA5050Master::on_factsheet(agv_id, factsheet);
}

}  // namespace vda5050_master_ros2
