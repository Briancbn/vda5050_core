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

#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>

#include "vda5050_master_ros2/msg/master_connection.hpp"
#include "vda5050_master_ros2/order_status_builder.hpp"

namespace vda5050_master_ros2 {
namespace {

// Build "hostname-pid" so multi-master deployments get unique
// master_ids without manual config.
std::string default_master_id()
{
  char host[256] = {0};
  if (::gethostname(host, sizeof(host) - 1) != 0)
  {
    host[0] = '\0';
  }
  std::string out = host[0] ? std::string(host) : std::string("master");
  out += "-";
  out += std::to_string(static_cast<std::int64_t>(::getpid()));
  return out;
}

}  // namespace

VDA5050MasterROS2::VDA5050MasterROS2(
  std::shared_ptr<vda5050_core::transport::MqttClientInterface> mqtt_client,
  rclcpp::Node::SharedPtr ros2_node, const std::string& topic_namespace,
  const std::string& master_id, const std::string& master_version)
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
    [this](const std::string& mfg, const std::string& serial) {
      return this->get_active_assignment_id(mfg, serial);
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
      ros2_node,
      [this]() {
        const auto snap = this->get_broker_status();
        GetMasterBrokerStatusService::StatusSnapshot out;
        out.connected = snap.connected;
        out.last_disconnect_at = snap.last_disconnect_at;
        out.reconnect_count = snap.reconnect_count;
        return out;
      },
      topic_namespace)),
  assignment_result_publisher_(
    std::make_shared<AssignmentResultPublisher>(ros2_node, topic_namespace)),
  assign_order_request_subscriber_(
    std::make_unique<AssignOrderRequestSubscriber>(
      ros2_node,
      [this](
        const std::string& mfg, const std::string& serial,
        const vda5050_core::types::Order& order) {
        return this->assign_order(mfg, serial, order);
      },
      [this](
        const std::string& mfg, const std::string& serial,
        const std::string& assignment_id, const std::string& order_id,
        std::uint32_t order_update_id) {
        this->record_assignment(
          mfg, serial, assignment_id, order_id, order_update_id);
      },
      assignment_result_publisher_, topic_namespace)),
  master_id_(master_id.empty() ? default_master_id() : master_id),
  master_connection_publisher_(std::make_unique<MasterConnectionPublisher>(
    ros2_node, master_id_, master_version,
    [this]() { return this->get_broker_status().connected; },
    [this]() {
      return static_cast<std::uint32_t>(this->get_onboarded_agvs().size());
    },
    topic_namespace)),
  fleet_roster_subscriber_(std::make_unique<FleetRosterSubscriber>(
    std::move(ros2_node), [this]() { return this->get_onboarded_agvs(); },
    [this](const std::vector<vda5050_core::master::VDA5050Master::OnboardSpec>&
             specs) { return this->onboard_agv_batch(specs); },
    [this](const std::vector<std::pair<std::string, std::string>>& keys) {
      return this->offboard_agv_batch(keys);
    },
    topic_namespace))
{
}

std::pair<std::string, std::string> VDA5050MasterROS2::split_agv_id(
  const std::string& agv_id)
{
  // master keeps agvs_ keyed as "{mfg}/{sn}" (master.cpp:228).
  const auto slash = agv_id.find('/');
  if (slash == std::string::npos) return {agv_id, std::string{}};
  return {agv_id.substr(0, slash), agv_id.substr(slash + 1)};
}

void VDA5050MasterROS2::on_state(
  const std::string& agv_id, const vda5050_core::types::State& state)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_state(mfg, serial, state);

  // OrderStatus + combined DeviceStatus alongside the per-component
  // State stream. handle_state has already cached the new State, so
  // both the order bundle and the status snapshot reflect it.
  if (auto agv = get_agv(mfg, serial))
  {
    auto bundle = agv->get_order_status_bundle();
    auto msg = build_order_status_msg(
      bundle, mfg, serial, get_active_assignment_id(mfg, serial));
    order_status_publisher_->publish_order_status(mfg, serial, msg);
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }

  vda5050_core::master::VDA5050Master::on_state(agv_id, state);
}

void VDA5050MasterROS2::on_connection(
  const std::string& agv_id, const vda5050_core::types::Connection& connection)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_connection(mfg, serial, connection);
  if (auto agv = get_agv(mfg, serial))
  {
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }
  vda5050_core::master::VDA5050Master::on_connection(agv_id, connection);
}

void VDA5050MasterROS2::on_factsheet(
  const std::string& agv_id, const vda5050_core::types::Factsheet& factsheet)
{
  auto [mfg, serial] = split_agv_id(agv_id);
  device_status_->publish_factsheet(mfg, serial, factsheet);
  if (auto agv = get_agv(mfg, serial))
  {
    device_status_->publish_device_status(
      mfg, serial, agv->get_status_snapshot());
  }
  vda5050_core::master::VDA5050Master::on_factsheet(agv_id, factsheet);
}

void VDA5050MasterROS2::on_broker_disconnected()
{
  if (master_connection_publisher_)
  {
    master_connection_publisher_->set_state(
      vda5050_master_ros2::msg::MasterConnection::DEGRADED);
  }
  vda5050_core::master::VDA5050Master::on_broker_disconnected();
}

void VDA5050MasterROS2::on_broker_reconnected()
{
  if (master_connection_publisher_)
  {
    master_connection_publisher_->set_state(
      vda5050_master_ros2::msg::MasterConnection::READY);
  }
  vda5050_core::master::VDA5050Master::on_broker_reconnected();
}

}  // namespace vda5050_master_ros2
