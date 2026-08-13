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

#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11_json/pybind11_json.hpp>

#include <algorithm>

#include "vda5050_core/execution/protocol_adapter.hpp"
#include "vda5050_core/json_utils/serialization.hpp"
#include "vda5050_core/transport/mqtt_client_interface.hpp"
#include "vda5050_core/types/agv_position.hpp"

#include "vda5050_core/rmf_migration.hpp"
#include "vda5050_core_py/rmf_migration.hpp"

namespace vda5050_core_py {

using vda5050_core::rmf_migration::ActionExecutor;
using vda5050_core::rmf_migration::ActivityIdentifier;
using vda5050_core::rmf_migration::Adapter;
using vda5050_core::rmf_migration::CommandExecution;
using vda5050_core::rmf_migration::Destination;
using vda5050_core::rmf_migration::FleetConfiguration;
using vda5050_core::rmf_migration::FleetUpdateHandle;
using vda5050_core::rmf_migration::LocalizationRequest;
using vda5050_core::rmf_migration::NavigationRequest;
using vda5050_core::rmf_migration::RobotCallbacks;
using vda5050_core::rmf_migration::RobotConfiguration;
using vda5050_core::rmf_migration::RobotState;
using vda5050_core::rmf_migration::RobotUpdateHandle;
using vda5050_core::rmf_migration::StopRequest;

//=============================================================================
void bind_rmf_migration(py::module_ & m)
{
  auto m_rmf_migration =
    m.def_submodule("rmf_migration", "Open-RMF style migration API");

  py::class_<ActivityIdentifier, std::shared_ptr<ActivityIdentifier>>(
    m_rmf_migration, "ActivityIdentifier")
    .def("__eq__", &ActivityIdentifier::operator==)
    .def("__ne__", &ActivityIdentifier::operator!=);

  py::class_<RobotState>(m_rmf_migration, "RobotState")
    .def(
      py::init<std::string, std::array<double, 3>, double>(), py::arg("map"),
      py::arg("position"), py::arg("battery_soc"))
    .def_property("map", &RobotState::map, &RobotState::set_map)
    .def_property("position", &RobotState::position, &RobotState::set_position)
    .def_property(
      "battery_state_of_charge", &RobotState::battery_state_of_charge,
      &RobotState::set_battery_state_of_charge);

  py::class_<RobotConfiguration>(m_rmf_migration, "RobotConfiguration")
    .def(
      py::init<std::string, std::string, std::string, std::string>(),
      py::arg("manufacturer"), py::arg("serial_number"),
      py::arg("interface_name") = "uagv", py::arg("version") = "2.0.0")
    .def_readwrite("manufacturer", &RobotConfiguration::manufacturer)
    .def_readwrite("serial_number", &RobotConfiguration::serial_number)
    .def_readwrite("interface_name", &RobotConfiguration::interface_name)
    .def_readwrite("version", &RobotConfiguration::version)
    .def_readwrite("factsheet", &RobotConfiguration::factsheet);

  py::class_<Destination>(m_rmf_migration, "Destination")
    .def_property_readonly("map", &Destination::map)
    .def_property_readonly("position", &Destination::position)
    .def_property_readonly("xy", &Destination::xy)
    .def_property_readonly("yaw", &Destination::yaw)
    .def_property_readonly("graph_index", &Destination::graph_index)
    .def_property_readonly("name", &Destination::name)
    .def_property_readonly("speed_limit", &Destination::speed_limit);

  py::class_<CommandExecution>(m_rmf_migration, "CommandExecution")
    .def("finished", &CommandExecution::finished)
    .def("failed", &CommandExecution::failed)
    .def("okay", &CommandExecution::okay)
    .def("is_finished", &CommandExecution::is_finished)
    .def_property_readonly("identifier", &CommandExecution::identifier);

  py::class_<RobotCallbacks>(m_rmf_migration, "RobotCallbacks")
    .def(
      py::init<NavigationRequest, StopRequest, ActionExecutor>(),
      py::arg("navigate"), py::arg("stop"), py::arg("action_executor"))
    .def_property_readonly("navigate", &RobotCallbacks::navigate)
    .def_property_readonly("stop", &RobotCallbacks::stop)
    .def_property_readonly("action_executor", &RobotCallbacks::action_executor)
    .def_property(
      "localize", &RobotCallbacks::localize,
      &RobotCallbacks::with_localization);

  py::class_<RobotUpdateHandle, std::shared_ptr<RobotUpdateHandle>>(
    m_rmf_migration, "RobotUpdateHandle")
    .def("update", &RobotUpdateHandle::update)
    .def("more", [](RobotUpdateHandle& self) { return self.more(); });

  py::class_<FleetConfiguration>(m_rmf_migration, "FleetConfiguration")
    .def(
      py::init<std::string, std::string, std::string, int>(),
      py::arg("fleet_name"), py::arg("broker_uri"), py::arg("client_id_prefix"),
      py::arg("update_interval") = 30)
    .def_property(
      "fleet_name", &FleetConfiguration::fleet_name,
      &FleetConfiguration::set_fleet_name)
    .def_property(
      "broker_uri", &FleetConfiguration::broker_uri,
      &FleetConfiguration::set_broker_uri)
    .def_property(
      "client_id_prefix", &FleetConfiguration::client_id_prefix,
      &FleetConfiguration::set_client_id_prefix)
    .def_property(
      "update_interval", &FleetConfiguration::update_interval,
      &FleetConfiguration::set_update_interval)
    .def_property_readonly("known_robots", &FleetConfiguration::known_robots)
    .def(
      "add_known_robot_configuration",
      &FleetConfiguration::add_known_robot_configuration)
    .def(
      "get_known_robot_configuration",
      &FleetConfiguration::get_known_robot_configuration);

  py::class_<FleetUpdateHandle, std::shared_ptr<FleetUpdateHandle>>(
    m_rmf_migration, "FleetUpdateHandle")
    .def("add_robot", &FleetUpdateHandle::add_robot);

  py::class_<Adapter, std::shared_ptr<Adapter>>(m_rmf_migration, "Adapter")
    .def_static("make", &Adapter::make)
    .def(
      "add_vda5050_fleet", &Adapter::add_vda5050_fleet,
      py::arg("configuration"))
    .def("start", &Adapter::start)
    .def("stop", &Adapter::stop);
}

}  // namespace vda5050_core_py

namespace vda5050_core {

namespace rmf_migration {

//=============================================================================
RobotState::RobotState(
  std::string map, std::array<double, 3> position, double battery_soc)
: map_(std::move(map)), position_(position), battery_soc_(battery_soc)
{
  // Nothing to do here ...
}

//=============================================================================
const std::string& RobotState::map() const
{
  return map_;
}

//=============================================================================
void RobotState::set_map(std::string value)
{
  map_ = std::move(value);
}

//=============================================================================
std::array<double, 3> RobotState::position() const
{
  return position_;
}

//=============================================================================
void RobotState::set_position(std::array<double, 3> value)
{
  position_ = value;
}

//=============================================================================
double RobotState::battery_state_of_charge() const
{
  return battery_soc_;
}

//=============================================================================
void RobotState::set_battery_state_of_charge(double value)
{
  battery_soc_ = value;
}

//=============================================================================
types::AGVPosition RobotState::to_agv_position(
  std::string map, const std::array<double, 3>& position)
{
  types::AGVPosition pos;
  pos.x = position[0];
  pos.y = position[1];
  pos.theta = position[2];
  pos.map_id = std::move(map);

  return pos;
}

//=============================================================================
types::BatteryState RobotState::to_battery_state(double soc)
{
  types::BatteryState b;
  b.battery_charge = soc * 100.0;
  return b;
}

//=============================================================================
RobotConfiguration::RobotConfiguration(
  std::string manufacturer, std::string serial_number,
  std::string interface_name, std::string version)
: manufacturer(std::move(manufacturer)),
  serial_number(std::move(serial_number)),
  interface_name(std::move(interface_name)),
  version(std::move(version))
{
  // Nothing to do here ...
}

//=============================================================================
const std::string& Destination::map() const
{
  return map_;
}

//=============================================================================
std::array<double, 3> Destination::position() const
{
  return position_;
}

//=============================================================================
std::array<double, 2> Destination::xy() const
{
  std::array<double, 2> destination;
  std::copy_n(position_.begin(), 2, destination.begin());
  return destination;
}

//=============================================================================
double Destination::yaw() const
{
  return position_[2];
}

//=============================================================================
std::optional<uint32_t> Destination::graph_index() const
{
  return graph_index_;
}

//=============================================================================
std::optional<std::string> Destination::name() const
{
  return name_;
}

//=============================================================================
std::optional<double> Destination::speed_limit() const
{
  return speed_limit_;
}

//=============================================================================
Destination::Destination(
  std::string map, std::array<double, 3> position,
  std::optional<uint32_t> graph_index, std::optional<std::string> name,
  std::optional<double> speed_limit)
: map_(std::move(map)),
  position_(position),
  graph_index_(graph_index),
  name_(std::move(name)),
  speed_limit_(speed_limit)
{
  // Nothing to do here
}

//=============================================================================
ActivityIdentifier::ActivityIdentifier(uint64_t identifier)
: identifier_(identifier)
{
  // Nothing to do here ...
}

//=============================================================================
bool ActivityIdentifier::operator==(const ActivityIdentifier& other) const
{
  return identifier_ == other.identifier_;
}

//=============================================================================
bool ActivityIdentifier::operator!=(const ActivityIdentifier& other) const
{
  return !(this->operator==(other));
}

//=============================================================================
void CommandExecution::finished()
{
  if (execution_)
  {
    execution_->finished();
  }
}

//=============================================================================
void CommandExecution::failed(const std::string& reason)
{
  if (execution_)
  {
    execution_->failed(reason);
  }
}

//=============================================================================
bool CommandExecution::okay() const
{
  if (!execution_)
  {
    return false;
  }
  return execution_->okay();
}

//=============================================================================
bool CommandExecution::is_finished() const
{
  if (!execution_)
  {
    return true;
  }

  return execution_->is_finished();
}

//=============================================================================
ConstActivityIdentifierPtr CommandExecution::identifier() const
{
  return identifier_;
}

//=============================================================================
CommandExecution::CommandExecution(
  std::shared_ptr<client::adapter::Execution> execution,
  ConstActivityIdentifierPtr identifier)
: execution_(std::move(execution)), identifier_(std::move(identifier))
{
  // Nothing to do here ...
}

//=============================================================================
RobotCallbacks::RobotCallbacks(
  NavigationRequest navigate, StopRequest stop, ActionExecutor action_executor)
: navigate_(std::move(navigate)),
  stop_(std::move(stop)),
  action_executor_(std::move(action_executor)),
  localize_(nullptr)
{
  // Nothing to do her ...
}

//=============================================================================
NavigationRequest RobotCallbacks::navigate() const
{
  return navigate_;
}

//=============================================================================
StopRequest RobotCallbacks::stop() const
{
  return stop_;
}

//=============================================================================
ActionExecutor RobotCallbacks::action_executor() const
{
  return action_executor_;
}

//=============================================================================
RobotCallbacks& RobotCallbacks::with_localization(
  LocalizationRequest localization)
{
  localize_ = std::move(localization);
  return *this;
}

//=============================================================================
LocalizationRequest RobotCallbacks::localize() const
{
  return localize_;
}

//=============================================================================
void RobotUpdateHandle::update(
  RobotState state, ConstActivityIdentifierPtr /*identifier*/)
{
  adapter_->state_manager()->set_position(
    state.position()[0], state.position()[1], state.position()[2], state.map());
  adapter_->state_manager()->set_battery_state(
    RobotState::to_battery_state(state.battery_state_of_charge()));
}

//=============================================================================
std::shared_ptr<client::adapter::StateManager> RobotUpdateHandle::more()
{
  return adapter_->state_manager();
}

//=============================================================================
RobotUpdateHandle::RobotUpdateHandle(
  std::shared_ptr<client::adapter::Adapter> adapter)
: adapter_(std::move(adapter))
{
  // Nothing to do here ...
}

//=============================================================================
FleetConfiguration::FleetConfiguration(
  std::string fleet_name, std::string broker_uri, std::string client_id_prefix,
  int update_interval)
: fleet_name_(std::move(fleet_name)),
  broker_uri_(std::move(broker_uri)),
  client_id_prefix_(std::move(client_id_prefix)),
  update_interval_(update_interval)
{
  // Nothing to do here ...
}

//=============================================================================
std::optional<FleetConfiguration> FleetConfiguration::from_config_files(
  const std::string& /*config_file*/)
{
  // TODO(sauk2): Define a config and read off it. Another option is to
  // define a conversion from RMF config to VDA5050 factsheet
  return std::nullopt;
}

//=============================================================================
const std::string& FleetConfiguration::fleet_name() const
{
  return fleet_name_;
}

//=============================================================================
void FleetConfiguration::set_fleet_name(std::string value)
{
  fleet_name_ = std::move(value);
}

//=============================================================================
const std::unordered_map<std::string, RobotConfiguration>&
FleetConfiguration::known_robot_configurations() const
{
  return known_robot_configurations_;
}

//=============================================================================
std::vector<std::string> FleetConfiguration::known_robots() const
{
  std::vector<std::string> result;
  result.reserve(known_robot_configurations_.size());

  for (const auto& [name, _] : known_robot_configurations_)
  {
    result.push_back(name);
  }
  return result;
}

//=============================================================================
void FleetConfiguration::add_known_robot_configuration(
  std::string robot_name, RobotConfiguration config)
{
  known_robot_configurations_.insert_or_assign(
    std::move(robot_name), std::move(config));
}

//=============================================================================
std::optional<RobotConfiguration>
FleetConfiguration::get_known_robot_configuration(const std::string& name) const
{
  auto it = known_robot_configurations_.find(name);
  if (it != known_robot_configurations_.end()) return it->second;
  return std::nullopt;
}

//=============================================================================
const std::string& FleetConfiguration::broker_uri() const
{
  return broker_uri_;
}

//=============================================================================
void FleetConfiguration::set_broker_uri(std::string value)
{
  broker_uri_ = std::move(value);
}

//=============================================================================
const std::string& FleetConfiguration::client_id_prefix() const
{
  return client_id_prefix_;
}

//=============================================================================
void FleetConfiguration::set_client_id_prefix(std::string value)
{
  client_id_prefix_ = std::move(value);
}

//=============================================================================
int FleetConfiguration::update_interval() const
{
  return update_interval_;
}

//=============================================================================
void FleetConfiguration::set_update_interval(int value)
{
  update_interval_ = value;
}

//=============================================================================
std::shared_ptr<RobotUpdateHandle> FleetUpdateHandle::add_robot(
  std::string name, RobotState initial_state, RobotConfiguration configuration,
  RobotCallbacks callbacks)
{
  auto mqtt_client = transport::create_default_client_unique(
    configuration_.broker_uri(),
    fmt::format("{}_{}", configuration_.client_id_prefix(), name));
  auto protocol_adapter = execution::ProtocolAdapter::make(
    std::move(mqtt_client), configuration.interface_name, configuration.version,
    configuration.manufacturer, configuration.serial_number);

  auto adapter = client::adapter::Adapter::make(protocol_adapter);
  adapter->state_manager()->set_position(
    initial_state.position()[0], initial_state.position()[1],
    initial_state.position()[2], initial_state.map());
  adapter->state_manager()->set_battery_state(
    RobotState::to_battery_state(initial_state.battery_state_of_charge()));

  adapter->on_navigate(
    [callbacks, this](
      client::adapter::NodeRequest node_request,
      std::optional<client::adapter::EdgeRequest> /*edge_request*/,
      std::shared_ptr<client::adapter::OrderExecution> execution) {
      const auto& node_position = node_request.node_position().value();
      auto destination = Destination(
        node_request.node_position().value().map_id,
        {node_position.x, node_position.y, node_position.theta.value_or(0.0)},
        node_request.sequence_id(), node_request.node_id());
      auto command = CommandExecution(
        execution, std::shared_ptr<ActivityIdentifier>(
                     new ActivityIdentifier(activity_count_++)));

      callbacks.navigate()(std::move(destination), std::move(command));
    });

  adapter->on_action(
    [callbacks, this](
      client::adapter::ActionRequest request,
      std::shared_ptr<client::adapter::ActionExecution> execution) {
      auto command = CommandExecution(
        execution, std::shared_ptr<ActivityIdentifier>(
                     new ActivityIdentifier(activity_count_++)));
      if (request.action_type() == "startPause")
      {
        callbacks.stop()(command.identifier());
        command.finished();
      }
      else
      {
        nlohmann::json description = {};
        if (request.action_parameters().has_value())
        {
          description = request.action_parameters().value();
        }
        callbacks.action_executor()(
          request.action_type(), description, std::move(command));
      }
    });

  if (callbacks.localize())
  {
    adapter->on_localize(
      [callbacks, this](
        client::adapter::LocalizationRequest request,
        std::shared_ptr<client::adapter::ActionExecution> execution) {
        auto destination = Destination(
          request.map_id(), {request.x(), request.y(), request.theta()});
        auto command = CommandExecution(
          execution, std::shared_ptr<ActivityIdentifier>(
                       new ActivityIdentifier(activity_count_++)));

        callbacks.localize()(std::move(destination), std::move(command));
      });
  }

  robots_.insert_or_assign(name, adapter);

  return std::shared_ptr<RobotUpdateHandle>(new RobotUpdateHandle(adapter));
}

//=============================================================================
FleetUpdateHandle::FleetUpdateHandle(FleetConfiguration configuration)
: configuration_(std::move(configuration))
{
  // Nothing to do here ...
}

//=============================================================================
std::shared_ptr<Adapter> Adapter::make()
{
  auto adapter = std::shared_ptr<Adapter>(new Adapter());
  return adapter;
}

//=============================================================================
std::shared_ptr<FleetUpdateHandle> Adapter::add_vda5050_fleet(
  FleetConfiguration configuration)
{
  auto fleet = std::shared_ptr<FleetUpdateHandle>(
    new FleetUpdateHandle(std::move(configuration)));
  fleets_.push_back(fleet);
  return fleet;
}

//=============================================================================
void Adapter::start()
{
  for (auto& fleet : fleets_)
  {
    for (auto& [name, adapter] : fleet->robots_)
    {
      adapter->start();
    }
  }
}

//=============================================================================
void Adapter::stop()
{
  for (auto& fleet : fleets_)
  {
    for (auto& [name, adapter] : fleet->robots_)
    {
      adapter->stop();
    }
  }
}

//=============================================================================
Adapter::Adapter()
{
  // Nothing to do here ...
}

}  // namespace rmf_migration
}  // namespace vda5050_core
