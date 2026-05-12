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

#include "vda5050_core/master/validation/pre_send_validator.hpp"

#include <string>

#include "vda5050_core/errors/error_codes.hpp"
#include "vda5050_core/errors/error_factory.hpp"

namespace vda5050_core::master {

vda5050_core::order_utils::ValidationResult validate_pre_send(
  const PreSendContext& ctx)
{
  vda5050_core::order_utils::ValidationResult res;

  auto add_error = [&](const std::string& description) {
    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::PreSendValidationError, description, {}));
  };

  // No-map gate (Task #39): the master cannot validate Order or Instant
  // Action node/edge identifiers against a topology it has not loaded.
  // Hard-reject — fail fast on missing config rather than silently
  // bypass map-integrity checks downstream.
  if (ctx.loaded_map == nullptr)
  {
    res.errors.push_back(vda5050_core::errors::create_error(
      vda5050_core::errors::MapValidationError,
      "Master has no map loaded — call load_map_from_config() before "
      "publishing.",
      {}));
    return res;  // remaining checks are meaningless without a map
  }

  // Connection ONLINE — practical guard; sending to OFFLINE / CONNECTIONBROKEN
  // is wasted MQTT.
  if (ctx.connection_status != vda5050_core::types::ConnectionState::ONLINE)
  {
    add_error("AGV connection_status is not ONLINE");
  }

  // Master-internal operational state. ERROR / STATE_UNKNOWN both
  // indicate the AGV is not in a position to receive new orders:
  //   - ERROR: AGV has reported a fatal error
  //   - STATE_UNKNOWN: AGV's state-topic heartbeat exceeded 30s
  //     (Task #28). Sending orders to a silent AGV is wasteful and
  //     potentially unsafe — the order would publish but never be
  //     acknowledged.
  // UNAVAILABLE (set on connection loss) is already rejected by the
  // connection-status guard above.
  if (
    ctx.operational_state == AGVState::ERROR ||
    ctx.operational_state == AGVState::STATE_UNKNOWN)
  {
    add_error(
      std::string("AGV operational_state is ") +
      (ctx.operational_state == AGVState::ERROR ? "ERROR" : "STATE_UNKNOWN"));
  }

  // Need a State message before we can reason about mode or position.
  if (!ctx.last_state.has_value())
  {
    add_error("AGV has not yet reported any State");
    return res;  // remaining checks need last_state
  }

  // Per VDA5050 v2.0.0 §6.10: only AUTOMATIC has master under "full control".
  // SEMIAUTOMATIC is technically permissive, but task tracker locks strict
  // AUTOMATIC for V0.
  if (
    ctx.last_state->operating_mode !=
    vda5050_core::types::OperatingMode::AUTOMATIC)
  {
    add_error("AGV operating_mode is not AUTOMATIC");
  }

  // Per VM-VDA-6-6-1-3 (BACKLOG): reject when AGV's position is not
  // initialized.
  if (
    !ctx.last_state->agv_position.has_value() ||
    !ctx.last_state->agv_position->position_initialized)
  {
    add_error("AGV position is not initialized");
  }

  return res;
}

}  // namespace vda5050_core::master
