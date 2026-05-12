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

#ifndef VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_
#define VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_

#include <string_view>
#include <vector>

#include "vda5050_core/types/error.hpp"
#include "vda5050_core/types/state.hpp"

namespace vda5050_core::master {
namespace event {

// =============================================================================
// State event detection — pure prev/curr diffs.
// =============================================================================
//
// State message trigger list:
//   • Driving over a node              → reached_node
//   • New base request                 → new_base_requested
//   • Switching the operating mode     → mode_changed
//   • Errors or warnings               → errors_appeared / errors_resolved
//   • Change in the "driving" field    → driving_changed
//   • Paused field                     → paused_changed
//   • Load status changes              → loads_changed
//
// Out of scope (handled elsewhere):
//   • action_states transitions        → ActionLifecycleTracker
//   • Order id / order_update_id       → OrderLifecycleManager
//   • Error level (WARNING vs FATAL)   → error classification layer

/**
 * @brief True iff `node_id` transitioned to `released == true`.
 *
 * "Released" is the VDA5050 flag the AGV sets when it has executed
 * the node. Rising edge: prev had it absent or unreleased; curr has
 * it released.
 */
bool reached_node(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr, std::string_view node_id);

/**
 * @brief Errors present in curr but not in prev.
 *
 * Compared by (error_type, error_description) since VDA5050 errors
 * don't carry stable IDs.
 */
std::vector<vda5050_core::types::Error> errors_appeared(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief Errors present in prev but no longer in curr.
 *
 * Symmetric counterpart to errors_appeared. Captures the spec's
 * "self-resolving WARNING" recovery (e.g., field violation cleared).
 */
std::vector<vda5050_core::types::Error> errors_resolved(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief Rising edge: new_base_request transitioned false → true.
 *
 * Idempotent on sustained "true" — only fires once per request. The
 * flag is `optional<bool>` per VDA5050; absent is treated as false.
 */
bool new_base_requested(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief True iff curr.operating_mode != prev.operating_mode.
 */
bool mode_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief True iff curr.paused != prev.paused (any flip in either
 * direction). The `paused` field is `optional<bool>`; absent is
 * treated as false.
 */
bool paused_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief True iff curr.driving != prev.driving.
 */
bool driving_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

/**
 * @brief True iff curr.loads != prev.loads (any change in the loads
 * vector — count, contents, or both).
 */
bool loads_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr);

}  // namespace event
}  // namespace vda5050_core::master

#endif  // VDA5050_CORE__MASTER__STATE__STATE_EVENT_DETECTOR_HPP_
