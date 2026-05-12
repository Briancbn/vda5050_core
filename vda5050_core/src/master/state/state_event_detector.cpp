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

#include "vda5050_core/master/state/state_event_detector.hpp"

#include <algorithm>

namespace vda5050_core::master {
namespace event {

namespace {

// Returns the released flag for node_id in the given State, or false
// if the node is not in node_states.
bool is_released(
  const vda5050_core::types::State& state, std::string_view node_id)
{
  auto it = std::find_if(
    state.node_states.begin(), state.node_states.end(),
    [node_id](const vda5050_core::types::NodeState& n) {
      return n.node_id == node_id;
    });
  return it != state.node_states.end() && it->released;
}

bool same_error(
  const vda5050_core::types::Error& a, const vda5050_core::types::Error& b)
{
  return a.error_type == b.error_type &&
         a.error_description == b.error_description;
}

// Returns errors in `from` that are not in `against` (set difference
// by error_type + error_description).
std::vector<vda5050_core::types::Error> errors_diff(
  const std::vector<vda5050_core::types::Error>& from,
  const std::vector<vda5050_core::types::Error>& against)
{
  std::vector<vda5050_core::types::Error> diff;
  for (const auto& e : from)
  {
    auto it = std::find_if(
      against.begin(), against.end(),
      [&e](const vda5050_core::types::Error& a) { return same_error(e, a); });
    if (it == against.end()) diff.push_back(e);
  }
  return diff;
}

}  // namespace

bool reached_node(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr, std::string_view node_id)
{
  return is_released(curr, node_id) && !is_released(prev, node_id);
}

std::vector<vda5050_core::types::Error> errors_appeared(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return errors_diff(curr.errors, prev.errors);
}

std::vector<vda5050_core::types::Error> errors_resolved(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return errors_diff(prev.errors, curr.errors);
}

bool new_base_requested(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  bool prev_set = prev.new_base_request.value_or(false);
  bool curr_set = curr.new_base_request.value_or(false);
  return curr_set && !prev_set;
}

bool mode_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.operating_mode != curr.operating_mode;
}

bool paused_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.paused.value_or(false) != curr.paused.value_or(false);
}

bool driving_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.driving != curr.driving;
}

bool loads_changed(
  const vda5050_core::types::State& prev,
  const vda5050_core::types::State& curr)
{
  return prev.loads != curr.loads;
}

}  // namespace event
}  // namespace vda5050_core::master
