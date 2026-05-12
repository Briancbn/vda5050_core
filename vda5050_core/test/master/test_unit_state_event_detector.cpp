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

#include <gtest/gtest.h>

#include "vda5050_core/master/state/state_event_detector.hpp"

namespace vda5050_core::master::event::test {

namespace {
// Helpers for building synthetic State objects without ceremony.
vda5050_core::types::NodeState make_node(const std::string& id, bool released)
{
  vda5050_core::types::NodeState n;
  n.node_id = id;
  n.released = released;
  return n;
}

vda5050_core::types::Error make_error(
  const std::string& type, const std::string& description = "")
{
  vda5050_core::types::Error e;
  e.error_type = type;
  if (!description.empty()) e.error_description = description;
  return e;
}

vda5050_core::types::Load make_load(const std::string& id)
{
  vda5050_core::types::Load l;
  l.load_id = id;
  return l;
}
}  // namespace

// ============================================================================
// reached_node
// ============================================================================

TEST(StateEventDetectorTest, ReachedNodeOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.node_states = {make_node("n1", false)};
  vda5050_core::types::State curr;
  curr.node_states = {make_node("n1", true)};

  EXPECT_TRUE(reached_node(prev, curr, "n1"));
}

TEST(StateEventDetectorTest, ReachedNodeFalseWhenAlreadyReleased)
{
  vda5050_core::types::State prev;
  prev.node_states = {make_node("n1", true)};
  vda5050_core::types::State curr;
  curr.node_states = {make_node("n1", true)};

  EXPECT_FALSE(reached_node(prev, curr, "n1"));
}

TEST(StateEventDetectorTest, ReachedNodeFalseWhenNodeAbsent)
{
  vda5050_core::types::State prev;
  vda5050_core::types::State curr;
  EXPECT_FALSE(reached_node(prev, curr, "n1"));
}

TEST(StateEventDetectorTest, ReachedNodeOnlyForRequestedId)
{
  vda5050_core::types::State prev;
  prev.node_states = {make_node("n1", false), make_node("n2", false)};
  vda5050_core::types::State curr;
  curr.node_states = {make_node("n1", true), make_node("n2", false)};

  EXPECT_TRUE(reached_node(prev, curr, "n1"));
  EXPECT_FALSE(reached_node(prev, curr, "n2"));
}

// ============================================================================
// errors_appeared / errors_resolved
// ============================================================================

TEST(StateEventDetectorTest, ErrorsAppearedReturnsNewOnly)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1", "old")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1", "old"), make_error("E2", "new")};

  auto appeared = errors_appeared(prev, curr);
  ASSERT_EQ(appeared.size(), 1u);
  EXPECT_EQ(appeared[0].error_type, "E2");
}

TEST(StateEventDetectorTest, ErrorsAppearedEmptyWhenNoNewErrors)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1")};

  EXPECT_TRUE(errors_appeared(prev, curr).empty());
}

TEST(StateEventDetectorTest, ErrorsResolvedReturnsRemovedOnly)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1", "old"), make_error("E2", "vanishing")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1", "old")};

  auto resolved = errors_resolved(prev, curr);
  ASSERT_EQ(resolved.size(), 1u);
  EXPECT_EQ(resolved[0].error_type, "E2");
}

TEST(StateEventDetectorTest, ErrorsResolvedEmptyWhenNothingRemoved)
{
  vda5050_core::types::State prev;
  prev.errors = {make_error("E1")};
  vda5050_core::types::State curr;
  curr.errors = {make_error("E1"), make_error("E2")};

  EXPECT_TRUE(errors_resolved(prev, curr).empty());
}

// ============================================================================
// new_base_requested
// ============================================================================

TEST(StateEventDetectorTest, NewBaseRequestedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.new_base_request = false;
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_TRUE(new_base_requested(prev, curr));
}

TEST(StateEventDetectorTest, NewBaseRequestedFalseWhenSustained)
{
  vda5050_core::types::State prev;
  prev.new_base_request = true;
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_FALSE(new_base_requested(prev, curr));
}

TEST(StateEventDetectorTest, NewBaseRequestedHandlesAbsentPrev)
{
  vda5050_core::types::State prev;  // optional<bool> defaults to nullopt
  vda5050_core::types::State curr;
  curr.new_base_request = true;

  EXPECT_TRUE(new_base_requested(prev, curr));
}

// ============================================================================
// mode_changed
// ============================================================================

TEST(StateEventDetectorTest, ModeChangedOnTransition)
{
  vda5050_core::types::State prev;
  prev.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::State curr;
  curr.operating_mode = vda5050_core::types::OperatingMode::MANUAL;

  EXPECT_TRUE(mode_changed(prev, curr));
}

TEST(StateEventDetectorTest, ModeChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;
  vda5050_core::types::State curr;
  curr.operating_mode = vda5050_core::types::OperatingMode::AUTOMATIC;

  EXPECT_FALSE(mode_changed(prev, curr));
}

// ============================================================================
// paused_changed
// ============================================================================

TEST(StateEventDetectorTest, PausedChangedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.paused = false;
  vda5050_core::types::State curr;
  curr.paused = true;

  EXPECT_TRUE(paused_changed(prev, curr));
}

TEST(StateEventDetectorTest, PausedChangedOnFallingEdge)
{
  vda5050_core::types::State prev;
  prev.paused = true;
  vda5050_core::types::State curr;
  curr.paused = false;

  EXPECT_TRUE(paused_changed(prev, curr));
}

TEST(StateEventDetectorTest, PausedChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.paused = true;
  vda5050_core::types::State curr;
  curr.paused = true;

  EXPECT_FALSE(paused_changed(prev, curr));
}

// ============================================================================
// driving_changed
// ============================================================================

TEST(StateEventDetectorTest, DrivingChangedOnRisingEdge)
{
  vda5050_core::types::State prev;
  prev.driving = false;
  vda5050_core::types::State curr;
  curr.driving = true;

  EXPECT_TRUE(driving_changed(prev, curr));
}

TEST(StateEventDetectorTest, DrivingChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.driving = true;
  vda5050_core::types::State curr;
  curr.driving = true;

  EXPECT_FALSE(driving_changed(prev, curr));
}

// ============================================================================
// loads_changed
// ============================================================================

TEST(StateEventDetectorTest, LoadsChangedOnAddition)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};

  EXPECT_TRUE(loads_changed(prev, curr));
}

TEST(StateEventDetectorTest, LoadsChangedOnRemoval)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{};

  EXPECT_TRUE(loads_changed(prev, curr));
}

TEST(StateEventDetectorTest, LoadsChangedFalseWhenSame)
{
  vda5050_core::types::State prev;
  prev.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};
  vda5050_core::types::State curr;
  curr.loads = std::vector<vda5050_core::types::Load>{make_load("L1")};

  EXPECT_FALSE(loads_changed(prev, curr));
}

}  // namespace vda5050_core::master::event::test
