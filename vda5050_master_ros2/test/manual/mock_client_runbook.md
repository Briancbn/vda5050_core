# Manual Integration Runbook — `example_master` + `mock_client`

Step-by-step scenarios for exercising the V0 `vda5050_master_ros2`
master process against the standalone `mock_client` AGV stub over a
local MQTT broker.

> **Scope.** This runbook covers manual fault-injection testing with
> the in-tree `mock_client` AGV stub (deterministic, no external
> dependencies). Companion to:
>
> - `interop_runbook.md` — master + `example_client` (independent
>   VDA5050 implementation) driven by `mock_fms` (FMS-side console).
> - Unit suite — gtest, run via `colcon test`.
>
> Real FMS / sim / hardware integration is out of scope here.

---

## Prerequisites

```bash
# One-time
sudo apt install ros-jazzy-desktop mosquitto mosquitto-clients \
    libpaho-mqttpp-dev libpaho-mqtt-dev libfmt-dev \
    nlohmann-json3-dev ros-jazzy-rmw-cyclonedds-cpp

# Per-shell
source /opt/ros/jazzy/setup.bash
cd /home/lyh/vda5050/working_repo/vda5050_client
colcon build --packages-select vda5050_core vda5050_master_ros2
source install/setup.bash
```

## Conventions

Throughout this runbook:

- **T1** = the terminal running `mosquitto -v`
- **T2** = the terminal running `example_master`
- **T3** = the terminal running `mock_client` (one per AGV)
- **T4** = a "driver" terminal where you invoke `ros2 service call …`

## Kill-switch

When a scenario goes off the rails:

```bash
pkill -INT -f mock_client
pkill -INT -f example_master
# Then re-launch from scratch with `mosquitto -v`.
```

## `example_master` log markers (the runbook's expected-output contract)

These are the structured prefixes example_master emits on every
overridden `on_*` virtual. They are the runbook's stable assertion
strings:

| Marker | Source virtual | Meaning |
|---|---|---|
| `[CONN]` | `on_connection` | Raw Connection message received |
| `[CONNECTED]` | `on_connect` | First ONLINE detected |
| `[OFFLINE]` | `on_offline` | Graceful OFFLINE detected |
| `[BROKEN]` | `on_connection_broken` | Last-will fired |
| `[STATE]` | `on_state` | Raw State received |
| `[FACTSHEET]` | `on_factsheet` | Factsheet cached |
| `[MODE]` | `on_mode_changed` | Operating-mode edge |
| `[ERROR]` | `on_errors_appeared` | New AGV error |
| `[TIMEOUT]` | `on_state_timeout` | 30 s state heartbeat lost |
| `[RECOVERED]` | `on_state_resumed` | State resumed after timeout |
| `[BROKER]` | `on_broker_(dis)connected` | Master ↔ broker link |

## Mock launch reference

```bash
ros2 run vda5050_master_ros2 mock_client \
    --serial MOCK001 \
    --mfg MockMfg \
    --map-id warehouse_floor1 \
    --x0 0.0 --y0 0.0 \
    --mode AUTOMATIC \
    --tick-ms 500
```

Signals to a running mock_client:

| Signal | Effect |
|---|---|
| `SIGINT` / `SIGTERM` | Graceful: publish `Connection{OFFLINE}` (retained), then disconnect |
| `SIGUSR1` | Toggle `operating_mode` AUTOMATIC ↔ MANUAL |
| `SIGUSR2` | Toggle `state_.paused` |
| `SIGKILL` (`kill -9`) | Ungraceful: broker fires the pre-registered `CONNECTIONBROKEN` last-will |

`--scenario` values:

| Value | What changes |
|---|---|
| `idle` (default) | Healthy AGV, ready to receive orders |
| `uninit-pose` | `state.agv_position.position_initialized = false` |
| `no-factsheet` | Skip the initial Factsheet publish |
| `malformed-state` | Emit one bad-JSON State at startup, then resume normal publishing |

## Multi-AGV (inline)

```bash
# T3a:
ros2 run vda5050_master_ros2 mock_client --serial MOCK001 &
# T3b:
ros2 run vda5050_master_ros2 mock_client --serial MOCK002 --x0 5.0 --y0 0.0 &
```

Each mock has its own MQTT client-id `mock_client-<serial>` so they
don't collide on the broker. The master treats each as a distinct AGV
in its onboarded-fleet map.

---

## Per-scenario template

```md
### S<N> — <name>
- Use case: <description>
- Spec ref: VDA5050 v2.0.0 §<x.y>

Setup
  T1: mosquitto -v
  T2: ros2 run vda5050_master_ros2 example_master
  T3: ros2 run vda5050_master_ros2 mock_client --serial MOCK001 ...

Steps
  1. T4: <ros2 service call …>
  2. ...

Expected
  T1: <broker traffic visible to mosquitto -v>
  T2: <example_master log marker(s)>
  T3: <mock_client log line(s)>
  Service response: <YAML-shape>

Cleanup
  <how to reset for next scenario>
```

---

## Tier-1 — must walk before V0 sign-off (10 scenarios)

### Last-walked status — 2026-05-14

| # | Scenario | Status | Notes |
|---|---|---|---|
| S1 | Master-broker bounce | ✅ verified | reconnect_count went 1 → 2 across bounce |
| S2 | Onboard idempotent | ✅ verified | SUCCESS → ALREADY_ONBOARDED |
| S3 | GetLoadedMap | ✅ verified | sample_map (4 nodes / 4 edges) + factsheet alignment |
| S4 | Happy-path order | ✅ verified | `decision=0` → mock walks N0→N1, `OrderLifecycle completed` logged |
| S5 | Horizon extension stitch | ✅ verified | base order with N0/N1 released + N2 horizon → mock parks at N1; update with same order_id update_id=1 releasing N2 + adding N3 → mock walks to N3. Required the mock fix in `fix(master_ros2): mock_client should not walk unreleased horizon nodes` |
| S6 | Schema reject bad version | ✅ verified | after the stitcher fix in `fix(master): stitcher should not reject new order_id after prior order complete`, surfaces `schemaValidationError: header.version '99.0' is not in SupportedSchemaVersions` |
| S7 | Traversability reject unreachable | ✅ verified | surfaces `traversabilityValidationError: AGV is not within the first node's allowed_deviation_x_y (distance=67.27 m, allowed=0.00 m)` plus map-deviation error |
| S8 | Pre-send reject MANUAL mode | ✅ verified | `decision=4` (AGV_MODE_NOT_AUTO), `preSendValidationError` |
| S9 | stateRequest instant action | ✅ verified | mock logged `action ia-stateReq-1 (stateRequest) acked` within 1 tick |
| S10 | kill -9 → BROKEN | ✅ verified | master logged `[CONN] CONNECTIONBROKEN` + `[BROKEN]` + `[AGV] Last-will fired ... Clearing pending queues` within 1 s of kill |

**Stitcher false-rejection — found and fixed during this walkthrough.**
The earlier round saw that after order `A` completed, any new order
`B` with a different order_id was stitch-rejected (`"cancel the
active order before sending a different order"`) — masking S6/S7's
intended validator errors. Per VDA5050 §6.6.1 the AGV legitimately
keeps `state.order_id = A` until a new order is accepted; the
master's lifecycle correspondingly keeps `has_active=true` with
`order_complete=true`. The stitcher previously rejected on
`has_active` alone, ignoring `order_complete`.

Fixed in `fix(master): stitcher should not reject new order_id after
prior order complete`: when `snapshot.order_complete == true`, a
candidate with a different `order_id` is treated as a fresh
assignment (`SEND_NOW`). Verified by re-running S6 and S7 — they now
surface the intended `schemaValidationError` and
`traversabilityValidationError` respectively.

### S1 — Master-broker status survives a broker bounce

- Spec ref: §6.14

**Setup**
- T1: `mosquitto -v`
- T2: `ros2 run vda5050_master_ros2 example_master`

**Steps**
1. T4: `ros2 service call /vda5050_master/get_master_broker_status vda5050_master_ros2/srv/GetMasterBrokerStatus '{}'`
2. T1: Ctrl-C the broker. Wait 5 s.
3. T4: Re-run the GetMasterBrokerStatus call.
4. T1: Restart `mosquitto -v`.
5. T4: Re-run a third time.

**Expected**
- T2 after step 2: `[ERROR] [BROKER] master lost MQTT broker connection`
- Service response after step 3:
  ```yaml
  connected: false
  last_disconnect_at: {sec: <now>, nanosec: <…>}
  reconnect_count: 1
  ```
- T2 after step 4: `[INFO] [BROKER] master reconnected to MQTT broker`
- Service response after step 5: `connected: true`, `reconnect_count: 2`

**Cleanup**
- None needed.

### S2 — Onboard idempotency


**Setup** Same as S1, plus T3 mock_client running.

**Steps**
1. T4: `ros2 service call /vda5050_master/onboard_agv vda5050_master_ros2/srv/OnboardAGV "{manufacturer: 'MockMfg', serial_number: 'MOCK001'}"`
2. T4: Re-run the same service call.

**Expected**
- First response: `status: 0` (SUCCESS), echoed `manufacturer` / `serial_number`.
- Second response: `status: 1` (ALREADY_ONBOARDED).
- T2 after step 1: `[VDA5050Master] Onboarded AGV: MockMfg/MOCK001` + `[CONN] MockMfg/MOCK001 state=ONLINE` + `[CONNECTED] MockMfg/MOCK001 came ONLINE` + `[FACTSHEET] MockMfg/MOCK001 series=MockAGV`.

**Cleanup**
- Offboard if not running further scenarios: `ros2 service call /vda5050_master/offboard_agv …`.

### S3 — `GetLoadedMap` returns map and per-AGV alignment


**Setup** Same as S2.

**Steps**
1. T4: `ros2 service call /vda5050_master/get_loaded_map vda5050_master_ros2/srv/GetLoadedMap '{}'`

**Expected**
- Response:
  ```yaml
  status: 0                   # SUCCESS
  map_id: 'warehouse_floor1'
  map_version: '1.0'
  node_count: 4
  edge_count: 4
  agv_alignments:
    - manufacturer: 'MockMfg'
      serial_number: 'MOCK001'
      …                       # findings populated from factsheet vs map
  ```

**Cleanup** None.

### S4 — Happy-path order lifecycle: assign → publish → state cycles

- Spec ref: §6.6.1

**Setup**
- T1: `mosquitto -v`
- T2: `ros2 run vda5050_master_ros2 example_master`
- T3: `ros2 run vda5050_master_ros2 mock_client --serial MOCK001`
- T4: prepare `order.yaml` (see appendix) at `/tmp/order.yaml`

**Steps**
1. T4: `ros2 service call /vda5050_master/onboard_agv vda5050_master_ros2/srv/OnboardAGV "{manufacturer: 'MockMfg', serial_number: 'MOCK001'}"` → SUCCESS.
2. T4: `ros2 service call /vda5050_master/assign_order vda5050_master_ros2/srv/AssignOrder "$(cat /tmp/order.yaml)"`
3. Wait ~10 s, observe mock state cycle.
4. T4: `ros2 topic echo --once /vda5050_master/MockMfg/MOCK001/order_status`

**Expected**
- Service response: `decision: 0` (ASSIGNED), `errors: []`.
- T3 log: `[mock_client] Received order id=demo-order-001 …`.
- T2 log: `[STATE] MockMfg/MOCK001 order=demo-order-001/0 last_node=N0 …` then progresses to `last_node=N1`.
- `order_status` topic shows `phase: PHASE_RUNNING` then `PHASE_COMPLETED` once mock reaches final node.

**Known issue (2026-05-14)**
The master's async OrderPublisher chain currently reports `Order
validation failed for MockMfg/MOCK001: 1 error(s)` after the sync
pre-flight returns ASSIGNED. The error is not surfaced to the
operator — fixing the master to log the specific error (or returning
via the planned AssignmentEvents topic) is a follow-up. Until then, S4 fails
on the publish→state cycle step but ASSIGNED is returned, confirming
the request side is wired correctly.

**Cleanup**
- Ctrl-C T3. Expect T2: `[OFFLINE] MockMfg/MOCK001 announced graceful OFFLINE`.

### S5 — Horizon extension at stitch point

- Spec ref: §6.6.2 (figure 8)

**Setup** Same as S4. Order from S4 still active.

**Steps**
1. Wait until mock's `last_node_id` shows `N1` (the last released node of the base from S4).
2. T4: `ros2 service call /vda5050_master/assign_order vda5050_master_ros2/srv/AssignOrder "$(cat /tmp/order_update.yaml)"` (update extends with N2, N3 — see appendix).

**Expected**
- Service response: `decision: 0` (ASSIGNED) or `decision: 8` (STITCH_QUEUED) depending on timing.
- T2 log: `[STATE] … order=demo-order-001/1 last_node=N1 …` after the update lands.

**FIWARE recipe** — the easiest stitch-test order_update payload is to
reverse the base's waypoint list and append, mirroring
`vda5050_fiware/mock_update_order.py`.

**Cleanup** Ctrl-C T3.

### S6 — Schema reject: bad header.version


**Setup** Same as S2.

**Steps**
1. T4: assign_order with `order.header.version = "99.0"`.

**Expected**
- Service response: `decision != 0`, `errors[0].error_type = 'schemaValidationError'`, `error_description` mentions `header.version`.

**Cleanup** None.

### S7 — Traversability reject: first node unreachable

- Spec ref: §6.6.1.3

**Setup**
- T3: `mock_client --x0 0 --y0 0` (default pose at N0=origin)
- Build an order whose first node is `N2 (5,5)` — 7 m from mock, > AGV deviation tolerance.

**Steps**
1. T4: assign that order.

**Expected**
- Service response: `decision != 0`, error mentioning unreachable / first node / deviation.

**Cleanup** None.

### S8 — Pre-send reject: mock in MANUAL → `AGV_MODE_NOT_AUTO`

- Spec ref: Table 10

**Setup** Same as S2.

**Steps**
1. T4 (or T3 console): `kill -USR1 $(pgrep -f 'mock_client.*MOCK001')`. T3 logs `[mock_client] mode flipped -> MANUAL`.
2. T4: assign order from S4.

**Expected**
- Service response: `decision: 4` (AGV_MODE_NOT_AUTO).
- T2 log shows no Order publish on `rmf2/v2/MockMfg/MOCK001/order`.

**Cleanup**
- T3: `kill -USR1 $(pgrep -f 'mock_client.*MOCK001')` to flip back to AUTOMATIC.

### S9 — `stateRequest` instant action → mock publishes fresh State

- Spec ref: §6.8.2.6

**Setup** Same as S2.

**Steps**
1. T4:
   ```bash
   ros2 service call /vda5050_master/assign_instant_actions \
       vda5050_master_ros2/srv/AssignInstantActions \
       "{manufacturer: 'MockMfg', serial_number: 'MOCK001',
         instant_actions: {header: {header_id: 1, timestamp: 0, version: '2.0.0',
           manufacturer: 'MockMfg', serial_number: 'MOCK001'},
         actions: [{action_id: 'ia-stateReq-1', action_type: 'stateRequest',
                    blocking_type: 'NONE', action_parameters: []}]}}"
   ```

**Expected**
- T3 log: `[mock_client] action ia-stateReq-1 (stateRequest) acked`.
- T2 log: a fresh `[STATE]` line within one tick.

**Cleanup** None.

### S10 — Broken connection: `kill -9` mock → master fires `on_connection_broken`

- Spec ref: §6.14

**Setup** Same as S2, with mock running.

**Steps**
1. T4: `kill -9 $(pgrep -f 'mock_client.*MOCK001')`.

**Expected**
- T2 log within ~1 s: `[WARN] [BROKEN] MockMfg/MOCK001 unexpectedly disconnected (last-will fired)`.

**Cleanup**
- Re-launch mock in T3 for subsequent scenarios.

---

## Tier-2 — documented, walk only if changes touch the related component (15 scenarios)

### S11 — Graph reject: disconnected nodes
Build an order with N0 and N1 but no E01 edge → graph validator rejects.

### S12 — Released-edge endpoint check
Released E01 but N1 not released → rejected.

### S13 — Traversability reject: action capability missing
Order contains `action_type: "customDock"` but mock's
factsheet lists no `customDock` in agv_actions → rejected.

### S14 — Pre-send reject: position not initialized
Launch mock with `--scenario uninit-pose`; assign order →
`AGV_POSITION_NOT_INITIALIZED`.

### S15 — Stitch guard: `order_update_id <= active`
With order `demo-order-001` order_update_id=1 active, send
update with order_update_id=0 → `STITCH_REJECTED`.

### S16 — Stitch guard: AGV past stitch node
Wait until mock progresses past stitch node, then send an
order update whose first base node is the now-passed node → rejected.

### S17 — Stitch guard: order_id mismatch (3-strike)
Send 3 consecutive order_update messages for an order_id
that mock is NOT executing → after 3 strikes master clears stale
tracking. Inspect log for `mismatch counter`.

### S18 — Action conflict: HARD blocks running pickup
Active order with `pickup` action RUNNING; send instant
action `dropoff` with `blocking_type=HARD` → rejected.

### S19 — Custom action routed through
Send `customBeep` instant action → mock acks FINISHED.

### S20 — Mode flip AUTOMATIC→MANUAL captures queue
Send 2 orders to mock. Before second completes, flip mode
to MANUAL via SIGUSR1. Master captures pending queue into
mode_cancelled_buffer. Verify via:
`ros2 service call /vda5050_master/get_device_status ...`.

### S21 — Mode return MANUAL→AUTOMATIC: resume queue
After S20, flip back to AUTOMATIC (SIGUSR1). ExampleMaster's
`on_mode_changed` override auto-calls `resume_mode_cancelled_queue()`.

### S22 — Mode-blocked instant action
With mock in MANUAL, send a `customBeep` instant
action → master rejects pre-send.

### S23 — State timeout: stop publishing → `[TIMEOUT]`
SIGSTOP mock (`kill -STOP $(pgrep -f mock_client)`). Wait
30+ s. Master logs `[WARN] [TIMEOUT] MockMfg/MOCK001 state heartbeat
lost`.

### S24 — State resumed → `[RECOVERED]`
SIGCONT the mock from S23. Within 1 tick, master logs
`[INFO] [RECOVERED] MockMfg/MOCK001 state heartbeat resumed`.

### S25 — Malformed state → schema reject
Launch mock with `--scenario malformed-state`. T2 logs
`[WARN] [AGV] Dropping malformed state from MockMfg/MOCK001: 1 schema
error(s)` once at startup, then normal `[STATE]` thereafter.

---

## Appendix — sample YAML payloads

### `/tmp/order.yaml` — base order (used in S4)

```yaml
manufacturer: 'MockMfg'
serial_number: 'MOCK001'
order:
  header:
    header_id: 1
    timestamp: 1747200000000
    version: '2.0.0'
    manufacturer: 'MockMfg'
    serial_number: 'MOCK001'
  order_id: 'demo-order-001'
  order_update_id: 0
  nodes:
    - {node_id: 'N0', sequence_id: 0, released: true, actions: [],
       node_position: [{x: 0.0, y: 0.0, theta: [0.0], allowed_deviation_x_y: [],
                        allowed_deviation_theta: [], map_id: 'warehouse_floor1',
                        map_description: []}],
       node_description: []}
    - {node_id: 'N1', sequence_id: 2, released: true, actions: [],
       node_position: [{x: 5.0, y: 0.0, theta: [0.0], allowed_deviation_x_y: [],
                        allowed_deviation_theta: [], map_id: 'warehouse_floor1',
                        map_description: []}],
       node_description: []}
  edges:
    - {edge_id: 'E01', sequence_id: 1, start_node_id: 'N0', end_node_id: 'N1',
       released: true, actions: [], edge_description: [], max_speed: [],
       max_height: [], min_height: [], orientation: [], orientation_type: [],
       direction: [], rotation_allowed: [], max_rotation_speed: [], trajectory: [],
       length: []}
  zone_set_id: []
```

### `/tmp/order_update.yaml` — order update extending the base (used in S5)

Same shape as `order.yaml`, but with `order_update_id: 1` and
additional `nodes` (`N2`, `N3`) + `edges` (`E12`, `E23`). When the
mock has progressed to `last_node_id=N1`, this update is accepted at
the stitch point.

---

## Known issues / debugging tips

1. **Master ↔ AGV `header.version` mismatch.** The master's
   `ProtocolAdapter` uses `Version="v2"` for BOTH topic path AND
   `header.version` JSON field, but the master's schema validator
   requires `header.version == "2.0.0"`. `mock_client` works around
   this by bypassing `ProtocolAdapter::publish` and emitting messages
   with `header.version = "2.0.0"` on the `rmf2/v2/...` topic path.
   The same workaround should be applied to the master's own outbound
   side at some point — it would currently fail-validate its own
   messages if it ever subscribed to itself.
2. **Retained Connection ONLINE not always delivered on first
   subscribe.** Some Paho client / broker combinations don't deliver
   the retained payload reliably to a subscribe-after-publish race.
   `mock_client` mitigates by re-publishing Connection ONLINE on every
   tick (per spec §6.14, 15 s heartbeat — we run faster).
3. **`Order validation failed for ...: 1 error(s)`** without
   specifics. The async OrderPublisher chain logs only the count, not
   the error description. To debug a rejected order, add a temporary
   `VDA5050_ERROR(..., result.errors[0].error_description.value_or(\"\"))`
   in `vda5050_core/src/master/agv.cpp`'s `publish_order` rejection
   site, rebuild, re-run. Follow-up: surface validator errors through
   the planned `AssignmentEvents` topic.

## Next milestones

- **`interop_runbook.md`** (`mock_fms` + Saurabh's `example_client`)
  covers cross-implementation interop testing.
- **Real FMS / sim / hardware integration** — deferred post-V0.
- **Automated `launch_test` wrapper** — defer until CI hosts a broker
  reliably.
