# vda5050_master_ros2

ROS 2 wrapper for the VDA5050 fleet master. Implements the FMS-side of
the VDA5050 v2.0.0 protocol: dispatches orders, validates them, tracks
per-AGV state, and exposes both an MQTT surface (to AGVs) and a ROS 2
surface (to FMS clients).

## What's in the package

| Component | Purpose |
|---|---|
| `VDA5050MasterROS2` library | Wraps `vda5050_core::master::VDA5050Master` + 10 ROS 2 services + 4 per-AGV topics |
| `example_master` | Reference master executable — loads `sample_map.json`, advertises all endpoints |
| `mock_client` | Standalone single-AGV pure-MQTT stub (no rclcpp) for fault-injection testing |
| `example_client` | Ported VDA5050 client framework + `MinimalStatePublisher` for interop testing |
| `mock_fms` | FMS-side driver: scripted end-to-end scenarios with pass/fail exit codes |

Per-AGV ROS 2 topics created on first `Connection ONLINE`:
- `/<ns>/<mfg>/<serial>/{state,connection,factsheet,order_status}`

Services (10) under `/<ns>/`:
- `assign_order`, `assign_instant_actions`
- `onboard_agv`, `offboard_agv`
- `get_device_status`, `get_order_status`, `get_loaded_map`, `get_master_broker_status`
- `resume_mode_cancelled_queue`, `discard_mode_cancelled_queue`

## Features implemented

**Order dispatch (master → AGV)**
- Validator chain: schema → graph → traversability → pre-send (AGV_NOT_ONBOARDED, AGV_OFFLINE, AGV_NO_STATE_YET, AGV_MODE_NOT_AUTO, AGV_POSITION_NOT_INITIALIZED, STATE_UNKNOWN)
- Order stitching (horizon extension via `order_update_id`), lifecycle tracking, mismatch recovery
- Instant actions with mode-aware allowlist

**AGV state ingestion (AGV → master)**
- Per-AGV cached State / Connection / Factsheet / Visualization with incoming schema validation
- Connection + State heartbeat monitoring; operational-state demotion on timeout
- Last-will detection (`kill -9` → CONNECTION_BROKEN)
- Mode-cancelled queue: capture on AUTOMATIC → non-AUTO flip; resume / discard on operator action

**Fleet operations**
- Onboard / offboard (idempotent), multi-AGV
- Loaded map + factsheet alignment validator (vehicle-vs-map structural diff)
- MQTT broker disconnect / reconnect tracking with observability snapshot

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd <workspace>
colcon build --packages-select vda5050_master_ros2
source install/setup.bash
```

## Run

```bash
# Broker (one time per machine)
mosquitto -v   # or: sudo systemctl start mosquitto

# Master
ros2 run vda5050_master_ros2 example_master

# AGV-side stub (for end-to-end testing)
ros2 run vda5050_master_ros2 mock_client --serial S001 --mfg Manufacturer
# OR for interop testing
ros2 run vda5050_master_ros2 example_client

# FMS-side driver (for end-to-end testing)
ros2 run vda5050_master_ros2 mock_fms --scenario happy-path
```

## Running the examples

**Master only** — exercise the 10 ROS 2 services with raw `ros2 service call` (no AGV side).
```bash
ros2 run vda5050_master_ros2 example_master &
ros2 service call /vda5050_master/onboard_agv \
    vda5050_master_ros2/srv/OnboardAGV \
    "{manufacturer: 'Manufacturer', serial_number: 'S001'}"
```
- Expected response: `status: 0` (SUCCESS). A second call returns `status: 1` (ALREADY_ONBOARDED).
- Try also: `get_loaded_map`, `get_master_broker_status`, `assign_order` (rejects pre-AGV with `decision: 2` AGV_OFFLINE).

**Master + `mock_client`** — full order lifecycle plus AGV-side fault injection (validator rejections, broker bounce, last-will on kill -9, multi-AGV).
```bash
ros2 run vda5050_master_ros2 example_master &                       # T1
ros2 run vda5050_master_ros2 mock_client --serial S001 --mfg Manufacturer    # T2
```
- Expected: T1 (master) logs `[CONN] ONLINE Manufacturer/S001` within ~1 s.
- From a 3rd terminal, dispatch a sample order:
  ```bash
  ORDER_DIR=$(ros2 pkg prefix vda5050_master_ros2)/share/vda5050_master_ros2/sample_data/orders
  ros2 service call /vda5050_master/onboard_agv \
      vda5050_master_ros2/srv/OnboardAGV \
      "{manufacturer: 'Manufacturer', serial_number: 'S001'}"
  ros2 service call /vda5050_master/assign_order \
      vda5050_master_ros2/srv/AssignOrder "$(cat $ORDER_DIR/happy_path.yaml)"
  ```
  Expected: `decision: 0` (ASSIGNED), master `[STATE]` log advances `last_node_id: N0 → N1`, `phase: COMPLETED`.
- Fault-injection variants live under `sample_data/orders/`: `schema_reject.yaml`, `traversability_reject_*.yaml`, `stitch_base.yaml` + `stitch_update.yaml`.

**Master + `example_client` + `mock_fms`** — cross-implementation interop with scripted scenarios (custom CLI harness; exit code 0 = PASS).
```bash
ros2 run vda5050_master_ros2 example_master &                # T1
ros2 run vda5050_master_ros2 example_client &                # T2
ros2 run vda5050_master_ros2 mock_fms --scenario happy-path  # T3
echo $?    # 0 = PASS
```
- Scenarios: `happy-path`, `stitch`, `broker-bounce`, `onboard-flood`.
- Each step prints `OK: <step>` or `FAIL: <step>` to stdout; final exit code rolls them up.

## Sample data

Under `sample_data/`:

```
sample_map.json                            # 4-node grid loaded by example_master
orders/                                    # YAML orders for `ros2 service call`
  happy_path.yaml
  stitch_base.yaml, stitch_update.yaml
  schema_reject.yaml
  traversability_reject_*.yaml
instant_actions/                           # YAML instant actions
  state_request.yaml
  factsheet_request.yaml
  custom_beep.yaml
```

Installed to `share/vda5050_master_ros2/sample_data/` for runtime use:

```bash
ORDER_DIR=$(ros2 pkg prefix vda5050_master_ros2)/share/vda5050_master_ros2/sample_data/orders
ros2 service call /vda5050_master/assign_order \
    vda5050_master_ros2/srv/AssignOrder \
    "$(cat $ORDER_DIR/happy_path.yaml)"
```
