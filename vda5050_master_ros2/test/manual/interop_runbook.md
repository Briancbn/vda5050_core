# End-to-End Interop Runbook — `example_master` + `example_client` + `mock_fms`

Scenarios for exercising the V0 `vda5050_master_ros2` master against an
independent VDA5050 client implementation (Saurabh's `example_client`,
ported in `examples/example_client.cpp`) driven by `mock_fms` from the
FMS side. Validates cross-implementation interoperability of the
master.

> **Scope.** This runbook covers interop testing with an independent
> VDA5050 client. Companion to:
>
> - `mock_client_runbook.md` — master + in-tree `mock_client` AGV stub.
>   Used for AGV-side fault-injection scenarios we control end-to-end.
> - Unit suite — gtest, run via `colcon test`.
>
> Scenarios here focus on **the master correctly interoperating with
> code it didn't co-design** — protocol compatibility, FMS-driven
> dispatch patterns, real-timing races.
>
> **Must-walk** scenarios are the ones to verify before tagging V0
> done. **Recommended** scenarios are demo recipes or rare-but-real
> failure modes worth knowing.

## Why some `mock_client_runbook` scenarios DON'T port here

The mock_client runbook's value is fault injection in a stub we
control. Many of those scenarios need AGV-side misbehaviour
(schema-violating messages, mid-order pose loss, fake mode flips).
`example_client` always sends valid messages — we can't easily make
it lie. The following scenarios stay mock_client-only:

| Scenario | Why no port |
|---|---|
| Schema reject | `example_client` always emits valid JSON |
| Traversability reject (limit overrun) | Mocked factsheet not exposed |
| Uninitialised-pose | `MinimalStatePublisher` always sets `position_initialized=true` |
| Malformed-State | Can't inject without code edits |
| MANUAL mode flip | No signal handler in `example_client` |
| Mode-cancelled queue capture | Same — mode never flips |
| State-timeout (kill -STOP) | Brittle on a real process |

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
colcon build --packages-select vda5050_master_ros2
source install/setup.bash
```

## Conventions

Throughout this runbook:

- **T1** = `mosquitto -v`
- **T2** = `ros2 run vda5050_master_ros2 example_master`
- **T3** = `ros2 run vda5050_master_ros2 example_client` (AGV-side)
- **T4** = `ros2 run vda5050_master_ros2 mock_fms ...` (FMS driver)
- **T5** = an optional `ros2 service call` / `mosquitto_sub` driver
  terminal for ad-hoc poking

## Shutdown — between scenarios

The default is **Ctrl-C in each terminal**. `signal_handler` in
example_master and example_client catches SIGINT and runs
`shutdown()` — graceful OFFLINE publishes, threads join, mqtt
disconnects cleanly. No `pkill` / `kill -9` required.

If you started a process detached (`nohup … &` from a different
terminal, no foreground Ctrl-C path) and need to stop it:

```bash
# Find the PID — exact name match, not regex (avoids matching this shell)
pgrep -x example_master  # or example_client
kill -INT <pid>          # graceful shutdown via signal_handler
# Only if it's hung 5s+ after SIGINT:
kill -9 <pid>            # last resort — skips OFFLINE publish
```

Do NOT use `pkill -f vda5050_master_ros2` — the `-f` flag matches
the *entire command line*, so if your shell's cwd contains
`vda5050_master_ros2/` the shell sees itself in the match and kills
itself before the children.

## `example_master` log markers

Same markers as mock_client — see `mock_client_runbook.md` §"example_master log markers".
The contract is unchanged here.

## `example_client` log lines

The expected progression lines are:

| Line | Source |
|---|---|
| `MQTT client [vda5050_client] connected to tcp://localhost:1883` | startup |
| `Initializing NavigationStrategy ...` | startup |
| `Initializing MinimalStatePublisher (test extension) ...` | startup |
| `Received order with order_id: …` | inbound order |
| `Adding nodes` | NavigationStrategy load |
| `Pushing node with sequence_id [N] to event queue` | per node |
| `Navigating to [x, y] with sequence N` | per node |
| `[MinimalStatePublisher] arrived at node seq=N; pushing NodeAckUpdate` | per node |

`Adding nodes` appears **once per (order_id, order_update_id)** — the
SECTION 1 guard breaks Saurabh's original infinite-walk loop. If you
see it spamming, the guard is broken.

## `mock_fms` modes summary

| Mode | Flag | Purpose |
|---|---|---|
| Scripted | `--scenario <name>` | Automated walks for runbook scenarios — exit code asserts pass/fail |
| Interactive | `--interactive` (default) | REPL — operator drives the master like a real FMS console |
| Monitor | `--monitor SERIAL` | Pretty-printed live state/connection/order_status diff for one AGV |

---

# Tier-1 Scenarios (must-walk)

## S1 — Master-broker bounce

**Goal:** Master survives a broker outage; reconnect count + last
disconnect time visible to FMS.

**Setup:** T1, T2, T3 running. `mock_fms` not needed for this one.

```bash
# T5 — snapshot before
ros2 service call /vda5050_master/get_master_broker_status \
    vda5050_master_ros2/srv/GetMasterBrokerStatus '{}'
# expect: connected=1 reconnect_count=N

# T1 — bounce
# (in T1 terminal: Ctrl-C mosquitto, then re-run `mosquitto -v`)
sudo systemctl restart mosquitto    # alternative

# T2 — expect markers
# [BROKER] master lost MQTT broker connection
# [BROKER] master reconnected to MQTT broker

# T5 — snapshot after
ros2 service call /vda5050_master/get_master_broker_status \
    vda5050_master_ros2/srv/GetMasterBrokerStatus '{}'
# expect: connected=1 reconnect_count=N+1 last_disconnect_at>0
```

**Verify:** Bounce visible to FMS; AGV reconnects automatically.

## S2 — Onboard idempotence

**Goal:** Onboarding the same (mfg, serial) twice is safe — second
call returns SUCCESS without disturbing state.

```bash
# T4
ros2 run vda5050_master_ros2 mock_fms --interactive
fms> onboard Manufacturer S001
  onboard status=0
fms> onboard Manufacturer S001
  onboard status=0    # idempotent
fms> status S001
  has_state=True has_connection=True ...
```

**Verify:** Second onboard returns 0 (SUCCESS); subsequent `status`
shows AGV still has cached state.

## S3 — GetLoadedMap

**Goal:** FMS sees the map the master booted with.

```bash
fms> map
  status=0 map_id=warehouse_floor1 version=1.0 (4 nodes, 4 edges)
```

## S4 — Happy-path order (FMS-driven, scripted)

**Goal:** Full FMS-side dispatch + assertion path is green end-to-end.

```bash
# T1, T2, T3 running
# T4
ros2 run vda5050_master_ros2 mock_fms --scenario happy-path
# expect: PASS: happy-path, exit code 0
```

The scripted walk runs onboard → wait for state → AssignOrder → wait
for OrderStatus topic to report `last_node_id=N1`. Each step prints a
✓ on success.

## S5 — Horizon extension stitch

**Goal:** A base+horizon order, then an update releasing the horizon
node, completes the extended path.

```bash
ros2 run vda5050_master_ros2 mock_fms --scenario stitch
# expect: PASS: stitch, exit code 0
```

Internally: send order with N0/N1 released + N2 horizon, wait for AGV
to reach N1 (parked at last released), send update with N2 released,
wait for AGV to reach N2. Decision may be `ASSIGNED` (immediate) or
`STITCH_QUEUED` (waiting for state at stitch point) — both are valid.

## S6 — kill -9 example_client → BROKEN marker

**Goal:** Master sees AGV's last-will when the AGV process is killed.

```bash
# T3 (running)
pgrep -f example_client
kill -9 <pid>

# T2 expect (within ~5s):
# [BROKEN] AGV last-will fired (connection_state=CONNECTIONBROKEN)
```

**Note:** This is the only failure-injection scenario that works on
`example_client` because the last-will is set via `mqtt_->set_will()`
in `MinimalStatePublisher::prepare_will_and_connect()` (SECTION 2),
which the broker delivers even on `kill -9` (no graceful close).

## S7 — Multi-AGV onboard flood

**Goal:** Master accepts 3 AGVs being onboarded back-to-back without
race conditions.

```bash
ros2 run vda5050_master_ros2 mock_fms --scenario onboard-flood
# expect: PASS: onboard-flood, exit code 0
```

Scripted walk onboards FLOOD01/FLOOD02/FLOOD03 then offboards them.
No state is published for these AGVs (no example_client backing them),
which is fine — onboard is purely master-side bookkeeping.

## S8 — Live monitor during order completion

**Goal:** Operator can watch state/connection/order_status changes in
real time while an order runs.

```bash
# T4 — start monitor
ros2 run vda5050_master_ros2 mock_fms --monitor S001

# T5 — dispatch order
ros2 service call /vda5050_master/assign_order \
    vda5050_master_ros2/srv/AssignOrder \
    "$(cat /tmp/happy_order.yaml)"

# T4 expect output like:
# [16:42:01] CONN  ONLINE
# [16:42:01] STATE order=/0 node=- driving=False mode=AUTOMATIC
# [16:42:05] STATE order=demo-l2-happy/0 node=N0 driving=False ...
# [16:42:06] STATE order=demo-l2-happy/0 node=N1 ...
# [16:42:06] ORDER phase=3 last_node=N1 base/horizon=2/0
```

`/tmp/happy_order.yaml` per `play_with_master_and_mock.md` §5a.

# Tier-2 Scenarios (recommended)

## S9 — `mock_fms` interactive demo flow

**Goal:** Walking a stakeholder through the master using the REPL —
no edit-and-recompile, no YAML wrangling, no raw service-call typing.

```bash
ros2 run vda5050_master_ros2 mock_fms --interactive
fms> map
fms> onboard
fms> status
fms> order
fms> order-status
fms> broker
fms> exit
```

**Verify:** Each command returns clean output (no exceptions). Useful
to retain post-V0 as the "FMS console" for new endpoints.

## S10 — Back-to-back orders (real-timing race surface)

**Goal:** Three sequential orders, each completing before the next is
dispatched. Exposes timing races that the mock_client.s predictable behaviour hides.

```bash
ros2 run vda5050_master_ros2 mock_fms --interactive
fms> onboard
fms> order
  decision=0 order_id=repl-0
# wait for completion via order-status
fms> order-status
fms> order                # second
fms> order                # third
```

Between each: `fms> order-status` should report `last_node_id=N1`
before the next `fms> order`. If a second `order` returns
`decision != 0`, that's a real concurrency bug worth investigating —
the master is supposed to accept once the prior order completes.

**Note:** This scenario uses the **same** `repl-N` order_ids, so
expect order #2 onwards to potentially trigger stitch logic (since
order_id collides with the active one). Use a unique order_id by
restarting mock_fms between orders, or accept stitch behaviour and
verify decision is ASSIGNED-or-STITCH_QUEUED.

## S11 — Live status query while order is mid-flight

**Goal:** Snapshot consistency — `get_order_status` mid-order reflects
the AGV's actual position, not stale data.

```bash
# T4 monitor
ros2 run vda5050_master_ros2 mock_fms --monitor S001 &

# T5 dispatch and poll
ros2 service call /vda5050_master/assign_order \
    vda5050_master_ros2/srv/AssignOrder \
    "$(cat /tmp/happy_order.yaml)"

# poll order-status every 500ms (5 times)
for i in 1 2 3 4 5; do
  ros2 service call /vda5050_master/get_order_status \
      vda5050_master_ros2/srv/GetOrderStatus \
      "{manufacturer: 'Manufacturer', serial_number: 'S001'}" \
      | grep -E "(last_node_id|phase)"
  sleep 0.5
done
```

**Verify:** Polled `last_node_id` matches the monitor's most-recent
`STATE` line at every poll. No regression to empty.

## S12 — Broker bounce status query

**Goal:** `mock_fms` scripted scenario shows broker connect state and
guides the operator through a bounce.

```bash
ros2 run vda5050_master_ros2 mock_fms --scenario broker-bounce
# expect: ✓ broker status reports connected (reconnect_count=N)
#         (operator: run `sudo systemctl restart mosquitto` and re-run ...)

# T1: sudo systemctl restart mosquitto
sleep 3
ros2 run vda5050_master_ros2 mock_fms --scenario broker-bounce
# expect: reconnect_count = N+1
```

# Walked verification log

Record each walk-through here with date + AGV serial(s) used + notes
on any non-deterministic behaviour observed.

| Date | Scenario | Result | Notes |
|---|---|---|---|
| 2026-05-14 | S4 (happy-path scripted) | 5/5 PASS | Deterministic with proper teardown between runs |
| 2026-05-14 | S5 (stitch scripted) | 5/5 PASS | After fixing scenario_stitch to send stitch-point-only update (spec §6.6.2 required) |
| 2026-05-14 | S7 (onboard-flood) | PASS | Three AGVs onboarded sequentially |
| 2026-05-14 | S12 (broker-bounce snapshot) | PASS | Reports connect status + reconnect_count |
| | | | |
| | | | |

---

## Appendix — files referenced

- `examples/example_client.cpp` — VDA5050 client; SECTION 1 is Saurabh's
  Handler/Strategy framework, SECTION 2 is the test-only
  `MinimalStatePublisher`.
- `examples/mock_fms.cpp` — FMS-side driver; 3 modes.
- `examples/example_master.cpp` — the master process under test.
- `sample_data/sample_map.json` — the 4-node grid map the master
  loads at startup; orders must use these node IDs.
- `vda5050_doc/manual/play_with_master_and_mock.md` — companion play
  doc covering the mock_client; `/tmp/happy_order.yaml` is
  defined there in §5a.
- `vda5050_doc/manual/ros2_endpoints_cheatsheet.md` — reference for
  all 10 ROS 2 service payloads (used implicitly throughout this
  runbook).
