# Mission Language v1
This document defines the version 1 YAML schema consumed by `overrack_mission` for autonomous shelf-inspection missions.

## Quick Reference
| Key | Type | Required | Description |
| --- | --- | --- | --- |
| `api_version` | int | yes | Must be `1`; used to guard parser changes. |
| `defaults` | n/a | no | Non supportato: rimosso in v1, imposta quota/hover nel route file. |
| `route_file` | string | yes | Path to a YAML route loaded by the mission runner (precomputed mode is implicit). |
| `inspection` | map | optional | Enables the inspection stage, timeout, and optional acknowledgement gating. |
| `fallback` | map | optional | Trigger → action list map; defaults ensure safe landing if omitted. |
| `return_home_and_land_on_finish` | bool | optional | When true the FSM executes `return_home` + `land` at the end. |

## Defaults Block
Non più supportato: il parser rifiuta `defaults`. Quota e hover vanno definiti nel route file (`default_altitude_m`, `default_hover_s`, `hover_s` per step).

## Precomputed Routes (Only Mode)
Missions implicitly use precomputed routes; no `mode` flag is required (any other value is rejected). Reference an external route file:

```yaml
route_file: routes/overrack_default.yaml
```
The referenced YAML follows the schema in `planning/precomputed_routes.py`, letting each leg define `position`, optional `hover_s`, `inspect`, `yaw_deg`, and named actions (e.g., `snapshot`, `align_camera`). Route-level actions are non-blocking tags; flight commands such as `land` are not supported here (use fallbacks or `return_home_and_land_on_finish`).

## Example Mission
```yaml
api_version: 1
route_file: routes/overrack_default.yaml
inspection:
  enable: true
  timeout_s: 4.0
fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]
return_home_and_land_on_finish: true
```

## Workspace Bounds and Cruise Speed Limits
The runtime enforces the indoor volume declared via the ROS parameter `world_bounds.{x,y,z}` (meters in PX4’s NED frame; see `config/sim/default.yaml`). Every ENU waypoint is mirrored into those bounds during parsing; if any `x`, `y`, or `z` lies outside the envelope the mission loader aborts with a descriptive error. At runtime, PX4 setpoints are also clamped to the same box after subtracting the spawn offset measured from `/fmu/out/vehicle_local_position`, ensuring the drone cannot drift through walls even if a waypoint was hand-edited.

La cruise speed non viene più validata né usata dal mission runner: la velocità di crociera è gestita dai parametri PX4 (es. MPC_XY_VEL_MAX).

## Inspection Configuration
Inspection inserts a dedicated stage after each hover so that `inspection_node.py` can classify the rack slot. The block looks like:

```yaml
inspection:
  enable: true              # default false
  timeout_s: 4.0            # maximum dwell time awaiting inspection result
  require_ack: false        # optional; if true mission waits for explicit OK/SUSPECT (LOW_LIGHT only toggles the torch)
  image_topic: /camera/image_raw  # overrides default ROS topic for the node
```
- When disabled, the mission runner simply respects `hover_time_s` and advances.
- When enabled, the inspector publishes `overrack/inspection` events that feed metrics and torch control; they no longer trigger FSM fallbacks.
- Setting `require_ack` to `true` forces the FSM to wait for an inspection verdict (or timeout) before advancing to the next waypoint.

## Fallback Triggers and Actions
Fallback entries map trigger names to ordered lists of actions:

```yaml
fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]
```

### Supported Triggers
- `battery_warning`, `battery_critical` – derived from PX4 `BatteryStatus` thresholds.
- Link-loss is handled directly by PX4 failsafes when Offboard setpoints stop; any `link_lost` entries in a mission file are ignored by the FSM.
- Inspection events (`OK`, `SUSPECT`, `LOW_LIGHT`) do not start fallbacks; `LOW_LIGHT` only toggles the torch via `torch_controller`.

### Supported Actions
| Action | Effect |
| --- | --- |
| `return_home` | Fly to the stored home waypoint using current cruise speed, then hold. |
| `land` | Issue `VEHICLE_CMD_NAV_LAND` immediately. |
| `hold:<duration>` | Pause in place for `<duration>` seconds (e.g., `hold:5s`). |
| `increase_hover:<duration>` | Extend the next hover window by the specified amount. |
| `resume` | Explicitly continue the mission after any temporary hold (optional). |

Actions execute sequentially. If an action is unknown the parser logs a warning and skips it, keeping backward compatibility with older plans. When `return_home_and_land_on_finish: true` is set, the mission runner implicitly appends `["return_home", "land"]` once all waypoints are complete.

## Mission Runner Internals

The ROS 2 package `overrack_mission` exposes three primary entry points via `colcon`/`setup.py`:

| Console script | Source file | Purpose |
| --- | --- | --- |
| `mission_runner` | `overrack_mission/nodes/mission_control_node.py` | Loads the mission YAML, drives the `MissionStateMachine`, and publishes PX4 setpoints. |
| `inspection_node` | `overrack_mission/nodes/inspection_node.py` | Subscribes to camera images + mission events, emits inspection verdicts (`OK`, `SUSPECT`, `LOW_LIGHT`). |
| `mission_metrics` | `overrack_mission/nodes/metrics_node.py` | Aggregates timestamps, fallback causes, and inspection results into CSV artefacts under `data/metrics/`. |

### Execution Flow
1. `mission_runner` parses the YAML (this document), loads the referenced precomputed route, and injects it into the mission FSM.
2. `MissionController` translates waypoints into PX4 Offboard setpoints (`/fmu/in/vehicle_local_position_setpoint`, `/fmu/in/trajectory_setpoint`).
3. `MissionStateMachine` enforces the phase progression (ARM → TAKEOFF → TRANSIT → HOVER/INSPECT → HOLD/RETURN) and calls fallback handlers when triggers fire.
4. Telemetry helpers (`px4io/telemetry.py`) subscribe to PX4 topics such as `/fmu/out/vehicle_status`, `/fmu/out/vehicle_local_position`, `/fmu/out/battery_status` to gate transitions (arming allowed, offboard ready, battery thresholds, heartbeat timeout).
5. When `inspection.enable` is true, the state machine notifies `inspection_node`, which publishes verdicts on `overrack/inspection`. Those verdicts can satisfy `require_ack` and feed torch/metrics consumers; they no longer trigger FSM fallbacks.
6. `mission_metrics` listens to both mission state and inspection streams, recording per-phase timing plus the final outcome; files are written to `data/metrics/<timestamp>/` when the mission ends.

### Step Semantics and Stabilization Logic
Each entry in a route step list is treated as an atomic navigation goal: the controller flies to the specified position/yaw, enters `HOVER`, and immediately fires any configured inspection or action hooks. There is currently no delay between entering `HOVER` and triggering the action, so a `snapshot` can occur while PX4 is still finishing the last few centimetres of translation or yaw alignment.

Missions that require perfectly steady captures can encode a two-step convention at the route level:

```yaml
- name: F_arrival
  position: [0.0, 3.5, 1.2]
  hover_s: 1.0
  inspect: false

- name: F_photo
  position: [0.0, 3.5, 1.2]
  hover_s: 10.0
  inspect: true
  action: snapshot
```

`F_arrival` commands the transit and provides a short dwell so PX4’s attitude and position controllers can settle, while the following `F_photo` duplicates the setpoint but keeps the vehicle in a longer hover window where inspections or actions run. This split-step approach yields deterministic timing for `inspection_node` and `metrics_node` consumers and prevents race conditions where a camera trigger would otherwise fire during the final approach. Future FSM revisions may add a built-in stabilization grace period, but until then stabilization is handled explicitly in the mission’s route definition.

### Key Topics and Parameters
| Topic | Direction | Produced by | Notes |
| --- | --- | --- | --- |
| `/fmu/in/offboard_control_mode` | publish | `mission_runner` | Enables position control during Offboard flight. |
| `/fmu/in/trajectory_setpoint` | publish | `mission_runner` | XY setpoints generated from the loaded route. |
| `/fmu/out/vehicle_status` | subscribe | Telemetry helpers | Used to detect `nav_state`, arming state, failsafes. |
| `/fmu/out/battery_status` | subscribe | Telemetry helpers | Feeds `battery_warning`/`battery_critical` triggers. |
| `/overrack/inspection` | publish | `inspection_node` | Verdicts (`OK`, `SUSPECT`, `LOW_LIGHT`, custom tags). |
| `/overrack/mission_state` | publish | `mission_runner` | High-level FSM state changes (useful for dashboards/scripts). |

Relevant ROS parameters:
- `mission_file` (string, default `config/mission.yaml`): path to the YAML plan. Overridden via `--ros-args -p mission_file:=...` when launched by `run_system.sh` (or legacy `run_ros2_system.sh`).
- `sim.disable_link_after_s` (float, default `-1.0`): `<0` disables link-loss simulation, `0` triggers immediately, `>0` triggers after N seconds by stopping Offboard setpoints/heartbeats so PX4 native failsafe engages.
- `image_topic` (string): forwarded to `inspection_node` to match your camera stream.
- `agent_cmd` (string): handled outside of ROS but documented in `.env` as `SSDT_AGENT_CMD` for consistency.

### Fallback and Events
Fallback triggers are telemetry-derived events raised inside `mission_runner` (`battery_warning`, `battery_critical`). Link loss is not modelled as a fallback; PX4’s native failsafe takes over if Offboard setpoints stop. Inspection verdicts on `/overrack/inspection` do not alter the FSM; `LOW_LIGHT` is reserved for torch control only.

Actions specified in the YAML are executed by the `MissionStateMachine` as synchronous routines. The controller inhibits new setpoints while actions marked as blocking (`return_home`, `land`) are running to avoid conflicting commands. Non-blocking actions (`hold`, `increase_hover`) simply adjust timers and resume the nominal mission.

For details on how PX4 topics are bridged through Micro XRCE-DDS, refer to `docs/ROS2_PX4_BRIDGING.md`.
