# Mission Language v1
This document defines the version 1 YAML schema consumed by `overrack_mission` for autonomous shelf-inspection missions.

## Quick Reference
| Key | Type | Required | Description |
| --- | --- | --- | --- |
| `api_version` | int | yes | Must be `1`; used to guard parser changes. |
| `frame_id` | string | yes | Reference frame for positions (default `map`). |
| `defaults` | map | yes | Global parameters (altitude, hover time, cruise speed, planner options). |
| `mode` | string | yes | `explicit`, `precomputed`, or `coverage`. Drives how the path is produced. |
| `waypoints` | list | conditional | Ordered `[x, y]` pairs used when `mode: explicit`. |
| `route_file` | string | conditional | Path to a YAML route loaded when `mode: precomputed`. |
| `area.polygon` | list | conditional | Closed polygon (list of `[x, y]`) for `mode: coverage`. |
| `inspection` | map | optional | Enables the inspection stage, timeout, and optional acknowledgement gating. |
| `fallback` | map | optional | Trigger → action list map; defaults ensure safe landing if omitted. |
| `avoidance` | enum | optional | Behaviour when avoidance is triggered (`stop`, `rtl`, `land`). |
| `ignore_gps` | bool | optional | Force PX4 to rely on vision/position estimates indoors. |
| `land_on_finish` | bool | optional | When true the FSM executes `return_home` + `land` at the end. |

## Defaults Block
The `defaults` section seeds the mission planner and inspection behaviour:

```yaml
defaults:
  altitude_m: 2.5          # commanded Z (positive up) in metres
  hover_time_s: 2.0        # dwell time at each inspection point
  cruise_speed_mps: 1.0    # XY speed setpoint between waypoints
  yaw_deg: 0.0             # optional, overrides yaw for explicit and precomputed legs
  fov_deg: 70              # camera field of view for coverage planning
  overlap: 0.25            # fractional overlap between coverage lanes
```

`altitude_m`, `hover_time_s`, and `cruise_speed_mps` are always read; planners fall back to safe defaults if omitted but declaring them makes intent explicit. Coverage missions additionally honour `fov_deg` and `overlap` when generating serpentine paths.

## Mission Modes
### Explicit Mode
Provide `mode: explicit` and a `waypoints` array of `[x, y]` pairs measured in metres relative to `frame_id`. Each waypoint inherits `defaults.altitude_m` and can be extended with inline dictionaries if per-waypoint overrides are needed (e.g., `{x: 2.0, y: 1.0, hover_time_s: 4.0}`).

### Precomputed Route Mode
Set `mode: precomputed` and reference an external route file:

```yaml
route_file: routes/overrack_default.yaml
```
The referenced YAML reuses the same schema used by `planning/precomputed_routes.py`, allowing each leg to specify named actions (e.g., `snapshot`, `align_camera`) that the mission runner publishes as mission-state annotations.

### Coverage Mode
Coverage missions define an area polygon:

```yaml
mode: coverage
area:
  polygon:
    - [0.0, 0.0]
    - [12.0, 0.0]
    - [12.0, 6.0]
    - [0.0, 6.0]
```
`coverage_planner.py` applies the `defaults.fov_deg` and `defaults.overlap` parameters, computes a lawnmower pattern, and produces synthetic waypoints that are fed back into the mission engine. You can still toggle `inspection.enable` and fallback rules exactly as in other modes.

## Workspace Bounds and Cruise Speed Limits
The runtime enforces the indoor volume declared via the ROS parameter `world_bounds.{x,y,z}` (meters in PX4’s NED frame; see `config/sim/default.yaml`). Every ENU waypoint is mirrored into those bounds during parsing; if any `x`, `y`, or `z` lies outside the envelope the mission loader aborts with a descriptive error. At runtime, PX4 setpoints are also clamped to the same box after subtracting the spawn offset measured from `/fmu/out/vehicle_local_position`, ensuring the drone cannot drift through walls even if a waypoint was hand-edited.

Similarly, `cruise_speed_limits: [min, max]` guards the `defaults.cruise_speed_mps` value. Missions that specify a cruise speed outside this range fail fast so operators know when a plan is too aggressive for the current room.

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

Actions execute sequentially. If an action is unknown the parser logs a warning and skips it, keeping backward compatibility with older plans. When `land_on_finish: true` is set, the mission runner implicitly appends `["return_home", "land"]` once all waypoints are complete.

## Optional Flags
- `avoidance`: choose how to react to an avoidance event (`stop`, `rtl`, `land`).
- `ignore_gps`: set to `true` for indoor missions relying on vision/odometry; the PX4 commander switches to the appropriate estimator combination.
- `land_on_finish`: described above; defaults to `false` for hover-and-hold behaviour.

## Example (Explicit Mission)
```yaml
api_version: 1
frame_id: map
mode: explicit
defaults:
  altitude_m: 2.5
  hover_time_s: 2.0
  cruise_speed_mps: 1.0
  fov_deg: 70
  overlap: 0.2
waypoints:
  - [0.0, 0.0]
  - [2.0, 0.0]
  - [2.0, 1.0]
  - [0.0, 1.0]
inspection:
  enable: true
  timeout_s: 4.0
fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]
land_on_finish: true
```

## Example (Coverage Mission)
```yaml
api_version: 1
frame_id: map
mode: coverage
defaults:
  altitude_m: 2.8
  hover_time_s: 1.5
  cruise_speed_mps: 1.2
  fov_deg: 68
  overlap: 0.3
area:
  polygon:
    - [0.0, 0.0]
    - [10.0, 0.0]
    - [10.0, 5.0]
    - [0.0, 5.0]
inspection:
  enable: false
fallback:
  battery_warning: ["return_home", "land"]
ignore_gps: true
```

Use these templates as starting points and keep `api_version: 1` up to date whenever the parser evolves to ensure missions fail fast when the schema changes.

## Mission Runner Internals

The ROS 2 package `overrack_mission` exposes three primary entry points via `colcon`/`setup.py`:

| Console script | Source file | Purpose |
| --- | --- | --- |
| `mission_runner` | `overrack_mission/nodes/mission_control_node.py` | Loads the mission YAML, drives the `MissionStateMachine`, and publishes PX4 setpoints. |
| `inspection_node` | `overrack_mission/nodes/inspection_node.py` | Subscribes to camera images + mission events, emits inspection verdicts (`OK`, `SUSPECT`, `LOW_LIGHT`). |
| `mission_metrics` | `overrack_mission/nodes/metrics_node.py` | Aggregates timestamps, fallback causes, and inspection results into CSV artefacts under `data/metrics/`. |

### Execution Flow
1. `mission_runner` parses the YAML (this document) and instantiates the planner according to `mode` (`explicit`, `precomputed`, `coverage`).
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
| `/fmu/in/trajectory_setpoint` | publish | `mission_runner` | XY setpoints generated from mission waypoints or coverage planner. |
| `/fmu/out/vehicle_status` | subscribe | Telemetry helpers | Used to detect `nav_state`, arming state, failsafes. |
| `/fmu/out/battery_status` | subscribe | Telemetry helpers | Feeds `battery_warning`/`battery_critical` triggers. |
| `/overrack/inspection` | publish | `inspection_node` | Verdicts (`OK`, `SUSPECT`, `LOW_LIGHT`, custom tags). |
| `/overrack/mission_state` | publish | `mission_runner` | High-level FSM state changes (useful for dashboards/scripts). |

Relevant ROS parameters:
- `mission_file` (string, default `config/mission.yaml`): path to the YAML plan. Overridden via `--ros-args -p mission_file:=...` in `run_ros2_system.sh`.
- `sim.disable_link_after_s` (float, default `-1.0`): `<0` disables link-loss simulation, `0` triggers immediately, `>0` triggers after N seconds by stopping Offboard setpoints/heartbeats so PX4 native failsafe engages.
- `image_topic` (string): forwarded to `inspection_node` to match your camera stream.
- `agent_cmd` (string): handled outside of ROS but documented in `.env` as `SSDT_AGENT_CMD` for consistency.

### Fallback and Events
Fallback triggers are telemetry-derived events raised inside `mission_runner` (`battery_warning`, `battery_critical`). Link loss is not modelled as a fallback; PX4’s native failsafe takes over if Offboard setpoints stop. Inspection verdicts on `/overrack/inspection` do not alter the FSM; `LOW_LIGHT` is reserved for torch control only.

Actions specified in the YAML are executed by the `MissionStateMachine` as synchronous routines. The controller inhibits new setpoints while actions marked as blocking (`return_home`, `land`) are running to avoid conflicting commands. Non-blocking actions (`hold`, `increase_hover`) simply adjust timers and resume the nominal mission.

For details on how PX4 topics are bridged through Micro XRCE-DDS, refer to `docs/ROS2_PX4_BRIDGING.md`.
