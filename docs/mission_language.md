# Mission Language

Authoring guidelines for the current single-drone mission YAMLs. The canonical schema is in `docs/mission_v1.md`; this page keeps a concise view of what to set, how the FSM interprets it, and what will change once multi-drone support lands.

## What We Support Today
- Single vehicle, ENU mission files loaded via `mission_runner.ros__parameters.mission_file` in `config/sim/default.yaml`.
- Three modes: `explicit` (inline waypoints), `precomputed` (route file), `coverage` (polygon → planner-generated lanes).
- Inspection is optional and no longer drives fallbacks; it just gates mission advancement when `require_ack` is set.
- Fallbacks are limited to battery triggers; link-loss is owned by PX4’s native failsafe when setpoints stop.

## Minimum Skeleton
```yaml
api_version: 1
mode: explicit
defaults:
  altitude_m: 2.5
  hover_time_s: 2.0
  cruise_speed_mps: 1.0
waypoints:
  - [0.0, 0.0]
  - [2.0, 0.5]
inspection:
  enable: true
  timeout_s: 4.0
fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]
land_on_finish: true
```
- Keep `api_version: 1` to guard parser changes.
- Waypoints are ENU; Z and yaw default to `defaults.altitude_m` and `defaults.yaw_deg` if present.
- Missions fail fast when a waypoint breaches `world_bounds` from `config/sim/default.yaml` or `cruise_speed_mps` falls outside `cruise_speed_limits`.

## Field Checklist
- `mode`: `explicit`, `precomputed`, or `coverage`. Coverage requires `area.polygon`.
- `defaults`: `altitude_m`, `hover_time_s`, `cruise_speed_mps` are the essentials; `fov_deg` and `overlap` influence coverage planning.
- `inspection`: `enable`, `timeout_s`, optional `require_ack`, `image_topic`. `LOW_LIGHT` only toggles the torch via `torch_controller`.
- `fallback`: map battery triggers to actions (`return_home`, `land`, `hold:<duration>`, `increase_hover:<duration>`, `resume`). Unknown actions are ignored with a warning.
- `land_on_finish`: appends `return_home` + `land` when the route is complete.

## Looking Ahead: Multi-Drone
- The ongoing multi-drone branch will likely add per-vehicle mission sections or multiple mission files with a shared supervisor. Keep mission content vehicle-agnostic where possible so it can be reused.
- Per-vehicle ports/namespaces (PX4 UDP, XRCE agent command, ROS 2 topics) will need to be parameterised; avoid hardcoding them inside mission files.

## Further Reading
- Deep-dive, examples, and FSM internals: `docs/mission_v1.md`.
- Runtime parameters (world, bounds, agent, battery): `config/sim/default.yaml`.
