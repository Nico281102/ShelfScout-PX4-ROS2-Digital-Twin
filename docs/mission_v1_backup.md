# Mission Language v1

This document summarises the YAML schema consumed by `overrack_mission`.

```yaml
api_version: 1
frame_id: map

defaults:
  altitude_m: 2.5
  hover_time_s: 2.0
  cruise_speed_mps: 1.0
  fov_deg: 70
  overlap: 0.25

mode: explicit            # or "coverage"
waypoints:
  - [0.0, 0.0]
  - [2.0, 0.0]
  - [2.0, 1.0]
  - [0.0, 1.0]
# when mode=coverage provide:
# area:
#   polygon: [[0,0],[12,0],[12,6],[0,6]]

inspection:
  enable: true
  timeout_s: 4.0

fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]
  low_light: ["increase_hover:2s"]
  link_lost: ["hold:5s", "land"]

avoidance: stop
ignore_gps: true
land_on_finish: false
```

* `mode: explicit` uses the `waypoints` list as-is; `mode: coverage` runs the
  lawnmower planner on the provided `area.polygon` using the camera defaults.
* `inspection.enable` inserts an `Inspect` stage with the given timeout. The
  inspection node should publish `OK`, `SUSPECT`, or `LOW_LIGHT` on
  `overrack/inspection`.
* `fallback` entries list actions executed in order when a trigger fires. The
  supported actions are:
  * `return_home` – fly to the home waypoint.
  * `land` – issue `VEHICLE_CMD_NAV_LAND`.
  * `hold:5s` – hold position for the given duration.
  * `increase_hover:2s` – extend the current/next hover window.

Triggers currently recognised: `battery_warning`, `battery_critical`,
`low_light`, `link_lost`. Additional triggers may be added later without
breaking compatibility.
