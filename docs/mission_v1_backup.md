# Mission Language v1

This document summarises the YAML schema consumed by `overrack_mission`.

```yaml
api_version: 1

defaults: {}

route_file: routes/overrack_default.yaml

inspection:
  enable: true
  timeout_s: 4.0

fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]

land_on_finish: false
```

* Missions are always precomputed: `route_file` is required and the `mode` flag is no longer needed.
* `inspection.enable` inserts an `Inspect` stage with the given timeout. The inspection node should publish `OK`, `SUSPECT`, or `LOW_LIGHT` on `overrack/inspection`.
* `fallback` entries list actions executed in order when a trigger fires. The supported actions are:
  * `return_home` – fly to the home waypoint.
  * `land` – issue `VEHICLE_CMD_NAV_LAND`.
  * `hold:5s` – hold position for the given duration.
  * `increase_hover:2s` – extend the current/next hover window.

Triggers currently recognised: `battery_warning`, `battery_critical`. Link loss is handled by PX4 failsafes when Offboard setpoints stop; any `link_lost` entries are ignored. `LOW_LIGHT` is kept for torch control only and no longer starts mission fallbacks. Additional triggers may be added later without breaking compatibility.
