# Run ROS 2 System

`scripts/run_ros2_system.sh` is the one-stop launcher for the OverRack Scan stack. It sources environment defaults, builds the ROS 2 workspace if needed, starts PX4 + Gazebo, brings up the Micro XRCE-DDS Agent, and finally launches the ROS 2 nodes (mission runner, inspection, metrics). This page documents how it resolves inputs, what it launches, where it writes logs, and what you would change to optimise or extend it (e.g., multi-drone).

## Inputs and Defaults
- **Environment file**: `scripts/.env` (required). Must define at least `SSDT_PX4_DIR` (PX4 checkout) and optionally `SSDT_ROOT`, `SSDT_ROS_WS`, `SSDT_WORLD`, `SSDT_MISSION_FILE`, `SSDT_PARAM_FILE`, `SSDT_AGENT_CMD`, `SSDT_HEADLESS_DEFAULT`. Legacy `SHELFSCOUT_*` keys are also honoured.
- **CLI flags**: `--gui` or `--headless` (default headless). No other flags are accepted.
- **Parameter YAML**: defaults to `config/sim/default.yaml` unless overridden by `SSDT_PARAM_PATH` (falls back to the legacy `ros2_ws/.../param/sim.yaml` if the new file is missing). The script reads three keys from this file (if present under `run_ros2_system.ros__parameters` or `mission_runner.ros__parameters`):
  - `mission_file`
  - `world_file`
  - `agent_cmd` / `agent_cmd_default` (command line for the Micro XRCE-DDS agent; in multi si usa per-drone)
- **Agent command precedence**: `agent_cmd` in `drones_yaml` → `agent_cmd_default` in YAML → env overrides (`AGENT_CMD`, `XRCE_AGENT_CMD`, `MICROXRCE_AGENT_CMD`, `MICRORTPS_AGENT_CMD`) → default `MicroXRCEAgent udp4 -p 8888 -v 6`. In multi-drone il launcher avvia un agent per drone con il comando completo (porta inclusa); nel path single-drone parte un solo agent.
- **World and mission precedence**: values in YAML → env defaults (`SSDT_WORLD`, `SSDT_MISSION_FILE`) → baked-in defaults (`worlds/overrack_indoor.world`, `config/mission_precomputed.yaml`).
- **Path resolution**: relative paths are resolved against `ROOT_DIR` (from `SSDT_ROOT` or repo root). Absolute paths pass through unchanged.

## Launch Sequence (single drone)
1. **Source environment**: loads `scripts/.env`, resolves `ROOT_DIR`, `PX4_DIR`, `ROS2_WS`, `LOG_DIR`. Exits early if `PX4_DIR` or the param file are missing.
2. **Workspace check**: runs `colcon build --packages-select px4_msgs overrack_mission` if `ros2_ws/install/setup.bash` is absent.
3. **ROS environment**: sources `/opt/ros/<distro>/setup.bash` (default `humble`) and the workspace overlay. Custom `ssdt_source_ros`/`shelfscout_source_ros` functions are used if present.
4. **PX4 + Gazebo**: starts `scripts/launch_px4_gazebo.sh` in a new process group, with `PX4_DIR` and optional `--headless`. Logs go to `data/logs/px4_gazebo.out` and `data/logs/px4_sitl_default.out`.
5. **Readiness gates**:
   - Waits for `gzserver` to appear (up to 40s).
   - Waits for `Ready for takeoff` in `px4_sitl_default.out` (up to 120s).
6. **Micro XRCE-DDS Agent**: single-drone path lancia un agent unico (log `data/logs/micro_xrce_agent.out`); il path multi avvia un agent per ogni drone (log per-namespace, es. `data/logs/px4_1/micro_xrce_agent.out`).
7. **Topic wait**: polls for `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position` (60s each) to confirm the bridge is alive.
8. **ROS 2 nodes**: starts `ros2 launch overrack_mission mission.sim.launch.py` with `params_file:=<param>` and `mission_file:=<mission>`; logs to `data/logs/mission_runner.out`.
9. **Cleanup**: a trap on `INT/TERM/EXIT` stops mission runner, agent, and the PX4/Gazebo process group; it force-kills `gzserver/gzclient` if needed.

## Logs Produced
- `data/logs/px4_sitl_default.out` – PX4 SITL console (from `PX4_SITL_LOG_FILE`).
- `data/logs/px4_gazebo.out` – stdout/stderr of `launch_px4_gazebo.sh`.
- `data/logs/micro_xrce_agent.out` – Micro XRCE-DDS Agent output (single drone). In multi i log degli agent sono per-namespace (`data/logs/<ns>/micro_xrce_agent.out`).
- `data/logs/mission_runner.out` – ROS 2 launch (mission runner + inspection + metrics).




## Multi-Drone Considerations
Pass `--params config/sim/multi.yaml` (or another multi-drone param file) to declare the drone list via `run_ros2_system.ros__parameters.drones_yaml`; il launcher avvia PX4 istanze, un agent XRCE per drone (usa l’`agent_cmd` per drone o il fallback `agent_cmd_default`), e lancia i nodi ROS namespaced.
