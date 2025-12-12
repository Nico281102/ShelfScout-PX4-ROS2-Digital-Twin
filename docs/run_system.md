# Run System (multi-first)

`scripts/run_system.sh` is the preferred launcher for the OverRack Scan stack. It always uses the multi-drone flow (even for one drone) to keep a single codepath. For the architectural view of components/topics, see `docs/architecture.md`; this page describes how the launcher resolves inputs and starts the processes. The older `run_ros2_system.sh` remains as a legacy single-path wrapper.

## Inputs and Defaults
- **Environment file (required)**: `scripts/.env` must set `SSDT_PX4_DIR` (PX4 checkout) and optionally `SSDT_ROOT`, `SSDT_ROS_WS`, `SSDT_WORLD`, `SSDT_MISSION_FILE`, `SSDT_PARAM_PATH`, `SSDT_AGENT_CMD`, `SSDT_HEADLESS_DEFAULT`. Legacy `SHELFSCOUT_*` keys are honored.
- **CLI flags**: `--gui` or `--headless` (default headless); `--params <file>` to override the YAML.
- **Parameter YAML**: defaults to `config/sim/multi_1drone.yaml` (falls back to `config/sim/multi.yaml`). Keys read via `overrack_mission.param_utils`:
  - `mission_file`
  - `world_file`
  - `agent_cmd` / `agent_cmd_default`
  - `drones` list (required; even single-drone uses this list)
- **Agent command precedence**: YAML `agent_cmd` → env overrides (`AGENT_CMD`, `XRCE_AGENT_CMD`, `MICROXRCE_AGENT_CMD`, `MICRORTPS_AGENT_CMD`) → default `MicroXRCEAgent udp4 -p 8888 -v 6`. A single shared agent is launched for all drones.
- **World / mission precedence**: YAML → env (`SSDT_WORLD`, `SSDT_MISSION_FILE`) → baked-in defaults (`worlds/overrack_indoor.world`, `config/mission_precomputed.yaml`).
- **Path resolution**: relative paths are resolved against `ROOT_DIR` (from `SSDT_ROOT` or repo root); absolute paths are used as-is.
- **Health checks / timeouts** (configurable): `SSDT_GZSERVER_TIMEOUT` (default 40s) and `SSDT_TOPIC_TIMEOUT` (default 60s) gate startup; `SSDT_REQUIRE_GZSERVER` and `SSDT_REQUIRE_PX4_TOPICS` (default `1`) decide if missing Gazebo/process or PX4 topics abort or only warn; `SSDT_GZ_GRACE` (default 8s) controls how long cleanup waits after SIGINT before SIGKILL on Gazebo.

## Param Parsing
- Parsing is centralized in `overrack_mission.param_utils`, shared between the Bash launcher and `mission.sim.launch.py`.
- Quick CLI: `PYTHONPATH="$PWD/ros2_ws/src" python3 -m overrack_mission.param_utils --file config/sim/multi_1drone.yaml defaults|drones --format shell|json`.
- Override order:
  - Global blocks `mission_runner` / `inspection_node` / `torch_controller` apply to all drones.
  - Per-drone overrides inside each `drones` entry take precedence over globals.
  - Mission: per-drone `mission_runner.mission_file` → per-drone `mission_file` → launch arg `mission_file` → global `mission_runner.ros__parameters.mission_file` → `run_ros2_system.ros__parameters.mission_file`.
  - MAVLink URL: per-drone `mavlink_url` → per-drone `mavlink_udp_port` (translated to `udp://:<port>`) → default `px4_param_setter.ros__parameters.mavlink_url`.
  - Agent: YAML `agent_cmd` → env overrides → default `MicroXRCEAgent udp4 -p 8888 -v 6`.

## Launch Sequence (always multi flow)
1. Source `scripts/.env`, resolve `ROOT_DIR`, `PX4_DIR`, `ROS2_WS`, `LOG_DIR`; verify param file exists and contains a `drones` list.
2. Build workspace if `ros2_ws/install/setup.bash` is missing (`colcon build --packages-select px4_msgs overrack_mission`).
3. Source ROS distro (`/opt/ros/<distro>`, default `humble`) and the workspace overlay; respect custom `ssdt_source_ros`/`shelfscout_source_ros` if present.
4. Start PX4 + Gazebo via `scripts/launch_px4_gazebo_multi.sh` (headless if requested); log to `data/logs/px4_gazebo.out`.
5. Wait for a stable `gzserver` (fails on timeout if `SSDT_REQUIRE_GZSERVER=1`, default).
6. Start Micro XRCE-DDS Agent with the resolved command (single shared agent); log to `data/logs/micro_xrce_agent.out` and ensure it stays alive.
7. Wait for `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position` to confirm the bridge (fails on timeout if `SSDT_REQUIRE_PX4_TOPICS=1`, default).
8. Launch ROS 2 nodes: `ros2 launch overrack_mission mission.sim.launch.py params_file:=<param> mission_file:=<mission>`, which spins per-namespace groups (`mission_runner`, `inspection_node`, `mission_metrics`, `torch_controller`, `px4_param_setter`). Log: `data/logs/mission_runner.out`.
9. Trap `INT/TERM/EXIT` to cleanly stop mission runner(s), agent, and PX4/Gazebo (SIGINT to Gazebo, SIGKILL only if still alive); cleanup is idempotent.

## Health Checks
- **Gazebo**: waits for `gzserver` to appear and stay up (`SSDT_GZSERVER_TIMEOUT`, default 40s). With `SSDT_REQUIRE_GZSERVER=1` startup aborts on timeout; set to `0` to only warn.
- **PX4 topics**: waits for `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position` (`SSDT_TOPIC_TIMEOUT`, default 60s). With `SSDT_REQUIRE_PX4_TOPICS=1` startup aborts on timeout; set to `0` to continue best-effort.
- **Agent liveness**: after spawning the Micro XRCE Agent, the script checks it did not exit immediately and tails logs on failure.
- **Cleanup nuance (PX4 SITL)**: Gazebo is spawned by PX4 in its own session (not the wrapper PGID), so cleanup targets `gzserver/gzclient` directly: SIGINT first, wait `SSDT_GZ_GRACE` seconds, then SIGKILL only if still running. This avoids persistent “forcing Gazebo” warnings when shutdown is clean.

## Logs Produced
- `data/logs/px4_gazebo.out` – stdout/stderr of `launch_px4_gazebo_multi.sh`.
- `data/logs/micro_xrce_agent.out` – Micro XRCE-DDS Agent (single shared agent).
- `data/logs/mission_runner.out` – ROS 2 launch (mission runner + inspection + metrics + torch).

## Where to look next
- Architecture and topic wiring: `docs/architecture.md` (PX4/ROS topic map and per-namespace nodes).
- Mission language and FSM: `docs/mission_language.md`, `docs/mission_v1.md`.
- PX4↔ROS bridging details: `docs/ROS2_PX4_BRIDGING.md`.
