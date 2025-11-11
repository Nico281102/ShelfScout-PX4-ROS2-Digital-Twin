# ShelfScout PX4‑ROS2 Digital Twin
This README describes ShelfScout PX4‑ROS2 Digital Twin's architecture (SSDT), setup workflow, launch procedure, and troubleshooting guidance for contributors.

## Overview
OverRack Scan is an indoor PX4 SITL + Gazebo Classic sandbox that exercises a ROS 2 mission runner, perception node, and metrics pipeline against shelf-inspection scenarios. The repository provides the Gazebo world, PX4 model patches, mission YAML language, and orchestration scripts needed to reproduce full-stack simulations headless or with a GUI.

## Architecture at a Glance
```
PX4 SITL (gazebo-classic_iris_opt_flow)
    ↕  UDP 2019/2020 (uORB <-> RTPS)
Micro XRCE-DDS Agent (udp4 -p 8888)
    ↕  DDS QoS (Fast DDS / XRCE)
ROS 2 Workspace (ros2_ws)
    ├─ overrack_mission.mission_runner  → publishes Offboard setpoints
    ├─ overrack_mission.inspection_node → emits inspection events
    └─ overrack_mission.metrics         → exports KPI CSV files
```
- The `MissionStateMachine` enforces TAKEOFF → TRANSIT → HOVER/INSPECT → HOLD, applies tolerance windows, and triggers fallback actions (`return_home`, `hold`, `land`, etc.).
- `inspection_node.py` watches camera frames and mission state, marks each waypoint as `OK`, `SUSPECT`, or `LOW_LIGHT`.
- `metrics_node.py` records mission phases, fallback reasons, and inspection verdicts into `data/metrics/` for offline review.

## Repository Layout
| Path | Description |
| --- | --- |
| `ros2_ws/src/overrack_mission/` | Primary ROS 2 package (mission runner, mission engine, perception, metrics, planning utilities). |
| `ros2_ws/src/px4_msgs`, `px4_ros_com` | PX4-official message and bridge packages; kept pristine for compatibility. |
| `config/` | Mission YAML files and optional PX4 parameter overrides. |
| `routes/` | Pre-computed trajectory templates (e.g., `overrack_default.yaml`). |
| `worlds/`, `models/` | Gazebo Classic indoor world plus the custom rack and drone assets. |
| `scripts/` | Launchers (`run_ros2_system.sh`, `launch_px4_gazebo.sh`), debug helpers, and env file. |
| `data/` | Runtime artefacts: `logs/`, `metrics/`, `images/`. |
| `docs/` | Mission language reference, bridge details, structure review, and archived notes. |

## Environment Setup
### Prerequisites
- Ubuntu 22.04 with ROS 2 Humble (desktop install) and `colcon`.
- PX4-Autopilot sources (tested with v1.14.x) built at least once via `make px4_sitl_default`.
- Micro XRCE-DDS Agent available on the `PATH` (or built locally and referenced in scripts).
- Gazebo Classic 11 with `gzserver`, `gzclient`, and PX4 dependencies.
- Python 3.10 + `pip` (create a virtual environment if you need isolated tooling).

### Workspace Preparation
1. Copy `scripts/.env.example` to `scripts/.env` and adjust the SSDT variables (`SSDT_PX4_DIR`, `SSDT_ROS_WS`, `SSDT_AGENT_CMD`, etc.) so they point to your PX4 checkout, ROS workspace, and preferred defaults.
2. (Optional) create a virtual environment for the auxiliary scripts:
   ```bash
   python3.10 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
3. Build the ROS 2 workspace if you wish to iterate manually:
   ```bash
   cd ros2_ws
   colcon build --symlink-install --packages-select px4_msgs overrack_mission
   source install/setup.bash
   ```
4. Ensure PX4 SITL models and this repository are on `GAZEBO_MODEL_PATH`. The launch scripts append both automatically, but you can export them manually if needed:
   ```bash
   export GAZEBO_MODEL_PATH="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$(pwd)/models:${GAZEBO_MODEL_PATH:-}"
   ```

### Env profile
The ShelfScout Digital Twin (SSDT) profile lives in `scripts/.env` and is sourced by every launcher. The file:
- Defines all input/output paths (`SSDT_PX4_DIR`, `SSDT_ROS_WS`, `SSDT_WORLD`, `SSDT_LOG_DIR`, ...), so you only update one place when moving the workspace.
- Exposes helpers (`ssdt_resolve_path`, `ssdt_source_ros`, `ssdt_prepend_path`) that launchers reuse instead of duplicating ROS sourcing logic or PATH edits.
- Ships with legacy aliases (`PX4_DIR`, `MICRO_XRCE_AGENT_DIR`) to remain compatible with older scripts until they are migrated.

Always `source scripts/.env` before manual commands, or let the provided scripts do it for you. To refresh your setup, edit `scripts/.env` (or start from `scripts/.env.example`) and re-run the launch script.

## Launch with `run_ros2_system.sh`
Use the orchestrator from the repository root to start PX4, Gazebo, the Micro XRCE-DDS Agent, and all ROS 2 nodes:

```bash
PX4_DIR=/absolute/path/to/PX4-Autopilot \
./scripts/run_ros2_system.sh \
  --gui \                           # use --headless for CI or SSH
  --world worlds/overrack_indoor.world \
  --mission config/mission.yaml \
  --agent-cmd "MicroXRCEAgent udp4 -p 8888 -v 6"
```
The script performs the following:
- Verifies and (re)builds `ros2_ws` if needed.
- Launches PX4 SITL + Gazebo via `scripts/launch_px4_gazebo.sh`, extending `GAZEBO_MODEL_PATH` for the custom rack world.
- Starts or reuses the Micro XRCE-DDS Agent and tails its log to `data/logs/micro_xrce_agent.out`.
- Runs `ros2 run overrack_mission mission_runner --ros-args -p mission_file:=...` logging to `data/logs/mission_runner.out`.
- Streams PX4 output to `data/logs/px4_sitl_default.out` for later inspection.

If you ever need to kill everything quickly, `scripts/stop_manual_like.sh` still terminates PX4, Gazebo, the agent, and the mission runner.

## Mission Runner and ROS 2 Nodes
- **Mission Runner (`mission_node.py`)**: loads the YAML plan supplied via `mission_file`, instantiates `MissionController`, `MissionStateMachine`, and `InspectionNode`, and publishes Offboard setpoints at 20 Hz once PX4 reports readiness.
- **Mission Engine**: `mission_engine/controller.py`, `plan_parser.py`, `state_machine.py`, and `setpoints.py` translate mission plans into PX4 topic interactions. Modes include `explicit` waypoint execution, `precomputed` route playback (from `routes/*.yaml`), and `coverage` patterns generated by `planning/coverage_planner.py`.
- **Telemetry helpers**: `mission_engine/telemetry.py` aggregates `VehicleLocalPosition`, `VehicleStatus`, `BatteryStatus`, and inspection events to gate arming/offboard transitions and to enforce tolerances.
- **Perception (`perception/inspection_node.py`)**: subscribes to `sensor_msgs/Image` (configurable `image_topic`) and mission state updates to label each inspection stage as `OK`, `SUSPECT`, or `LOW_LIGHT`.
- **Metrics (`metrics/metrics_node.py`)**: listens to mission states and inspection events, counts fallback occurrences, and writes summary + per-inspection CSV artefacts into `data/metrics/` at shutdown.
The ROS 2 package is registered through `ros2_ws/src/overrack_mission/setup.py`, which wires the entry points above so `colcon build` exposes `mission_runner`, `inspection_node`, and `mission_metrics` as console scripts after installation.

## Data, Logs, and Analysis Tools
- `data/logs/` collects PX4, agent, mission runner, and auxiliary script logs for regression tracking.
- `data/metrics/` stores missions KPI CSVs (duration, fallback counts, inspection outcomes).
- `data/images/` holds camera captures for post-mission analysis or barcode decoding.
- `scripts/run_vision.py` and `scripts/run_visual_demo.py` provide optional visualisation/analytics entry points.

## Troubleshooting
- **Offboard rejected**: confirm at least 20 trajectory setpoints are streamed before `VehicleCommand.DO_SET_MODE`, and verify `vehicle_status.nav_state` plus pe-flight checks via `mission_engine/telemetry.py` helpers.
- **No `/fmu/out/*` topics**: check `data/logs/micro_xrce_agent.out` for agent crashes, ensure the PX4 branch matches the agent binary, and confirm UDP ports 2019/2020 are free.
- **Fallback not firing**: ensure triggers in the mission YAML match published event names (e.g., `battery_warning`, `low_light`) and monitor `overrack/inspection` output.
- **Gazebo refuses to start**: rebuild PX4 (`make px4_sitl_default`), try `--headless`, and verify that `GAZEBO_MODEL_PATH` contains both PX4 and OverRack models.
  

### PX4 + Gazebo Classic startup quirk (race condition)

We observed that sometimes Gazebo Classic starts (process is up, port is open) but PX4’s `sitl_run.sh` tries to spawn the vehicle model too early, right after `gzserver` is launched. In those runs the PX4 log stops at:

```text
[px4] Using: /.../models/iris_opt_flow/iris_opt_flow.sdf
````

and nothing else happens.

This is a small timing issue: `gzserver` is running, but it has not finished loading the world and plugins yet, so the CLI call

```bash
gz model --spawn-file=... --model-name=...
```

returns “An instance of Gazebo is not running.” and the flow looks stuck.

#### Workaround we applied

We patched PX4’s `Tools/simulation/gazebo-classic/sitl_run.sh` to give Gazebo a short grace period **after** starting `gzserver` and **before** spawning the model:

```bash
gzserver $verbose $world_path $ros_args &
SIM_PID=$!

# ... resolve modelpath ...

echo "Using: ${modelpath}/${model}/${model}.sdf"

# [BEGIN Customization]
sleep 4 # give gazebo a moment to finish loading plugins/world
# [END Customization]

while gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" \
  --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | \
  grep -q "An instance of Gazebo is not running."; do
  echo "gzserver not ready yet, trying again!"
  sleep 1
done
```

We noticed that running the same script from an SSH / VS Code remote shell sometimes worked, probably because the environment was a bit slower, which unintentionally gave Gazebo the extra time, or gazebo needed less time to fully initialize because the gzclient would not be started.

#### Why we document it here

* the ROS 2 workspace, Micro XRCE agent and mission runner were fine;
* the flakiness was specifically in the PX4 + Gazebo Classic launch path;
* keeping the explanation close to the launch instructions will help anyone who sees “stuck at ‘Using: …iris_opt_flow.sdf’” and thinks PX4 is broken.



but the default is: put it right after the PX4 + Gazebo section so operators will actually see it.

- **Known PX4 + Gazebo Classic startup quirk**: occasionally `gzserver` accepts connections before the world is fully initialised, so PX4’s `gz model --spawn-file=...` call fails with “An instance of Gazebo is not running.” Patch `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh` to add a short delay after starting `gzserver` (e.g., `sleep 4`) and retry the spawn loop until it succeeds. This prevents runs getting stuck after printing `Using: .../iris_opt_flow.sdf` and matches the workaround already applied in our local PX4 fork.

## How AI was used in this project
- The markdow is almost fully IA written, of course is carefully checked by the developer, but checking is faster than starting writing from zero.
The integratex visualstudio code chatbot is used, becouse allow the develper to carefully check the diff between a previous version of the document and the newone propose by the IA.

## Documentation Index
| Document | Purpose |
| --- | --- |
| `docs/mission_v1.md` | Authoritative mission YAML schema (api_version 1) with defaults, modes, triggers, and examples. |
| `docs/mission_v1_backup.md` | Legacy snapshot of the mission language kept for reference. |
| `docs/ROS2_PX4_BRIDGING.md` | Details on the PX4 ↔ Micro XRCE-DDS ↔ ROS 2 bridge, message flows, and troubleshooting. |
| `docs/structure_review.md` | Component inventory and status (active vs legacy/zombie subsystems). |
| `docs/README_legacy.md` | Historical notes preserved for completeness; not actively maintained. |
