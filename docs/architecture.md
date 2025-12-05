<!-- Scope: high-level architecture for the current single-drone stack. Multi-drone notes are included as outlook only. -->

# Architecture

OverRack Scan runs PX4 SITL inside a Gazebo Classic world and drives it with ROS 2 nodes. A Micro XRCE-DDS bridge keeps PX4 and ROS 2 in sync; the bridge internals live in `docs/ROS2_PX4_BRIDGING.md`. The focus here is how the project is structured and how the pieces are wired together for the single-drone pipeline.

## Stack Overview (single drone)
```
Gazebo Classic world (overrack_indoor.world)
    ↕  PX4 SITL (gazebo-classic_iris_opt_flow)
    ↕  XRCE bridge (see ROS2_PX4_BRIDGING.md)
ROS 2 nodes (mission_runner + inspection_node + metrics_node + torch plugin)
    ↕  data/metrics, data/logs, data/images
```

- The launch scripts read `config/sim/default.yaml` to choose the world, mission file, and agent command. Edit that YAML instead of passing long CLI flags.
- `mission_runner` applies the mission YAML (see `docs/mission_language.md`) and publishes Offboard setpoints; inspection + metrics live in the same executor to keep clocks aligned.
- PX4 ↔ ROS traffic uses `px4_msgs` over the `/fmu/in/*` and `/fmu/out/*` topics. Topic mapping, QoS, and agent setup are detailed in `docs/ROS2_PX4_BRIDGING.md`.
- Runtime artefacts are persisted under `data/` (logs, metrics, images) so every run is auditable.

## Runtime Flow
1. `scripts/run_ros2_system.sh` sources `scripts/.env`, reads `config/sim/default.yaml`, and spawns PX4 + Gazebo via `scripts/launch_px4_gazebo.sh`.
2. The same script launches the XRCE bridge (defaults to `MicroXRCEAgent udp4 ...`) and tails logs to `data/logs/micro_xrce_agent.out`.
3. After ensuring `ros2_ws/install/setup.bash` exists (builds if missing), it launches `mission_runner` (with inspection + metrics) using the same param file.
4. `px4_param_setter` (optional) pushes battery and failsafe parameters to PX4 using MAVLink UDP, keeping SITL settings consistent across runs.
5. Data products (metrics, images, logs) are written under `data/` and stay aligned to the mission timestamp for later analysis.

## Data and Coordinates
- Mission plans stay ENU; the PX4 adapter converts to NED, subtracts the spawn offset learned from the first `/fmu/out/vehicle_local_position`, and clamps setpoints to `world_bounds` from `config/sim/default.yaml`.
- Metrics and images are written under `data/metrics/<timestamp>/` and `data/images/`; logs for PX4, the agent, and the mission runner live in `data/logs/`.
- Gazebo models from PX4 and this repo are added to `GAZEBO_MODEL_PATH` automatically by the launch scripts; the torch plugin is loaded from `ros2_ws/install/lib` via `GAZEBO_PLUGIN_PATH`.

## Multi-Drone Outlook (WIP branch)
- Architecture will mirror the single-drone stack with separate vehicle IDs, ports, and namespaces per drone (e.g., distinct agent commands or XRCE sessions).
- Mission files will need either per-vehicle sections or multiple mission files; the current `mission_language.md` documents the single-vehicle schema and will be extended when the branch lands.
- Troubleshooting and PX4/Gazebo notes already call out where per-vehicle parameters (ports, model names, uORB↔DDS mapping) will need duplication.
