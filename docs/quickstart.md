# Quickstart

This guide will get you up and running with a ShelfScout simulation in minutes.

## Prerequisites
Before starting, ensure your system is prepared.

- Install Dependencies: Follow the detailed steps in `docs/environment_setup.md` to install ROS 2, PX4, and the Micro XRCE-DDS Agent.
- Configure Environment: You must copy `scripts/.env.example` to `scripts/.env` and adjust the paths to match your local setup (specifically SSDT_PX4_DIR).
  ```bash
  cp scripts/.env.example scripts/.env
  nano scripts/.env  # Adjust SSDT_PX4_DIR and SSDT_ROS_WS
  source scripts/.env
  ```
- Patch PX4 multi-run script and DDS topics for battery status (details in `docs/environment_setup.md`):
  ```bash
  scripts/apply_px4_sitl_multiple_run_patch.sh --px4-dir "$SSDT_PX4_DIR"

  ./scripts/apply_patch_dds_topics.sh
  ```

## 2. Key Directories

A quick overview of where files are located:

* **`config/`**: Simulation parameters (e.g., `sim/multi.yaml`), mission behaviors, and geometric routes (`routes/`).
* **`models/` & `worlds/`**: Custom Gazebo Classic assets (racks, warehouse environment).
* **`scripts/`**: Launch orchestrators (`run_system.sh`; legacy `run_ros2_system.sh`) and helper utilities.
* **`ros2_ws/`**: The ROS 2 workspace containing the `overrack_mission` package (runner, inspection, metrics) and PX4 bridges.
* **`data/`**: Runtime artifacts (logs, CSV metrics, camera snapshots).

## 3. Configuration Hierarchy

The system uses a three-layer configuration approach to separate hardware/simulation settings from mission logic and flight paths.

### Level 1: Simulation Configuration
This is the entry point (passed via `--params`). It defines the world, the number of drones, their spawn points, and which mission file each drone follows.

*Example (`config/sim/multi_1drone.yaml`):*
```yaml
run_ros2_system:
  ros__parameters:
    world_file: worlds/overrack_indoor.world
    agent_cmd_default: MicroXRCEAgent udp4 -p 8888 -v 4 
    drones_yaml: |
      - name: drone1
        namespace: px4_1
        model: iris_opt_flow
        spawn: {x: 1.5, y: 0.5, z: 0.3, yaw: 1.57}
        mavlink_udp_port: 14541
        px4_params:
          SIM_BAT_DRAIN: 120.0
          SIM_BAT_MIN_PCT: 0.03
          BAT1_N_CELLS: 3
          BAT1_V_CHARGED: 4.15
          BAT1_V_EMPTY: 3.55
          BAT1_CAPACITY: 3500.0
          BAT_LOW_THR: 0.25
          BAT_CRIT_THR: 0.15
          BAT_EMERGEN_THR: 0.08
          COM_LOW_BAT_ACT: 0
        px4_namespace: "/px4_1"
        log_dir: data/logs/drone1
        mission_runner:
          mission_file: config/mission_drone1.yaml
          sim.disable_link_after_s: -1.0
        inspection_node:
          image_topic: overrack/iris/front_camera/image_raw
          low_light_threshold: 90.0
          suspect_variance: 12.0
          publish_period_s: 0.5

```
For a deep dive into available parameters and creating custom routes, refer to the Mission Language Documentation `docs/mission_language.md`.
### Level 2: Mission Configuration
Defines behavior: inspection timeout and fallback safety triggers (e.g., what to do on low battery). It points to the specific geometric route, which carries altitude/hover defaults.

*Example (`config//mission_drone1.yaml`):*
```yaml
api_version: 1

route_file: routes/drone1_shelf_north.yaml

inspection:
  enable: true
  timeout_s: 5.0

fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]

return_home_and_land_on_finish: true
```
### Level 3: Route Configuration
Defines geometry: the specific sequence of waypoints (Steps), coordinates, and specific actions (like snapshots) to perform at each point.

*Example (`config/routes/drone1_shelf_north.yaml`):*
```yaml
route:
  name: north_inspection
  default_altitude_m: 1
  default_hover_s: 2.0

  steps:      

    - name: A
      position: [2, 0.0, 1.0]
      hover_s: 2.0
      inspect: false


    - name: B_arrival
      position: [2, 4, 1.0]
      yaw_deg: 180
      inspect: false
      hover_s: 5.0          


    - name: BC_arrival # distance of 0.8
      position: [0.8, 4, 1.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: BC_photo
      position: [0.8, 4, 1.5]
      yaw_deg: 180
      inspect: true
      action: snapshot


    - name: BC_arrival_down # distance of 0.8
      position: [0.8, 4, 0.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: BC_photo_down
      position: [0.8, 4, 0.5]
      yaw_deg: 180
      inspect: true
      action: snapshot

    - name: C_arrival_down
      position: [0.0, 4, 0.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: C_photo_down
      position: [0.0, 4, 0.5]
      yaw_deg: 180
      inspect: true
      action: snapshot

    - name: C_arrival
      position: [0.0, 4, 1.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: C_photo
      position: [0.0, 4, 1.5]
      yaw_deg: 180
      inspect: true
      action: snapshot

    - name: CD_arrival
      position: [-0.8, 4, 1.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: CD_photo
      position: [-0.8, 4, 1.5]
      yaw_deg: 180
      inspect: true
      action: snapshot

    - name: CD_arrival_down
      position: [-0.8, 4, 0.5]
      yaw_deg: 180
      inspect: false
      hover_s: 15.0

    - name: CD_photo_down
      position: [-0.8, 4, 0.5]
      yaw_deg: 180
      inspect: true
      action: snapshot

    - name: D
      position: [-2, 4, 1.5]
      yaw_deg: 180
      inspect: true

    - name: H
      position: [-3, 1.5, 1.5]
      yaw_deg: 180
      inspect: false

```
## Run The simulation
From the repository root (ensure your environment is sourced), run the orchestrator script:
```bash
./scripts/run_system.sh --gui --params config/sim/multi_1drone.yaml
```
### Simulation Parameters
The --params flag points to a high-level simulation configuration file. This file tells the launcher:

- Which world to load.
- Which mission file to execute.
- How many drones to spawn (for multi-agent setups).+

## What happen next?
1. Gazebo Classi will open
2. PX4 will initialize.
3. The mission runner will arm the drone, take off and execute the waypoints
4. Metrics and Logs
  
This same workflow is captured in the screencast below.

To stop the simulation, simply press Ctrl+C in the terminal; `run_system.sh` gracefully shuts down Gazebo, PX4, the agent, and the mission node.

> **Note:** the demo uses a single-drone params file, but you can point `run_system.sh` at any other YAML (e.g. `config/sim/multi.yaml`,`config/sim/multi_3drones.yaml` ) to exercise multi-agent worlds. Re-run the command with your chosen file to generate a matching screencast:
> ```bash
> ./scripts/run_system.sh --gui --params config/sim/mission_3drones.yaml
> ```
