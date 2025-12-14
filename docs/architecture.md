<!-- Scope: high-level architecture for the current single-drone stack. Multi-drone notes are included as outlook only. -->

# Architecture

OverRack Scan runs PX4 SITL inside a Gazebo Classic world and drives it with ROS 2 nodes. A Micro XRCE-DDS bridge keeps PX4 and ROS 2 in sync; the bridge internals live in `docs/uxrce_dds_px4_ros_bridge.md`. The focus here is how the project is structured and how the pieces are wired together for the single-drone pipeline. (we need to abstract)

**Read this first:** if you need a conceptual overview of PX4, Gazebo, and ROS 2/DDS before diving deeper, start with [docs/foundations.md](docs/foundations.md).

```mermaid
flowchart LR
    %% ---------------------------------------------------------
    %% 1. SIMULATION ENVIRONMENT
    %% ---------------------------------------------------------
    subgraph Sim_Env["Simulation environment (host PC)"]
        direction TB
        GZ["Gazebo Classic<br>(shared world: physics, sensors, rendering)"]
        
        subgraph Fleet["PX4 fleet"]
            PX4_1["PX4 Autopilot (drone 1)"]
            PX4_N["PX4 Autopilot (drone N)"]
        end

        PX4_1 <== "MAVLink (TCP/UDP)" ==> GZ
        PX4_N <== "MAVLink (TCP/UDP)" ==> GZ
    end

    %% ---------------------------------------------------------
    %% 2. MIDDLEWARE (BRIDGE)
    %% ---------------------------------------------------------
    PX4_1 <--> AGENT
    PX4_N <--> AGENT

    AGENT["Micro XRCE-DDS Agent<br>(bridge UDP ↔ DDS)"]

    %% ---------------------------------------------------------
    %% 3. ROS 2 SYSTEM
    %% ---------------------------------------------------------
    subgraph ROS_System["ROS 2 system (ros2_ws)"]
        direction TB

        %% ----- DRONE 1 -----
        subgraph NS_1["Namespace: /px4_1"]
            direction TB

            %% PX4-facing ROS nodes
            subgraph PX4_NODES_1["PX4-facing ROS 2 nodes"]
                direction TB
                TOPICS_1[("/px4_1/fmu/*<br>topics")]
                Mission_1["mission_runner"]
                Param_1["px4_param_setter"]
                Other_1["other PX4-aware nodes"]

                TOPICS_1 <==> Mission_1
                TOPICS_1 <==> Param_1
                TOPICS_1 -.-> Other_1
            end

            %% Payload/inspection pipeline
            subgraph PAYLOAD_1["Payload and inspection"]
                direction TB
                Vision_1["inspection_node"]
                Torch_1["torch_controller"]
                Vision_1 --> Torch_1
            end
        end

        %% ----- DRONE N -----
        subgraph NS_N["Namespace: /px4_N"]
            direction TB

            subgraph PX4_NODES_N["PX4-facing ROS 2 nodes"]
                direction TB
                TOPICS_N[("/px4_N/fmu/*<br>topics")]
                Mission_N["mission_runner"]
                Param_N["px4_param_setter"]
                Other_N["other PX4-aware nodes"]

                TOPICS_N <==> Mission_N
                TOPICS_N <==> Param_N
                TOPICS_N -.-> Other_N
            end

            subgraph PAYLOAD_N["Payload and inspection"]
                direction TB
                Vision_N["inspection_node"]
                Torch_N["torch_controller"]
                Vision_N --> Torch_N
            end
        end
    end

    %% ---------------------------------------------------------
    %% CONNECTIONS (bridge to topic bus)
    %% ---------------------------------------------------------
    AGENT <==> TOPICS_1
    AGENT <==> TOPICS_N

    %% ---------------------------------------------------------
    %% DIRECT SENSOR/ACTUATOR (bypassing bridge)
    %% ---------------------------------------------------------
    GZ -- "camera stream<br>.../camera/front/image_raw" --> Vision_1
    Torch_1 -- "light control<br>.../overrack/torch_enable" --> GZ

    GZ -- "camera stream<br>.../camera/front/image_raw" --> Vision_N
    Torch_N -- "light control<br>.../overrack/torch_enable" --> GZ

```

### PX4 facing ROS2 nodes
```mermaid
flowchart TB
    subgraph PX4_SIDE["PX4 autopilot (/px4_n)"]
        PX4[" PX4 Autopliot"]
    end

    subgraph ROS_SIDE["ROS 2 (/px4_n namespace)"]
        subgraph Mission["mission_runner"]
            Tele["telemetry helper<br>(subscribers)"]
            Setp["setpoints helper<br>(publishers)"]
        end
        Param["px4_param_setter"]
    end

    %% Telemetry consumers
    PX4 -- "/px4_n/fmu/out/vehicle_local_position<br>/vehicle_status<br>/battery_status" --> Tele
    PX4 -- "/px4_n/fmu/out/vehicle_status" --> Param

    %% Offboard/command publishers
    Setp -- "/px4_n/fmu/in/offboard_control_mode<br>/trajectory_setpoint<br>/vehicle_command" --> PX4

    %% MAVLink param push (outside DDS topics)
    Param -. "MAVSDK <br>(no /fmu/in/* topic)" .-> PX4
```
`mission_runner` is the only ROS node here; it embeds two helpers: `telemetry` (subscribes to `/fmu/out/*`) and `setpoints` (publishes to `/fmu/in/*`). `px4_param_setter` listens to `/fmu/out/vehicle_status` to detect PX4 STANDBY and then uses MAVSDK (over MAVLink) to push parameters directly to the autopilot, bypassing the `/fmu/in/*` bus.*¹

¹ MAVSDK is limited to parameter push/edge cases because building a MAVSDK backend would force us to re-implement the arbitration, queuing, and conflict-resolution logic that DDS/ROS 2 already routes through the mission runner suite; see [ROS 2 bridge](uxrce_dds_px4_ros_bridge.md). for the full explanation.

## Stack Overview (single drone)
- Simulation: Gazebo Classic world + PX4 SITL model chosen from `config/sim/default.yaml` (defaults to `worlds/overrack_indoor.world` and `iris_opt_flow` unless overridden via env/params).
- Bridge: Micro XRCE-DDS Agent (`MicroXRCEAgent udp4 ...` by default) exposes `/fmu/in/*` and `/fmu/out/*` on DDS.
- ROS 2 nodes per vehicle: `mission_runner` (telemetry + setpoints helpers), `inspection_node`, `mission_metrics`, `torch_controller`; `px4_param_setter` optionally pushes parameters via MAVSDK/MAVLink once PX4 reaches STANDBY.
- Outputs: logs under `data/logs/` (PX4, Gazebo, micro_xrce_agent, mission_runner), metrics CSVs under `data/metrics/`, images/snapshots under `data/images/`.

## Runtime Flow
This section is a high-level summary of the orchestrator; for CLI flags, env resolution, and logging paths see `docs/run_system.md`.
1. `scripts/run_system.sh` (multi-first) loads `scripts/.env`, resolves paths (`PX4_DIR`, ROS workspace), and reads the param YAML (`config/sim/multi_1drone.yaml` by default; falls back to `config/sim/multi.yaml`). The legacy `run_ros2_system.sh` still exists for the old single-path flow.
2. Derives world, mission file, agent command, and `drones` list via `param_utils`; builds the ROS 2 workspace if needed and sources the chosen ROS distro.
3. Starts PX4 + Gazebo via `launch_px4_gazebo_multi.sh`, with `--headless` if requested.
4. Starts the Micro XRCE Agent with the resolved command and waits for `/fmu/out/vehicle_status` and `/fmu/out/vehicle_local_position` to appear.
5. Launches `ros2 launch overrack_mission mission.sim.launch.py params_file:=<...> mission_file:=<...>`, which spins up (per namespace) `mission_runner`, `inspection_node`, `mission_metrics`, `torch_controller`, and `px4_param_setter` using the same params file/overrides.

## Models and PX4 airframes
- Gazebo loads the SDF model you pick (e.g., `models/iris_opt_flow/iris_opt_flow.sdf`), while PX4 flies an airframe selected by `PX4_SIM_MODEL` (mapped in `launch_px4_gazebo_multi.sh`). Keep those aligned: motor layout, mass/inertia, and sensor plugins in the SDF should match what the chosen airframe expects.
- Custom models can live under `models/<name>/<name>.sdf.jinja`. Ensure `GAZEBO_MODEL_PATH` includes `models/`, extend the PX4 `sitl_multiple_run.sh` whitelist for `<name>`, and map it to an appropriate PX4 airframe (or add a new one in PX4) so the control stack remains stable.
- If you change prop layout or mass, update both the SDF and the PX4 airframe constants; mismatches show up as EKF instability or sluggish control.

## Data 
- Metrics, inspection logs, and snapshots are written under `data/metrics/` and `data/images/`; PX4/agent/mission logs go to `data/logs/`.
