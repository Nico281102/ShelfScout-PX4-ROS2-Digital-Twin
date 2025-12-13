# Configuration & Simulation Reference
This guide documents the complete configuration hierarchy for ShelfScout. To run a simulation, you interact with three layers of YAML files:

1. Simulation Config (config/sim/*.yaml): Defines the environment, the fleet of drones, and hardware/network parameters.

2. Mission Config (config/mission*.yaml): Defines the behavior (inspection logic, safety fallbacks) for a specific drone.

3. Route Config (config/routes/*.yaml): Defines the geometric path (waypoints).
  
## 1. Simulation Configuration
**File path:** Passed via --params (e.g., config/sim/multi.yaml). Purpose: Orchestrates the launch. It tells the script which world to load and which drones to spawn.

### Top-Level Blocks
```yaml
# Global environment settings
run_ros2_system:
  ros__parameters:
    world_file: worlds/overrack_indoor.world  # Path to the Gazebo SDF world
    agent_cmd_default: MicroXRCEAgent udp4 -p 8888 -v 4  # Default DDS agent command

# The Fleet Definition
drones:
  - name: drone1
    namespace: px4_1

    model: iris_opt_flow
    spawn: {x: 1.5, y: 0.5, z: 0.3, yaw: 1.57}
    mission_file: config/mission_drone1.yaml
    mavlink_udp_port: 14541
    # ... (see Drone Definition details below)

# Global ROS 2 Node Parameters (applied to all drones unless overridden)
mission_runner:
  ros__parameters:
    world_bounds.x: [-5.0, 5.0]
    return_home_safe_z: 2.0

inspection_node:
  ros__parameters:
    low_light_threshold: 90.0
```
### The `drones` List Definition
Each entry in the `drones` list configures a single PX4 instance and its ROS 2 stack.

| Field | Description | Example |
| :--- | :--- | :--- |
| `name` | Unique identifier used for logs and fallback naming. | `drone1` |
| `namespace` | **ROS Node Namespace**. Isolates the processes (e.g., `/px4_1/mission_runner`). | `px4_1` |
| `px4_namespace` | **Topic Prefix**. Must match the DDS bridge topics (e.g., subscribes to `/px4_1/fmu/out/...`). | `/px4_1` |
| `model` | Gazebo model name (must exist in `models/`). | `iris_opt_flow` |
| `spawn` | Initial pose `{x, y, z, yaw}`. | `{x: 0, y: 0, z: 0.2, yaw: 0}` |
| `mission_file` | Path to the mission logic for this drone. | `config/mission_drone1.yaml` |
| `mavlink_udp_port`| Unique UDP port for PX4 instance. | `14541` (increment for each drone) |
| `agent_cmd` | Specific XRCE agent command (needs unique port). | `"MicroXRCEAgent udp4 -p 8888 -v 4"` |
| `px4_params` | Dictionary of PX4 parameters to set at startup. | `{SIM_BAT_DRAIN: 60.0}` |

**Note on Namespaces:**
* `namespace` prevents node name collisions (process isolation).
* `px4_namespace` matches the data stream coming from PX4 (data isolation).
* In multi-drone setups, these usually match (e.g., `px4_1`) to keep the system coherent.

## 2\. Mission Configuration

**File path:** Referenced by `mission_file` in the simulation config.
**Purpose:** Defines *how* the drone executes the task (behavior).
For the full mission schema and FSM behavior, see `docs/mission_v1.md`.

### Schema (v1)

```yaml
api_version: 1

# The geometric path
route_file: routes/drone1_shelf_north.yaml

# Inspection Logic
inspection:
  enable: true              # Se true, ogni waypoint entra in INSPECT (anche se il route non ha inspect: true)
  timeout_s: 4.0            # Max time to wait for inspection result
  require_ack: false        # If true, holds until 'OK'/'SUSPECT' is received

# Safety Fallbacks
fallback:
  battery_warning: ["return_home"]
  battery_critical: ["land"]

# Completion Behavior
return_home_and_land_on_finish: true
```


**Inspection.enable:** if `true`, every step enters INSPECT (even if the route step lacks `inspect: true`). If `false`, only steps with `inspect: true` in the route enter INSPECT. It does not start/stop the `inspection_node` (always launched); it only gates the FSM’s INSPECT state and the wait for `require_ack`/`timeout_s`.

### Fallback Actions

Triggers (`battery_warning`, `battery_critical`) can start these sequences:

  * `return_home`: Fly to start position/home.
  * `land`: Emergency land immediately.
  * `hold:Ns`: Hover in place for N seconds.
  * `increase_hover:Ns`: Extend current waypoint hover time.

-----

## 3\. Route Configuration

**File path:** Referenced by `route_file` in the mission config.
**Purpose:** Defines *where* the drone flies (geometry).

```yaml
route:
  name: north_inspection
  default_altitude_m: 1
  default_hover_s: 2.0

  steps:
    - name: A
      position: [2, 0.0, 1.0]  # x, y, z (ENU)
      hover_s: 2.0
      yaw_deg: 180
      inspect: false

    - name: BC_photo
      position: [0.8, 4, 1.5]
      yaw_deg: 180
      inspect: true
      action: snapshot         # Emits event "snapshot" on ROS topic
```

  * **`position`**: `[x, y, z]` coordinates in ENU (East-North-Up) frame.
  * **`inspect`**: Overrides the global `inspection.enable` setting for this specific point.
  * **`action`**: A string tag published to `/overrack/mission_state` (e.g., to trigger a camera capture).

-----

## Summary of Relationships

1.  **`run_ros2_system.sh`** reads the **Simulation Config** (`sim/multi.yaml`).
2.  It spawns Gazebo and PX4 instances based on the **`drones`** list.
3.  It launches `mission_runner` nodes, passing the specific **Mission Config** (`mission_drone1.yaml`) to each.
4.  The mission runner loads the **Route** (`routes/drone1.yaml`) and executes the flight path defined therein.
