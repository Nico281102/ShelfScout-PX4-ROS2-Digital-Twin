# PX4 ↔ ROS 2 Bridging
This document explains the PX4 ↔ Micro XRCE-DDS ↔ ROS 2 bridge used in the project, including message flows, agent settings, and troubleshooting steps.

## Architecture
The bridge translates PX4 uORB messages into ROS 2 topics via Micro XRCE-DDS. It is the only ROS/DDS touchpoint; the rest of the architecture is covered in `docs/architecture.md`.

```mermaid
flowchart LR
    subgraph Sim["Gazebo Classic + PX4 SITL"]
        GZ["Gazebo Classic world"]
        PX4["PX4 SITL<br>uORB ↔ uXRCE-DDS client"]
        GZ --> PX4
    end


    subgraph Bridge["Micro XRCE-DDS Agent"]
        AGENT[MicroXRCEAgent
        udp4 -p 8888]
    end

    subgraph ROS2["ROS 2 (Fast DDS)"]
        RCL[Fast DDS graph]
        NODES[mission_runner
        inspection_node
        metrics_node]
        RCL --> NODES
        NODES --> RCL
    end

    PX4 -- XRCE session (UDP, CDR) --> AGENT
    AGENT -- DDS entities (/fmu/in,out) --> RCL
```

- PX4 builds and launches the uXRCE-DDS client alongside SITL; it serialises uORB topics into an XRCE session and connects to the agent.
- `MicroXRCEAgent` translates the XRCE session into DDS entities on port 8888. Multiple ROS 2 participants can subscribe via Fast DDS once the agent is running.
- ROS 2 nodes built against `px4_msgs` publish to `/fmu/in/*` topics and subscribe to `/fmu/out/*`, matching PX4 QoS (`BEST_EFFORT`, depth 1).

## Multi-drone architecture (default)
The multi path keeps a single agent and multiple PX4 clients, while ROS 2 nodes are namespaced per vehicle. It is driven by `run_system.sh --params config/sim/multi.yaml` (or `config/sim/multi_1drone.yaml` for one drone using the same flow).

```mermaid
flowchart LR
    subgraph Sim["Gazebo Classic + PX4 SITL (multi)"]
        PX41["PX4 SITL #1
        (uXRCE client, px4_instance=1 → SYSID=2)"]
        PX42["PX4 SITL #2
        (uXRCE client, px4_instance=2 → SYSID=3)"]
        PX41 --- PX42
    end

    subgraph Bridge["Micro XRCE-DDS Agent"]
        AGENT[MicroXRCEAgent
        udp4 -p 8888]
    end

    subgraph ROS2["ROS 2 (Fast DDS)"]
        N1["mission_runner / inspection / metrics
        ns=px4_1 (vehicle_ns)"]
        N2["mission_runner / inspection / metrics
        ns=px4_2 (vehicle_ns)"]
    end

    PX41 -- XRCE session (UDP) --> AGENT
    PX42 -- XRCE session (UDP) --> AGENT
    AGENT -- DDS topics (/px4_1/fmu/in|out/*) --> N1
    AGENT -- DDS topics (/px4_2/fmu/in|out/*) --> N2
```

- **PX4 instances**: `launch_px4_gazebo_multi.sh` (called by `run_system.sh`) uses `sitl_multiple_run.sh` to spawn one PX4 per drone with unique SYSID/MAVLink ports and its own uXRCE client.
- **Agent**: one `MicroXRCEAgent` on the configured port (default 8888). Each PX4 client opens its own XRCE session; multiple agents are optional (set `agent_cmd` per drone with distinct `-p` if you need isolation).
- **ROS 2 namespaces**: `mission.sim.launch.py` wraps each drone in `PushRosNamespace(ns)` where `ns` comes from `vehicle_ns`/`namespace` in the YAML (`px4_1`, `px4_2`, ...). Node names are suffixed (`mission_runner_px4_1`, etc.) to avoid clashes.
- **PX4 topic prefix**: parameters `px4_namespace` optionally prepend a PX4 prefix to `/fmu/*` (left empty by default because the bridge exposes `/fmu/...` without namespace). Publishers/subscribers call `namespaced()` to add `px4_namespace` only if provided.
- **PX4 namespace vs SYSID**: PX4’s `rcS` sets `MAV_SYS_ID=$((px4_instance+1))` and the XRCE namespace to `px4_<px4_instance>` (`-n px4_1`, `-n px4_2`, ...). Result: the PX4 instance that publishes `/px4_1/fmu/...` reports `MAV_SYS_ID=2`, `/px4_2/fmu/...` reports `MAV_SYS_ID=3`, etc. QGC will show Vehicle 2 on namespace `/px4_1`, Vehicle 3 on `/px4_2`. Aligning SYSID and namespace would require changing `px4_instance` numbering or overriding `PX4_UXRCE_DDS_NS`/`MAV_SYS_ID`.
- **Who sets the XRCE topic namespace?** PX4 itself. In `rcS` the uXRCE client starts with `-n px4_<px4_instance>` (or `PX4_UXRCE_DDS_NS` if set), so PX4 emits `/px4_k/fmu/in|out/*`. The agent just relays what PX4 publishes; it does not invent prefixes. ROS nodes must use the same `px4_namespace` to match those topics.
- **What is Fast DDS?** The DDS implementation used by ROS 2 (rmw_fastrtps). It handles peer-to-peer discovery and transport of DDS entities; when the agent creates `/px4_k/fmu/*` endpoints, Fast DDS announces them on the ROS graph so nodes can publish/subscribe directly (no central broker).
- **Logs**: agent logs stay in `data/logs/micro_xrce_agent.out`. ROS node logs go under `ROS_LOG_DIR/<ns>/` (set by the launch file). PX4/Gazebo stdout is still aggregated in `data/logs/px4_sitl_default.out` and `data/logs/px4_gazebo.out`.

## Component Responsibilities
- **PX4 SITL**: started through `scripts/launch_px4_gazebo.sh`. The relevant binaries live in `PX4_DIR/build/px4_sitl_default/` and include the uXRCE-DDS client.
- **Micro XRCE-DDS Agent**: either installed system-wide or referenced through `MICRO_XRCE_AGENT_DIR`. `scripts/run_system.sh` spawns it with the command supplied in `config/sim/multi_1drone.yaml` / `config/sim/multi.yaml`.
- **ROS 2 Workspace (`ros2_ws`)**: houses `px4_msgs`, `px4_ros_com`, and `overrack_mission`. After sourcing `ros2_ws/install/setup.bash`, any ROS 2 node can interact with PX4 topics using the shared message definitions.

## Why uXRCE-DDS instead of MAVROS 2?

We evaluated MAVROS 2 for the link between PX4 (flight controller) and the companion computer (ROS 2), but we chose the native uXRCE-DDS (DDS-XRCE) bridge for these technical reasons.

### 1) Protocol translation vs DDS gateway
- **MAVROS 2 is a protocol translator:** it consumes MAVLink streams, decodes them, applies semantic conversions (e.g., NED↔ENU frame transforms), and republishes ROS 2 messages/services. This adds an extra translation layer and per-message processing.
- **uXRCE-DDS is a DDS gateway/proxy:** PX4 runs a DDS-XRCE client; the Micro XRCE-DDS Agent creates DDS entities on behalf of that client and bridges data into the DDS global data space so ROS 2 sees plain DDS topics.

### 2) Data format and “conversion cost”
- **MAVROS:** MAVLink payloads must be parsed and mapped into ROS message types (plugin-dependent), often with unit/frame conversions.
- **uXRCE-DDS:** messages are serialized on the PX4 side using a CDR-family encoding (Micro CDR / XCDR) compatible with DDS. The Agent primarily forwards serialized payloads while handling XRCE sessions and reliability/QoS translation.

### 3) Topic access, timestamps, and performance expectations
- **Topic coverage:** MAVROS is bounded by the MAVLink message set; DDS-XRCE can expose any PX4 uORB topic that is bridged and present in `px4_msgs`.
- **Timestamps:** with MAVLink, getting PX4-origin timestamps in ROS 2 depends on time sync and message-specific handling; missing/unstable sync can devolve to “receive time” and add jitter. With DDS-XRCE, uORB timestamps can be carried through more directly, which helps sensor fusion and visual odometry.
- **Latency/CPU/throughput:** DDS-XRCE can reduce overhead because there is no MAVLink→ROS semantic translation and fewer per-message transforms, but performance still depends on transport (serial/UDP), XRCE reliability settings, and DDS QoS.
### 4) QoS semantics and hidden limitations

* **MAVROS 2 exposes ROS 2 topics with configurable QoS, but these QoS are not enforced end-to-end.**
  MAVLink is not data-centric and does not support durability, history, or late-joiner semantics. As a result, QoS policies such as `TRANSIENT_LOCAL` or `KEEP_ALL` cannot be meaningfully implemented, even if they appear configurable at the ROS 2 layer.

* **This can hide fundamental limitations rather than solving them.**
  MAVROS 2 may accept QoS configurations that are semantically incompatible with PX4/MAVLink, leading to mismatches, degraded behavior, or false expectations (e.g., replay for late joiners).

* **With uXRCE-DDS, QoS constraints are explicit and enforced by DDS matching rules.**
  PX4 publishes data with well-defined DDS semantics (typically `VOLATILE` and `KEEP_LAST`), and subscribers must adapt accordingly. Unsupported QoS (e.g., `TRANSIENT_LOCAL`, `KEEP_ALL`) simply do not match, making the system behavior predictable and transparent.


### Conclusion
For telemetry-heavy workloads, uXRCE-DDS avoids the MAVLink bottleneck and keeps the stack closer to ROS 2’s native DDS data path, which is a better fit for a real-time digital twin.


## Why we don’t use a MAVSDK backend

DDS topics (`/fmu/in/*`, `/fmu/out/*`) cover telemetry and Offboard control well, but not every PX4 feature is exposed as a uORB/DDS topic.

In particular, **parameter read/write is still a MAVLink workflow**: there is no `/fmu/in/*` “set parameter” topic in the bridge. For this reason, `px4_param_setter` uses MAVSDK (over MAVLink) only to push parameters (typically when PX4 is in STANDBY).

We avoid a full “MAVSDK backend” for all data/commands because it would duplicate what DDS already provides (fan-out, discovery, QoS) and add another single point of failure and arbitration layer.

### Additional notes: DDS typing, sensor streaming, and high-frequency telemetry

**DDS is strongly typed**

ROS 2/DDS enforces strong typing: every topic has a fixed, IDL-defined message type.
This guarantees:

- code-generated serializers and deserializers
- unambiguous message structure
- automatic incompatibility detection between publishers and subscribers

Because PX4 exposes uORB messages as DDS topics, strong typing ensures that the
structure of vehicle_local_position, vehicle_status, battery_status, and
all /fmu/in/* command topics remains consistent across PX4, the Micro XRCE-DDS
agent, and all ROS 2 nodes. This is essential in a multi-process system:
mission runner, metrics, inspection, and GUI nodes can all subscribe to the same
topics safely without risking runtime mismatches or silent failures.

**High-frequency telemetry with low overhead**

The PX4 uXRCE-DDS client and the Micro XRCE Agent are optimized for real-time
telemetry and typically use:

- compact CDR serialization
- UDP transport
- statically allocated buffers on the PX4 side
- BEST_EFFORT QoS for high-rate data streams

This configuration allows PX4 to publish telemetry at 100–400 Hz with minimal
CPU overhead and without unbounded message queues.
BEST_EFFORT ensures that PX4 never accumulates stale data: if a subscriber
cannot keep up, samples are simply dropped, which is the correct behavior in a
real-time control system.

**Why cameras work better in ROS 2 than in MAVLink**

MAVLink is a vehicle control and telemetry protocol; it is not designed for
high-bandwidth sensor streaming. While image transport over MAVLink exists, it
is not suitable for sustained transmission of:

- depth images
- raw or high-resolution frames
- real-time machine vision pipelines
- multi-camera setups
MAVLink does not define a native image message type, so implementations either
tunnel JPEG fragments or rely on vendor-specific extensions. As a result,
camera pipelines over MAVLink are brittle, bandwidth-limited, and incompatible
across simulators and hardware.
ROS 2 instead provides native message types (sensor_msgs/Image,
CameraInfo), efficient memory handling (including zero-copy-capable paths),
and direct integration with Gazebo/Ignition camera plugins and vision
frameworks. The perception pipeline remains unchanged even if the underlying
camera hardware or simulator changes, as long as a ROS 2 driver is available — a
level of modularity that MAVLink does not provide.

## Bridge Lifecycle
1. `scripts/launch_px4_gazebo_multi.sh` starts PX4 with the uXRCE-DDS client enabled so PX4 opens XRCE sessions toward the agent.
2. `scripts/run_system.sh` launches the agent with the command from `config/sim/multi_1drone.yaml` or `config/sim/multi.yaml` (`run_ros2_system.ros__parameters.agent_cmd_default`) and tails its output to `data/logs/micro_xrce_agent.out`.
3. `mission.sim.launch.py` spins one group of nodes per drone under its ROS namespace. Each group targets the `/px4_k/fmu/*` topics exposed by PX4 via the single agent, using `px4_namespace` to match the XRCE namespace (`/px4_1`, `/px4_2`, …); without that prefix the nodes would look at plain `/fmu/*` and see nothing.
4. Once the agent establishes XRCE sessions, Fast DDS discovery exposes `/fmu/in/*` and `/fmu/out/*` to the ROS graph. `mission_runner` waits for `/fmu/out/vehicle_status` before entering Offboard.
5. If PX4 updates add or remove topics, rebuild PX4 (`make px4_sitl_default`) so the generated uXRCE client and `px4_msgs` stay in sync.

## Message Flows
### PX4 → ROS 2 (`/fmu/out/*`)
| Topic | Description | Notes |
| --- | --- | --- |
| `/fmu/out/vehicle_local_position` | Position, velocity, reference frame data | Consumed by `px4io/telemetry.py` for state transitions. |
| `/fmu/out/vehicle_status` | Arming state, nav state, failsafes | Used to gate Offboard requests and detect link issues. |
| `/fmu/out/battery_status` | Voltage, percentage, warning bits | Drives fallback triggers (`battery_warning`, `battery_critical`). |
| `/fmu/out/vehicle_attitude` | Quaternion orientation | Optional; useful for perception alignment. |
| `/fmu/out/trajectory_setpoint` | Echoed setpoints for verification | Helpful for debugging mission scripts. |

### ROS 2 → PX4 (`/fmu/in/*`)
| Topic | Publisher | Purpose |
| --- | --- | --- |
| `/fmu/in/offboard_control_mode` | `MissionController` | Keeps PX4 in position-control Offboard mode. |
| `/fmu/in/trajectory_setpoint` | `MissionController` | Publishes position/yaw commands at 20 Hz. |
| `/fmu/in/vehicle_command` | `MissionController` | Sends `DO_SET_MODE`, `COMPONENT_ARM_DISARM`, and fallback commands (e.g., land). |
| `/fmu/in/vehicle_command` | `metrics_node`/others | May inject additional commands (e.g., land on finish). |

All OverRack Scan publishers follow the PX4 recommendation of sending setpoints for at least 1 s before arming or switching to Offboard. QoS profiles mirror PX4 defaults (`BEST_EFFORT`, `depth=1`) to avoid incompatibilities.

> **Frame conversions.** Mission plans remain ENU-centric. The `px4io/setpoints.py` adapter converts each command into PX4’s NED frame, converts yaw from degrees to radians, subtracts the spawn offset captured from the first `/fmu/out/vehicle_local_position`, and clamps the result to the configured `world_bounds`. The same helper is reused both for publishing and debugging so what you log is exactly what PX4 receives.

## Agent Command Options and Ports
The agent command is resolved from the params YAML (`run_ros2_system.ros__parameters.agent_cmd_default` or per-drone `agent_cmd` in multi) and, if missing, from env overrides (`AGENT_CMD`/`XRCE_AGENT_CMD`/`MICROXRCE_AGENT_CMD`/`MICRORTPS_AGENT_CMD`). 

```bash
MicroXRCEAgent udp4 -p 8888 -v 6            # default verbose UDP agent
MicroXRCEAgent udp4 -p 8888 -v 4            # Less verbose UDP agent
```

Port 8888 is the UDP endpoint where the Micro XRCE-DDS Agent listens for incoming XRCE client sessions from PX4. PX4’s uXRCE-DDS client connects to this port to publish and receive DDS topics.

> Quick port map (as used here): 8888 = XRCE agent; 14550 = QGC MAVLink; 14540–14549 = companion MAVLink; 4560 = Gazebo TCP.

## Troubleshooting
- **Agent starts but no ROS 2 topics appear**: confirm PX4 and `px4_msgs` are built from the same PX4 release. A mismatch in message definitions silently drops samples. Rebuild `px4_msgs` after updating PX4.
- **`MicroXRCEAgent` exits immediately**: check for port conflicts on 8888 or invalid `--agent-cmd` syntax. Use `MicroXRCEAgent udp4 -p 8888 -v 6 --refs` to list active sessions.


Further reading: official PX4 uXRCE-DDS guide (PX4 v1.14) — https://docs.px4.io/v1.14/ko/middleware/uxrce_dds.html
