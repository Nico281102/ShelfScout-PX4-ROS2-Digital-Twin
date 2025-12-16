# Foundations: PX4, Gazebo Classic, and ROS 2/DDS
This primer gives newcomers the minimum context before diving into `docs/architecture.md` or the detailed bridge guide (`docs/uxrce_dds_px4_ros_bridge.md`). It focuses on concepts and why the stack is built this way; implementation specifics live in the other docs.

## 1) PX4 Autopilot — Fundamentals
- **What PX4 is:** a modular, real-time autopilot that runs a publish/subscribe bus called **uORB**. PX4 modules exchange telemetry and control data through uORB topics.
- **Protocols on the wire:**
  - **MAVLink** for SITL/HIL, telemetry, and command/control over serial/UDP/TCP.
  - **uXRCE-DDS** for direct DDS/ROS 2 integration: PX4 publishes/subscribes DDS entities via the embedded Micro XRCE-DDS client.
- **Offboard mode:** PX4 expects a continuous stream of setpoints from a single publisher at a steady rate. Dropping heartbeats or mixing multiple competing publishers can cause Offboard to disengage or behave unpredictably.
- **Where to go deeper:** the PX4 ↔ DDS bridge, topic namespaces, and frame conversions are explained in [ROS 2 Bridge](uxrce_dds_px4_ros_bridge.md).

## 2) Gazebo Classic — Role in Simulation
- **What it is:** a physics engine plus sensor simulator. Worlds and robots are described in **SDF** files; we use **Jinja-templated SDF** to parameterize models (airframe, sensors, environment).
- **PX4–Gazebo link:** the MAVLink SITL plugin bridges PX4 and Gazebo: PX4 outputs actuator commands, Gazebo simulates physics/sensors and feeds PX4 with IMU/baro/vision data.
- **Sensors/actuators:** cameras, IMU, barometer, and motors are defined in SDF; plugins publish simulated sensor data and consume actuator commands.
- **Multi-drone sims:** multiple PX4 instances connect to the same Gazebo world; each instance has its own MAVLink ports and uXRCE namespace (see `docs/architecture.md` for the multi-drone topology).

## 3) ROS 2 and DDS — Fundamentals
- **DDS basics:** a distributed pub/sub middleware with automatic discovery, QoS policies, and strong typing (IDL-defined message types with generated serializers/deserializers).
- **QoS essentials:** `reliable` vs `best_effort`; depth/history to bound queues. PX4 uses `BEST_EFFORT` and depth=1 for high-rate telemetry so stale data is dropped instead of queued.
- **uXRCE client vs Micro XRCE Agent:** PX4 firmware embeds the **Micro XRCE-DDS client** (lightweight, runs on the autopilot) while the **Micro XRCE Agent** runs on the host (PC/edge), translates XRCE sessions to full DDS for ROS 2 nodes.
- **Why DDS here instead of a MAVSDK backend:** DDS already provides discovery, fan-in/fan-out, and QoS for many ROS 2 nodes; application-level logic (mission runner, fallbacks, inspection) arbitrates commands. See the rationale in [ROS 2 Bridge](uxrce_dds_px4_ros_bridge.md#why-we-dont-use-a-mavsdk-backend).

## 4) Glossary and Resources
- **uORB** – PX4’s internal pub/sub bus.
- **MAVLink** – telemetry/command protocol used by PX4 and SITL/HIL.
- **DDS** – distributed pub/sub middleware; ROS 2 uses DDS under the hood.
- **XRCE** – “eXtremely Resource Constrained Environments”; the Micro XRCE-DDS client in PX4 talks to the Micro XRCE Agent on the host.
- **SDF** – Simulation Description Format used by Gazebo; templates can be written with Jinja for parametrization.
- **SITL** – Software-in-the-Loop simulation (PX4 runs as a host process and talks to Gazebo).
- **Internal docs:** `docs/architecture.md`, `docs/uxrce_dds_px4_ros_bridge.md`, `docs/run_system.md`, `docs/mission_language.md`.
- **Official references:** PX4 ([https://docs.px4.io](https://docs.px4.io)), ROS 2 ([https://docs.ros.org](https://docs.ros.org)), Gazebo Classic ([http://classic.gazebosim.org/tutorials](http://classic.gazebosim.org/tutorials)).
