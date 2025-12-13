# ShelfScout PX4‑ROS2 Digital Twin
Concise guide to the simulation workflow. Full details live in [docs/](docs/).

**Project origin.** This started as a tiny prototype inspired by [overrack-inspect](docs/px4-vision-overrack-spec-v1.pdf); I used it as my first hands-on experiment with ROS 2 and PX4. It grew into an indoor rack-inspection digital twin: multi-drone Gazebo Classic worlds, PX4 SITL bridged via Micro XRCE-DDS, ROS 2 mission runner and YAML-driven mission/route configs you can extend.


## Overview
- Indoor PX4 SITL + Gazebo Classic sandbox with ROS 2 mission runner, inspection, and metrics.
- Architecture and flow: [Architecture](docs/architecture.md).
- PX4 ↔ ROS 2 bridging and topic map: [ROS 2 bridge](docs/uxrce_dds_px4_ros_bridge.md).

## Quick start
- Prerequisites and environment setup: [Environment setup](docs/environment_setup.md).
- Launch everything (multi flow, GUI):
```bash
./scripts/run_system.sh --gui --params config/sim/multi_2drones.yaml
```
- For the mega indoor four-drone preset:
 ```bash
 ./scripts/run_system.sh --gui --params config_mega_overrack_indoor/sim/multi_4drones.yaml
 ```
- Use `--headless` on CI/SSH. Pick mission/world/agent in the params YAML.

### Expected output

ShelfScout multi-drone demo in the smaller indoor world:
[![ShelfScout Multi-Drone Demo](docs/preview/multli_2drones_preview.png)](https://youtu.be/KUdoRD7VAOU)

Mega over-rack four-drone preset:
[![Mega ShelfScout Multi-Drone](docs/preview/multli_4drones_preview.png)](https://youtu.be/LL4VuSNlRJE)

For more, follow the walkthrough: [Quickstart](docs/quickstart.md).

## Configuration
- Simulation params (world, agents, drones): `config/sim/*.yaml` — structure described in [Mission language](docs/mission_language.md).
- Mission and route schema and FSM behavior: [Mission v1](docs/mission_v1.md) (referenced by [Mission language](docs/mission_language.md)).

## Developer docs
- FSM, launch flow, per-node notes: [FSM](docs/fsm.md).
- PX4/Gazebo models, plugins, and custom drone notes: see [Architecture](docs/architecture.md) (models/airframes) and [Troubleshooting](docs/troubleshooting.md) (multi-drone/custom model pitfalls).

## Troubleshooting
- Common issues and fixes: [Troubleshooting](docs/troubleshooting.md).

## How AI was used in this project
- Assisted with heavy refactors such as this README overhaul.
- Generated the initial documentation drafts that the developer later revised (quickstart, architecture, etc.).
- Suggested cover art/thumbnail ideas so the screencast is easier to understand at a glance.
- Served as a fast prototyping partner for text/code snippets.
- Helped clarify the ROS 2 ↔ PX4 bridge by summarizing the original PX4 docs: https://docs.px4.io/v1.14/ko/middleware/uxrce_dds.html.
- Caveats to remember:
  - Don’t hand over total control; review every suggestion before applying it.
  - Use the tool as a guided assistant, not an autopilot—steer it, don’t let it steer you.
