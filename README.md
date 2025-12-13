# ShelfScout PX4‑ROS2 Digital Twin
Concise guide to the simulation workflow. Full details live in `docs/`.
Come è nato questo progetto, l'ispirazione iniziale era data dal documento [overrack-inspect](docs/px4-vision-overrack-spec-v1.pdf), poi pero si è evoluto, perchè ho voluto di mia volontà esplorare le possibilia con ROS2, questa infatti è la mia primissima esperienza in ambiente ROS2.


## Overview
- Indoor PX4 SITL + Gazebo Classic sandbox with ROS 2 mission runner, inspection, and metrics.
- Architecture and flow: `docs/architecture.md`.
- PX4 ↔ ROS 2 bridging and topic map: `docs/uxrce_dds_px4_ros_bridge.md`.

## Quick start
- Prerequisites and environment setup: `docs/environment_setup.md`.
- Launch everything (multi flow, GUI):  
  ```bash
  ./scripts/run_system.sh --gui --params config/sim/multi_2drones.yaml
  ```  
  Use `--headless` on CI/SSH. Pick mission/world/agent in the params YAML.

## Expected output

[![ShelfScout Multi-Drone Demo](docs/preview/multli_2drones_preview.png)](https://youtu.be/KUdoRD7VAOU)

Watch the screencast and follow the walkthrough: `docs/quickstart.md`.

## Configuration
- Simulation params (world, agents, drones): `config/sim/*.yaml` — structure described in `docs/mission_language.md`.
- Mission + route schema and FSM behavior: `docs/mission_v1.md` (referenced by `docs/mission_language.md`).

## Developer docs
- FSM, launch flow, per-node notes: `docs/fsm.md`, `docs/developer_guide.md`.
- PX4/Gazebo models, plugins, and custom drone notes: see `docs/architecture.md` (models/airframes) and `docs/troubleshooting.md` (multi-drone/custom model pitfalls).

## Troubleshooting
- Common issues and fixes: `docs/troubleshooting.md`.

## How AI was used in this project
- Assisted with heavy refactors such as this README overhaul.
- Generated the initial documentation drafts that the developer later revised (quickstart, architecture, etc.).
- Suggested cover art/thumbnail ideas so the screencast is easier to understand at a glance.
- Served as a fast prototyping partner for text/code snippets.
- Helped clarify the ROS 2 ↔ PX4 bridge by summarizing the original PX4 docs: https://docs.px4.io/v1.14/ko/middleware/uxrce_dds.html.
- Caveats to remember:
  - Don’t hand over total control; review every suggestion before applying it.
  - Use the tool as a guided assistant, not an autopilot—steer it, don’t let it steer you.
