# ShelfScout PX4‑ROS2 Digital Twin
Concise guide to the simulation workflow. Full details live in `docs/`.

## Overview
- Indoor PX4 SITL + Gazebo Classic sandbox with ROS 2 mission runner, inspection, and metrics.
- Architecture and flow: `docs/architecture.md`.
- PX4 ↔ ROS 2 bridging and topic map: `docs/ROS2_PX4_BRIDGING.md`.

## Quick start
- Prerequisites and environment setup: `docs/environment_setup.md`.
- Launch everything (multi flow, GUI):  
  ```bash
  ./scripts/run_system.sh --gui --params config/sim/multi_2drones.yaml
  ```  
  Use `--headless` on CI/SSH. Pick mission/world/agent in the params YAML. Short walkthrough and expected output: `docs/quickstart.md`.

## Configuration
- Simulation params (world, agents, drones): `config/sim/*.yaml` — structure described in `docs/mission_language.md`.
- Mission + route schema and FSM behavior: `docs/mission_v1.md` (referenced by `docs/mission_language.md`).

## Developer docs
- FSM, launch flow, per-node notes: `docs/fsm.md`, `docs/developer_guide.md`.
- PX4/Gazebo models, plugins, and custom drone notes: `docs/px4_gazebo_notes.md`.

## Troubleshooting
- Common issues and fixes: `docs/troubleshooting.md`.
