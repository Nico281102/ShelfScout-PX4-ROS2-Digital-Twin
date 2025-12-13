<!-- Practical setup notes for contributors. -->

# Environment Setup

These steps prepare the simulation stack using the multi-first flow (`scripts/run_system.sh`).

## Prerequisites
- Ubuntu 22.04 LTS with ROS 2 Humble (`ros-humble-desktop` installed).
- PX4-Autopilot checked out locally and built once with `make px4_sitl_default`.
- Gazebo Classic 11.x available on PATH (`gazebo`, `gzserver`, `gzclient`).
- Micro XRCE-DDS Agent installed or built from source (tested with `v2.4.3`).
- Python 3.10 with `pip`; `colcon` tooling (`colcon-core`, `colcon-ros`, `colcon-cmake`) for the ROS workspace.

## Required patches (run once)
- PX4 DDS topics (battery status): from repo root
  ```bash
  chmod +x scripts/apply_patch_dds_topics.sh   # one-time, if needed
  ./scripts/apply_patch_dds_topics.sh
  ```
  Patches `src/modules/uxrce_dds_client/dds_topics.yaml` under `SSDT_PX4_DIR` to publish `/fmu/out/battery_status`.
- PX4 multi-run helper (if you use the PX4 sitl_multiple_run wrapper):
  ```bash
  chmod +x scripts/apply_px4_sitl_multiple_run_patch.sh   # one-time
  ./scripts/apply_px4_sitl_multiple_run_patch.sh --px4-dir "$SSDT_PX4_DIR"
  ```

## One-Time Workspace Prep
1. Copy `scripts/.env.example` to `scripts/.env` and set the absolute paths: at minimum `SSDT_PX4_DIR` (your PX4 checkout) and optionally `SSDT_ROOT`, `SSDT_ROS_WS`, `SSDT_WORLD`, `SSDT_MISSION_FILE`, `SSDT_PARAM_PATH`, `SSDT_AGENT_CMD`, `SSDT_HEADLESS_DEFAULT`. Health-check toggles (`SSDT_REQUIRE_*`, `SSDT_*TIMEOUT*`, `SSDT_GZ_GRACE`) are documented in `docs/run_system.md`.
2. (Optional) create a virtual environment for helper scripts:
   ```bash
   python3.10 -m venv .venv
   source .venv/bin/activate
   pip install --upgrade pip
   pip install -r requirements.txt
   ```
3. Build PX4 SITL once in your PX4 tree:
   ```bash
   cd "$SSDT_PX4_DIR"
   make px4_sitl_default
   ```
4. Build the ROS 2 workspace (mission, inspection, metrics, plugins):
   ```bash
   cd "$SSDT_ROS_WS"
   colcon build --symlink-install --packages-select px4_msgs overrack_mission
   # add overrack_light_plugin if you need the torch plugin; px4_ros_com only if you use its nodes
   source install/setup.bash
   ```
5. Verify Gazebo paths: `GAZEBO_MODEL_PATH` should include both PX4’s `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models` and this repo’s `models`; `GAZEBO_PLUGIN_PATH` must include `ros2_ws/install/lib` so the torch plugin loads. The launch scripts set these automatically when you use them.

## Daily Workflow
1. Run `./scripts/run_system.sh ...` 
2. Edit `config/sim/multi_1drone.yaml` (or `config/sim/multi.yaml`) to pick mission file, world, agent command, and the `drones` list. Keep `gazebo_model_name` aligned with your PX4 build.
3. If you change anything under `ros2_ws/src/**`, rebuild the touched packages and re-source:
   ```bash
   cd "$SSDT_ROS_WS"
   colcon build --symlink-install --packages-select overrack_mission   # add overrack_light_plugin if you changed it
   source install/setup.bash
   ```
4. Launch with `./scripts/run_system.sh --gui` (or `--headless` on CI/SSH). Logs land in `data/logs/` for PX4, the agent, and mission nodes. The legacy `run_ros2_system.sh` remains available but follows the old single-drone path.



