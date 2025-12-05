<!-- Practical setup notes for contributors. Quickstart remains deferred until the multi-drone branch stabilises. -->

# Environment Setup

These steps prepare the single-drone simulation stack. Multi-drone work is happening on a separate branch; when it lands, revisit port assignments and namespaces before copying these steps verbatim.

## Prerequisites
- Ubuntu 22.04 LTS with ROS 2 Humble (`ros-humble-desktop` installed).
- PX4-Autopilot checked out locally and built once with `make px4_sitl_default`.
- Gazebo Classic 11.x available on PATH (`gazebo`, `gzserver`, `gzclient`).
- Micro XRCE-DDS Agent installed or built from source (tested with `v2.4.3`).
- Python 3.10 with `pip`; `colcon` tooling (`colcon-core`, `colcon-ros`, `colcon-cmake`) for the ROS workspace.

## One-Time Workspace Prep
1. Copy `scripts/.env.example` to `scripts/.env` and set the absolute paths: `SSDT_PX4_DIR` (your PX4 checkout), `SSDT_ROS_WS` (this repo’s `ros2_ws`), `SSDT_AGENT_CMD` (agent command), and `SSDT_WORLD` if you want a non-default world.
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
   colcon build --symlink-install --packages-select px4_msgs px4_ros_com overrack_mission overrack_light_plugin
   source install/setup.bash
   ```
5. Verify Gazebo paths: `GAZEBO_MODEL_PATH` should include both PX4’s `Tools/simulation/gazebo-classic/sitl_gazebo-classic/models` and this repo’s `models`; `GAZEBO_PLUGIN_PATH` must include `ros2_ws/install/lib` so the torch plugin loads. The launch scripts set these automatically when you use them.

## Daily Workflow
1. `source scripts/.env` (or just run `./scripts/run_ros2_system.sh ...` which sources it for you).
2. Edit `config/sim/default.yaml` to pick the mission file, world, and agent command. Keep `gazebo_model_name` aligned with your PX4 build.
3. If you change anything under `ros2_ws/src/**`, rebuild and re-source:
   ```bash
   cd "$SSDT_ROS_WS"
   colcon build --symlink-install --packages-select overrack_mission overrack_light_plugin
   source install/setup.bash
   ```
4. Launch with `./scripts/run_ros2_system.sh --gui` (or `--headless` on CI/SSH). Logs land in `data/logs/` for PX4, the agent, and mission nodes.

## Notes and Gotchas
- Battery simulation relies on PX4 parameters pushed by `px4_param_setter` (see `config/sim/default.yaml`); Gazebo’s battery plugin is ignored.
- If Gazebo cannot find models it may try to download them from Fuel and stall; set `GAZEBO_MODEL_PATH` explicitly or stay online for the first run.
- Multi-drone will require per-vehicle ports and namespaces; avoid hardcoding ports in scripts—prefer the env-driven defaults in `scripts/.env`.
