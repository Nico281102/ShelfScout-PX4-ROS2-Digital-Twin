# PX4 + Gazebo Classic Notes

Operational notes for the PX4 SITL + Gazebo Classic stack. Focus is on the single-drone setup; multi-drone will reuse the same patterns with per-vehicle ports and namespaces.

## Simulator Basics
- World: `worlds/overrack_indoor.world` by default (see `config/sim/default.yaml`). Models from PX4 and this repo are appended to `GAZEBO_MODEL_PATH` by the launch scripts.
- Vehicle: `iris_opt_flow` (or another PX4 model) controlled by PX4 SITL. Keep `mission_runner.ros__parameters.gazebo_model_name` aligned with the chosen SDF.
- Launch: `scripts/run_ros2_system.sh --gui` (or `--headless`) wraps PX4, Gazebo, the XRCE agent, and ROS 2 nodes with consistent logging in `data/logs/`.

## Battery Simulation (no Gazebo battery plugin)
- Gazebo’s battery plugin in the SDF is ignored; PX4’s internal SITL battery simulator is used instead.
- Battery parameters are pushed via `px4_param_setter` in `config/sim/default.yaml`:
  - `SIM_BAT_DRAIN` (seconds to drain fully), `SIM_BAT_MIN_PCT` (minimum percentage).
  - `BAT1_*` voltage/cell/capacity values to mirror the intended pack.
  - `BAT_LOW_THR`, `BAT_CRIT_THR`, `BAT_EMERGEN_THR` to align PX4 warnings with mission fallbacks.
  - `COM_LOW_BAT_ACT` is set to `0` so mission-controlled fallbacks stay in charge.
- Adjust these in the YAML to model realistic runtimes; rebuilding PX4 is not required after parameter tweaks.

## uORB ↔ DDS Mapping
- The PX4 ↔ ROS 2 bridge relies on the uORB ↔ RTPS mapping generated from PX4’s `uorb_rtps_message_ids.yaml`. If you add new uORB topics (e.g., for multi-drone coordination), regenerate the agent/client code in the PX4 tree and rebuild `px4_msgs` + `px4_ros_com`:
  ```bash
  cd "$SSDT_PX4_DIR"
  make px4_sitl_default
  cd "$SSDT_ROS_WS"
  colcon build --symlink-install --packages-select px4_msgs px4_ros_com
  ```
- Keep the XRCE agent command in `config/sim/default.yaml` in sync with any port or topic list changes (`-r`/`-s` flags for PX4 UDP, `-p` for XRCE).

## Rendering and Headless Mode
- `--gui` starts `gzclient`; `--headless` only runs `gzserver`. On headless servers, ensure `DISPLAY` is unset or use `xvfb-run` if you need the client.
- When Gazebo stalls on first run while downloading models, either stay online or pre-populate `GAZEBO_MODEL_PATH` with PX4’s model directory to avoid Fuel lookups.

## Logs Worth Checking
- `data/logs/px4_sitl_default.out` — PX4 startup, parameter pushes, Offboard status, battery warnings.
- `data/logs/micro_xrce_agent.out` — bridge health (session creation, port binding).
- `data/logs/mission_runner.out` — mission parsing, bounds validation, fallback reasons.
