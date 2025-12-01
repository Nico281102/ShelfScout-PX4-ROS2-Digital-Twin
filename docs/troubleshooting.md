# Troubleshooting

## Gazebo stalls after `Using: ...iris_opt_flow.sdf`

**Symptoms**

- `run_ros2_system.sh --gui` prints `Using: /.../iris_opt_flow.sdf` and then hangs.
- No new log entries appear in `data/logs/px4_sitl_default.out`.

**Context**

- GUI or headless start on Ubuntu 22.04 with ROS 2 Humble.
- First run on a fresh or offline machine.

**Likely cause**

- Gazebo Classic cannot find models locally and waits to download them from Fuel, or `GAZEBO_MODEL_PATH` does not include PX4/repo model paths.

**Fix**

1. Export model paths before launch (or confirm `scripts/.env` adds them):
   ```bash
   export GAZEBO_MODEL_PATH="$SSDT_PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models:$(pwd)/models:${GAZEBO_MODEL_PATH:-}"
   ```
2. Relaunch `./scripts/run_ros2_system.sh --headless` to verify `gzserver` starts; retry `--gui` if you need the client.
3. If offline, run once with network available so PX4 models are cached; after the first download they remain local.

**Notes (optional)**

- `gzserver` prints `Using: ...iris_opt_flow.sdf` before resolving model dependencies. If those dependencies are missing from `GAZEBO_MODEL_PATH`, try launching `gazebo -s libgazebo_ros_api_plugin.so` to see the full warnings.
- If the spawn call races `gzserver` (PX4 stops after `Using: ...iris_opt_flow.sdf`), patch `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh` to delay the spawn and retry until Gazebo is ready. See the detailed workaround below.

### PX4 + Gazebo Classic spawn race (code workaround)
When PX4 prints `Using: ...iris_opt_flow.sdf` and stalls, `gz model --spawn-file=...` is reaching Gazebo before the world is fully initialised. The fix we applied in our PX4 fork adds a grace period and keeps retrying the spawn:

```bash
gzserver $verbose $world_path $ros_args &
SIM_PID=$!

# ... resolve modelpath ...

echo "Using: ${modelpath}/${model}/${model}.sdf"

# [BEGIN Customization]
sleep 4 # give gazebo a moment to finish loading plugins/world
# [END Customization]

while gz model --verbose --spawn-file="${modelpath}/${model}/${model_name}.sdf" \
  --model-name=${model} -x 1.01 -y 0.98 -z 0.83 2>&1 | \
  grep -q "An instance of Gazebo is not running."; do
  echo "gzserver not ready yet, trying again!"
  sleep 1
done
```
Place this in `PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_run.sh` after the `gzserver` start to prevent the race.

## Battery stuck near 50% or warnings fire too early

**Symptoms**

- PX4 logs report `battery warning` too early or never drop below ~50%.
- `mission_runner` does not enter `battery_warning`/`battery_critical` fallbacks as expected.

**Context**

- Single-drone simulation with `run_ros2_system.sh` on PX4 v1.14 SITL.
- Battery parameters left at PX4 defaults.

**Likely cause**

- The Gazebo battery plugin is ignored; PX4’s internal battery simulator uses default values (slow drain and high thresholds), so warnings do not match the mission.

**Fix**

1. Open `ros2_ws/src/overrack_mission/overrack_mission/param/sim.yaml` and tune `px4_param_setter.px4_params`:
   - `SIM_BAT_DRAIN` and `SIM_BAT_MIN_PCT` for discharge rate.
   - `BAT1_*` (cells, voltages, capacity) to model the pack.
   - `BAT_LOW_THR`, `BAT_CRIT_THR`, `BAT_EMERGEN_THR` to align warnings.
   - Leave `COM_LOW_BAT_ACT: 0` if you want mission fallbacks to own the response.
2. Relaunch `./scripts/run_ros2_system.sh`; PX4 does not need a rebuild for these parameters.
3. Check `data/logs/px4_sitl_default.out` to confirm `px4_param_setter` applied the values (search for `param set`).

**Notes (optional)**

- Mission fallbacks (`return_home`, `land`, etc.) trigger only when `battery_status` topics reflect the chosen thresholds. Misaligned values can keep the drone flying past the planned budget.
- Gazebo’s battery plugin is ignored: PX4 uses its internal SITL battery simulator. Tune PX4 parameters instead of editing SDF battery plugins.

## Battery topic missing on ROS 2

**Symptoms**

- `battery_status` (or another battery-related uORB topic) is absent from `/fmu/out/*` on the ROS 2 graph.

**Likely cause**

- The uORB ↔ DDS mapping omits the topic; PX4’s default RTPS mapping may not include custom or less common battery messages.

**Fix**

1. In your PX4 tree, edit `src/modules/uxrce_dds_client/dds_topics.yaml` and add the publication under the `publications:` list:
   ```yaml
   - topic: /fmu/out/battery_status
     type: px4_msgs::msg::BatteryStatus
   ```
   (If other battery-related topics are needed, list them here as well.)
2. Rebuild PX4 (regenerates the uXRCE-DDS client code and the mapping):
   ```bash
   cd "$SSDT_PX4_DIR"
   make px4_sitl_default
   ```
3. Rebuild ROS messages/bridge so ROS 2 sees the updated mapping:
   ```bash
   cd "$SSDT_ROS_WS"
   colcon build --symlink-install --packages-select px4_msgs px4_ros_com
   ```
4. Relaunch `run_ros2_system.sh`; `/fmu/out/battery_status` should now appear.

## Drone spawns off-origin (diagonal takeoff)

**Symptoms**

- The Iris model spawns near shelves instead of the world origin.
- PX4 odometry starts at NED (0, 0, 0).
- Takeoff shows a low diagonal drift instead of a clean vertical climb.
- Bootstrap setpoints in logs already show non-zero ENU X/Y.

**Root cause (frames & spawn offset)**

- Missions use the map/world ENU frame; PX4 flies in its local NED frame.
- A non-zero spawn offset exists when the drone spawns away from ENU (0, 0). If subtracted too early, PX4 flies diagonally at low altitude.

**How the stack handles it now**

1. `Telemetry` subscribes to `/gazebo/model_states` and polls `/gazebo/get_entity_state` or `/gazebo/get_model_state` to cache:
   ```text
   spawn_offset_enu = (x, y, z)  # Gazebo world frame
   ```
2. `MissionController._check_spawn_sync_ready()` blocks until:
   - spawn sync is READY (or times out),
   - PX4 local position is valid (`local_position().xy_valid == True`),
   - `spawn_offset_enu` is present.
   It logs messages like:
   - `Waiting for Gazebo model detection before capturing spawn offset...`
   - `Waiting for Gazebo spawn offset (query in flight)...`
   - `Waiting for EKF local position...`
   - `Spawn offset fetched from Gazebo (...): ENU (x=..., y=..., z=...)`
3. Bootstrap states send 20 pure-vertical setpoints using `MissionContext.bootstrap_position_enu()`:
   ```text
   (0.0, 0.0, plan.altitude_m) -> NED (0.0, 0.0, -ALT)
   ```
   `SetpointPublisher` runs with `bootstrap_mode=True`, so no spawn offset is applied.
4. On exiting bootstrap, the controller calls `set_bootstrap_mode(False)` and starts subtracting `spawn_offset_enu` before converting to NED:
   ```text
   local_enu = enu_position - spawn_offset_enu
   NED       = (local_enu.y, local_enu.x, -local_enu.z)
   ```

**Fix**

1. Confirm `gazebo_model` matches `/gazebo/model_states` (e.g., `iris_opt_flow`).
2. Look for spawn-offset logs; if missing, wait for Gazebo pose and PX4 odometry.
3. Use a safe takeoff altitude (2–3 m) so horizontal motion begins above obstacles.
4. If return-home must go back to the spawn point, set `home_position` in the mission YAML to the ENU X/Y printed in the spawn-offset log:
   ```yaml
   home_position: [1.01, 0.98]  # example ENU spawn coordinates
   ```

## Light plugin appears on when toggled off

**Symptoms**

- Torch still lights the scene even after the plugin reports switching OFF.

**Likely cause**

- Only client-side OGRE properties were changed; the server-side light entity remained active.

**Fix**

1. Use Gazebo’s server-side light control (`/world/<world>/light/modify`) via `gazebo::msgs::Light`.
2. When toggling OFF, set near-zero range, disable shadows/visibility, and publish the update with the scoped light name.
3. When toggling ON, restore range/attenuation/diffuse values and re-enable shadows so renderer and physics stay in sync.

**Code reference (old vs new approach)**

Old client-side-only attempt (insufficient):
```c++
if (on)
{
    light_->SetDiffuseColor(ignition::math::Color(1, 0.95, 0.8, 1));
    light_->SetSpecularColor(ignition::math::Color(0.3, 0.3, 0.3, 1));
    light_->SetAttenuation(1.0, 0.1, 0.01);
}
else
{
    light_->SetDiffuseColor(ignition::math::Color(0, 0, 0, 1));
    light_->SetSpecularColor(ignition::math::Color(0, 0, 0, 1));
    light_->SetAttenuation(0.0, 1.0, 1.0);
    light_->SetRange(0.01);
    light_->SetVisible(false);
    light_->SetCastShadows(false);
}
```

Working approach (client + server):
```c++
// Client-side update
if (on)
{
    light_->SetDiffuseColor(ignition::math::Color(1, 0.95, 0.8, 1));
    light_->SetSpecularColor(ignition::math::Color(0.3, 0.3, 0.3, 1));
    light_->SetAttenuation(1.0, 0.1, 0.01);
    light_->SetRange(20.0);
    light_->SetVisible(true);
    light_->SetCastShadows(true);
}
else
{
    light_->SetDiffuseColor(ignition::math::Color(0, 0, 0, 1));
    light_->SetSpecularColor(ignition::math::Color(0, 0, 0, 1));
    light_->SetAttenuation(0.0, 1.0, 1.0);
    light_->SetRange(0.01);
    light_->SetVisible(false);
    light_->SetCastShadows(false);
}

// Server-side authoritative update
msgs::Light msg;
msg.set_name(scoped_light_name_);
msg.set_cast_shadows(on);
msg.set_range(on ? 20.0 : 0.01);
auto *diff = msg.mutable_diffuse();
if (on)
{
    diff->set_r(1.0); diff->set_g(0.95); diff->set_b(0.8); diff->set_a(1.0);
}
else
{
    diff->set_r(0.0); diff->set_g(0.0); diff->set_b(0.0); diff->set_a(1.0);
}
light_pub_->Publish(msg);
```

## Transparent roof leaks sunlight

**Symptoms**

- Indoor scene stays bright; LOW_LIGHT events are unreliable because sun light passes through.

**Likely cause**

- The sun light passes through the collision mesh; visual transparency only affects the viewer.

**Fix**

1. Remove the global sun include in the world file.
2. Keep the roof visual transparent for debugging if needed.
3. Light the scene with indoor spotlights (“neons”) and the drone torch to stabilise LOW_LIGHT detection.

**XML snippets**

Disable the sun:
```xml
<!-- Sun disabled: its light interferes with indoor lighting -->
<!--
<include>
  <uri>model://sun</uri>
</include>
-->
```

Keep a transparent visual:
```xml
<visual name="vis">
  <geometry><box><size>12 10 0.1</size></box></geometry>
  <material>
    <script>
      <uri>file://../models/materials/scripts/custom.material</uri>
      <name>Custom/AlphaBlend</name>
    </script>
    <ambient>1 1 1 0.1</ambient>
    <diffuse>1 1 1 0.1</diffuse>
  </material>
</visual>
```

## Offboard rejected

**Symptoms**

- PX4 rejects Offboard activation or arming when running the mission runner.

**Likely cause**

- Offboard setpoints are not streamed before `VehicleCommand.DO_SET_MODE`, or preflight checks fail.

**Fix**

1. Ensure at least 20 trajectory setpoints are sent before switching to Offboard.
2. Inspect `vehicle_status.nav_state` and preflight status via `px4io/telemetry.py` helpers or `px4_sitl_default.out`.

## Drift or “drone leaves the room”

**Symptoms**

- The vehicle drifts or exits the expected indoor volume.

**Likely cause**

- World bounds or spawn-offset corrections are misaligned.

**Fix**

1. Set `debug_frames:=true` to log ENU/NED/offset snapshots per waypoint.
2. Verify `world_bounds` in `param/sim.yaml` and ensure waypoints fall inside the envelope.

## PX4 battery failsafe overrides mission fallbacks

**Symptoms**

- PX4 triggers RTL/land before mission fallbacks run.

**Likely cause**

- PX4 `COM_LOW_BAT_ACT` is set to a mode that preempts the FSM.

**Fix**

1. Set `COM_LOW_BAT_ACT` to `Warning` (or adjust thresholds) via `px4_param_setter` in `param/sim.yaml`.
2. Keep mission fallbacks in sync with PX4 thresholds.

## No `/fmu/out/*` topics

**Symptoms**

- ROS 2 graph is missing PX4 telemetry topics.

**Likely cause**

- Micro XRCE-DDS Agent crash, message-definition mismatch, or UDP port conflict.

**Fix**

1. Check `data/logs/micro_xrce_agent.out` for errors.
2. Ensure the PX4 branch matches the agent binary and that UDP ports 2019/2020 are free.

## Fallback not firing

**Symptoms**

- Mission fallbacks never execute.

**Likely cause**

- Triggers in the mission YAML do not match supported events.

**Fix**

1. Use supported triggers (`battery_warning`, `battery_critical`). Link loss is handled directly by PX4 when Offboard heartbeats stop.
2. Confirm the mission YAML matches `docs/mission_v1.md`.

## Gazebo refuses to start (general)

**Symptoms**

- Gazebo fails at startup without reaching the world load.

**Likely cause**

- Stale PX4 build or missing model paths.

**Fix**

1. Rebuild PX4 (`make px4_sitl_default`).
2. Try `--headless` to rule out GUI issues.
3. Ensure `GAZEBO_MODEL_PATH` includes both PX4 models and OverRack models.
