#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[run_ros2_system] Missing environment file: $ENV_FILE" >&2
  exit 1
fi

# shellcheck disable
set +u
source "$ENV_FILE"
set -u

ROOT_DIR="${SSDT_ROOT:-${SHELFSCOUT_ROOT:-$ROOT_DIR}}"
PX4_DIR="${SSDT_PX4_DIR:-${SHELFSCOUT_PX4_DIR:-${PX4_DIR:-}}}"
if [[ -z "${PX4_DIR:-}" ]]; then
  echo "[run_ros2_system] PX4_DIR is not set. Check $ENV_FILE." >&2
  exit 1
fi

resolve_with_root() {
  local candidate="$1"
  if declare -F ssdt_resolve_path >/dev/null 2>&1; then
    ssdt_resolve_path "$candidate"
    return
  elif declare -F shelfscout_resolve_path >/dev/null 2>&1; then
    shelfscout_resolve_path "$candidate"
    return
  fi
  if [[ -z "$candidate" ]]; then
    printf '%s\n' "$ROOT_DIR"
    return
  fi
  if [[ "$candidate" = /* ]]; then
    printf '%s\n' "$candidate"
  else
    printf '%s/%s\n' "$ROOT_DIR" "$candidate"
  fi
}

# Read YAML defaults (mission/world/agent) via python+PyYAML to keep parsing logic simple.
load_yaml_defaults() {
  local yaml_file="$1"
  local -a defaults
  if [[ ! -f "$yaml_file" ]]; then
    return
  fi
  if ! command -v python3 >/dev/null 2>&1; then
    echo "[run_ros2_system] python3 not found; skipping YAML defaults" >&2
    return
  fi
  if mapfile -t defaults < <(python3 - "$yaml_file" <<'PY'
import sys
try:
    import yaml
except ImportError:
    sys.stderr.write("[run_ros2_system] PyYAML is required to parse YAML defaults.\n")
    sys.exit(1)
from pathlib import Path
path = Path(sys.argv[1])
with path.open('r', encoding='utf-8') as handle:
    data = yaml.safe_load(handle) or {}
def nested(container, keys):
    for key in keys:
        if not isinstance(container, dict):
            return ""
        container = container.get(key)
        if container is None:
            return ""
    return container if isinstance(container, str) else ""
mission = (
    nested(data, ["run_ros2_system", "ros__parameters", "mission_file"])
    or nested(data, ["mission_runner", "ros__parameters", "mission_file"])
    or ""
)
world = nested(data, ["run_ros2_system", "ros__parameters", "world_file"]) or ""
agent = nested(data, ["run_ros2_system", "ros__parameters", "agent_cmd"]) or ""
agent_default = nested(data, ["run_ros2_system", "ros__parameters", "agent_cmd_default"]) or agent
drones_yaml = nested(data, ["run_ros2_system", "ros__parameters", "drones_yaml"]) or ""
dr = []
if drones_yaml:
    try:
        import yaml as _yaml  # type: ignore
        dr = _yaml.safe_load(drones_yaml) or []
    except Exception:
        dr = []
if not isinstance(dr, list):
    dr = []
print(mission)
print(world)
print(agent_default)
print(len(dr))
PY
); then
    YAML_MISSION_DEFAULT="${defaults[0]:-}"
    YAML_WORLD_DEFAULT="${defaults[1]:-}"
    YAML_AGENT_DEFAULT="${defaults[2]:-}"
    YAML_DRONES_COUNT="${defaults[3]:-0}"
  else
    echo "[run_ros2_system] Unable to parse YAML defaults from $yaml_file" >&2
  fi
}

# Export drone definitions from the params file as shell assignments.
load_drones_env() {
  local yaml_file="$1"
  if [[ ! -f "$yaml_file" ]]; then
    return
  fi
  if ! command -v python3 >/dev/null 2>&1; then
    return
  fi
  python3 - "$yaml_file" <<'PY'
import json
import shlex
import sys
from pathlib import Path
try:
    import yaml
except ImportError:
    sys.exit(0)

path = Path(sys.argv[1])
if not path.is_file():
    sys.exit(0)
data = yaml.safe_load(path.read_text()) or {}
ros_params = ((data.get("run_ros2_system") or {}).get("ros__parameters") or {})
drones_yaml = ros_params.get("drones_yaml") or ""
if not drones_yaml:
    sys.exit(0)
try:
    drones = yaml.safe_load(drones_yaml) or []
except Exception:
    drones = []
if not isinstance(drones, list):
    sys.exit(0)
print(f"DRONE_COUNT={len(drones)}")
for idx, drone in enumerate(drones):
    name = drone.get("name") or f"drone{idx+1}"
    ns = (drone.get("namespace") or name).lstrip("/")
    model = drone.get("model") or drone.get("gazebo_model_name") or "iris_opt_flow"
    spawn = drone.get("spawn") or {}
    mission = drone.get("mission_file") or ""
    mav_sys_id = drone.get("mav_sys_id") or ""
    mavlink_port = drone.get("mavlink_udp_port") or ""
    agent_cmd = drone.get("agent_cmd") or ""
    log_dir = drone.get("log_dir") or ""
    px4_params = drone.get("px4_params") or {}
    def emit(key, value):
        print(f'DRONE_{key}[{idx}]={shlex.quote("" if value is None else str(value))}')
    emit("NAME", name)
    emit("NS", ns)
    emit("MODEL", model)
    emit("SPAWN_X", spawn.get("x", 0.0))
    emit("SPAWN_Y", spawn.get("y", 0.0))
    emit("SPAWN_Z", spawn.get("z", 0.0))
    emit("SPAWN_YAW", spawn.get("yaw", 0.0))
    emit("MISSION", mission)
    emit("MAV_SYS_ID", mav_sys_id)
    emit("MAVLINK_PORT", mavlink_port)
    emit("AGENT_CMD", agent_cmd)
    emit("LOG_DIR", log_dir)
    if isinstance(px4_params, dict):
        emit("PX4_PARAMS", json.dumps(px4_params))
    else:
        emit("PX4_PARAMS", "{}")
PY
}

ROS2_WS="$(resolve_with_root "${SSDT_ROS_WS:-${SHELFSCOUT_ROS_WS:-$ROOT_DIR/ros2_ws}}")"
LOG_DIR="$(resolve_with_root "${SSDT_LOG_DIR:-${SHELFSCOUT_LOG_DIR:-$ROOT_DIR/data/logs}}")"
mkdir -p "$LOG_DIR"

HEADLESS="${SSDT_HEADLESS_DEFAULT:-${SHELFSCOUT_HEADLESS_DEFAULT:-1}}"
DEFAULT_AGENT_CMD="${SSDT_AGENT_CMD:-${SHELFSCOUT_AGENT_CMD:-MicroXRCEAgent udp4 -p 8888 -v 6}}"
AGENT_CMD_ENV_OVERRIDE="${AGENT_CMD:-${XRCE_AGENT_CMD:-${MICROXRCE_AGENT_CMD:-${MICRORTPS_AGENT_CMD:-}}}}"
AGENT_CMD=""
CLEANED_UP=0
PX4_LOG="$(resolve_with_root "${SSDT_PX4_LOG:-${SHELFSCOUT_PX4_LOG:-$LOG_DIR/px4_sitl_default.out}}")"
GAZEBO_LOG="$(resolve_with_root "${SSDT_GAZEBO_LOG:-${SHELFSCOUT_GAZEBO_LOG:-$LOG_DIR/px4_gazebo.out}}")"
DEFAULT_WORLD="${SSDT_WORLD:-${SHELFSCOUT_WORLD:-worlds/overrack_indoor.world}}"
DEFAULT_MISSION_FILE="${SSDT_MISSION_FILE:-${SHELFSCOUT_MISSION_FILE:-config/mission_precomputed.yaml}}"
# Prefer the top-level config; keep legacy ROS package path as a fallback for older setups.
DEFAULT_PARAM_FILE_PRIMARY="config/sim/default.yaml"
DEFAULT_PARAM_FILE_LEGACY="ros2_ws/src/overrack_mission/overrack_mission/param/sim.yaml"
DEFAULT_PARAM_FILE="${SSDT_PARAM_FILE:-${SHELFSCOUT_PARAM_FILE:-$DEFAULT_PARAM_FILE_PRIMARY}}"
WORLD_PATH=""
MISSION_PATH=""
PARAM_FILE_PATH="${SSDT_PARAM_PATH:-${SHELFSCOUT_PARAM_PATH:-$(resolve_with_root "$DEFAULT_PARAM_FILE")}}"
YAML_MISSION_DEFAULT=""
YAML_WORLD_DEFAULT=""
YAML_AGENT_DEFAULT=""
YAML_DRONES_COUNT="0"
YAML_AGENT_DEFAULT_OVERRIDE=""

declare -a DRONE_NAME DRONE_NS DRONE_MODEL DRONE_SPAWN_X DRONE_SPAWN_Y DRONE_SPAWN_Z DRONE_SPAWN_YAW DRONE_MISSION DRONE_MAV_SYS_ID DRONE_MAVLINK_PORT DRONE_AGENT_CMD DRONE_LOG_DIR DRONE_PX4_PARAMS
DRONE_COUNT=0
PX4_WRAPPER_PIDS=()
AGENT_PIDS=()
PX4_LOGS=()

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1
      shift
      ;;
    --gui)
      HEADLESS=0
      shift
      ;;
    --params)
      PARAM_FILE_PATH="$(resolve_with_root "${2:-}")"
      shift 2
      ;;
    *)
      echo "[run_ros2_system] Unknown argument: $1 (supported: --gui, --headless, --params <file>)" >&2
      exit 1
      ;;
  esac
done

PRIMARY_PARAM_PATH="$(resolve_with_root "$DEFAULT_PARAM_FILE_PRIMARY")"
LEGACY_PARAM_PATH="$(resolve_with_root "$DEFAULT_PARAM_FILE_LEGACY")"

# Allow transparent fallback to the legacy ROS package param if the new config path is missing.
if [[ ! -f "$PARAM_FILE_PATH" ]] && [[ "$PARAM_FILE_PATH" == "$PRIMARY_PARAM_PATH" ]] && [[ -f "$LEGACY_PARAM_PATH" ]]; then
  echo "[run_ros2_system] Missing config/sim/default.yaml; falling back to legacy params at $LEGACY_PARAM_PATH"
  PARAM_FILE_PATH="$LEGACY_PARAM_PATH"
fi

if [[ ! -f "$PARAM_FILE_PATH" ]]; then
  echo "[run_ros2_system] Parameter file not found: $PARAM_FILE_PATH" >&2
  exit 1
fi

# Allow the YAML file to drive mission/world/agent defaults.
load_yaml_defaults "$PARAM_FILE_PATH"
if [[ -n "$YAML_WORLD_DEFAULT" ]]; then
  WORLD_PATH="$(resolve_with_root "$YAML_WORLD_DEFAULT")"
else
  WORLD_PATH="$(resolve_with_root "$DEFAULT_WORLD")"
fi
if [[ -n "$YAML_MISSION_DEFAULT" ]]; then
  MISSION_PATH="$(resolve_with_root "$YAML_MISSION_DEFAULT")"
else
  MISSION_PATH="$(resolve_with_root "$DEFAULT_MISSION_FILE")"
fi
if [[ -n "$YAML_AGENT_DEFAULT" ]]; then
  AGENT_CMD="$YAML_AGENT_DEFAULT"
elif [[ -n "$AGENT_CMD_ENV_OVERRIDE" ]]; then
  AGENT_CMD="$AGENT_CMD_ENV_OVERRIDE"
else
  AGENT_CMD="$DEFAULT_AGENT_CMD"
fi

read -r -a AGENT_CMD_ARRAY <<< "$AGENT_CMD"
if [[ ${#AGENT_CMD_ARRAY[@]} -eq 0 ]]; then
  echo "[run_ros2_system] Invalid AGENT_CMD: $AGENT_CMD" >&2
  exit 1
fi
if [[ "${YAML_DRONES_COUNT:-0}" -gt 0 ]]; then
  echo "[run_ros2_system] Drones declared in params: ${YAML_DRONES_COUNT}"
fi

# Populate drone arrays from drones_yaml (if any).
eval "$(load_drones_env "$PARAM_FILE_PATH")" || true
if (( DRONE_COUNT > 0 )); then
  echo "[run_ros2_system] Drones loaded:"
  for idx in $(seq 0 $((DRONE_COUNT - 1))); do
    echo "  - ${DRONE_NAME[$idx]:-drone$((idx+1))} (ns=${DRONE_NS[$idx]:-}, agent_cmd=${DRONE_AGENT_CMD[$idx]:-<unset>})"
  done
fi

resolve_agent_cmd_for_drone() {
  local idx="$1"
  local default_cmd="${2:-$DEFAULT_AGENT_CMD}"
  local cmd="${DRONE_AGENT_CMD[$idx]:-}"
  if [[ -z "$cmd" ]]; then
    cmd="$default_cmd"
    # Se più droni e il comando ha "-p <porta>", incrementa la porta per evitare collisioni.
    if (( DRONE_COUNT > 1 )) && [[ "$cmd" =~ (.*-p[[:space:]]*)([0-9]+)(.*) ]]; then
      local base_port="${BASH_REMATCH[2]}"
      cmd="${BASH_REMATCH[1]}$((base_port + idx))${BASH_REMATCH[3]}"
    fi
  fi
  printf '%s\n' "$cmd"
}

ensure_colcon_workspace() {
  if ! command -v colcon >/dev/null 2>&1; then
    echo "[run_ros2_system] colcon build not found in PATH" >&2
    exit 1
  fi
  if [[ ! -d "$ROS2_WS/src" ]]; then
    echo "[run_ros2_system] Missing ROS2 workspace: $ROS2_WS/src" >&2
    exit 1
  fi
  if [[ ! -f "$ROS2_WS/install/setup.bash" ]]; then
    echo "[run_ros2_system] Building ROS2 workspace"
    ( cd "$ROS2_WS" && colcon build --symlink-install --packages-select px4_msgs overrack_mission )
  fi
}

source_ros2() {
  echo "[run_ros2_system] source_ros2(): start"

  if declare -F ssdt_source_ros >/dev/null 2>&1; then
    echo "[run_ros2_system] delegating to ssdt_source_ros()"
    ssdt_source_ros
  elif declare -F shelfscout_source_ros >/dev/null 2>&1; then
    echo "[run_ros2_system] delegating to shelfscout_source_ros()"
    shelfscout_source_ros
  else
    local ros_distro="${SSDT_ROS_DISTRO:-${SHELFSCOUT_ROS_DISTRO:-humble}}"
    local had_u=0
    if [[ $- == *u* ]]; then
      had_u=1
      set +u
    fi

    echo "[run_ros2_system] listing /opt/ros/${ros_distro}"
    ls -1 "/opt/ros/${ros_distro}" 2>/dev/null || echo "[run_ros2_system] /opt/ros/${ros_distro} is missing"

    local distro_setup="/opt/ros/${ros_distro}/setup.bash"
    if [[ -f "$distro_setup" ]]; then
      echo "[run_ros2_system] sourcing $distro_setup"
      # shellcheck disable=SC1090
      source "$distro_setup"
    else
      echo "[run_ros2_system] $distro_setup not found"
    fi

    local overlay_setup="$ROS2_WS/install/setup.bash"
    if [[ -f "$overlay_setup" ]]; then
      echo "[run_ros2_system] sourcing $overlay_setup"
      # shellcheck disable=SC1090
      source "$overlay_setup"
    else
      echo "[run_ros2_system] $overlay_setup not found"
    fi

    if [[ $had_u -eq 1 ]]; then
      set -u
    fi
  fi

  command -v ros2 >/dev/null 2>&1 \
    && echo "[run_ros2_system] ros2 found at: $(command -v ros2)" \
    || echo "[run_ros2_system] ros2 still missing after sourcing"

  echo "[run_ros2_system] PATH: $PATH"
}



ensure_colcon_workspace
source_ros2

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[run_ros2_system] ros2 CLI not found after sourcing workspace" >&2
  exit 1
fi

LAUNCH_SCRIPT="$ROOT_DIR/scripts/launch_px4_gazebo.sh"
LAUNCH_MULTI_SCRIPT="$ROOT_DIR/scripts/launch_px4_gazebo_multi.sh"
if [[ ! -x "$LAUNCH_SCRIPT" ]]; then
  echo "[run_ros2_system] Missing launch script: $LAUNCH_SCRIPT" >&2
  exit 1
fi
if (( DRONE_COUNT > 0 )) && [[ ! -x "$LAUNCH_MULTI_SCRIPT" ]]; then
  echo "[run_ros2_system] Missing multi launch script: $LAUNCH_MULTI_SCRIPT" >&2
  exit 1
fi

echo "[run_ros2_system] PX4 world => $WORLD_PATH (from $PARAM_FILE_PATH)"
echo "[run_ros2_system] Mission   => $MISSION_PATH (from $PARAM_FILE_PATH)"
echo "[run_ros2_system] Agent cmd => $AGENT_CMD"
[[ $HEADLESS -eq 1 ]] && echo "[run_ros2_system] Gazebo client disabled (headless)"

PX4_HEADLESS_ARGS=()
[[ $HEADLESS -eq 1 ]] && PX4_HEADLESS_ARGS+=(--headless)

wait_for_gzserver() {
  local timeout=${1:-40}
  local stable=0
  local start
  start=$(date +%s)
  echo "[run_ros2_system] Waiting for gzserver (timeout ${timeout}s)"
  while (( $(date +%s) - start < timeout )); do
    if pgrep -f 'gzserver' >/dev/null 2>&1; then
      stable=$((stable + 1))    # <-- niente ((stable++)) che rompe con -e
      if (( stable >= 3 )); then
        echo "[run_ros2_system] gzserver is up and stable"
        return 0
      fi
    else
      stable=0
    fi
    sleep 1
  done
  echo "[run_ros2_system] gzserver not seen, continuing"
}

wait_for_topic() {
  local topic=$1
  local timeout=${2:-60}
  local deadline=$(( $(date +%s) + timeout ))
  echo "[run_ros2_system] Waiting for topic $topic (timeout ${timeout}s)"
  while (( $(date +%s) <= deadline )); do
    if ros2 topic list 2>/dev/null | grep -F -- "$topic" >/dev/null; then
      echo "[run_ros2_system] Topic $topic detected"
      return 0
    fi
    sleep 1
  done
  echo "[run_ros2_system] Timeout waiting for topic $topic" >&2
  return 1
}

wait_for_px4_ready() {
  local timeout=${1:-120}
  local deadline=$(( $(date +%s) + timeout ))
  echo "[run_ros2_system] Waiting for PX4 readiness (timeout ${timeout}s)"
  while (( $(date +%s) <= deadline )); do
    if [[ -f "$PX4_LOG" ]] && grep -q "Ready for takeoff" "$PX4_LOG" 2>/dev/null; then
      echo "[run_ros2_system] PX4 reports Ready for takeoff"
      return 0
    fi
    sleep 1
  done
  echo "[run_ros2_system] Timeout waiting for PX4 readiness" >&2
  return 1
}

if (( DRONE_COUNT > 0 )); then
  echo "[run_ros2_system] Multi-drone mode: ${DRONE_COUNT} instances"

  # Avvia PX4 + Gazebo multi-istanza con lo script dedicato (un solo gzserver).
  "$LAUNCH_MULTI_SCRIPT" "${PX4_HEADLESS_ARGS[@]}" --world "$WORLD_PATH" --params "$PARAM_FILE_PATH" &
  PX4_WRAPPER_PID=$!

  wait_for_gzserver 40
  sleep 2

  # Avvia un unico agent XRCE condiviso (porta fissa dal comando globale).
  AGENT_LOG="$LOG_DIR/micro_xrce_agent.out"
  : > "$AGENT_LOG"
  echo "[run_ros2_system] Agent command (shared): ${AGENT_CMD_ARRAY[*]}"
  "${AGENT_CMD_ARRAY[@]}" >"$AGENT_LOG" 2>&1 &
  AGENT_PID=$!
  echo "[run_ros2_system] Micro XRCE Agent started (logs -> $AGENT_LOG)"

  MISSION_LOG="$LOG_DIR/mission_runner.out"
  : > "$MISSION_LOG"
  ros2 launch overrack_mission mission.sim.launch.py \
    params_file:="$PARAM_FILE_PATH" mission_file:="$MISSION_PATH" \
    >"$MISSION_LOG" 2>&1 &
  MISSION_PID=$!
  echo "[run_ros2_system] Mission launch started (logs -> $MISSION_LOG)"

  cleanup_multi() {
    if [[ $CLEANED_UP -eq 1 ]]; then return; fi
    CLEANED_UP=1
    echo "[run_ros2_system] Cleaning up (multi)..."
    if [[ -n "${MISSION_PID:-}" ]]; then
      kill "$MISSION_PID" 2>/dev/null || true
      wait "$MISSION_PID" 2>/dev/null || true
    fi
    if [[ -n "${AGENT_PID:-}" ]]; then
      kill "$AGENT_PID" 2>/dev/null || true
      wait "$AGENT_PID" 2>/dev/null || true
    fi
    if [[ -n "${PX4_WRAPPER_PID:-}" ]]; then
      kill -- -"$PX4_WRAPPER_PID" 2>/dev/null || true
      wait -- -"$PX4_WRAPPER_PID" 2>/dev/null || wait "$PX4_WRAPPER_PID" 2>/dev/null || true
    fi
    if pgrep -f gzserver >/dev/null 2>&1 || pgrep -f gzclient >/dev/null 2>&1; then
      echo "[run_ros2_system] Forcing Gazebo shutdown..."
      pkill -9 -f gzserver 2>/dev/null || true
      pkill -9 -f gzclient 2>/dev/null || true
      wait_gz_dead || echo "[run_ros2_system] gzserver/gzclient still alive after cleanup"
    fi
  }

  trap cleanup_multi INT TERM EXIT
  wait "$MISSION_PID"
  cleanup_multi
  exit 0
fi

# -------------------------
# Single-drone legacy path
# -------------------------
: > "$PX4_LOG"
: > "$GAZEBO_LOG"
sleep 5
setsid env PX4_DIR="$PX4_DIR" PX4_SITL_LOG_FILE="$PX4_LOG" \
  "$LAUNCH_SCRIPT" "${PX4_HEADLESS_ARGS[@]}" --world "$WORLD_PATH" \
  >>"$GAZEBO_LOG" 2>&1 &
PX4_WRAPPER_PID=$!

wait_for_gzserver 40
sleep 2   # piccolo margine
echo "[run_ros2_system] PX4/Gazebo wrapper started (logs -> $GAZEBO_LOG)"

wait_for_px4_ready 120

AGENT_LOG="$LOG_DIR/micro_xrce_agent.out"
: > "$AGENT_LOG"
# Start the Micro XRCE Agent with the resolved command
"${AGENT_CMD_ARRAY[@]}" >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!
echo "[run_ros2_system] Micro XRCE Agent started (logs -> $AGENT_LOG)"

# questi 2 wait c’erano e funzionavano
wait_for_topic "/fmu/out/vehicle_status" 60
wait_for_topic "/fmu/out/vehicle_local_position" 60

echo "[run_ros2_system] PX4 RTPS bridge ready; launching mission runner"

MISSION_LOG="$LOG_DIR/mission_runner.out"
: > "$MISSION_LOG"

ros2 launch overrack_mission mission.sim.launch.py \
  params_file:="$PARAM_FILE_PATH" mission_file:="$MISSION_PATH" \
  >"$MISSION_LOG" 2>&1 &
MISSION_PID=$!
echo "[run_ros2_system] Mission launch started (logs -> $MISSION_LOG)"

wait_gz_dead() {
  local tries=15
  while (( tries > 0 )); do
    if ! pgrep -f gzserver >/dev/null 2>&1 && ! pgrep -f gzclient >/dev/null 2>/dev/null; then
      return 0
    fi
    sleep 0.3
    ((tries--))
  done
  return 1
}

cleanup() {
  if [[ $CLEANED_UP -eq 1 ]]; then return; fi
  CLEANED_UP=1
  echo "[run_ros2_system] Cleaning up..."

  if [[ -n "${MISSION_PID:-}" ]]; then
    kill "$MISSION_PID" 2>/dev/null || true
    wait "$MISSION_PID" 2>/dev/null || true
  fi
  if [[ -n "${AGENT_PID:-}" ]]; then
    kill "$AGENT_PID" 2>/dev/null || true
    wait "$AGENT_PID" 2>/dev/null || true
  fi
  if [[ -n "${PX4_WRAPPER_PID:-}" ]]; then
    # Stop the entire PX4/Gazebo process group
    kill -- -"$PX4_WRAPPER_PID" 2>/dev/null || true
    local deadline=$(( $(date +%s) + 10 ))
    while pgrep -g "$PX4_WRAPPER_PID" >/dev/null 2>&1; do
      (( $(date +%s) >= deadline )) && break
      sleep 0.2
    done
    # Try to reap the whole PGID; fall back to the leader if bash rejects the PGID form
    wait -- -"$PX4_WRAPPER_PID" 2>/dev/null || wait "$PX4_WRAPPER_PID" 2>/dev/null || true
  fi

  # Final safety net only if Gazebo bits remain
  if pgrep -f gzserver >/dev/null 2>&1 || pgrep -f gzclient >/dev/null 2>&1; then
    echo "[run_ros2_system] Forcing Gazebo shutdown..."
    pkill -9 -f gzserver 2>/dev/null || true
    pkill -9 -f gzclient 2>/dev/null || true
    wait_gz_dead || echo "[run_ros2_system] gzserver/gzclient still alive after cleanup"
  fi
}

trap cleanup INT TERM EXIT

wait "$MISSION_PID"
cleanup
