#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[run_ros2_system] Missing environment file: $ENV_FILE" >&2
  exit 1
fi

# shellcheck disable=SC1090
set +u
source "$ENV_FILE"
set -u

if [[ -z "${PX4_DIR:-}" ]]; then
  echo "[run_ros2_system] PX4_DIR is not set. Check $ENV_FILE." >&2
  exit 1
fi

ROS2_WS="$ROOT_DIR/ros2_ws"
LOG_DIR="$ROOT_DIR/data/logs"
mkdir -p "$LOG_DIR"

HEADLESS=1
WORLD_OPT=""
MISSION_OPT=""
DEFAULT_AGENT_CMD="MicroXRCEAgent udp4 -p 8888 -v 6"
AGENT_CMD="${AGENT_CMD:-${XRCE_AGENT_CMD:-${MICROXRCE_AGENT_CMD:-${MICRORTPS_AGENT_CMD:-$DEFAULT_AGENT_CMD}}}}"
CLEANED_UP=0
PX4_LOG="$LOG_DIR/px4_sitl_default.out"
GAZEBO_LOG="$LOG_DIR/px4_gazebo.out"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS=1; shift ;;
    --gui)      HEADLESS=0; shift ;;
    --world)    WORLD_OPT=${2:-}; shift 2 ;;
    --mission)  MISSION_OPT=${2:-}; shift 2 ;;
    --agent-cmd) AGENT_CMD=${2:-}; shift 2 ;;
    *) echo "[run_ros2_system] Unknown argument: $1" >&2; exit 1 ;;
  esac
done

read -r -a AGENT_CMD_ARRAY <<< "$AGENT_CMD"
if [[ ${#AGENT_CMD_ARRAY[@]} -eq 0 ]]; then
  echo "[run_ros2_system] Invalid AGENT_CMD: $AGENT_CMD" >&2
  exit 1
fi

WORLD_PATH="$ROOT_DIR/worlds/overrack_indoor.world"
if [[ -n "$WORLD_OPT" ]]; then
  [[ "$WORLD_OPT" = /* ]] && WORLD_PATH="$WORLD_OPT" || WORLD_PATH="$ROOT_DIR/$WORLD_OPT"
fi

MISSION_PATH="$ROOT_DIR/config/mission.yaml"
if [[ -n "$MISSION_OPT" ]]; then
  [[ "$MISSION_OPT" = /* ]] && MISSION_PATH="$MISSION_OPT" || MISSION_PATH="$ROOT_DIR/$MISSION_OPT"
fi

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
  echo "[run_ros2_system] source_ros2(): entro"

  local had_u=0
  # se lo script è in set -u lo spengo un attimo
  if [[ $- == *u* ]]; then
    had_u=1
    set +u
  fi

  # voglio vedere cosa c'è
  echo "[run_ros2_system] ls /opt/ros/humble:"
  ls -1 /opt/ros/humble 2>/dev/null || echo "[run_ros2_system] /opt/ros/humble NON esiste"

  # 1) ROS2 di sistema
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    echo "[run_ros2_system] sourcing /opt/ros/humble/setup.bash"
    # shellcheck disable=SC1090
    source /opt/ros/humble/setup.bash
  else
    echo "[run_ros2_system] /opt/ros/humble/setup.bash NON trovato"
  fi

  # 2) overlay del progetto
  if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
    echo "[run_ros2_system] sourcing $ROS2_WS/install/setup.bash"
    # shellcheck disable=SC1090
    source "$ROS2_WS/install/setup.bash"
  else
    echo "[run_ros2_system] $ROS2_WS/install/setup.bash NON trovato"
  fi

  # riaccendo -u se c’era
  if [[ $had_u -eq 1 ]]; then
    set -u
  fi

  # ora vediamo chi è ros2
  command -v ros2 >/dev/null 2>&1 \
    && echo "[run_ros2_system] ros2 trovato in: $(command -v ros2)" \
    || echo "[run_ros2_system] ros2 ANCORA non trovato dopo i source"

  echo "[run_ros2_system] PATH: $PATH"
}



ensure_colcon_workspace
source_ros2

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[run_ros2_system] ros2 CLI not found after sourcing workspace" >&2
  exit 1
fi

LAUNCH_SCRIPT="$ROOT_DIR/scripts/launch_px4_gazebo.sh"
if [[ ! -x "$LAUNCH_SCRIPT" ]]; then
  echo "[run_ros2_system] Missing launch script: $LAUNCH_SCRIPT" >&2
  exit 1
fi

echo "[run_ros2_system] PX4 world => $WORLD_PATH"
echo "[run_ros2_system] Mission   => $MISSION_PATH"
echo "[run_ros2_system] Agent cmd => $AGENT_CMD"
[[ $HEADLESS -eq 1 ]] && echo "[run_ros2_system] Gazebo client disabled (headless)"

PX4_HEADLESS_ARGS=()
[[ $HEADLESS -eq 1 ]] && PX4_HEADLESS_ARGS+=(--headless)

: > "$PX4_LOG"
: > "$GAZEBO_LOG"
sleep 5 # 
setsid env PX4_DIR="$PX4_DIR" PX4_SITL_LOG_FILE="$PX4_LOG" \
  "$LAUNCH_SCRIPT" "${PX4_HEADLESS_ARGS[@]}" --world "$WORLD_PATH" \
  >>"$GAZEBO_LOG" 2>&1 &
PX4_WRAPPER_PID=$!

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

wait_for_gzserver 40
sleep 2   # piccolo margine
echo "[run_ros2_system] PX4/Gazebo wrapper started (logs -> $GAZEBO_LOG)"


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
ros2 run overrack_mission mission_runner --ros-args -p mission_file:="$MISSION_PATH" \
  >"$MISSION_LOG" 2>&1 &
MISSION_PID=$!
echo "[run_ros2_system] Mission runner started (logs -> $MISSION_LOG)"

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
