#!/usr/bin/env bash
# Orchestrate PX4 SITL + Gazebo + Micro XRCE Agent + ROS2 mission nodes.
#
# Usage:
#   PX4_DIR=/path/to/PX4-Autopilot ./scripts/run_ros2_system.sh \
#     [--headless|--gui] [--world <path>] [--mission <path>] [--agent-cmd "MicroXRCEAgent udp4 -p 8888 -v 6"]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[run_ros2_system] Missing environment file: $ENV_FILE" >&2
  exit 1
fi
ENV_NOUNSET=0
if [[ $- == *u* ]]; then
  set +u
  ENV_NOUNSET=1
fi
# shellcheck disable=SC1090
source "$ENV_FILE"
if [[ $ENV_NOUNSET -eq 1 ]]; then
  set -u
fi

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
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1; shift ;;
    --gui)
      HEADLESS=0; shift ;;
    --world)
      WORLD_OPT=${2:-}; shift 2 ;;
    --mission)
      MISSION_OPT=${2:-}; shift 2 ;;
    --agent-cmd)
      AGENT_CMD=${2:-}; shift 2 ;;
    *)
      echo "[run_ros2_system] Unknown argument: $1" >&2
      exit 1 ;;
  esac
done

if [[ ! -d "$PX4_DIR" ]]; then
  echo "[run_ros2_system] PX4_DIR does not exist: $PX4_DIR" >&2
  exit 1
fi

WORLD_PATH="$ROOT_DIR/worlds/overrack_indoor.world"
if [[ -n "$WORLD_OPT" ]]; then
  if [[ "$WORLD_OPT" = /* ]]; then
    WORLD_PATH="$WORLD_OPT"
  else
    WORLD_PATH="$ROOT_DIR/$WORLD_OPT"
  fi
fi
if [[ ! -f "$WORLD_PATH" ]]; then
  echo "[run_ros2_system] World file not found: $WORLD_PATH" >&2
  exit 1
fi

MISSION_PATH="$ROOT_DIR/config/mission.yaml"
if [[ -n "$MISSION_OPT" ]]; then
  if [[ "$MISSION_OPT" = /* ]]; then
    MISSION_PATH="$MISSION_OPT"
  else
    MISSION_PATH="$ROOT_DIR/$MISSION_OPT"
  fi
fi
if [[ ! -f "$MISSION_PATH" ]]; then
  echo "[run_ros2_system] Mission file not found: $MISSION_PATH" >&2
  exit 1
fi

if [[ -z "$AGENT_CMD" ]]; then
  AGENT_CMD="$DEFAULT_AGENT_CMD"
fi
read -r -a AGENT_ARGS <<< "$AGENT_CMD"
if [[ ${#AGENT_ARGS[@]} -eq 0 ]]; then
  echo "[run_ros2_system] Invalid agent command" >&2
  exit 1
fi
if ! command -v "${AGENT_ARGS[0]}" >/dev/null 2>&1 && [[ ! -x "${AGENT_ARGS[0]}" ]]; then
  echo "[run_ros2_system] MicroXRCEAgent executable not found: ${AGENT_ARGS[0]}" >&2
  echo "[run_ros2_system] Ensure scripts/.env exports MICRO_XRCE_AGENT_DIR and PATH_MICRO_XRCE_AGENT." >&2
  exit 1
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
  local needs_build=0
  if [[ ! -f "$ROS2_WS/install/setup.bash" ]]; then
    needs_build=1
  elif [[ ! -d "$ROS2_WS/install/overrack_mission" ]] || [[ ! -d "$ROS2_WS/install/px4_msgs" ]]; then
    needs_build=1
  fi
  if [[ $needs_build -eq 1 ]]; then
    echo "[run_ros2_system] Building ROS2 workspace"
    ( cd "$ROS2_WS" && colcon build --symlink-install --packages-select px4_msgs overrack_mission )
  fi
}

source_ros2() {
  local setup_bash="$ROS2_WS/install/setup.bash"
  if [[ ! -f "$setup_bash" ]]; then
    echo "[run_ros2_system] Missing $setup_bash" >&2
    exit 1
  fi
  local nounset=0
  if [[ $- == *u* ]]; then
    set +u
    nounset=1
  fi
  # shellcheck disable=SC1090
  source "$setup_bash"
  if [[ $nounset -eq 1 ]]; then
    set -u
  fi
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
echo "[run_ros2_system] Mission => $MISSION_PATH"
echo "[run_ros2_system] Agent cmd => $AGENT_CMD"
if [[ $HEADLESS -eq 1 ]]; then
  echo "[run_ros2_system] Gazebo client disabled (headless)"
fi

PX4_HEADLESS_ARGS=()
if [[ $HEADLESS -eq 1 ]]; then
  PX4_HEADLESS_ARGS+=(--headless)
fi

: > "$PX4_LOG"
PX4_DIR="$PX4_DIR" "$LAUNCH_SCRIPT" "${PX4_HEADLESS_ARGS[@]}" --world "$WORLD_PATH" &
PX4_WRAPPER_PID=$!

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
"${AGENT_ARGS[@]}" >"$AGENT_LOG" 2>&1 &
AGENT_PID=$!
echo "[run_ros2_system] Micro XRCE Agent started (logs -> $AGENT_LOG)"

wait_for_topic "/fmu/out/vehicle_status" 60
wait_for_topic "/fmu/out/vehicle_local_position" 60

echo "[run_ros2_system] PX4 RTPS bridge ready; launching mission runner"

MISSION_LOG="$LOG_DIR/mission_runner.out"
: > "$MISSION_LOG"
ros2 run overrack_mission mission_runner --ros-args -p mission_file:="$MISSION_PATH" \
  >"$MISSION_LOG" 2>&1 &
MISSION_PID=$!
echo "[run_ros2_system] Mission runner started (logs -> $MISSION_LOG)"

cleanup() {
  if [[ $CLEANED_UP -eq 1 ]]; then
    return
  fi
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
    kill "$PX4_WRAPPER_PID" 2>/dev/null || true
    wait "$PX4_WRAPPER_PID" 2>/dev/null || true
  fi
  pkill -f MicroXRCEAgent 2>/dev/null || true
  pkill -f px4 2>/dev/null || true
  pkill -f gzserver 2>/dev/null || true
  pkill -f gzclient 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait "$MISSION_PID"
cleanup
