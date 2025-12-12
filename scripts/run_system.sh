#!/usr/bin/env bash
# Multi-path-only launcher for PX4 + Gazebo + XRCE Agent + ROS 2 nodes.
# Uses the multi-drone launch flow even for a single drone to avoid legacy codepaths.

set -euo pipefail

# ---------------------------
# Paths / env bootstrap
# ---------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"

log()  { echo "[run_system] $*"; }
warn() { echo "[run_system] WARNING: $*" >&2; }
err()  { echo "[run_system] ERROR: $*" >&2; }

die() {
  err "$*"
  exit 1
}

require_cmd() {
  command -v "$1" >/dev/null 2>&1 || die "Missing required command: $1"
}

# Load .env (allow undefined vars during source)
[[ -f "$ENV_FILE" ]] || die "Missing environment file: $ENV_FILE"
# shellcheck disable=SC1090
set +u
source "$ENV_FILE"
set -u

ROOT_DIR="${SSDT_ROOT:-${SHELFSCOUT_ROOT:-$ROOT_DIR}}"
PX4_DIR="${SSDT_PX4_DIR:-${SHELFSCOUT_PX4_DIR:-${PX4_DIR:-}}}"
[[ -n "${PX4_DIR:-}" ]] || die "PX4_DIR is not set. Check $ENV_FILE."

resolve_with_root() {
  local candidate="${1:-}"
  if declare -F ssdt_resolve_path >/dev/null 2>&1; then
    ssdt_resolve_path "$candidate"; return
  elif declare -F shelfscout_resolve_path >/dev/null 2>&1; then
    shelfscout_resolve_path "$candidate"; return
  fi

  if [[ -z "$candidate" ]]; then
    printf '%s\n' "$ROOT_DIR"; return
  fi
  if [[ "$candidate" = /* ]]; then
    printf '%s\n' "$candidate"
  else
    printf '%s/%s\n' "$ROOT_DIR" "$candidate"
  fi
}

# param_utils wrapper
PARAM_UTILS_PYTHONPATH="$ROOT_DIR/ros2_ws/src:${PYTHONPATH:-}"
param_utils() {
  PYTHONPATH="$PARAM_UTILS_PYTHONPATH" python3 -m overrack_mission.param_utils "$@"
}

# ---------------------------
# Defaults (overridable)
# ---------------------------
HEADLESS="${SSDT_HEADLESS_DEFAULT:-${SHELFSCOUT_HEADLESS_DEFAULT:-1}}"

DEFAULT_AGENT_CMD="${SSDT_AGENT_CMD:-${SHELFSCOUT_AGENT_CMD:-MicroXRCEAgent udp4 -p 8888 -v 6}}"
AGENT_CMD_ENV_OVERRIDE="${AGENT_CMD:-${XRCE_AGENT_CMD:-${MICROXRCE_AGENT_CMD:-${MICRORTPS_AGENT_CMD:-}}}}"

DEFAULT_WORLD="${SSDT_WORLD:-${SHELFSCOUT_WORLD:-worlds/overrack_indoor.world}}"
DEFAULT_MISSION_FILE="${SSDT_MISSION_FILE:-${SHELFSCOUT_MISSION_FILE:-config/mission_precomputed.yaml}}"

PRIMARY_PARAM_FILE="config/sim/multi_1drone.yaml"
SECONDARY_PARAM_FILE="config/sim/multi.yaml"
PARAM_FILE_PATH="${SSDT_PARAM_PATH:-${SHELFSCOUT_PARAM_PATH:-$(resolve_with_root "$PRIMARY_PARAM_FILE")}}"

ROS2_WS="$(resolve_with_root "${SSDT_ROS_WS:-${SHELFSCOUT_ROS_WS:-$ROOT_DIR/ros2_ws}}")"
LOG_DIR="$(resolve_with_root "${SSDT_LOG_DIR:-${SHELFSCOUT_LOG_DIR:-$ROOT_DIR/data/logs}}")"
mkdir -p "$LOG_DIR"

# Timeouts / gating behavior (configurabili)
GZSERVER_TIMEOUT="${SSDT_GZSERVER_TIMEOUT:-40}"
TOPIC_TIMEOUT="${SSDT_TOPIC_TIMEOUT:-60}"
REQUIRE_PX4_TOPICS="${SSDT_REQUIRE_PX4_TOPICS:-1}"  # 1 = fail-fast, 0 = best-effort
REQUIRE_GZSERVER="${SSDT_REQUIRE_GZSERVER:-1}"      # 1 = fail-fast, 0 = best-effort
GZ_GRACE="${SSDT_GZ_GRACE:-8}"                      # seconds to wait after SIGINT before SIGKILL

# Topics check (substring match OK for /px4_n/...)
PX4_REQUIRED_TOPICS=(
  "/fmu/out/vehicle_status"
  "/fmu/out/vehicle_local_position"
)

# ---------------------------
# CLI args
# ---------------------------
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless) HEADLESS=1; shift ;;
    --gui)      HEADLESS=0; shift ;;
    --params)
      PARAM_FILE_PATH="$(resolve_with_root "${2:-}")"
      shift 2
      ;;
    *)
      die "Unknown argument: $1 (supported: --gui, --headless, --params <file>)"
      ;;
  esac
done

# Fallback param file if primary missing
PRIMARY_PARAM_PATH="$(resolve_with_root "$PRIMARY_PARAM_FILE")"
SECONDARY_PARAM_PATH="$(resolve_with_root "$SECONDARY_PARAM_FILE")"
if [[ ! -f "$PARAM_FILE_PATH" ]] && [[ "$PARAM_FILE_PATH" == "$PRIMARY_PARAM_PATH" ]] && [[ -f "$SECONDARY_PARAM_PATH" ]]; then
  log "Missing $PRIMARY_PARAM_PATH; falling back to $SECONDARY_PARAM_PATH"
  PARAM_FILE_PATH="$SECONDARY_PARAM_PATH"
fi
[[ -f "$PARAM_FILE_PATH" ]] || die "Parameter file not found: $PARAM_FILE_PATH"

require_cmd python3

# Read YAML defaults/drones (drones may fail in some edge cases; keep tolerant but validate DRONE_COUNT)
if ! eval "$(param_utils --file "$PARAM_FILE_PATH" defaults --format shell)"; then
  die "Unable to read defaults from $PARAM_FILE_PATH"
fi
eval "$(param_utils --file "$PARAM_FILE_PATH" drones --format shell)" || true

if [[ -z "${DRONE_COUNT:-}" || ! "${DRONE_COUNT}" =~ ^[0-9]+$ || "${DRONE_COUNT}" -le 0 ]]; then
  die "No drones declared in params. Add a 'drones' list (see config/sim/multi_1drone.yaml)."
fi

WORLD_PATH="$(resolve_with_root "${YAML_WORLD_DEFAULT:-$DEFAULT_WORLD}")"
MISSION_PATH="$(resolve_with_root "${YAML_MISSION_DEFAULT:-$DEFAULT_MISSION_FILE}")"

# Agent cmd precedence: YAML > env override > default
if [[ -n "${YAML_AGENT_DEFAULT:-}" ]]; then
  AGENT_CMD="$YAML_AGENT_DEFAULT"
elif [[ -n "${AGENT_CMD_ENV_OVERRIDE:-}" ]]; then
  AGENT_CMD="$AGENT_CMD_ENV_OVERRIDE"
else
  AGENT_CMD="$DEFAULT_AGENT_CMD"
fi

read -r -a AGENT_CMD_ARRAY <<< "$AGENT_CMD"
(( ${#AGENT_CMD_ARRAY[@]} > 0 )) || die "Invalid AGENT_CMD: $AGENT_CMD"

# ---------------------------
# ROS2 workspace bootstrap
# ---------------------------
ensure_colcon_workspace() {
  require_cmd colcon
  [[ -d "$ROS2_WS/src" ]] || die "Missing ROS2 workspace: $ROS2_WS/src"

  if [[ ! -f "$ROS2_WS/install/setup.bash" ]]; then
    log "Building ROS2 workspace"
    ( cd "$ROS2_WS" && colcon build --symlink-install --packages-select px4_msgs overrack_mission )
  fi
}

source_ros2() {
  log "source_ros2(): start"

  if declare -F ssdt_source_ros >/dev/null 2>&1; then
    log "delegating to ssdt_source_ros()"
    ssdt_source_ros
    return
  fi
  if declare -F shelfscout_source_ros >/dev/null 2>&1; then
    log "delegating to shelfscout_source_ros()"
    shelfscout_source_ros
    return
  fi

  local ros_distro="${SSDT_ROS_DISTRO:-${SHELFSCOUT_ROS_DISTRO:-humble}}"
  local had_u=0
  if [[ $- == *u* ]]; then
    had_u=1
    set +u
  fi

  local distro_setup="/opt/ros/${ros_distro}/setup.bash"
  if [[ -f "$distro_setup" ]]; then
    log "sourcing $distro_setup"
    # shellcheck disable=SC1090
    source "$distro_setup"
  else
    warn "$distro_setup not found"
  fi

  if [[ -f "$ROS2_WS/install/setup.bash" ]]; then
    # shellcheck disable=SC1091
    source "$ROS2_WS/install/setup.bash"
  else
    warn "ROS2 overlay missing: $ROS2_WS/install/setup.bash"
  fi

  (( had_u == 1 )) && set -u
  log "source_ros2(): done"
}

# ---------------------------
# Wait helpers
# ---------------------------
wait_for_process_pattern() {
  local pattern="$1"
  local timeout="${2:-30}"
  local stable_needed="${3:-3}"

  local stable=0
  local start
  start="$(date +%s)"

  log "Waiting for process pattern '$pattern' (timeout ${timeout}s)"
  while (( $(date +%s) - start < timeout )); do
    if pgrep -f -- "$pattern" >/dev/null 2>&1; then
      stable=$((stable + 1))
      if (( stable >= stable_needed )); then
        log "Process '$pattern' is up and stable"
        return 0
      fi
    else
      stable=0
    fi
    sleep 1
  done

  err "Timeout waiting for process '$pattern'"
  return 1
}

wait_for_topic() {
  local topic="$1"
  local timeout="${2:-60}"
  local deadline=$(( $(date +%s) + timeout ))

  log "Waiting for topic substring '$topic' (timeout ${timeout}s)"
  while (( $(date +%s) <= deadline )); do
    # ros2 topic list can be slow early on; keep quiet
    if ros2 topic list 2>/dev/null | grep -F -- "$topic" >/dev/null; then
      log "Topic detected: $topic"
      return 0
    fi
    sleep 1
  done

  err "Timeout waiting for topic: $topic"
  return 1
}

tail_log_on_failure() {
  local title="$1"
  local file="$2"
  local n="${3:-80}"

  echo "----- [run_system] ${title} (last ${n} lines): ${file} -----" >&2
  if [[ -f "$file" ]]; then
    tail -n "$n" "$file" >&2 || true
  else
    echo "(missing log file)" >&2
  fi
  echo "----------------------------------------------------------" >&2
}

# ---------------------------
# Main
# ---------------------------
ensure_colcon_workspace
source_ros2
require_cmd ros2

LAUNCH_MULTI_SCRIPT="$ROOT_DIR/scripts/launch_px4_gazebo_multi.sh"
[[ -x "$LAUNCH_MULTI_SCRIPT" ]] || die "Missing multi launch script: $LAUNCH_MULTI_SCRIPT"

log "PX4 world => $WORLD_PATH (from $PARAM_FILE_PATH)"
log "Mission   => $MISSION_PATH (from $PARAM_FILE_PATH)"
log "Agent cmd => $AGENT_CMD"
(( HEADLESS == 1 )) && log "Gazebo client disabled (headless)"

PX4_HEADLESS_ARGS=()
(( HEADLESS == 1 )) && PX4_HEADLESS_ARGS+=(--headless)

GAZEBO_LOG="$LOG_DIR/px4_gazebo.out"
AGENT_LOG="$LOG_DIR/micro_xrce_agent.out"
MISSION_LOG="$LOG_DIR/mission_runner.out"
: > "$GAZEBO_LOG"
: > "$AGENT_LOG"
: > "$MISSION_LOG"

PX4_WRAPPER_PID=""
AGENT_PID=""
MISSION_PID=""

CLEANUP_DONE=0
cleanup() {
  (( CLEANUP_DONE )) && return
  CLEANUP_DONE=1

  log "Cleanup start"

  if [[ -n "${MISSION_PID:-}" ]]; then
    kill "$MISSION_PID" 2>/dev/null || true
    wait "$MISSION_PID" 2>/dev/null || true
  fi

  if [[ -n "${AGENT_PID:-}" ]]; then
    kill "$AGENT_PID" 2>/dev/null || true
    wait "$AGENT_PID" 2>/dev/null || true
  fi

  if [[ -n "${PX4_WRAPPER_PID:-}" ]]; then
    # setsid creates a new session where pid == pgid, so kill -PID targets the group
    kill -- -"$PX4_WRAPPER_PID" 2>/dev/null || true
    wait -- -"$PX4_WRAPPER_PID" 2>/dev/null || wait "$PX4_WRAPPER_PID" 2>/dev/null || true
  fi

  # Gazebo is spawned by PX4 in a separate session (not the wrapper PGID), handle explicitly.
  if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
    log "Stopping Gazebo (external PX4 session)..."
    pkill -INT -x gzserver 2>/dev/null || true
    pkill -INT -x gzclient 2>/dev/null || true

    local waited=0
    while (( waited < GZ_GRACE )); do
      if ! pgrep -x gzserver >/dev/null 2>&1 && ! pgrep -x gzclient >/dev/null 2>&1; then
        break
      fi
      sleep 1
      ((waited++))
    done

    if pgrep -x gzserver >/dev/null 2>&1 || pgrep -x gzclient >/dev/null 2>&1; then
      warn "Gazebo did not exit gracefully; forcing shutdown"
      pkill -KILL -x gzserver 2>/dev/null || true
      pkill -KILL -x gzclient 2>/dev/null || true
    fi
  fi

  log "Cleanup done"
}
trap cleanup INT TERM EXIT

# 1) Start PX4+Gazebo wrapper (in its own session)
log "Starting PX4/Gazebo multi wrapper (logs -> $GAZEBO_LOG)"
setsid env PX4_DIR="$PX4_DIR" \
  "$LAUNCH_MULTI_SCRIPT" "${PX4_HEADLESS_ARGS[@]}" --world "$WORLD_PATH" --params "$PARAM_FILE_PATH" \
  >>"$GAZEBO_LOG" 2>&1 &
PX4_WRAPPER_PID="$!"
log "PX4/Gazebo wrapper PID: $PX4_WRAPPER_PID"

# 2) Wait for gzserver (blocking unless REQUIRE_GZSERVER=0)
if (( REQUIRE_GZSERVER )); then
  if ! wait_for_process_pattern "gzserver" "$GZSERVER_TIMEOUT" 3; then
    tail_log_on_failure "PX4/Gazebo wrapper log" "$GAZEBO_LOG" 120
    die "Gazebo did not start; aborting."
  fi
else
  wait_for_process_pattern "gzserver" "$GZSERVER_TIMEOUT" 3 || warn "gzserver not detected (continuing due to REQUIRE_GZSERVER=0)"
fi

sleep 1

# 3) Start XRCE agent
log "Starting Micro XRCE Agent (logs -> $AGENT_LOG)"
log "Agent command: ${AGENT_CMD_ARRAY[*]}"
"${AGENT_CMD_ARRAY[@]}" >>"$AGENT_LOG" 2>&1 &
AGENT_PID="$!"
log "Micro XRCE Agent PID: $AGENT_PID"

# Sanity: ensure agent didn't exit immediately
sleep 0.2
if ! kill -0 "$AGENT_PID" 2>/dev/null; then
  tail_log_on_failure "Micro XRCE Agent log" "$AGENT_LOG" 120
  die "Micro XRCE Agent exited immediately."
fi

# 4) Wait for PX4 topics (blocking unless REQUIRE_PX4_TOPICS=0)
for t in "${PX4_REQUIRED_TOPICS[@]}"; do
  if (( REQUIRE_PX4_TOPICS )); then
    if ! wait_for_topic "$t" "$TOPIC_TIMEOUT"; then
      tail_log_on_failure "PX4/Gazebo wrapper log" "$GAZEBO_LOG" 120
      tail_log_on_failure "Micro XRCE Agent log" "$AGENT_LOG" 120
      die "PX4/XRCE topics not ready; aborting."
    fi
  else
    wait_for_topic "$t" "$TOPIC_TIMEOUT" || warn "Missing topic '$t' (continuing due to REQUIRE_PX4_TOPICS=0)"
  fi
done

log "PX4 RTPS bridge ready; launching mission runner(s)"

# 5) Launch mission
ros2 launch overrack_mission mission.sim.launch.py \
  params_file:="$PARAM_FILE_PATH" mission_file:="$MISSION_PATH" \
  >>"$MISSION_LOG" 2>&1 &
MISSION_PID="$!"
log "Mission launch PID: $MISSION_PID (logs -> $MISSION_LOG)"

# Wait mission, propagate exit code
set +e
wait "$MISSION_PID"
MISSION_RC="$?"
set -e

if (( MISSION_RC != 0 )); then
  tail_log_on_failure "Mission runner log" "$MISSION_LOG" 200
  warn "Mission exited with code $MISSION_RC"
fi

exit "$MISSION_RC"
