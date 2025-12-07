#!/usr/bin/env bash
# -----------------------------------------------------------------------------
# launch_px4_gazebo_multi.sh
# Thin orchestrator to start multiple PX4 SITL instances in Gazebo Classic.
#
# Assumptions:
#   - gzserver/gzclient are managed by PX4 make targets (same as single-drone script).
#   - drones configuration is read from the params YAML (run_ros2_system.ros__parameters.drones_yaml).
#
# Usage (normally internal from run_ros2_system.sh):
#   scripts/launch_px4_gazebo_multi.sh --params config/sim/multi.yaml [--world <world>] [--headless]
# -----------------------------------------------------------------------------
# -----------------------------------------------------------------------------

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
# Env is assumed to be loaded by the caller (run_ros2_system.sh)
if [[ -z "${PX4_DIR:-}" ]]; then
  echo "[launch_px4_gazebo_multi] PX4_DIR is not set. Source scripts/.env first." >&2
  exit 1
fi
resolve_with_root() {
  local candidate="$1"
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

# Shared Python utilities to parse YAML defaults/drones.
PARAM_UTILS_PYTHONPATH="$ROOT_DIR/ros2_ws/src:${PYTHONPATH:-}"
param_utils() {
  PYTHONPATH="$PARAM_UTILS_PYTHONPATH" python3 -m overrack_mission.param_utils "$@"
}

# -----------------------------------------------------------------------------
# CLI parsing
# -----------------------------------------------------------------------------

HEADLESS=0
WORLD_PATH=""
PARAM_FILE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --params)
      PARAM_FILE="${2:-}"
      shift 2
      ;;
    --headless)
      HEADLESS=1
      shift
      ;;
    --world)
      WORLD_PATH="${2:-$WORLD_PATH}"
      shift 2
      ;;
    *)
      echo "[launch_px4_gazebo_multi] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

# Load drones from params file
eval "$(param_utils --file "$PARAM_FILE" defaults --format shell)" || true
eval "$(param_utils --file "$PARAM_FILE" drones --format shell)" || true

if [[ -z "${DRONE_COUNT:-}" || ! "${DRONE_COUNT}" =~ ^[0-9]+$ || "${DRONE_COUNT}" -le 0 ]]; then
  echo "[launch_px4_gazebo_multi] DRONE_COUNT missing or invalid; check --params file" >&2
  exit 1
fi

if [[ -z "$WORLD_PATH" ]]; then
  if [[ -n "${YAML_WORLD_DEFAULT:-}" ]]; then
    WORLD_PATH="$(resolve_with_root "$YAML_WORLD_DEFAULT")"
  else
    WORLD_PATH="$ROOT_DIR/worlds/overrack_indoor.world"
  fi
fi
if [[ ! -f "$WORLD_PATH" ]]; then
  echo "[launch_px4_gazebo_multi] World not found: $WORLD_PATH" >&2
  exit 1
fi

echo "[launch_px4_gazebo_multi] Spawning ${DRONE_COUNT} PX4 + Gazebo instances"
echo "[launch_px4_gazebo_multi] World = $WORLD_PATH"
export PX4_SITL_WORLD="$WORLD_PATH"
export PX4_GAZEBO_WORLD="$WORLD_PATH"
export PX4_GZ_WORLD="$WORLD_PATH"

# -----------------------------------------------------------------------------
# MODEL PATHS
# -----------------------------------------------------------------------------

PX4_MODELS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
PX4_WORLDS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/worlds"
CUSTOM_MODELS="$ROOT_DIR/models"
if [[ -d "$CUSTOM_MODELS" ]]; then
  export GAZEBO_MODEL_PATH="$CUSTOM_MODELS:$PX4_MODELS:${GAZEBO_MODEL_PATH:-}"
else
  export GAZEBO_MODEL_PATH="$PX4_MODELS:${GAZEBO_MODEL_PATH:-}"
fi
# Ensure ROS2 gazebo plugins are reachable (needed by worlds using gazebo_ros_* plugins)
if [[ -n "${ROS_DISTRO:-}" && -d "/opt/ros/${ROS_DISTRO}/lib" ]]; then
  export GAZEBO_PLUGIN_PATH="/opt/ros/${ROS_DISTRO}/lib:${GAZEBO_PLUGIN_PATH:-}"
fi

echo "[launch_px4_gazebo_multi] GAZEBO_MODEL_PATH = $GAZEBO_MODEL_PATH"

# Ensure world is discoverable by sitl_multiple_run.sh (expects name under PX4 worlds)
WORLD_BASENAME="$(basename "$WORLD_PATH")"
WORLD_NAME="${WORLD_BASENAME%.world}"
if [[ ! -f "$PX4_WORLDS/$WORLD_NAME.world" ]]; then
  ln -sf "$WORLD_PATH" "$PX4_WORLDS/$WORLD_NAME.world"
fi

# -----------------------------------------------------------------------------
# INSTANCE LOOP
# -----------------------------------------------------------------------------

SCRIPT_ENTRIES=()
SUPPORTED_MODELS=("iris" "plane" "standard_vtol" "rover" "r1_rover" "typhoon_h480" "iris_opt_flow")

for idx in $(seq 0 $((DRONE_COUNT - 1))); do
  NS="${DRONE_NS[$idx]}"
  MODEL="${DRONE_MODEL[$idx]:-iris_opt_flow}"
  TARGET_MODEL="$MODEL"
  if [[ ! " ${SUPPORTED_MODELS[*]} " =~ " ${MODEL} " ]]; then
    echo "[launch_px4_gazebo_multi] Model $MODEL not supported by sitl_multiple_run.sh; falling back to iris"
    TARGET_MODEL="iris"
  fi
  SPAWN_X="${DRONE_SPAWN_X[$idx]:-0}"
  SPAWN_Y="${DRONE_SPAWN_Y[$idx]:-0}"
  SPAWN_Z="${DRONE_SPAWN_Z[$idx]:-0}"
  SPAWN_YAW="${DRONE_SPAWN_YAW[$idx]:-0}"
  SCRIPT_ENTRIES+=("${TARGET_MODEL}:${idx+1}:${SPAWN_X}:${SPAWN_Y}:${SPAWN_Z}:${SPAWN_YAW}")
  echo "[launch_px4_gazebo_multi] Drone $idx ns=$NS model=$MODEL (runner=${TARGET_MODEL}) spawn=($SPAWN_X,$SPAWN_Y,$SPAWN_Z,$SPAWN_YAW)"
done

SCRIPT_ARG=$(IFS=,; echo "${SCRIPT_ENTRIES[*]}")

echo "[launch_px4_gazebo_multi] Using sitl_multiple_run.sh script=$SCRIPT_ARG world=$WORLD_NAME"

(
  cd "$PX4_DIR"
  HEADLESS_ENV=()
  [[ $HEADLESS -eq 1 ]] && HEADLESS_ENV+=( "NO_GZCLIENT=1" )
  env "${HEADLESS_ENV[@]}" \
    "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_multiple_run.sh" \
      -s "$SCRIPT_ARG" -w "$WORLD_NAME" -t px4_sitl_default
)
