#!/usr/bin/env bash
# Launch PX4 SITL together with Gazebo Classic.
#
# Usage:
#   PX4_DIR=/path/to/PX4-Autopilot scripts/launch_px4_gazebo.sh [--headless] [--world path/to/world.sdf]
#
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[launch_px4_gazebo] Missing environment file: $ENV_FILE" >&2
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
  echo "[launch_px4_gazebo] PX4_DIR is not set. Check $ENV_FILE." >&2
  exit 1
fi

HEADLESS=0
WORLD_OPT=""
INSTANCE_ID=""
MODEL_NAME_OVERRIDE=""
MODEL_POSE=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1
      shift
      ;;
    --world)
      WORLD_OPT=${2:-}
      shift 2
      ;;
    --id|--instance)
      INSTANCE_ID=${2:-}
      shift 2
      ;;
    --model-name)
      MODEL_NAME_OVERRIDE=${2:-}
      shift 2
      ;;
    --model-pose)
      MODEL_POSE=${2:-}
      shift 2
      ;;
    *)
      echo "[launch_px4_gazebo] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ ! -d "$PX4_DIR" ]]; then
  echo "[launch_px4_gazebo] PX4_DIR does not exist: $PX4_DIR" >&2
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
  echo "[launch_px4_gazebo] World file not found: $WORLD_PATH" >&2
  exit 1
fi

export PX4_SITL_WORLD="$WORLD_PATH"
export PX4_GZ_WORLD="$WORLD_PATH"
export PX4_GAZEBO_WORLD="$WORLD_PATH"

BUILD_TARGET="px4_sitl_default"
BUILD_DIR="$PX4_DIR/build/${BUILD_TARGET}"
mkdir -p "$ROOT_DIR/data/logs"
LOG_FILE="${PX4_SITL_LOG_FILE:-$ROOT_DIR/data/logs/${BUILD_TARGET}.out}"
if [[ -n "$INSTANCE_ID" && -z "${PX4_SITL_LOG_FILE:-}" ]]; then
  LOG_FILE="$ROOT_DIR/data/logs/${BUILD_TARGET}_${INSTANCE_ID}.out"
fi
mkdir -p "$(dirname "$LOG_FILE")"
: > "$LOG_FILE"

# If a user param file exists, expose it to PX4 via px4-rc.params
PX4_PARAMS_CANDIDATE="$ROOT_DIR/config/px4_sitl.params"
# if [[ -f "$PX4_PARAMS_CANDIDATE" ]]; then # NON Attivare da problemi!
  #export PX4_RC_PARAMS_FILE="$PX4_PARAMS_CANDIDATE"
  # Ensure px4-rc.params (loader) is discoverable by rcS PATH lookup
  #export PATH="$ROOT_DIR/scripts:${PATH}"
# fi

if [[ ! -x "$BUILD_DIR/bin/px4" ]]; then
  echo "[launch_px4_gazebo] Building ${BUILD_TARGET} target (first run)"
  ( cd "$PX4_DIR" && make "${BUILD_TARGET}" )
fi

echo "[launch_px4_gazebo] Starting PX4 SITL (${BUILD_TARGET}) with world: $WORLD_PATH"
echo "[launch_px4_gazebo] Logs -> $LOG_FILE"
echo "[launch_px4_gazebo] Exporting PX4_SITL_WORLD=$WORLD_PATH"
if [[ -n "${PX4_RC_PARAMS_FILE:-}" ]]; then
  echo "[launch_px4_gazebo] PX4 params -> $PX4_RC_PARAMS_FILE"
fi

# 1) Dai precedenza ai modelli custom (consente override controllati come la camera frontale)
PX4_MODELS="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models"
CUSTOM_MODELS="$ROOT_DIR/models"
if [[ -d "$CUSTOM_MODELS" ]]; then
  export GAZEBO_MODEL_PATH="$CUSTOM_MODELS:$PX4_MODELS:${GAZEBO_MODEL_PATH:-}"
else
  export GAZEBO_MODEL_PATH="$PX4_MODELS:${GAZEBO_MODEL_PATH:-}"
fi

echo "[launch_px4_gazebo] GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"

if [[ -f "$CUSTOM_MODELS/iris_opt_flow/iris_opt_flow.sdf" ]]; then
  echo "[launch_px4_gazebo] Using custom iris_opt_flow model from $CUSTOM_MODELS"
fi

# forza simulatore e modello corretti
export PX4_SIMULATOR="gazebo-classic"
export PX4_SIM_MODEL="${PX4_SIM_MODEL:-iris_opt_flow}"

echo "[launch_px4_gazebo] simulator=$PX4_SIMULATOR  model=$PX4_SIM_MODEL"
if [[ -n "$INSTANCE_ID" ]]; then
  echo "[launch_px4_gazebo] PX4_INSTANCE=$INSTANCE_ID"
fi
if [[ -n "$MODEL_NAME_OVERRIDE" ]]; then
  echo "[launch_px4_gazebo] PX4_GZ_MODEL_NAME=$MODEL_NAME_OVERRIDE"
fi
if [[ -n "$MODEL_POSE" ]]; then
  echo "[launch_px4_gazebo] PX4_GZ_MODEL_POSE=$MODEL_POSE"
fi



MAKE_ENV=(
  "PX4_GZ_WORLD=$WORLD_PATH"
  "PX4_GAZEBO_WORLD=$WORLD_PATH"
  "PX4_SITL_WORLD=$WORLD_PATH"
  "PX4_SIM_MODEL=${PX4_SIM_MODEL:-iris_opt_flow}"
)
if [[ -n "$INSTANCE_ID" ]]; then
  MAKE_ENV+=("PX4_INSTANCE=$INSTANCE_ID")
fi
if [[ $HEADLESS -eq 1 ]]; then
  MAKE_ENV+=("HEADLESS=1")
fi
if [[ -n "$MODEL_NAME_OVERRIDE" ]]; then
  MAKE_ENV+=("PX4_GZ_MODEL_NAME=$MODEL_NAME_OVERRIDE")
fi
if [[ -n "$MODEL_POSE" ]]; then
  MAKE_ENV+=("PX4_GZ_MODEL_POSE=$MODEL_POSE")
fi

cleanup() {
  if [[ -n "${MAKE_PID:-}" ]]; then
    if kill -0 "$MAKE_PID" 2>/dev/null; then
      echo "[launch_px4_gazebo] Stopping PX4/Gazebo (pid=$MAKE_PID)"
      kill "$MAKE_PID" 2>/dev/null || true
      wait "$MAKE_PID" 2>/dev/null || true
    fi
  fi
}
trap cleanup INT TERM

cd "$PX4_DIR"
stdbuf -oL -eL env "${MAKE_ENV[@]}" \
  make "${BUILD_TARGET}" "gazebo-classic_${PX4_SIM_MODEL}" \
  | sed -u "s/^/[px4] /" | tee "$LOG_FILE"
