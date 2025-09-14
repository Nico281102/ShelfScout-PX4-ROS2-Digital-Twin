#!/usr/bin/env bash
#
# Launch PX4 SITL + Gazebo Classic with the Over-Rack indoor world.
#
# Usage:
#   PX4_DIR=/path/to/PX4-Autopilot scripts/launch_px4_gazebo.sh [--headless] [--world <path/to/world.sdf>]
#
# Notes:
# - Requires PX4-Autopilot built for SITL (px4_sitl_default) and Gazebo Classic.
# - Sources PX4's Gazebo setup (legacy or new path) to set plugins/models.
# - Extends GAZEBO_MODEL_PATH to include this repo's models.
# - World path is worlds/overrack_indoor.world.
# - If --headless is passed or DISPLAY is empty, uses gzserver instead of gazebo GUI.
set -euo pipefail

HEADLESS=0
WORLD_OPT=""
while [[ $# -gt 0 ]]; do
  case "$1" in
    --headless)
      HEADLESS=1; shift ;;
    --world)
      WORLD_OPT=${2:-}; shift 2 ;;
    *)
      echo "[launch_px4_gazebo] Unknown arg: $1" >&2; shift ;;
  esac
done

PX4_DIR="${PX4_DIR:-}"  # e.g., $HOME/PX4-Autopilot
if [[ -z "$PX4_DIR" ]]; then
  echo "[!] Please export PX4_DIR to your PX4-Autopilot repo path." >&2
  exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
  echo "[!] PX4_DIR does not exist: $PX4_DIR" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORLD_PATH="$ROOT_DIR/worlds/overrack_indoor.world"
if [[ -n "$WORLD_OPT" ]]; then
  if [[ "$WORLD_OPT" = /* ]]; then
    WORLD_PATH="$WORLD_OPT"
  else
    WORLD_PATH="$ROOT_DIR/$WORLD_OPT"
  fi
fi
BUILD_DIR="$PX4_DIR/build/px4_sitl_default"

if [[ ! -f "$WORLD_PATH" ]]; then
  echo "[!] World not found: $WORLD_PATH" >&2
  exit 1
fi
if [[ ! -d "$BUILD_DIR" ]] || [[ ! -x "$BUILD_DIR/bin/px4" ]]; then
  echo "[Build] PX4 SITL not found; running 'make px4_sitl gazebo'..." >&2
  pushd "$PX4_DIR" >/dev/null
  make px4_sitl gazebo
  popd >/dev/null
fi

# Source PX4 Gazebo Classic environment (plugins, models, resources)
# Support both legacy and current paths in PX4 repo structure.
# Avoid errors with 'set -u' when sourced script expands unset vars
: "${GAZEBO_PLUGIN_PATH:=}"
: "${GAZEBO_MODEL_PATH:=}"
: "${GAZEBO_RESOURCE_PATH:=}"
: "${IGN_GAZEBO_RESOURCE_PATH:=}"
: "${LD_LIBRARY_PATH:=}"
if [[ -f "$PX4_DIR/Tools/setup_gazebo.bash" ]]; then
  # shellcheck disable=SC1091
  source "$PX4_DIR/Tools/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR"
elif [[ -f "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" ]]; then
  # shellcheck disable=SC1091
  source "$PX4_DIR/Tools/simulation/gazebo-classic/setup_gazebo.bash" "$PX4_DIR" "$BUILD_DIR"
else
  echo "[!] Non trovo lo script di setup Gazebo in PX4_DIR. Controlla l'installazione." >&2
  exit 1
fi

# Extend model path to include our local models (shelves, markers, barcode boards).
export GAZEBO_MODEL_PATH="$ROOT_DIR/models:${GAZEBO_MODEL_PATH:-}"

echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"
echo "WORLD=$WORLD_PATH"
echo "[Launch] World => $WORLD_PATH"
echo "HEADLESS=$HEADLESS"

# Validate world SDF if tools are available
if command -v gz >/dev/null 2>&1; then
  echo "[SDF] Validating with 'gz sdf -k'..."
  if ! gz sdf -k "$WORLD_PATH"; then
    echo "[SDF] Validation failed for $WORLD_PATH" >&2
    exit 1
  fi
else
  echo "[SDF] 'gz' not found; skipping validation"
fi

# Avoid conflicts with existing gazebo/gzserver
pkill -f gzserver 2>/dev/null || true
pkill -f "gazebo(?!.*run_full_mission)" 2>/dev/null || true

# Start PX4 SITL in background (vehicle: iris)
PX4_BUILD="$BUILD_DIR"
PX4_BIN="$PX4_BUILD/bin/px4"
PX4_TMP="$PX4_BUILD/tmp"

# Auto-detect rcS layout (modern first, then rootfs, then legacy)
if [[ -f "$PX4_BUILD/etc/init.d/rcS" ]]; then
  PX4_ROOT="$PX4_BUILD"; PX4_RCS="etc/init.d/rcS"
  echo "[PX4] Using modern rcS path."
elif [[ -f "$PX4_BUILD/tmp/rootfs/etc/init.d/rcS" ]]; then
  PX4_ROOT="$PX4_BUILD/tmp/rootfs"; PX4_RCS="etc/init.d/rcS"
  echo "[PX4] Using rootfs rcS path."
elif [[ -f "$PX4_DIR/posix-configs/SITL/init/ekf2/iris" ]]; then
  PX4_ROOT="$PX4_DIR"; PX4_RCS="posix-configs/SITL/init/ekf2/iris"
  echo "[PX4] Using legacy rcS path."
else
  echo "[!] rcS non trovato in build o nella repo PX4." >&2
  exit 1
fi

mkdir -p "$ROOT_DIR/data/logs" "$PX4_TMP"
echo "[PX4] Launching SITL..."
echo "[PX4] BIN=$PX4_BIN"
echo "[PX4] ROOT=$PX4_ROOT"
echo "[PX4] rcS=$PX4_RCS"

# Compute absolute rcS and detect sitl_run.sh
PX4_RCS_ABS="$PX4_RCS"
if [[ -f "$PX4_ROOT/$PX4_RCS" ]]; then
  PX4_RCS_ABS="$PX4_ROOT/$PX4_RCS"
fi
echo "[PX4] rcS(abs)=$PX4_RCS_ABS"

if [[ -x "$PX4_DIR/Tools/simulation/gazebo-classic/sitl_run.sh" ]]; then
  SITL_RUN="$PX4_DIR/Tools/simulation/gazebo-classic/sitl_run.sh"; SITL_RUN_MODE="new"
elif [[ -x "$PX4_DIR/Tools/sitl_run.sh" ]]; then
  SITL_RUN="$PX4_DIR/Tools/sitl_run.sh"; SITL_RUN_MODE="legacy"
else
  SITL_RUN=""; SITL_RUN_MODE="none"
fi

if [[ -n "$SITL_RUN" ]]; then
  echo "[PX4] Using sitl_run.sh ($SITL_RUN_MODE): $SITL_RUN"
  if [[ "$SITL_RUN_MODE" == "new" ]]; then
    # New: bin debugger model world src_path build_path
    "$SITL_RUN" "$PX4_BIN" none iris "$WORLD_PATH" "$PX4_DIR" "$PX4_BUILD" \
      >"$ROOT_DIR/data/logs/px4_sitl.out" 2>&1 &
    PX4_PID=$!
  else
    # Legacy: bin debugger program model world src_path build_path (program=gazebo)
    "$SITL_RUN" "$PX4_BIN" none gazebo iris "$WORLD_PATH" "$PX4_DIR" "$PX4_BUILD" \
      >"$ROOT_DIR/data/logs/px4_sitl.out" 2>&1 &
    PX4_PID=$!
  fi
else
  echo "[PX4] sitl_run.sh not found; falling back to direct px4 invocation"
  pushd "$PX4_ROOT" >/dev/null
  PX4_SIM_MODEL=iris "$PX4_BIN" -w "$PX4_ROOT" -s "$PX4_RCS" -t "$PX4_TMP" \
    >"$ROOT_DIR/data/logs/px4_sitl.out" 2>&1 &
  PX4_PID=$!
  popd >/dev/null
fi

sleep 2
echo "[PX4] SITL PID: $PX4_PID"

# GUI: attach only gzclient (sitl_run launches server). Headless: do nothing.
if [[ $HEADLESS -eq 0 && -n "${DISPLAY:-}" ]]; then
  if command -v gzclient >/dev/null 2>&1; then
    echo "[GAZEBO] Attaching gzclient..."
    gzclient &
  else
    echo "[GAZEBO] gzclient not found; proceeding headless"
  fi
fi

echo "[OK] PX4 PID=$PX4_PID"
echo "Press Ctrl+C to stop."

trap 'echo Stopping...; pkill -f gzclient 2>/dev/null || true; pkill -f gzserver 2>/dev/null || true; kill $PX4_PID 2>/dev/null || true; wait || true' INT TERM
wait
