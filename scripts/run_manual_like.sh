#!/usr/bin/env bash
# Minimal orchestrator: avvia MicroXRCEAgent in un terminale separato
# e poi PX4 + Gazebo + mission_runner come fai tu a mano
# + monitoraggio missione (status + ack)
#
# Usage:
#   ./run_manual_like.sh [--gui|--headless]

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
if [[ ! -f "$ENV_FILE" ]]; then
  echo "[manual_like] Missing environment file: $ENV_FILE" >&2
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
  echo "[manual_like] PX4_DIR is not set. Check $ENV_FILE." >&2
  exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
  echo "[manual_like] PX4_DIR does not exist: $PX4_DIR" >&2
  exit 1
fi

MISSION_FILE="$ROOT_DIR/config/mission.yaml"
if [[ ! -f "$MISSION_FILE" ]]; then
  echo "[manual_like] Mission file not found: $MISSION_FILE" >&2
  exit 1
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "[manual_like] ros2 CLI not found after sourcing $ENV_FILE" >&2
  exit 1
fi
if ! command -v MicroXRCEAgent >/dev/null 2>&1; then
  echo "[manual_like] MicroXRCEAgent not found in PATH after sourcing $ENV_FILE" >&2
  exit 1
fi

# Default: headless
HEADLESS=1

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)
      HEADLESS=0
      shift
      ;;
    --headless)
      HEADLESS=1
      shift
      ;;
    *)
      echo "[manual_like] Unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

# 1) Avvia Micro XRCE-DDS Agent in un nuovo terminale
echo "[manual_like] Avvio MicroXRCEAgent in nuovo terminale..."
gnome-terminal -- bash -c "source \"$ENV_FILE\"; MicroXRCEAgent udp4 -p 8888 -v 6; exec bash"

sleep 3

# 2) Avvia PX4 SITL + Gazebo con modello iris_opt_flow
echo "[manual_like] Avvio PX4 SITL + Gazebo..."
(
  cd "$PX4_DIR"
  if [[ $HEADLESS -eq 1 ]]; then
    HEADLESS=1 make px4_sitl_default gazebo-classic_iris_opt_flow
  else
    make px4_sitl_default gazebo-classic_iris_opt_flow
  fi
) & PX4_PID=$!

sleep 15

# 3) Avvia ROS 2 mission_runner
echo "[manual_like] Avvio mission_runner..."
(
  ros2 run overrack_mission mission_runner --ros-args -p mission_file:="$MISSION_FILE"
) & MISSION_PID=$!

sleep 15

# 4) Monitoraggio stato missione in un nuovo terminale
echo "[manual_like] Avvio monitoraggio vehicle_status..."
gnome-terminal -- bash -c "source \"$ENV_FILE\"; ros2 topic echo /fmu/out/vehicle_status; exec bash"

# 5) Monitoraggio ack comandi in un nuovo terminale
echo "[manual_like] Avvio monitoraggio vehicle_command_ack..."
gnome-terminal -- bash -c "source \"$ENV_FILE\"; ros2 topic echo /fmu/out/vehicle_command_ack; exec bash"

# Cleanup alla chiusura
cleanup() {
  echo "[manual_like] Stop processi..."
  kill $MISSION_PID $PX4_PID 2>/dev/null || true
}
trap cleanup INT TERM EXIT

wait $MISSION_PID
