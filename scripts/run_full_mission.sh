#!/usr/bin/env bash
#
# One-command indoor mission run:
#  1) Auto-build PX4 if missing
#  2) Launch PX4 SITL + Gazebo with the Over-Rack world
#  3) Upload waypoints from config/mission.yaml to SITL and start mission
#  4) Start vision pipeline to decode barcodes from the down-facing camera frames
#  5) On mission completion, stop everything and leave logs in data/logs
#
# Usage:
#   PX4_DIR=/path/to/PX4-Autopilot scripts/run_full_mission.sh [--headless]
#
set -euo pipefail
: "${PX4_DIR:=$HOME/PX4/PX4-Autopilot}"

HEADLESS=0
if [[ "${1:-}" == "--headless" ]]; then
  HEADLESS=1
  shift || true
fi

PX4_DIR="${PX4_DIR:-}"
if [[ -z "$PX4_DIR" ]]; then
  echo "[!] Please export PX4_DIR to your PX4-Autopilot path" >&2
  exit 1
fi
if [[ ! -d "$PX4_DIR" ]]; then
  echo "[!] PX4_DIR not found: $PX4_DIR" >&2
  exit 1
fi

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# Ensure output dirs
mkdir -p "$ROOT_DIR/data/logs" "$ROOT_DIR/data/images/downcam"

# Clean camera frames dir
CAM_DIR="$ROOT_DIR/data/images/downcam"
rm -f "$CAM_DIR"/* || true

# Launch PX4 SITL + Gazebo (auto-build if needed)
if [[ -z "${DISPLAY:-}" && $HEADLESS -eq 0 ]]; then
  echo "[Info] DISPLAY not set; switching to headless mode."
  HEADLESS=1
fi

echo "[Launch] Starting PX4 SITL + Gazebo (headless=$HEADLESS)..."
if [[ $HEADLESS -eq 1 ]]; then
  PX4_DIR="$PX4_DIR" "$ROOT_DIR/scripts/launch_px4_gazebo.sh" --headless &
else
  PX4_DIR="$PX4_DIR" "$ROOT_DIR/scripts/launch_px4_gazebo.sh" &
fi
GZ_WRAPPER_PID=$!

# Wait for PX4 comms to be ready: UDP 14540 (SDK) or TCP 4560 (gzserver MAVLink)
echo "[Wait] Waiting for PX4 comms (UDP 14540 or TCP 4560)..."
READY=0
for i in $(seq 1 45); do
  UDP_OK=0; TCP_OK=0
  if command -v ss >/dev/null 2>&1; then
    ss -lun | grep -q ":14540" && UDP_OK=1 || true
    ss -ltn | grep -q ":4560" && TCP_OK=1 || true
  elif command -v netstat >/dev/null 2>&1; then
    netstat -lun | grep -q ":14540" && UDP_OK=1 || true
    netstat -ltn | grep -q ":4560" && TCP_OK=1 || true
  fi
  if [[ $UDP_OK -eq 1 || $TCP_OK -eq 1 ]]; then
    echo "[Wait] Comms ready: UDP14540=$UDP_OK TCP4560=$TCP_OK (t=${i}s)"; READY=1; break
  fi
  sleep 1
done
if [[ $READY -ne 1 ]]; then
  echo "[Warn] Neither UDP 14540 nor TCP 4560 visible after 45s; proceeding anyway"
fi

# Start vision pipeline in background
echo "[Vision] Starting barcode pipeline..."
${PY:-/usr/bin/python3} "$ROOT_DIR/scripts/run_vision.py" --watch "$CAM_DIR" --csv "$ROOT_DIR/data/logs/vision.csv" --with-mavsdk &
VISION_PID=$!

# Upload and start mission (blocks until completion). Run from repo root so it finds config/mission.yaml.
echo "[Mission] Upload + start from config/mission.yaml"
pushd "$ROOT_DIR" >/dev/null
echo "[Bridge] MAVSDK_URL=${MAVSDK_URL:-auto}"
${PY:-/usr/bin/python3} scripts/mission_bridge.py --start || true
popd >/dev/null

cleanup() {
  echo "[Cleanup] Stopping vision + Gazebo + PX4..."
  kill $VISION_PID 2>/dev/null || true
  kill $GZ_WRAPPER_PID 2>/dev/null || true
  # Best-effort shutdown
  pkill -f gzserver 2>/dev/null || true
  pkill -f gazebo 2>/dev/null || true
  pkill -f px4 2>/dev/null || true
}
trap cleanup INT TERM
cleanup

echo "[Done] Logs in data/logs; frames in data/images/downcam"
