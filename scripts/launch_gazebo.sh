#!/usr/bin/env bash
#
# Launch only Gazebo Classic with the Over-Rack indoor world (no PX4).
# Helpful for quickly previewing the environment and models.
set -euo pipefail

# Add local models to Gazebo model path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
export GAZEBO_MODEL_PATH="${ROOT_DIR}/models:${GAZEBO_MODEL_PATH:-}"

WORLD="${ROOT_DIR}/worlds/overrack_indoor.world"

echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}"
echo "Launching Gazebo with world: ${WORLD}"

if command -v gazebo >/dev/null 2>&1; then
  exec gazebo "${WORLD}" --verbose
elif command -v gz >/dev/null 2>&1; then
  # Fallback for Ignition Gazebo; world should still load, but PX4 Classic assets may differ.
  echo "Detected gz; attempting to open with gz sim (Ignition)."
  exec gz sim "${WORLD}"
else
  echo "Neither 'gazebo' nor 'gz' found in PATH. Please install Gazebo Classic or Ignition."
  exit 1
fi
