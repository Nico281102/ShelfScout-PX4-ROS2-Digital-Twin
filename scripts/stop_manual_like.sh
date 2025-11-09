#!/usr/bin/env bash
# Ferma Micro XRCE Agent, PX4, Gazebo e il mission runner in modo aggressivo.

set -euo pipefail

echo "[manual_like] Sto killando i processi..."

# Tentativo "soft"
pkill -f MicroXRCEAgent 2>/dev/null || true
pkill -f mission_runner 2>/dev/null || true
pkill -f px4 2>/dev/null || true
pkill -f gzserver 2>/dev/null || true
pkill -f gzclient 2>/dev/null || true
pkill -f "sitl_run.sh" 2>/dev/null || true
pkill -f "gazebo-classic_iris_opt_flow" 2>/dev/null || true

# Attendi un attimo e forza la chiusura se qualcosa è rimasto
sleep 1
pkill -9 -f MicroXRCEAgent 2>/dev/null || true
pkill -9 -f mission_runner 2>/dev/null || true
pkill -9 -f px4 2>/dev/null || true
pkill -9 -f gzserver 2>/dev/null || true
pkill -9 -f gzclient 2>/dev/null || true
pkill -9 -f "sitl_run.sh" 2>/dev/null || true
pkill -9 -f "gazebo-classic_iris_opt_flow" 2>/dev/null || true
pkill -9 -f "px4_sitl_default" 2>/dev/null || true
pkill -9 -f "gazebo-classic_iris_opt_flow" 2>/dev/null || true

# Cleanup eventuali build in corso (make/cmake)
pkill -9 -f "cmake --build" 2>/dev/null || true
pkill -9 -f "make px4_sitl_default" 2>/dev/null || true

# Doppio check per eventuali processi gazebo* residui
pgrep -f gz  >/dev/null && pkill -9 -f gz 2>/dev/null || true

echo "[manual_like] Tutto killato ✅"
