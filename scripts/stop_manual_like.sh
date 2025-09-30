#!/usr/bin/env bash
# Stoppa MicroXRCEAgent, PX4, Gazebo e mission_runner

echo "[manual_like] Sto killando i processi..."

pkill -f MicroXRCEAgent || true
pkill -f px4 || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f mission_runner || true

echo "[manual_like] Tutto killato ✅"
