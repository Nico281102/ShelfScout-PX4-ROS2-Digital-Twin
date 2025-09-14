#!/usr/bin/env bash
# Create and populate a Python virtualenv for the project.
# Usage: scripts/setup_venv.sh [PYTHON]
set -euo pipefail

PYTHON_BIN="${1:-python3}"
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="$ROOT_DIR/.venv"

echo "[VENV] Using Python: $PYTHON_BIN"
"$PYTHON_BIN" -m venv "$VENV_DIR"
source "$VENV_DIR/bin/activate"
pip install --upgrade pip
pip install -r "$ROOT_DIR/requirements.txt"
echo "[VENV] Ready. Activate with: source $VENV_DIR/bin/activate"

