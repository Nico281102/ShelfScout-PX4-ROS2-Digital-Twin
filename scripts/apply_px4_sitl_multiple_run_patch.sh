#!/usr/bin/env bash
# Apply the local sitl_multiple_run.sh patch into a PX4 checkout using sed.
# Usage:
#   scripts/apply_px4_sitl_multiple_run_patch.sh [--px4-dir <path>]

set -euo pipefail

usage() {
  cat <<'EOF'
Usage: apply_px4_sitl_multiple_run_patch.sh [--px4-dir <path>]

Options:
  --px4-dir   Path to PX4-Autopilot (default: $PX4_DIR or ~/PX4/PX4-Autopilot)
  -h, --help  Show this help
EOF
}

PX4_DIR_DEFAULT="${PX4_DIR:-$HOME/PX4/PX4-Autopilot}"
PX4_DIR="$PX4_DIR_DEFAULT"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --px4-dir)
      PX4_DIR="${2:-}"
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage
      exit 1
      ;;
  esac
done

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PATCH_CONTENT="${SCRIPT_DIR}/patches/px4_sitl_multiple_run.patched"
TARGET="${PX4_DIR}/Tools/simulation/gazebo-classic/sitl_multiple_run.sh"

if [[ ! -f "$PATCH_CONTENT" ]]; then
  echo "Patch content missing: $PATCH_CONTENT" >&2
  exit 1
fi

if [[ ! -f "$TARGET" ]]; then
  echo "Target file not found: $TARGET" >&2
  exit 1
fi

if cmp -s "$PATCH_CONTENT" "$TARGET"; then
  echo "[patch] Already applied: $TARGET"
  exit 0
fi

tmp="$(mktemp)"
cleanup() { rm -f "$tmp"; }
trap cleanup EXIT

# Replace file content using sed (keeps shebang permissions after chmod).
sed -n -e "1{r $PATCH_CONTENT" -e 'q' -e '}' "$TARGET" > "$tmp"

chmod +x "$tmp"
mv "$tmp" "$TARGET"

echo "[patch] Applied sitl_multiple_run.sh patch to $TARGET"
