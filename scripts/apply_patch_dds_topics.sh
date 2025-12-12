#!/usr/bin/env bash
# Ensure PX4 uXRCE DDS topics include battery_status publication.
# Safe no-op if already present.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"

log()  { echo "[patch_dds_topics] $*"; }
err()  { echo "[patch_dds_topics] ERROR: $*" >&2; }
die()  { err "$@"; exit 1; }

# Load env if available (to resolve SSDT_PX4_DIR)
if [[ -f "$ENV_FILE" ]]; then
  set +u
  # shellcheck disable=SC1090
  source "$ENV_FILE"
  set -u
fi

PX4_DIR="${SSDT_PX4_DIR:-${SHELFSCOUT_PX4_DIR:-${PX4_DIR:-}}}"
[[ -n "${PX4_DIR:-}" ]] || die "PX4_DIR is not set (populate scripts/.env or export SSDT_PX4_DIR)."

TARGET="$PX4_DIR/src/modules/uxrce_dds_client/dds_topics.yaml"
[[ -f "$TARGET" ]] || die "dds_topics.yaml not found at $TARGET"

BATTERY_TOPIC="/fmu/out/battery_status"

if grep -Fq "$BATTERY_TOPIC" "$TARGET"; then
  log "battery_status already present in dds_topics.yaml (no changes)"
  exit 0
fi

# Insert battery_status publication after vehicle_status block.
tmp_file="$(mktemp)"
trap 'rm -f "$tmp_file"' EXIT

if ! sed '/\/fmu\/out\/vehicle_status/{
  n
  a\
  - topic: /fmu/out/battery_status\
    type: px4_msgs::msg::BatteryStatus
}' "$TARGET" > "$tmp_file"; then
  die "sed patch failed"
fi

mv "$tmp_file" "$TARGET"
log "battery_status publication added to $TARGET"
