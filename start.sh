#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/scripts/env.sh"
if [[ ! -x "$ROOT/.venv/bin/python3" ]]; then
  echo "Run ./build.sh first." >&2
  exit 2
fi
case "${1:-readonly}" in
  readonly)
    export AUBO_ENABLE_MOTION=0
    MANIFEST="$ROOT/robonix_manifest.yaml"
    ;;
  motion)
    if [[ "${AUBO_ENABLE_MOTION:-0}" != 1 ]]; then
      echo "Motion mode requires AUBO_ENABLE_MOTION=1 ./start.sh motion" >&2
      exit 2
    fi
    MANIFEST="$ROOT/robonix_manifest.motion.yaml"
    ;;
  *)
    echo "Usage: ./start.sh [readonly|motion]" >&2
    exit 2
    ;;
esac
mkdir -p "$ROOT/.runtime"
exec rbnx boot -f "$MANIFEST"
