#!/usr/bin/env bash
set -euo pipefail
source "$(dirname "$0")/scripts/env.sh"
if [[ -f "$ROOT/rbnx-boot/state.json" ]]; then
  echo "Stop the deployment with ./stop.sh before building." >&2
  exit 2
fi
python3.10 -m venv --system-site-packages "$ROOT/.venv"
"$ROOT/.venv/bin/python3" -m pip install -r "$ROOT/requirements.txt" \
  -e "$(rbnx path robonix-api)"
exec rbnx build -f "$ROOT/robonix_manifest.motion.yaml"
