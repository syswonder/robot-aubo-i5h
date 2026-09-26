#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
export PATH="$HOME/.cargo/bin:$PATH"
# Both profiles share this directory's rbnx-boot/state.json.
exec rbnx shutdown -f "$ROOT/robonix_manifest.yaml"
