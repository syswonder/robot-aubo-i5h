#!/usr/bin/env bash
# Sourced by build.sh and start.sh on Ubuntu 22.04 / ROS 2 Humble.
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source "${ROS_SETUP:-/opt/ros/humble/setup.bash}"
set -u
export PATH="$ROOT/.venv/bin:$HOME/.cargo/bin:$PATH"
export RBNX_CODEGEN_PYTHON="$ROOT/.venv/bin/python3"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-77}"
export ROS_LOCALHOST_ONLY=1
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
export ROBONIX_PROVIDER_BIND_HOST=127.0.0.1
export ROBONIX_ADVERTISE_HOST=127.0.0.1
export AUBO_URDF="$ROOT/model/aubo_i5H.urdf"
export AUBO_STATUS_FILE="$ROOT/.runtime/arm-status.json"
export AUBO_RPC_CREDENTIALS="${AUBO_RPC_CREDENTIALS:-$ROOT/.runtime/sdk-credentials.json}"
cd "$ROOT"
