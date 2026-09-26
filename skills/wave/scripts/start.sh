#!/usr/bin/env bash
set -eo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
cd "$PKG_ROOT"

for _ros_setup in \
  "${ROS_SETUP:-}" \
  /opt/ros/humble/setup.bash \
  /opt/ros/jazzy/setup.bash; do
  if [ -n "$_ros_setup" ] && [ -f "$_ros_setup" ]; then
    # shellcheck disable=SC1090
    source "$_ros_setup"
    break
  fi
done

export PYTHONPATH="$(rbnx path robonix-api):$PKG_ROOT:${PYTHONPATH:-}"
export PYTHONPATH="$PKG_ROOT/rbnx-build/codegen/proto_gen:$PKG_ROOT/rbnx-build/codegen/robonix_mcp_types:${PYTHONPATH:-}"

exec python3 -m wave.main
