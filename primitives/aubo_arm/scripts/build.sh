#!/usr/bin/env bash
set -euo pipefail
PKG_ROOT="${RBNX_PACKAGE_ROOT:-$(cd "$(dirname "$0")/.." && pwd)}"
# Code generation and execution use the same Python environment.
# RBNX_CODEGEN_POLICY_EXEMPT: host interpreter and runtime interpreter are identical.
rbnx codegen -p "$PKG_ROOT"
python3 -m compileall -q "$PKG_ROOT/aubo_arm"
