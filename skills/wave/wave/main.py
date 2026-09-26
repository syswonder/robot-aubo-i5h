#!/usr/bin/env python3
"""wave — slightly wiggle each arm joint to verify controllability."""
from __future__ import annotations

import logging
import math
import threading
import time
from typing import Any

from robonix_api import Deferred, Err, Ok, Skill

from wave.arm_client import ArmRosClient

provider = Skill(id="wave", namespace="robonix/skill/wave")

from wave_mcp import Wave_Request, Wave_Response  # noqa: E402

log = logging.getLogger("wave")

_state: dict[str, Any] = {
    "arm": None,
    "lock": threading.Lock(),
    "defaults": {
        "delta_deg": 0.3,
        "settle_tolerance_rad": 0.001,
        "settle_timeout_s": 15.0,
        "pause_s": 0.5,
    },
}


def _parse_float(text: str, *, default: float, label: str) -> float:
    raw = (text or "").strip()
    if not raw:
        return default
    try:
        value = float(raw)
    except ValueError as exc:
        raise ValueError(f"{label} must be numeric, got {text!r}") from exc
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be positive, got {value}")
    return value


def _run_wave(
    arm: ArmRosClient,
    *,
    delta_rad: float,
    settle_tolerance_rad: float,
    settle_timeout_s: float,
    pause_s: float,
) -> str:
    names, positions = arm.snapshot()
    home = list(positions)
    dof = len(home)
    log.info("wave start — %d joints, delta=%.3f rad", dof, delta_rad)

    for joint_idx in range(dof):
        label = names[joint_idx] if names and joint_idx < len(names) else f"joint_{joint_idx + 1}"

        plus = list(home)
        plus[joint_idx] += delta_rad
        arm.command_positions(names, plus)
        arm.wait_until_near(
            plus,
            tolerance_rad=settle_tolerance_rad,
            timeout_s=settle_timeout_s,
        )
        if arm.cancel.wait(pause_s):
            raise RuntimeError("wave cancelled")

        minus = list(home)
        minus[joint_idx] -= delta_rad
        arm.command_positions(names, minus)
        arm.wait_until_near(
            minus,
            tolerance_rad=settle_tolerance_rad,
            timeout_s=settle_timeout_s,
        )
        if arm.cancel.wait(pause_s):
            raise RuntimeError("wave cancelled")

        arm.command_positions(names, home)
        arm.wait_until_near(
            home,
            tolerance_rad=settle_tolerance_rad,
            timeout_s=settle_timeout_s,
        )
        log.info("joint %s ok", label)

    return f"wave completed for {dof} joints (±{math.degrees(delta_rad):.1f}°)"


@provider.mcp("robonix/skill/wave/wave")
def wave(req: Wave_Request) -> Wave_Response:
    """轻微摆动各关节以检查机械臂是否可控。

    依次让每个关节在当前位置附近做小幅度正负摆动，然后回到起始姿态。
    可选参数 delta_deg（摆动幅度，度）和 settle_timeout_s（单次等待超时，秒）。
    """
    arm = _state["arm"]
    if arm is None:
        return Wave_Response(ok=False, message="skill not activated (arm not connected)")

    defaults = _state["defaults"]
    try:
        delta_deg = _parse_float(
            req.delta_deg,
            default=float(defaults["delta_deg"]),
            label="delta_deg",
        )
        settle_timeout_s = _parse_float(
            req.settle_timeout_s,
            default=float(defaults["settle_timeout_s"]),
            label="settle_timeout_s",
        )
    except ValueError as exc:
        return Wave_Response(ok=False, message=f"bad argument: {exc}")

    delta_rad = math.radians(delta_deg)
    if delta_deg > 0.5 or settle_timeout_s > 30:
        return Wave_Response(ok=False, message="commissioning wave is limited to 0.5 degrees and 30 seconds per target")
    if not _state["lock"].acquire(blocking=False):
        return Wave_Response(ok=False, message="wave already running; request not queued")
    try:
        message = _run_wave(
                arm,
                delta_rad=delta_rad,
                settle_tolerance_rad=float(defaults["settle_tolerance_rad"]),
                settle_timeout_s=settle_timeout_s,
                pause_s=float(defaults["pause_s"]),
            )
    except Exception as exc:  # noqa: BLE001
        log.exception("wave failed")
        try:
            arm.stop_arm()
        except Exception as stop_error:
            return Wave_Response(ok=False, message=f"{exc}; STOP FAILED: {stop_error}")
        return Wave_Response(ok=False, message=str(exc))
    finally:
        _state["lock"].release()

    return Wave_Response(ok=True, message=message)


@provider.on_init
def init(cfg: dict):
    cfg = cfg or {}
    _state["defaults"] = {
        "delta_deg": float(cfg.get("delta_deg", 0.3)),
        "settle_tolerance_rad": float(cfg.get("settle_tolerance_rad", 0.001)),
        "settle_timeout_s": float(cfg.get("settle_timeout_s", 15.0)),
        "pause_s": float(cfg.get("pause_s", 0.5)),
    }
    if any(not math.isfinite(v) or v <= 0 for v in _state["defaults"].values()):
        return Err("wave configuration must be finite and positive")
    if not 0.0001 <= _state["defaults"]["settle_tolerance_rad"] <= 0.001:
        return Err("settle tolerance must be 0.0001 to 0.001 rad")
    _state["arm_provider_id"] = str(cfg.get("arm_provider_id", "aubo_arm"))
    log.info("init ok — arm_provider_id=%s", _state["arm_provider_id"])
    return Ok()


@provider.on_activate
def activate():
    provider_id = str(_state.get("arm_provider_id", "aubo_arm"))
    try:
        arm = ArmRosClient(provider, arm_provider_id=provider_id)
        arm.connect()
    except Exception as exc:  # noqa: BLE001
        log.exception("failed to connect arm primitive")
        return Deferred(str(exc))

    _state["arm"] = arm
    log.info("activated — connected to arm provider %s", provider_id)
    return Ok()


@provider.on_deactivate
def deactivate():
    arm = _state["arm"]
    if arm is not None:
        if _state["lock"].locked():
            try:
                arm.stop_arm()
            except Exception as exc:
                return Err(str(exc))
        arm.close()
    _state["arm"] = None
    log.info("deactivated")
    return Ok()


@provider.on_shutdown
def shutdown():
    return deactivate()


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="[wave] %(levelname)s %(message)s")
    provider.run()
