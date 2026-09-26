"""Bounded ARCS JSON-RPC transport. Commands are never retried."""

import itertools
import json
import math
import threading
import urllib.parse
import urllib.request


READS = frozenset({
    "getRobotNames", "SystemInfo.getControlSoftwareFullVersion",
    "SystemInfo.getInterfaceVersionCode", "RuntimeMachine.getStatus",
    "RobotConfig.getRobotType", "RobotConfig.getControlBoxType",
    "RobotConfig.getJointMinPositions", "RobotConfig.getJointMaxPositions",
    "RobotConfig.getJointMaxSpeeds", "RobotConfig.getKinematicsParam",
    "RobotConfig.getTcpOffset", "RobotConfig.getMountingPose",
    "RobotState.getRobotModeType", "RobotState.getSafetyModeType",
    "RobotState.getJointPositions", "RobotState.getJointSpeeds",
    "RobotState.getTcpPose", "MotionControl.getExecId",
    "RobotAlgorithm.forwardKinematics", "RobotAlgorithm.inverseKinematics",
    "Math.rpyToQuaternion", "Math.quaternionToRpy",
})
WRITES = frozenset({"MotionControl.moveJoint", "MotionControl.stopJoint"})


class RpcError(RuntimeError):
    """Transport, protocol or controller error; a write may have executed."""


class NoRedirect(urllib.request.HTTPRedirectHandler):
    """Never forward robot requests to another endpoint."""

    def redirect_request(self, req, fp, code, msg, headers, newurl):
        raise RpcError("controller redirect rejected")


def vector(value, size, label):
    """Require an exact finite numeric vector without coercing booleans/strings."""
    if not isinstance(value, (list, tuple)) or len(value) != size:
        raise ValueError(f"{label} must contain {size} numbers")
    if any(type(v) not in (int, float) or not math.isfinite(v) for v in value):
        raise ValueError(f"{label} must contain only finite numbers")
    return tuple(float(v) for v in value)


class Client:
    """Allowlist calls, disable environment proxies, and bound every request."""

    def __init__(self, endpoint, robot="rob1", timeout=0.5, motion=False):
        parsed = urllib.parse.urlsplit(endpoint)
        if (parsed.scheme != "http" or not parsed.hostname or parsed.username
                or parsed.password or parsed.query or parsed.fragment
                or parsed.path != "/jsonrpc"):
            raise ValueError("endpoint must be http://HOST:PORT/jsonrpc")
        if not robot.isidentifier() or not 0 < timeout <= 5:
            raise ValueError("invalid robot name or RPC timeout")
        self.endpoint, self.robot, self.timeout = endpoint, robot, timeout
        self.motion = motion is True
        self._ids = itertools.count(1)
        self._lock = threading.Lock()
        self._opener = urllib.request.build_opener(
            urllib.request.ProxyHandler({}), NoRedirect())

    def call(self, method, params=()):
        """Perform one call; reject writes unless explicitly motion-enabled."""
        if method not in READS and not (self.motion and method in WRITES):
            raise RpcError(f"RPC method is not permitted: {method}")
        full = method if method.split(".")[0] in {
            "getRobotNames", "SystemInfo", "RuntimeMachine", "Math"
        } else f"{self.robot}.{method}"
        with self._lock:
            request_id = next(self._ids)
            payload = json.dumps({"jsonrpc": "2.0", "id": request_id,
                                  "method": full, "params": list(params)},
                                 allow_nan=False).encode()
            req = urllib.request.Request(self.endpoint, data=payload,
                                         headers={"Content-Type": "application/json"})
            try:
                with self._opener.open(req, timeout=self.timeout) as response:
                    raw = response.read(1024 * 1024 + 1)
                if len(raw) > 1024 * 1024:
                    raise RpcError("controller response exceeds limit")
                result = json.loads(raw)
            except (OSError, ValueError) as exc:
                raise RpcError(f"{method}: {type(exc).__name__}") from exc
            if (not isinstance(result, dict) or result.get("jsonrpc") != "2.0"
                    or type(result.get("id")) is not int or result["id"] != request_id
                    or "error" in result or "result" not in result):
                error = result.get("error", {}) if isinstance(result, dict) else {}
                raise RpcError(f"{method}: invalid response or controller error {error}")
            return result["result"]

    def command(self, method, params):
        """Require the documented zero success code, not truthy/falsy coercion."""
        code = self.call(method, params)
        if type(code) is not int or code != 0:
            raise RpcError(f"{method} rejected by controller: {code!r}")
