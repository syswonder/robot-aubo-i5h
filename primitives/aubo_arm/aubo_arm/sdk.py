"""Official Python SDK backend using the colleague's RpcClient connection pattern.

Unlike automatic startup examples, connecting here never modifies power,
brakes, speed fraction, runtime state, TCP calibration or safety settings.
"""

import json
import os
from pathlib import Path
import threading
import urllib.parse

from .rpc import READS, WRITES, RpcError


class SdkClient:
    """Expose the same bounded allowlisted operations through pyaubo_sdk 0.26."""

    def __init__(self, endpoint, robot="rob1", timeout=0.5, motion=False):
        """Authenticate a bounded TCP connection without changing controller state."""
        import pyaubo_sdk
        parsed = urllib.parse.urlsplit(endpoint)
        if (parsed.scheme != "http" or not parsed.hostname or parsed.username
                or parsed.password or parsed.query or parsed.fragment
                or parsed.path != "/jsonrpc"):
            raise ValueError("endpoint must be http://HOST:PORT/jsonrpc")
        if not robot.isidentifier() or not 0 < timeout <= 5:
            raise ValueError("invalid robot name or RPC timeout")
        path = Path(os.environ["AUBO_RPC_CREDENTIALS"])
        if path.stat().st_mode & 0o077:
            raise ValueError("SDK credentials file must have mode 0600")
        auth = json.loads(path.read_text())
        self._lock = threading.Lock()
        self.motion = motion is True
        self.robot_name = robot
        self.rpc = pyaubo_sdk.RpcClient()
        self.rpc.setRequestTimeout(int(timeout*1000))
        self._robot = None
        try:
            self.rpc.connect(parsed.hostname, 30004)
            if not self.rpc.hasConnected():
                raise RpcError("SDK connection failed")
            self.rpc.login(auth["username"],auth["password"])
            if not self.rpc.hasLogined():
                raise RpcError("SDK authentication failed")
            if robot not in self.rpc.getRobotNames():
                raise RpcError("configured robot is absent")
            self._robot = self.rpc.getRobotInterface(robot)
        except Exception:
            self.close()
            raise
        finally:
            auth.clear()

    def call(self, method, params=()):
        """Map only approved reads/commands; normalize SDK enums and tuples to wire shapes."""
        if method not in READS and not (self.motion and method in WRITES):
            raise RpcError(f"SDK method is not permitted: {method}")
        with self._lock:
            if method == "getRobotNames":
                return list(self.rpc.getRobotNames())
            group, name = method.split(".")
            parent = self.rpc if group in {"SystemInfo","RuntimeMachine","Math"} else self._robot
            interface = getattr(parent,f"get{group}")()
            try:
                result = getattr(interface,name)(*params)
            except Exception as exc:
                raise RpcError(f"{method}: SDK request failed ({type(exc).__name__})") from exc
            if hasattr(result,"name"):
                return result.name
            return list(result) if isinstance(result,tuple) else result

    def command(self, method, params):
        """Check SDK return codes, with no retry after an uncertain command."""
        result = self.call(method,params)
        if type(result) is not int or result != 0:
            raise RpcError(f"{method} rejected by controller: {result!r}")

    def close(self):
        """Release only this connection; never call RobotManage shutdown/poweroff."""
        try:
            if self.rpc.hasLogined():
                self.rpc.logout()
        finally:
            if self.rpc.hasConnected():
                self.rpc.disconnect()
