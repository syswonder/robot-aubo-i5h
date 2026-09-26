"""Single point-to-point motion with validated input and measured completion.

No power, brake release, runtime start, safety reset, or automatic retry is
performed. The operator must prepare the controller before enabling motion.
"""

from dataclasses import dataclass
import math
import threading
import time

from .kinematics import normalized_quaternion, quaternion, rpy
from .rpc import RpcError, vector


@dataclass(frozen=True)
class Settings:
    endpoint: str
    robot: str
    expected_model: str
    joint_names: tuple
    transport: str = "http"
    base_frame: str = "base_link"
    motion_enabled: bool = False
    poll_hz: float = 10.0
    rpc_timeout_s: float = 0.5
    max_sample_age_s: float = 0.5
    joint_speed_rad_s: float = 0.03
    joint_accel_rad_s2: float = 0.1
    stop_accel_rad_s2: float = 0.5
    max_joint_step_rad: float = 0.05
    max_tcp_step_m: float = 0.01
    max_tcp_rotation_rad: float = 0.1
    command_timeout_s: float = 15.0
    position_tolerance_rad: float = 0.001

    @classmethod
    def parse(cls, cfg):
        """Reject unknown or invalid values instead of silently enabling motion."""
        cfg = dict(cfg)
        cfg["joint_names"] = tuple(cfg["joint_names"])
        settings = cls(**cfg)
        if (len(settings.joint_names) != 6 or len(set(settings.joint_names)) != 6
                or any(not isinstance(n, str) or not n for n in settings.joint_names)):
            raise ValueError("six unique joint_names required")
        if type(settings.motion_enabled) is not bool:
            raise ValueError("motion_enabled must be a boolean")
        if settings.transport not in {"http", "sdk"}:
            raise ValueError("transport must be http or sdk")
        if not settings.expected_model or not settings.base_frame:
            raise ValueError("expected_model and base_frame are required")
        for name in cls.__dataclass_fields__:
            v = getattr(settings, name)
            if name.endswith(("_s", "_s2", "_rad", "_m", "_hz")):
                if type(v) not in (int, float) or not math.isfinite(v) or v <= 0:
                    raise ValueError(f"{name} must be finite and positive")
        if not 1 <= settings.poll_hz <= 30:
            raise ValueError("poll_hz must be between 1 and 30")
        if settings.joint_speed_rad_s > 0.2 or settings.max_joint_step_rad > 0.2:
            raise ValueError("commissioning speed/step may not exceed 0.2")
        return settings


@dataclass(frozen=True)
class Sample:
    positions: tuple
    speeds: tuple
    tcp: tuple
    mode: str
    safety: str
    exec_id: int
    began: float
    ended: float


class Arm:
    """Own one outstanding target. Faults latch until a new activation."""

    def __init__(self, settings, reader, writer, clock=time.monotonic):
        self.settings, self.reader, self.writer = settings, reader, writer
        self.clock = clock
        self.lock = threading.RLock()
        self.active = False
        self.fault = ""
        self.sample = None
        self.pending = None
        self.stop_requested = False
        self.minimum = self.maximum = None
        self.tcp_offset = None
        self.last_result = "not commanded"

    def connect(self):
        """Check identity and physical limits without modifying the controller."""
        s = self.settings
        if s.robot not in self.reader.call("getRobotNames"):
            raise ValueError("configured robot is not present")
        if self.reader.call("RobotConfig.getRobotType") != s.expected_model:
            raise ValueError("controller model does not match deployment")
        self.minimum = vector(self.reader.call("RobotConfig.getJointMinPositions"), 6, "minimum")
        self.maximum = vector(self.reader.call("RobotConfig.getJointMaxPositions"), 6, "maximum")
        if any(a >= b for a, b in zip(self.minimum, self.maximum)):
            raise ValueError("invalid controller joint limits")
        self.tcp_offset = vector(self.reader.call("RobotConfig.getTcpOffset"), 6, "TCP offset")
        self.refresh()

    def activate(self):
        """Validate a fresh sample and enable callbacks; send no motion RPC."""
        with self.lock:
            if self.pending is not None:
                raise RuntimeError("unresolved previous motion; cannot activate")
            self.refresh()
            self.fault = ""
            self.active = True

    def refresh(self):
        """Read actual feedback; never publish stale or partially read samples."""
        began = self.clock()
        r = self.reader.call
        sample = Sample(
            vector(r("RobotState.getJointPositions"), 6, "positions"),
            vector(r("RobotState.getJointSpeeds"), 6, "speeds"),
            vector(r("RobotState.getTcpPose"), 6, "TCP pose"),
            r("RobotState.getRobotModeType"), r("RobotState.getSafetyModeType"),
            r("MotionControl.getExecId"), began, self.clock())
        if (not isinstance(sample.mode, str) or not isinstance(sample.safety, str)
                or type(sample.exec_id) is not int or sample.exec_id < -1):
            raise RpcError("invalid controller state")
        if sample.ended - began > self.settings.max_sample_age_s:
            raise RpcError("state acquisition exceeded freshness limit")
        with self.lock:
            if self.sample is None or sample.began >= self.sample.began:
                self.sample = sample
                self._monitor(sample)
        return sample

    def _monitor(self, sample):
        """Complete only at a measured stationary target; fault on unsafe state/timeout."""
        if self.pending is None:
            return
        target, issued, count = self.pending
        if sample.began < issued:
            return
        if self.stop_requested:
            stopped = sample.exec_id == -1 and max(abs(v) for v in sample.speeds) < 0.003
            count = count + 1 if stopped else 0
            if count >= 3:
                self.pending = None
                self.last_result = "stopped: fresh feedback confirms stationary joints"
            else:
                self.pending = (target, issued, count)
            return
        if sample.mode != "Running" or sample.safety != "Normal":
            self.halt("controller left Running/Normal during motion")
            return
        if self.clock() - issued > self.settings.command_timeout_s:
            self.halt("motion completion timeout")
            return
        reached = (sample.exec_id == -1 and max(abs(a-b) for a,b in
                   zip(sample.positions, target)) <= self.settings.position_tolerance_rad
                   and max(abs(v) for v in sample.speeds) < 0.003)
        count = count + 1 if reached else 0
        if count >= 3:
            self.pending = None
            self.last_result = "completed: measured target reached"
        else:
            self.pending = (target, issued, count)

    def _ready(self):
        """Require explicit enablement, fresh feedback, stationary robot and idle queue."""
        if not self.settings.motion_enabled or not self.active or self.fault:
            raise ValueError(f"motion unavailable: {self.fault or 'disabled/inactive'}")
        if self.pending is not None:
            raise ValueError("another target is in progress")
        self.refresh()
        sample = self.sample
        if self.clock() - sample.began > self.settings.max_sample_age_s:
            raise ValueError("feedback is stale")
        if sample.mode != "Running" or sample.safety != "Normal":
            raise ValueError("controller must be Running with Normal safety")
        if sample.exec_id != -1 or max(abs(v) for v in sample.speeds) >= 0.003:
            raise ValueError("controller must be stationary with an empty motion queue")
        if self.reader.call("RuntimeMachine.getStatus") != "Running":
            raise ValueError("controller runtime must be Running; prepare it onsite")
        offset = vector(self.reader.call("RobotConfig.getTcpOffset"), 6, "TCP offset")
        if any(abs(a-b) > 1e-9 for a,b in zip(offset, self.tcp_offset)):
            raise ValueError("TCP offset changed; reload model and reactivate")

    def _validate_target(self, target):
        """Use controller ordering/limits and bound movement from measured joints."""
        target = vector(target, 6, "target")
        for q, low, high, actual in zip(target, self.minimum, self.maximum, self.sample.positions):
            if q < low or q > high:
                raise ValueError("target exceeds controller joint limits")
            if abs(q-actual) > self.settings.max_joint_step_rad:
                raise ValueError("target exceeds configured maximum joint step")
        return target

    def _send(self, target):
        """Send once; a lost reply leaves motion uncertain and triggers stop/fault."""
        target = self._validate_target(target)
        if max(abs(a-b) for a,b in zip(target,self.sample.positions)) <= self.settings.position_tolerance_rad:
            self.last_result = "already at target; no motion sent"
            return
        self.pending = (target, self.clock(), 0)
        self.stop_requested = False
        try:
            self.writer.command("MotionControl.moveJoint", [list(target),
                self.settings.joint_accel_rad_s2, self.settings.joint_speed_rad_s, 0, 0])
        except Exception as exc:
            self.halt(f"motion RPC uncertain or rejected: {exc}")
            raise
        self.last_result = "accepted; waiting for measured completion"

    def joints(self, names, positions, velocity=(), effort=(), stamp_s=None):
        """Accept all six named positions; reject unsupported fields and stale stamps."""
        if not self.lock.acquire(blocking=False):
            raise ValueError("controller is busy; command was not queued")
        try:
            if (len(names) != 6 or len(set(names)) != 6
                    or set(names) != set(self.settings.joint_names)):
                raise ValueError("exactly the six configured joint names are required")
            values = vector(positions, 6, "joint positions")
            if len(velocity) or len(effort):
                raise ValueError("velocity and effort commands are unsupported")
            if stamp_s is not None and not 0 <= time.time()-stamp_s <= 1.0:
                raise ValueError("joint command timestamp is stale or in the future")
            self._ready()
            mapping = dict(zip(names, values))
            self._send(tuple(mapping[n] for n in self.settings.joint_names))
        finally:
            self.lock.release()

    def pose(self, position, orientation):
        """Solve TCP IK near measured joints, then execute bounded joint interpolation."""
        if not self.lock.acquire(blocking=False):
            raise ValueError("controller is busy; command was not queued")
        try:
            xyz = vector(position, 3, "position")
            quat = normalized_quaternion(orientation)
            self._ready()
            if math.dist(xyz, self.sample.tcp[:3]) > self.settings.max_tcp_step_m:
                raise ValueError("TCP displacement exceeds configured step")
            actual_quat = quaternion(self.sample.tcp[3:])
            angle = 2*math.acos(min(1, abs(sum(a*b for a,b in zip(quat,actual_quat)))))
            if angle > self.settings.max_tcp_rotation_rad:
                raise ValueError("TCP rotation exceeds configured step")
            result = self.reader.call("RobotAlgorithm.inverseKinematics",
                                      [list(self.sample.positions), list(xyz + rpy(quat))])
            if (not isinstance(result, list) or len(result) != 2
                    or type(result[1]) is not int or result[1] != 0):
                raise ValueError("controller inverse kinematics failed")
            self._send(result[0])
        finally:
            self.lock.release()

    def halt(self, reason):
        """Latch fault and stop only owned/uncertain motion; failed stop stays unresolved."""
        with self.lock:
            if not self.stop_requested:
                self.fault = str(reason)
            if self.pending is not None and not self.stop_requested:
                self.stop_requested = True
                self.pending = (self.pending[0], self.clock(), 0)
                try:
                    self.writer.command("MotionControl.stopJoint", [self.settings.stop_accel_rad_s2])
                except Exception as exc:
                    self.fault += f"; STOP NOT CONFIRMED: {exc}"
                    raise RpcError(self.fault) from exc
                # A successful stop RPC is not evidence of physical stopping.
                self.last_result = "stop accepted; physical stop requires fresh feedback"

    def deactivate(self):
        """Block further commands before requesting an owned-motion stop."""
        with self.lock:
            self.active = False
            self.halt("deactivated")
        deadline = time.monotonic() + 5
        while self.pending is not None and time.monotonic() < deadline:
            self.refresh()
            time.sleep(0.05)
        if self.pending is not None:
            raise RpcError("deactivation could not verify physical stopping")
