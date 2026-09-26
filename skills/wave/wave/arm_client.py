"""ROS 2 consumer for the Aubo arm primitive joint contracts."""
from __future__ import annotations

import threading
import math
import time
from typing import Any

from robonix_api import ATLAS
from robonix_api.atlas_types import Kind, Transport
from robonix_api.ros import RosBackend, resolve_msg_type


def _joint_state_type():
    return resolve_msg_type("sensor_msgs/JointState")


class ArmRosClient:
    """Publish joint commands and read joint states via Atlas-discovered ROS topics."""

    def __init__(self, skill: Any, *, arm_provider_id: str) -> None:
        self._skill = skill
        self._arm_provider_id = arm_provider_id
        self._lock = threading.Lock()
        self._latest: Any | None = None
        self._cmd_pub = None
        self._subscription = None
        self._received = 0.0
        self._sequence = 0
        self.cancel = threading.Event()

    def connect(self) -> None:
        states_cap = ATLAS.find_unique_capability(
            contract_id="robonix/primitive/arm/joint_states",
            transport=Transport.ROS2,
            provider_kind=Kind.PRIMITIVE,
            provider_id=self._arm_provider_id,
        )
        cmd_cap = ATLAS.find_unique_capability(
            contract_id="robonix/primitive/arm/joint_command",
            transport=Transport.ROS2,
            provider_kind=Kind.PRIMITIVE,
            provider_id=self._arm_provider_id,
        )
        states_ch = self._skill.connect_capability(
            states_cap,
            "robonix/primitive/arm/joint_states",
            Transport.ROS2,
        )
        cmd_ch = self._skill.connect_capability(
            cmd_cap,
            "robonix/primitive/arm/joint_command",
            Transport.ROS2,
        )
        joint_state = _joint_state_type()
        self._subscription = self._skill.create_subscription_from_channel(
            states_ch,
            msg_type=joint_state,
            callback=self._on_joint_state,
        )
        self._cmd_pub = RosBackend.get().create_publisher(
            joint_state,
            cmd_ch.endpoint,
            "reliable",
        )
        self._wait_for_joint_state(timeout_s=15.0)

    def _on_joint_state(self, msg: Any) -> None:
        with self._lock:
            self._latest = msg
            self._received = time.monotonic()
            self._sequence += 1

    def _wait_for_joint_state(self, timeout_s: float) -> None:
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if self._latest is not None and list(getattr(self._latest, "position", []) or []):
                    return
            time.sleep(0.05)
        raise RuntimeError(
            f"timed out waiting for joint_states from arm provider '{self._arm_provider_id}'"
        )

    def snapshot(self) -> tuple[list[str], list[float]]:
        with self._lock:
            if self._latest is None:
                raise RuntimeError("no joint_states received yet")
            if time.monotonic() - self._received > 0.5:
                raise RuntimeError("joint feedback is stale")
            names = list(getattr(self._latest, "name", []) or [])
            positions = [float(v) for v in (getattr(self._latest, "position", []) or [])]
        if len(positions) != 6 or any(not math.isfinite(v) for v in positions):
            raise RuntimeError("joint_states must contain six finite positions")
        if len(names) != len(positions) or len(set(names)) != 6:
            raise RuntimeError("joint_states name/position length mismatch")
        return names, positions

    def command_positions(self, names: list[str], positions: list[float]) -> None:
        if self.cancel.is_set():
            raise RuntimeError("wave cancelled")
        if self._cmd_pub is None:
            raise RuntimeError("arm client is not connected")
        msg = _joint_state_type()()
        msg.header.stamp = RosBackend.get().node.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.name = list(names)
        msg.position = [float(v) for v in positions]
        self._cmd_pub.publish(msg)

    def wait_until_near(
        self,
        target: list[float],
        *,
        tolerance_rad: float,
        timeout_s: float,
    ) -> None:
        deadline = time.monotonic() + timeout_s
        stable, previous_sequence = 0, -1
        while time.monotonic() < deadline:
            if self.cancel.is_set():
                raise RuntimeError("wave cancelled")
            _, current = self.snapshot()
            if len(current) != len(target):
                raise RuntimeError(
                    f"joint count mismatch: state={len(current)} target={len(target)}"
                )
            with self._lock:
                sequence = self._sequence
                speeds = list(self._latest.velocity)
            if sequence != previous_sequence:
                near = all(abs(c-t) <= tolerance_rad for c,t in zip(current,target))
                stopped = len(speeds) == 6 and max(abs(v) for v in speeds) < 0.003
                stable = stable + 1 if near and stopped else 0
                previous_sequence = sequence
                if stable >= 5:
                    return
            time.sleep(0.05)
        raise RuntimeError(
            f"joints did not settle within {timeout_s:.1f}s "
            f"(tolerance {tolerance_rad:.3f} rad)"
        )

    def stop_arm(self):
        """Use the existing Driver DEACTIVATE to stop the arm on cancellation/failure."""
        self.cancel.set()
        import grpc
        import lifecycle_pb2
        import robonix_contracts_pb2_grpc as contracts
        cap = ATLAS.find_unique_capability(contract_id="robonix/lifecycle/driver",
            transport=Transport.GRPC, provider_kind=Kind.PRIMITIVE,
            provider_id=self._arm_provider_id)
        ch = self._skill.connect_capability(cap, "robonix/lifecycle/driver", Transport.GRPC)
        with grpc.insecure_channel(ch.endpoint) as channel:
            response = contracts.RobonixLifecycleDriverStub(channel).Driver(
                lifecycle_pb2.Driver_Request(command=2, config_json="{}"), timeout=12)
        if not response.ok:
            raise RuntimeError(f"arm stop/deactivation failed: {response.error}")

    def close(self):
        """Release this consumer's ROS entities; leave provider registration intact."""
        self.cancel.set()
        node = RosBackend.get().node
        if self._subscription is not None:
            node.destroy_subscription(self._subscription)
            self._subscription = None
        if self._cmd_pub is not None:
            node.destroy_publisher(self._cmd_pub)
            self._cmd_pub = None
