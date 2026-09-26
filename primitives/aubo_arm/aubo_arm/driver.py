"""ROS 2 arm feedback and commands registered with Robonix Atlas."""

import json
import logging
import os
from pathlib import Path
import subprocess
import tempfile
import threading
import time

from robonix_api import Err, Ok, Primitive
from robonix_api.ros import RosBackend

from .control import Arm, Settings
from .kinematics import quaternion
from .rpc import Client


log = logging.getLogger("aubo_arm")
provider = Primitive(id="aubo_arm", namespace="robonix/primitive/arm")
arm = None
settings = None
worker = None
model_process = None
stop_event = threading.Event()
entities = []
status_path = Path(os.environ.get("AUBO_STATUS_FILE", "/workspace/.runtime/arm-status.json"))


def status():
    """Persist observable progress locally; include freshness, fault and completion."""
    with arm.lock:
        sample = arm.sample
        doc = {"time": time.time(), "active": arm.active,
           "motion_enabled": settings.motion_enabled, "fault": arm.fault,
           "pending": arm.pending is not None, "last_result": arm.last_result,
           "sample_age_s": None if sample is None else time.monotonic()-sample.began,
           "positions": None if sample is None else sample.positions}
    status_path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(mode="w", dir=status_path.parent, delete=False) as f:
        json.dump(doc, f, indent=2)
        temporary = Path(f.name)
    temporary.replace(status_path)


def report_status():
    """Keep feedback alive on a status-file error, but latch and stop owned motion."""
    try:
        status()
    except Exception as exc:
        log.error("cannot persist arm status: %s", exc)
        try:
            arm.halt("local status unavailable")
        except Exception as stop_error:
            log.critical("%s", stop_error)


def on_joints(msg):
    """Validate a stamped joint target; never queue callback failures for replay."""
    try:
        if msg.header.frame_id and msg.header.frame_id != settings.base_frame:
            raise ValueError("joint command frame must be empty or base_link")
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        if stamp <= 0:
            raise ValueError("joint command requires a fresh wall-clock timestamp")
        arm.joints(list(msg.name), list(msg.position), list(msg.velocity), list(msg.effort), stamp)
    except Exception as exc:
        log.error("joint command rejected/failed: %s", exc)
    report_status()


def on_pose(msg):
    """Interpret the v1 unstamped pose in the documented controller base frame."""
    try:
        arm.pose([msg.position.x, msg.position.y, msg.position.z],
                 [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    except Exception as exc:
        log.error("pose command rejected/failed: %s", exc)
    report_status()


def poll(joint_pub, pose_pub):
    """Publish only complete fresh samples; latch and stop on acquisition failure."""
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState
    previous_error = ""
    while not stop_event.is_set():
        began = time.monotonic()
        try:
            sample = arm.refresh()
            if model_process is not None and model_process.poll() is not None:
                raise RuntimeError("robot_state_publisher exited")
            msg = JointState()
            msg.header.stamp = RosBackend.get().node.get_clock().now().to_msg()
            msg.header.frame_id = settings.base_frame
            msg.name = list(settings.joint_names)
            msg.position, msg.velocity = list(sample.positions), list(sample.speeds)
            joint_pub.publish(msg)
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = sample.tcp[:3]
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quaternion(sample.tcp[3:])
            pose_pub.publish(pose)
            previous_error = ""
        except Exception as exc:
            try:
                arm.halt(f"feedback unavailable: {exc}")
            except Exception as stop_error:
                log.critical("%s", stop_error)
            if str(exc) != previous_error:
                log.error("feedback failed; motion requires reactivation: %s", exc)
                previous_error = str(exc)
        report_status()
        stop_event.wait(max(0, 1/settings.poll_hz - (time.monotonic()-began)))


@provider.on_init
def init(cfg):
    """Parse deployment config and establish a read-only identity/limit baseline."""
    global settings, arm
    connections = []
    try:
        settings = Settings.parse(cfg)
        if settings.motion_enabled and os.environ.get("AUBO_ENABLE_MOTION") != "1":
            raise ValueError("motion deployment also requires AUBO_ENABLE_MOTION=1")
        transport = Client
        if settings.transport == "sdk":
            from .sdk import SdkClient
            transport = SdkClient
        reader = transport(settings.endpoint, settings.robot, settings.rpc_timeout_s)
        connections.append(reader)
        writer = transport(settings.endpoint, settings.robot, settings.rpc_timeout_s,
                        motion=settings.motion_enabled)
        connections.append(writer)
        arm = Arm(settings, reader, writer)
        arm.connect()
    except Exception as exc:
        for connection in connections:
            if hasattr(connection, "close"):
                try:
                    connection.close()
                except Exception as cleanup_error:
                    log.error("connection cleanup failed: %s", cleanup_error)
        arm = None
        return Err(str(exc))
    return Ok()


@provider.on_activate
def activate():
    """Create feedback, TF and optional command subscriptions after INIT succeeds."""
    global worker, model_process
    from geometry_msgs.msg import Pose
    from sensor_msgs.msg import JointState
    from rclpy.duration import Duration
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    try:
        arm.activate()
        stop_event.clear()
        joint_pub = provider.create_publisher("robonix/primitive/arm/joint_states",
            topic="/aubo/joint_states", msg_type=JointState, qos="reliable")
        entities.append(("publisher", joint_pub))
        pose_pub = provider.create_publisher("robonix/primitive/arm/end_pose",
            topic="/aubo/end_pose", msg_type=Pose, qos="reliable")
        entities.append(("publisher", pose_pub))
        if settings.motion_enabled:
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE, lifespan=Duration(seconds=1))
            for name, msg_type, callback in (("joint_command", JointState, on_joints),
                                              ("pos_command", Pose, on_pose)):
                contract, topic = f"robonix/primitive/arm/{name}", f"/aubo/{name}"
                sub = provider.create_subscription(contract, topic=topic,
                    msg_type=msg_type, callback=callback, qos=qos, declare=False)
                entities.append(("subscription", sub))
                # A command sink is required, so declaration failure must fail activation.
                provider.declare_ros2_topic(contract, topic, qos="reliable")
        model = os.environ.get("AUBO_URDF", "/workspace/model/aubo_i5H.urdf")
        model_process = subprocess.Popen([
            "ros2", "run", "robot_state_publisher", "robot_state_publisher", model,
            "--ros-args", "-r", "joint_states:=/aubo/joint_states"])
        worker = threading.Thread(target=poll, args=(joint_pub,pose_pub), daemon=True)
        worker.start()
    except Exception as exc:
        deactivate()
        return Err(str(exc))
    return Ok()


@provider.on_deactivate
def deactivate():
    """Stop owned motion, stop telemetry worker, and release ROS/model resources."""
    global worker, model_process
    errors = []
    if arm is not None:
        try:
            arm.deactivate()
        except Exception as exc:
            errors.append(str(exc))
    stop_event.set()
    if worker is not None:
        worker.join(timeout=8)
        if worker.is_alive():
            errors.append("feedback worker did not stop")
        else:
            worker = None
    if model_process is not None:
        model_process.terminate()
        try:
            model_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            model_process.kill()
            model_process.wait(timeout=2)
        model_process = None
    if worker is None and entities:
        node = RosBackend.get().node
        for kind, entity in entities:
            getattr(node, f"destroy_{kind}")(entity)
        entities.clear()
    if arm is not None:
        report_status()
    return Err("; ".join(errors)) if errors else Ok()


@provider.on_shutdown
def shutdown():
    """Deactivate first, then close SDK connections without powering the arm off."""
    result = deactivate()
    if arm is not None:
        for transport in (arm.reader, arm.writer):
            if hasattr(transport,"close"):
                transport.close()
    return result


if __name__ == "__main__":
    provider.run()
