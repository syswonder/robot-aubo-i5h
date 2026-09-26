"""Pose conversions and calibrated modified-DH kinematics, in metres/radians."""

import math
from .rpc import vector


def quaternion(rpy):
    """Convert fixed-axis XYZ roll/pitch/yaw to ROS x,y,z,w quaternion."""
    r, p, y = vector(rpy, 3, "rpy")
    cr, cp, cy = (math.cos(v / 2) for v in (r, p, y))
    sr, sp, sy = (math.sin(v / 2) for v in (r, p, y))
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def normalized_quaternion(value):
    """Require a near-unit quaternion; reject invalid orientation inputs."""
    q = vector(value, 4, "quaternion")
    norm = math.sqrt(sum(v*v for v in q))
    if abs(norm - 1) > 0.001:
        raise ValueError("quaternion norm must be within 0.001 of one")
    return tuple(v / norm for v in q)


def rpy(value):
    """Convert ROS x,y,z,w quaternion into fixed-axis XYZ Euler angles."""
    x, y, z, w = normalized_quaternion(value)
    return (math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y)),
            math.asin(max(-1.0, min(1.0, 2*(w*y-z*x)))),
            math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z)))


def multiply(a, b):
    return [[sum(a[i][k]*b[k][j] for k in range(4))
             for j in range(4)] for i in range(4)]


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def mdh(a, alpha, d, theta):
    """Return Tx(a) Rx(alpha) Rz(theta) Tz(d) homogeneous transform."""
    ca, sa, ct, st = math.cos(alpha), math.sin(alpha), math.cos(theta), math.sin(theta)
    return [[ct, -st, 0, a], [ca*st, ca*ct, -sa, -d*sa],
            [sa*st, sa*ct, ca, d*ca], [0, 0, 0, 1]]


def matrix_rpy(t):
    return (math.atan2(t[2][1], t[2][2]),
            math.atan2(-t[2][0], math.hypot(t[0][0], t[1][0])),
            math.atan2(t[1][0], t[0][0]))


def forward(params, joints):
    """Compute flange pose; nonzero beta needs a separately verified convention."""
    q = vector(joints, 6, "joints")
    for key in ("a", "alpha", "d", "theta", "beta"):
        vector(params[key], 6, key)
    if any(abs(v) > 1e-12 for v in params["beta"]):
        raise ValueError("nonzero DH beta is not supported")
    t = identity()
    for i in range(6):
        t = multiply(t, mdh(params["a"][i], params["alpha"][i],
                            params["d"][i], params["theta"][i] + q[i]))
    return tuple(t[i][3] for i in range(3)) + matrix_rpy(t)
