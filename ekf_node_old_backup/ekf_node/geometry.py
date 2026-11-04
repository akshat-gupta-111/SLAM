"""
geometry.py - tiny geometry helpers (numpy only, no ROS).

Everything here is a pure function, so it is trivially unit-testable.
"""
import math

import numpy as np


def wrap_angle(a: float) -> float:
    """Map any angle (rad) into [-pi, pi).

    Why: yaw is a *circular* quantity. 3.14 and -3.14 are neighbours, but a
    naive subtraction says they are 6.28 apart. Every time we subtract two
    angles (a filter residual) or store one (the state) we wrap it.
    """
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quat(yaw: float):
    """Quaternion (x, y, z, w) for a pure rotation about Z (flat-floor robot)."""
    return 0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw)


def quat_to_rot(x: float, y: float, z: float, w: float) -> np.ndarray:
    """3x3 rotation matrix from a quaternion.

    A quaternion that is all zeros is invalid (some drivers publish it when
    "no orientation"); we treat it as identity instead of dividing by zero.
    """
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def twist_to_base(R: np.ndarray, t: np.ndarray, v_s: np.ndarray, w_s: np.ndarray):
    """Re-express a sensor's twist as the twist of the robot's base_link origin.

    A sensor bolted at position `t` (in base_link) with orientation `R`
    (sensor->base rotation) reports the velocity of ITS OWN origin:

        v_sensor_point = v_base_origin + w x t

    so   v_base = R @ v_s  -  w_b x t        (w_b = R @ w_s)

    The `w x t` "lever-arm" term matters: a camera 15 cm from the rotation
    centre sees 0.15 * w of sideways speed while the robot merely spins on
    the spot. Ignoring it makes a spinning robot look like it is translating.
    Angular velocity is a free vector - it only needs rotating, no lever arm.
    """
    w_b = R @ w_s
    v_b = R @ v_s - np.cross(w_b, t)
    return v_b, w_b
