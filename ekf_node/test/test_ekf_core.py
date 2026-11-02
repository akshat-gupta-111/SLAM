"""Unit tests for geometry + EKF core. Run:  python3 -m pytest test/ -v"""
import math

import numpy as np
import pytest

from ekf_node.ekf_core import (BGZ, H_GYRO, H_TWIST, N, VX, WZ, X, Y, YAW,
                               EKFParams, PlanarEKF)
from ekf_node.geometry import quat_to_rot, twist_to_base, wrap_angle, yaw_to_quat


# ----------------------------- geometry ----------------------------------
def test_wrap_angle_range_and_continuity():
    for a in np.linspace(-20, 20, 401):
        w = wrap_angle(a)
        assert -math.pi <= w < math.pi
        assert abs(math.sin(w) - math.sin(a)) < 1e-9
    assert wrap_angle(math.pi + 0.1) == pytest.approx(-math.pi + 0.1)


def test_quat_roundtrip_yaw():
    for yaw in (-3.0, -0.5, 0.0, 1.2, 3.1):
        R = quat_to_rot(*yaw_to_quat(yaw))
        assert math.atan2(R[1, 0], R[0, 0]) == pytest.approx(yaw, abs=1e-9)


def test_zero_quaternion_is_identity():
    assert np.allclose(quat_to_rot(0, 0, 0, 0), np.eye(3))


def test_lever_arm_pure_spin():
    """Robot spins in place at w. A sensor 0.1 m ahead sees 0.1*w sideways
    speed. After correction the base must be reported as NOT translating."""
    w, t = 0.8, np.array([0.1, 0.0, 0.0])
    v_s = np.array([0.0, 0.1 * w, 0.0])
    v_b, w_b = twist_to_base(np.eye(3), t, v_s, np.array([0, 0, w]))
    assert np.allclose(v_b, 0.0, atol=1e-12)
    assert w_b[2] == pytest.approx(w)


def test_sensor_yawed_90deg():
    """Sensor x-axis points along base +y. Sensor 'forward' = base 'left'."""
    R = quat_to_rot(*yaw_to_quat(math.pi / 2))
    v_b, _ = twist_to_base(R, np.zeros(3), np.array([1.0, 0, 0]), np.zeros(3))
    assert np.allclose(v_b, [0, 1, 0], atol=1e-12)


# ----------------------------- EKF core ----------------------------------
def numeric_jacobian(x, dt, prm):
    eps, J = 1e-6, np.zeros((N, N))
    for i in range(N):
        d = np.zeros(N)
        d[i] = eps
        fp, _ = PlanarEKF.propagate(x + d, np.eye(N), dt, prm)
        fm, _ = PlanarEKF.propagate(x - d, np.eye(N), dt, prm)
        diff = fp - fm
        diff[YAW] = wrap_angle(diff[YAW])
        J[:, i] = diff / (2 * eps)
    return J


def test_jacobian_matches_numerical():
    """The hand-derived F must equal a finite-difference Jacobian. This is
    THE test that catches sign errors in the motion model."""
    prm = EKFParams()
    rng = np.random.default_rng(1)
    for _ in range(20):
        x = rng.normal(size=N)
        x[YAW] = rng.uniform(-3, 3)
        dt = rng.uniform(0.001, 0.2)
        # Extract F by propagating P=I with Q removed.
        prm0 = EKFParams(q_pos=0, q_yaw=0, q_vel=0, q_wz=0, q_bias=0)
        _, Pn = PlanarEKF.propagate(x, np.eye(N), dt, prm0)
        F_num = numeric_jacobian(x, dt, prm)
        assert np.allclose(Pn, F_num @ F_num.T, atol=1e-6)


def test_straight_line_integration():
    ekf = PlanarEKF()
    ekf.x[VX] = 1.0
    for _ in range(100):
        ekf.predict(0.01)
    assert ekf.x[X] == pytest.approx(1.0, abs=1e-9)
    assert ekf.x[Y] == pytest.approx(0.0, abs=1e-9)


def test_circle_closes():
    """vx=1, wz=1 -> circle of radius 1. After one period we are back home."""
    ekf = PlanarEKF()
    ekf.x[VX], ekf.x[WZ] = 1.0, 1.0
    n = 2000
    for _ in range(n):
        ekf.predict(2 * math.pi / n)
    assert ekf.x[X] == pytest.approx(0.0, abs=1e-3)
    assert ekf.x[Y] == pytest.approx(0.0, abs=1e-3)


def test_update_wraps_yaw_residual():
    ekf = PlanarEKF()
    ekf.x[YAW] = math.pi - 0.01
    H = np.zeros((1, N))
    H[0, YAW] = 1
    ekf.P[YAW, YAW] = 1.0
    ekf.update([-math.pi + 0.01], np.array([[1e-4]]), H, angle_rows=(0,))
    # True gap is 0.02 rad (across the +-pi seam), NOT 6.26.
    assert abs(wrap_angle(ekf.x[YAW] - math.pi)) < 0.02


def test_gate_rejects_outlier_accepts_normal():
    ekf = PlanarEKF()
    R = np.diag([1e-4] * 3)
    ok = ekf.update([0.01, 0.0, 0.0], R, H_TWIST, gate_chi2=16.27)
    bad = ekf.update([5.0, 0.0, 0.0], R, H_TWIST, gate_chi2=16.27)
    assert ok.accepted and not bad.accepted and bad.reason == "gated"


def test_nan_measurement_rejected_state_untouched():
    ekf = PlanarEKF()
    x0, P0 = ekf.x.copy(), ekf.P.copy()
    r = ekf.update([float("nan"), 0, 0], np.eye(3), H_TWIST)
    assert not r.accepted and r.reason == "non_finite"
    assert np.array_equal(x0, ekf.x) and np.array_equal(P0, ekf.P)


def test_covariance_stays_symmetric_psd_over_many_updates():
    ekf = PlanarEKF()
    rng = np.random.default_rng(0)
    R3, R1 = np.diag([1e-4, 1e-4, 1e-3]), np.array([[1e-6]])
    for i in range(20000):
        ekf.predict(0.005)
        ekf.update([0.3 + rng.normal(0, 1e-2), rng.normal(0, 1e-2),
                    0.2 + rng.normal(0, 3e-2)], R3, H_TWIST)
        ekf.update([0.2 + rng.normal(0, 1e-3)], R1, H_GYRO)
    P = ekf.P
    assert np.allclose(P, P.T, atol=1e-15)
    assert np.linalg.eigvalsh(P).min() > -1e-12


def test_gyro_bias_is_learned_from_twist_sensor():
    """Gyro reads wz+0.02 while the lidar says wz=0 -> bias state -> 0.02."""
    ekf = PlanarEKF()
    rng = np.random.default_rng(3)
    for i in range(6000):
        ekf.predict(0.005)
        ekf.update([0.02 + rng.normal(0, 1e-3)], np.array([[1e-6]]), H_GYRO)
        if i % 20 == 0:
            ekf.update([rng.normal(0, 1e-2), rng.normal(0, 1e-2),
                        rng.normal(0, 2e-2)], np.diag([1e-4, 1e-4, 4e-4]), H_TWIST)
    assert ekf.x[BGZ] == pytest.approx(0.02, abs=2e-3)
    assert abs(ekf.x[WZ]) < 2e-3


def test_long_silence_stops_the_robot():
    ekf = PlanarEKF()
    ekf.x[VX] = 1.0
    ekf.predict(5.0)           # > max_predict_dt
    assert ekf.x[VX] == 0.0 and ekf.x[X] <= ekf.params.max_predict_dt + 1e-9
