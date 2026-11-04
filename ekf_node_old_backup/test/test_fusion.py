"""Tests for FusionManager + an end-to-end simulation. Run: python3 -m pytest test/ -v"""
import math
import time

import numpy as np
import pytest

from ekf_node.ekf_core import (H_GYRO, H_TWIST, VX, X, Y, YAW, BGZ,
                               EKFParams, PlanarEKF)
from ekf_node.fusion_manager import FusionManager, Measurement
from ekf_node.geometry import wrap_angle

T0 = 1.7e9  # realistic epoch-sized timestamps (tests float precision too)


# ------------------------------------------------------------------ helpers
def twist_m(stamp, v, sensor="rf2o", var=(1e-4, 1e-4, 1e-3), **kw):
    return Measurement(sensor, T0 + stamp, np.array(v, float), np.diag(var),
                       H_TWIST, observes_velocity=True, gate_chi2=16.27, **kw)


def gyro_m(stamp, gz, var=1e-6):
    return Measurement("imu", T0 + stamp, np.array([gz]), np.array([[var]]),
                       H_GYRO, gate_chi2=10.83)


def simulate(duration=60.0, seed=0, bias=0.02, agile=False, noise=1.0,
             rf_lat=0.08, vo_lat=0.05):
    """Ground-truth trajectory + 3 noisy, delayed sensors. Returns
    (truth, list of (arrival_time, Measurement)). `agile` = aggressive
    accelerations (where sensor latency visibly hurts)."""
    rng = np.random.default_rng(seed)
    dt = 1 / 1800
    n = int(duration / dt)
    t = np.arange(n) * dt
    if agile:
        vx = 0.4 + 0.3 * np.sin(1.5 * t)
        vy = 0.05 * np.sin(0.9 * t)
        wz = 1.2 * np.sin(1.0 * t)
    else:
        vx = 0.3 + 0.15 * np.sin(0.5 * t)
        vy = 0.02 * np.sin(0.3 * t)
        wz = 0.4 * np.sin(0.2 * t) + 0.1
    yaw = np.cumsum(wz) * dt
    xs = np.cumsum(vx * np.cos(yaw) - vy * np.sin(yaw)) * dt
    ys = np.cumsum(vx * np.sin(yaw) + vy * np.cos(yaw)) * dt
    truth = dict(t=t, x=xs, y=ys, yaw=yaw, vx=vx, vy=vy, wz=wz)

    out = []
    sig_rf = tuple(noise * v for v in (0.01, 0.01, 0.03))
    sig_vo = tuple(noise * v for v in (0.03, 0.03, 0.05))
    sig_gz = 1e-3

    def at(ts):
        return min(int(ts / dt), n - 1)

    for ts in np.arange(0.0, duration, 1 / 180):             # IMU 180 Hz, 5 ms latency
        i = at(ts)
        z = wz[i] + bias + rng.normal(0, sig_gz)
        out.append((ts + 0.005, gyro_m(ts, z, var=sig_gz ** 2)))
    for ts in np.arange(0.0, duration, 1 / 10):              # rf2o 10 Hz, latency rf_lat
        i = at(ts)
        z = [vx[i] + rng.normal(0, sig_rf[0]), vy[i] + rng.normal(0, sig_rf[1]),
             wz[i] + rng.normal(0, sig_rf[2])]
        out.append((ts + rf_lat, twist_m(ts, z, "rf2o", np.square(sig_rf))))
    for ts in np.arange(0.0, duration, 1 / 20):              # VO 20 Hz, latency vo_lat
        i = at(ts)
        z = [vx[i] + rng.normal(0, sig_vo[0]), vy[i] + rng.normal(0, sig_vo[1]),
             wz[i] + rng.normal(0, sig_vo[2])]
        out.append((ts + vo_lat, twist_m(ts, z, "vo", np.square(sig_vo))))
    out.sort(key=lambda p: p[0])                              # ARRIVAL order
    return truth, out


# -------------------------------------------------------------- ordering
def test_out_of_order_equals_in_order_bitwise():
    """THE key property: arrival order must not change the answer."""
    rng = np.random.default_rng(5)
    ms = []
    for k in range(200):
        ms.append(gyro_m(k * 0.01, 0.1 + rng.normal(0, 1e-3)))
        if k % 5 == 0:
            ms.append(twist_m(k * 0.01 + 0.0013, [0.3 + rng.normal(0, 1e-2),
                              rng.normal(0, 1e-2), 0.1 + rng.normal(0, 3e-2)]))

    def run(order):
        fm = FusionManager(PlanarEKF(), history_window_s=5.0)
        for m in order:
            fm.add(Measurement(m.sensor, m.stamp, m.z, m.R, m.H, m.observes_velocity,
                               m.angle_rows, m.gate_chi2))
        return fm.ekf.x.copy(), fm.ekf.P.copy()

    xa, Pa = run(sorted(ms, key=lambda m: m.stamp))
    # Make twist measurements arrive ~0.1 s late relative to gyro ones.
    late = sorted(ms, key=lambda m: m.stamp + (0.1 if m.sensor == "rf2o" else 0.0))
    xb, Pb = run(late)
    assert np.array_equal(xa, xb) and np.array_equal(Pa, Pb)


def test_too_old_measurement_is_dropped():
    fm = FusionManager(PlanarEKF(), history_window_s=1.0)
    for k in range(300):
        fm.add(gyro_m(k * 0.01, 0.0))
    assert fm.add(gyro_m(0.5, 0.0)) == "too_old"
    assert fm.stats["imu"].too_old == 1


def test_history_is_bounded():
    fm = FusionManager(PlanarEKF(), history_window_s=1.0)
    for k in range(5000):
        fm.add(gyro_m(k * 0.005, 0.0))
    assert len(fm._log) < 1.0 / 0.005 + 5


def test_replay_does_not_double_count_stats():
    fm = FusionManager(PlanarEKF(), history_window_s=5.0)
    for k in range(50):
        fm.add(gyro_m(k * 0.01, 0.0))
    fm.add(twist_m(0.2, [0, 0, 0]))     # late -> replay of ~30 gyro updates
    assert fm.stats["imu"].accepted == 50
    assert fm.stats["rf2o"].accepted == 1


# ------------------------------------------------------------- robustness
def test_gate_recovery_after_filter_diverged():
    """State is 'wrong' by a lot: sensor is gated repeatedly, then a soft
    re-entry pulls the state back so the gate re-opens."""
    fm = FusionManager(PlanarEKF(), max_consecutive_rejects=5, recovery_inflation=100.0)
    fm.add(twist_m(0.0, [0, 0, 0]))
    fm.ekf.x[VX] = 2.0                     # corrupt: filter believes 2 m/s
    fm.ekf.P[VX, VX] = 1e-8                # ...and is very sure about it
    statuses = [fm.add(twist_m(0.1 * (k + 1), [0.0, 0.0, 0.0])) for k in range(60)]
    assert "rejected" in statuses
    assert statuses[-1] == "accepted"
    assert abs(fm.ekf.x[VX]) < 0.2


def test_velocity_watchdog_stops_drift_when_lidar_dies():
    fm = FusionManager(PlanarEKF())
    for k in range(100):                               # drive at 0.5 m/s for 1 s
        fm.add(gyro_m(k * 0.01, 0.0))
        if k % 10 == 0:
            fm.add(twist_m(k * 0.01, [0.5, 0, 0]))
    x_at_loss = fm.ekf.x[X]
    for k in range(100, 600):                          # 5 s of IMU only
        fm.add(gyro_m(k * 0.01, 0.0))
    assert fm.in_dropout
    assert fm.ekf.x[VX] == 0.0
    assert fm.ekf.x[X] - x_at_loss < 0.5 * 0.6 + 0.05   # coasted <= ~timeout
    assert fm.state_at(T0 + 6.0, 0.2)[0][VX] == 0.0


def test_recovers_after_dropout():
    fm = FusionManager(PlanarEKF())
    fm.add(twist_m(0.0, [0.5, 0, 0]))
    for k in range(1, 300):
        fm.add(gyro_m(k * 0.01, 0.0))
    assert fm.in_dropout
    assert fm.add(twist_m(3.0, [0.5, 0, 0])) == "accepted"
    assert not fm.in_dropout and fm.ekf.x[VX] > 0.3


def test_nonfinite_input_never_corrupts_state():
    fm = FusionManager(PlanarEKF())
    fm.add(twist_m(0.0, [0.1, 0, 0]))
    fm.add(twist_m(0.1, [float("nan"), 0, 0]))
    fm.add(twist_m(0.2, [float("inf"), 0, 0]))
    assert fm.ekf.healthy()
    assert fm.stats["rf2o"].rejected == 2


def test_state_at_does_not_mutate_filter():
    fm = FusionManager(PlanarEKF())
    fm.add(twist_m(0.0, [1.0, 0, 0]))
    fm.add(gyro_m(0.0, 0.0))
    x0 = fm.ekf.x.copy()
    xs, _ = fm.state_at(T0 + 0.1, 0.2)
    assert np.array_equal(x0, fm.ekf.x)
    assert xs[X] > x0[X]


# ---------------------------------------------------------- end-to-end sim
# Process noise matched to the GENTLE simulated motion (true accel ~0.08 m/s^2,
# ~0.08 rad/s^2). The library defaults are deliberately more conservative.
GENTLE_Q = dict(q_vel=0.005, q_wz=0.005)


def run_sim(seed=0, bias=0.02, duration=60.0, **q):
    truth, arrivals = simulate(duration, seed, bias)
    fm = FusionManager(PlanarEKF(EKFParams(**q)), history_window_s=2.0)
    for _, m in arrivals:
        fm.add(Measurement(m.sensor, m.stamp - T0 + 100.0, m.z, m.R, m.H,
                           m.observes_velocity, m.angle_rows, m.gate_chi2))
    return truth, fm


def test_end_to_end_accuracy_with_biased_gyro():
    """A gyro with a 0.02 rad/s (1.1 deg/s) bias would drift ~69 deg in 60 s
    if integrated raw. The filter must learn the bias from lidar/VO."""
    truth, fm = run_sim(**GENTLE_Q)
    x = fm.ekf.x
    yaw_err = abs(wrap_angle(x[YAW] - truth["yaw"][-1]))
    pos_err = math.hypot(x[X] - truth["x"][-1], x[Y] - truth["y"][-1])
    path = np.sum(np.hypot(np.diff(truth["x"]), np.diff(truth["y"])))
    uncorrected = 0.02 * 60.0
    print(f"\n  yaw err {math.degrees(yaw_err):.2f} deg (uncorrected gyro: "
          f"{math.degrees(uncorrected):.0f} deg) | pos err {pos_err*100:.1f} cm over "
          f"{path:.1f} m | learned bias {x[BGZ]:.4f} (true 0.0200)")
    # Residual bias error is limited by lidar wz noise averaged over time
    # (~1 mrad/s here => a few degrees per minute). Demand >10x improvement.
    assert yaw_err < uncorrected / 10
    assert pos_err < 0.02 * path           # < 2 % of distance travelled
    assert x[BGZ] == pytest.approx(0.02, abs=3e-3)


def test_fusion_beats_lidar_only_dead_reckoning():
    truth, arrivals = simulate(60.0, seed=2)
    # Baseline: integrate ONLY rf2o twist at its own rate.
    x = y = yaw = 0.0
    last = None
    for _, m in arrivals:
        if m.sensor != "rf2o":
            continue
        if last is not None:
            dt = m.stamp - last
            vx, vy, wz = m.z
            ym = yaw + 0.5 * wz * dt
            x += (vx * math.cos(ym) - vy * math.sin(ym)) * dt
            y += (vx * math.sin(ym) + vy * math.cos(ym)) * dt
            yaw += wz * dt
        last = m.stamp
    base_err = math.hypot(x - truth["x"][-1], y - truth["y"][-1])
    _, fm = run_sim(seed=2, **GENTLE_Q)
    fus_err = math.hypot(fm.ekf.x[X] - truth["x"][-1], fm.ekf.x[Y] - truth["y"][-1])
    print(f"\n  lidar-only pos err {base_err*100:.1f} cm  vs  fused {fus_err*100:.1f} cm")
    assert fus_err < base_err


def test_filter_is_not_overconfident():
    """Mean NIS should be about the measurement dimension if R and Q are honest.

    NIS >> dim  => OVERCONFIDENT (dangerous: filter ignores real errors)
    NIS << dim  => conservative  (safe, just leaves accuracy on the table)

    rf2o (dim 3) is a slow sensor measured against an honest R, so we expect
    it near 3. The 180 Hz gyro (dim 1) is always somewhat conservative with a
    random-walk yaw-rate model (real motion is smoother than the model
    allows), so for it we only forbid overconfidence.
    """
    _, fm = run_sim(seed=4, **GENTLE_Q)
    nis_gyro = fm.stats["imu"].nis_mean
    nis_rf2o = fm.stats["rf2o"].nis_mean
    print(f"\n  mean NIS  gyro {nis_gyro:.2f} (<=1 is fine)   rf2o {nis_rf2o:.2f} (expect ~3)")
    assert nis_gyro < 2.5
    assert 1.0 < nis_rf2o < 6.0


def test_timestamp_fusion_beats_arrival_time_fusion_when_agile():
    """Aggressive motion + 150 ms lidar latency. Fusing by MEASUREMENT time
    (our design) must beat stamping data with its arrival time, on average
    over several random seeds (single runs are dominated by noise)."""
    def total_err(restamp, seed):
        truth, arrivals = simulate(25.0, seed, agile=True, noise=0.3,
                                   rf_lat=0.15, vo_lat=0.10)
        fm = FusionManager(PlanarEKF(EKFParams(q_vel=0.5, q_wz=2.0)),
                           history_window_s=2.0)
        for arr, m in arrivals:
            stamp = (arr if restamp else m.stamp) - T0 + 100.0
            fm.add(Measurement(m.sensor, stamp, m.z, m.R, m.H, m.observes_velocity,
                               m.angle_rows, m.gate_chi2))
        x = fm.ekf.x
        return math.hypot(x[X] - truth["x"][-1], x[Y] - truth["y"][-1])

    seeds = range(6)
    good = np.mean([total_err(False, s) for s in seeds])
    naive = np.mean([total_err(True, s) for s in seeds])
    print(f"\n  mean pos err: timestamp-correct {good*100:.1f} cm  vs  arrival-stamped {naive*100:.1f} cm")
    assert good < naive


def test_speed_budget():
    """The 180 Hz IMU path must be cheap. Budget generous: 2 ms/update
    (a Jetson Nano CPU is ~3-5x slower than a dev laptop)."""
    fm = FusionManager(PlanarEKF())
    n = 3000
    t0 = time.perf_counter()
    for k in range(n):
        fm.add(gyro_m(k / 180.0, 0.0))
    per = (time.perf_counter() - t0) / n
    print(f"\n  gyro update: {per*1e6:.0f} us each -> {per*180*100:.1f}% of one core at 180 Hz")
    assert per < 2e-3
