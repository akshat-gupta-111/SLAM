"""
fusion_manager.py - time handling and sensor policy around the EKF.

Still NO ROS imports. This layer answers the questions the math core ignores:

  1. Sensors deliver data LATE and OUT OF ORDER. rf2o publishes a velocity
     ~50-100 ms after the scan it was computed from, while the IMU arrives
     almost instantly. If we fused in arrival order, the lidar velocity would
     be applied to the wrong moment. We fuse in MEASUREMENT-TIME order:
     keep a short history of states, and when late data arrives, rewind to
     just before it and replay everything after it.

  2. A bad sensor must not poison the filter (gating), and a filter that
     rejected too much must be able to recover (gate recovery).

  3. If every velocity sensor dies, the robot must not "drive away"
     (velocity watchdog).
"""
from __future__ import annotations

import bisect
import logging
from dataclasses import dataclass
from typing import Callable, Dict, List, NamedTuple, Optional, Tuple

import numpy as np

from .ekf_core import PlanarEKF


@dataclass
class Measurement:
    """One sensor reading, already converted to the robot body frame.

    stamp is the time the sensor OBSERVED the world (header.stamp), not the
    time the message reached us.
    """
    sensor: str
    stamp: float                      # seconds
    z: np.ndarray                     # measured values
    R: np.ndarray                     # measurement noise covariance
    H: np.ndarray                     # maps state -> expected measurement
    observes_velocity: bool = False   # True if it constrains vx/vy (feeds watchdog)
    angle_rows: Tuple[int, ...] = ()
    gate_chi2: float = 0.0
    # --- filled in by the manager ---
    r_scale: float = 1.0              # R inflation chosen on first processing
    status: str = "pending"
    nis: float = float("nan")


@dataclass
class SensorStats:
    accepted: int = 0
    rejected: int = 0
    too_old: int = 0
    consecutive_rejects: int = 0
    last_stamp: Optional[float] = None
    rate_hz: float = 0.0
    nis_mean: float = float("nan")   # moving average over ACCEPTED updates


class Snapshot(NamedTuple):
    """Everything needed to rewind the filter to a past instant."""
    t: float
    x: np.ndarray
    P: np.ndarray
    last_vel_t: float
    in_dropout: bool


class FusionManager:
    def __init__(self, ekf: PlanarEKF, history_window_s: float = 2.0,
                 max_consecutive_rejects: int = 20,
                 recovery_inflation: float = 25.0,
                 on_event: Optional[Callable[[str, str], None]] = None):
        self.ekf = ekf
        self.history_window_s = history_window_s
        self.max_consecutive_rejects = max_consecutive_rejects
        self.recovery_inflation = recovery_inflation
        self.on_event = on_event          # callback(level, message)

        self.initialized = False
        self.t = 0.0                      # time of the current filter state
        self.last_vel_t = 0.0             # last accepted linear-velocity update
        self.in_dropout = False           # True while velocity assumed "stopped"
        self.base: Optional[Snapshot] = None   # state just before the log starts
        # Three parallel lists, sorted by measurement time:
        self._log: List[Measurement] = []
        self._stamps: List[float] = []
        self._snaps: List[Snapshot] = []  # _snaps[i] = state AFTER _log[i]
        self.stats: Dict[str, SensorStats] = {}

    # ------------------------------------------------------------------
    def _event(self, level: str, msg: str) -> None:
        if self.on_event:
            self.on_event(level, msg)
        else:
            logging.getLogger("ekf").log(
                {"info": 20, "warning": 30, "error": 40}.get(level, 20), msg)

    def _snapshot(self) -> Snapshot:
        return Snapshot(self.t, self.ekf.x.copy(), self.ekf.P.copy(),
                        self.last_vel_t, self.in_dropout)

    def _restore(self, s: Snapshot) -> None:
        self.t = s.t
        self.ekf.x, self.ekf.P = s.x.copy(), s.P.copy()
        self.last_vel_t, self.in_dropout = s.last_vel_t, s.in_dropout

    def initialize(self, stamp: float) -> None:
        self.ekf.reset()
        self.t = self.last_vel_t = stamp
        self.in_dropout = False
        self._log, self._stamps, self._snaps = [], [], []
        self.base = self._snapshot()
        self.initialized = True

    def reset(self) -> None:
        """Forget everything; re-initialises on the next measurement."""
        self.initialized = False
        self.stats.clear()

    # ------------------------------------------------------------------
    # PUBLIC: feed one measurement
    # ------------------------------------------------------------------
    def add(self, m: Measurement) -> str:
        """Insert a measurement in time order. Returns its status string."""
        st = self.stats.setdefault(m.sensor, SensorStats())
        if not self.initialized:
            self.initialize(m.stamp)

        # Older than anything we can still rewind to -> cannot be used.
        if m.stamp < self.base.t:
            st.too_old += 1
            return "too_old"

        idx = bisect.bisect_right(self._stamps, m.stamp)
        is_latest = idx == len(self._log)
        self._log.insert(idx, m)
        self._stamps.insert(idx, m.stamp)
        self._snaps.insert(idx, None)     # placeholder, filled below

        if not is_latest:
            # LATE measurement: rewind to the state just before it ...
            self._restore(self._snaps[idx - 1] if idx > 0 else self.base)
        # ... then (re)process from idx to the end. For the normal in-order
        # case that is just this one measurement.
        for j in range(idx, len(self._log)):
            self._step(self._log[j], first=(j == idx))
            self._snaps[j] = self._snapshot()

        self._prune()
        if not self.ekf.healthy():
            self._event("error", "Filter state became non-finite; re-initialising.")
            t = self.t
            self.initialize(t)
        return m.status

    # ------------------------------------------------------------------
    # INTERNAL: process one measurement at its own timestamp
    # ------------------------------------------------------------------
    def _step(self, m: Measurement, first: bool) -> None:
        ekf, prm = self.ekf, self.ekf.params
        st = self.stats[m.sensor]

        # Velocity watchdog: the IMU cannot observe linear velocity, so if
        # rf2o/VO have been silent too long we must not keep coasting.
        t_exp = self.last_vel_t + prm.velocity_timeout_s
        if not self.in_dropout and m.stamp > t_exp:
            if t_exp > self.t:
                ekf.predict(t_exp - self.t)       # coast until expiry ...
                self.t = t_exp
            ekf.x, ekf.P = ekf.assume_stopped(ekf.x, ekf.P, prm)   # ... then stop
            self.in_dropout = True
            if first:
                self._event("warning", "No linear-velocity sensor for "
                            f"{prm.velocity_timeout_s:.1f}s - assuming robot stopped.")

        dt = m.stamp - self.t
        if dt > 0.0:
            ekf.predict(dt)
            self.t = m.stamp

        # Gate recovery: a filter that rejects a sensor forever has probably
        # diverged from reality (not the sensor from the filter). After N
        # consecutive rejections, let one measurement in with inflated noise;
        # the state moves partway toward it and the gate re-opens naturally.
        if first and st.consecutive_rejects >= self.max_consecutive_rejects:
            m.r_scale = self.recovery_inflation
            self._event("warning", f"{m.sensor}: rejected "
                        f"{st.consecutive_rejects} in a row - forcing a soft re-entry.")
        res = ekf.update(m.z, m.R * m.r_scale, m.H, m.angle_rows, m.gate_chi2)
        m.status = "accepted" if res.accepted else "rejected"
        m.nis = res.nis

        if res.accepted and m.observes_velocity:
            self.last_vel_t = m.stamp
            self.in_dropout = False

        # Statistics only on first processing, so replays don't double-count.
        if first:
            if res.accepted:
                st.accepted += 1
                st.consecutive_rejects = 0
                st.nis_mean = (res.nis if np.isnan(st.nis_mean)
                               else 0.98 * st.nis_mean + 0.02 * res.nis)
            else:
                st.rejected += 1
                st.consecutive_rejects += 1
            if st.last_stamp is not None and m.stamp > st.last_stamp:
                inst = 1.0 / (m.stamp - st.last_stamp)
                st.rate_hz = inst if st.rate_hz == 0.0 else 0.95 * st.rate_hz + 0.05 * inst
            st.last_stamp = max(m.stamp, st.last_stamp or m.stamp)

    def _prune(self) -> None:
        """Drop history older than the window; the last dropped state becomes
        the new rewind base."""
        horizon = self.t - self.history_window_s
        n = bisect.bisect_left(self._stamps, horizon)
        if n > 0:
            self.base = self._snaps[n - 1]
            del self._log[:n], self._stamps[:n], self._snaps[:n]

    # ------------------------------------------------------------------
    # PUBLIC: read the estimate at an arbitrary time (for the output timer)
    # ------------------------------------------------------------------
    def state_at(self, t: float, max_extrap_s: float) -> Tuple[np.ndarray, np.ndarray]:
        """Estimate at time t WITHOUT modifying the filter.

        Extrapolates forward (bounded by max_extrap_s) so the published
        odometry is smooth at the output rate instead of stair-stepping at
        the sensor rate. Applies the velocity watchdog to the copy as well,
        so a dead lidar shows up in the output even before the next IMU tick.
        """
        prm = self.ekf.params
        x, P = self.ekf.x, self.ekf.P
        if not self.in_dropout and t > self.last_vel_t + prm.velocity_timeout_s:
            x, P = PlanarEKF.assume_stopped(x, P, prm)
        dt = min(max(t - self.t, 0.0), max_extrap_s)
        return PlanarEKF.propagate(x, P, dt, prm)

    def velocity_stale(self, t: float) -> bool:
        return self.in_dropout or t > self.last_vel_t + self.ekf.params.velocity_timeout_s
