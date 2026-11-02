"""
ekf_core.py - the mathematical heart of the filter.

NO ROS IMPORTS HERE. This file is plain numpy so it can be unit-tested on a
laptop and reasoned about in isolation. The ROS node only converts messages
into numbers and calls this module.

State vector (7 states), all in the planar (2D) world:

    index  name  frame   meaning
    -----  ----  ------  ---------------------------------------------
      0    x     odom    position of base_link            [m]
      1    y     odom    position of base_link            [m]
      2    yaw   odom    heading of base_link             [rad]
      3    vx    body    forward velocity (base_link x)   [m/s]
      4    vy    body    sideways velocity (base_link y)  [m/s]
      5    wz    body    yaw rate                         [rad/s]
      6    bgz   -       gyro-z bias (gz_measured = wz + bgz) [rad/s]

Why velocities live in the BODY frame: every sensor we fuse (rf2o, visual
odometry, gyro) natively reports body-frame velocities, so each measurement
model is a trivial "pick these states" matrix - exactly linear, no
linearisation error. Only the motion model (rotating body velocity into the
odom frame to integrate position) is non-linear.

Why a gyro-bias state: the MPU6050's gyro bias drifts with temperature. The
calibration file captured it at one instant. With rf2o/VO also measuring wz,
the filter can keep re-estimating the bias online for free.
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import NamedTuple, Sequence, Tuple

import numpy as np

from .geometry import wrap_angle

# --------------------------------------------------------------------------
# State layout
# --------------------------------------------------------------------------
X, Y, YAW, VX, VY, WZ, BGZ = range(7)
N = 7
STATE_NAMES = ("x", "y", "yaw", "vx", "vy", "wz", "bgz")

# --------------------------------------------------------------------------
# Measurement matrices H (z = H @ state). They are constants because all of
# our sensor models are linear in this state parameterisation.
# --------------------------------------------------------------------------
# Gyro measures the true yaw rate PLUS its bias.
H_GYRO = np.zeros((1, N))
H_GYRO[0, WZ] = 1.0
H_GYRO[0, BGZ] = 1.0
# A body-frame twist sensor (rf2o, visual odometry) measures [vx, vy, wz].
H_TWIST = np.zeros((3, N))
H_TWIST[0, VX] = 1.0
H_TWIST[1, VY] = 1.0
H_TWIST[2, WZ] = 1.0
for _h in (H_GYRO, H_TWIST):
    _h.flags.writeable = False  # shared constants: make accidental edits fail loudly


@dataclass
class EKFParams:
    """All tunable numbers of the filter in one place (loaded from YAML).

    Process noise (q_*) is a *rate*: variance added per second of prediction.
    Over a step of length dt the filter adds q * dt to that state's variance.
    Larger q  => "this quantity may change quickly" => trust measurements more.
    """
    # Unmodelled drift of the integrated pose (wheel-less, so mostly small).
    q_pos: float = 1.0e-6        # (m^2)/s
    q_yaw: float = 1.0e-6        # (rad^2)/s
    # How fast the true velocity can change = acceleration noise density.
    # Rule of thumb: sigma_accel^2. A 0.3 m/s^2 robot -> ~0.1.
    q_vel: float = 0.1           # (m/s)^2 / s
    q_wz: float = 1.0            # (rad/s)^2 / s   (~1 rad/s^2 angular accel)
    # Gyro bias random walk. Tiny: bias changes slowly (thermal).
    q_bias: float = 1.0e-8       # (rad/s)^2 / s

    # Initial uncertainty (variance) of each state at start-up.
    p0_pos: float = 1.0e-6       # we DEFINE the start pose as the origin
    p0_yaw: float = 1.0e-6
    p0_vel: float = 0.1
    p0_wz: float = 0.1
    p0_bias: float = 4.0e-4      # (0.02 rad/s)^2 residual after calibration

    # --- dropout policy (robustness) ---
    # Never coast on a stale velocity longer than this (seconds).
    max_predict_dt: float = 0.5
    # If no sensor that observes linear velocity (rf2o / VO) has been
    # accepted for this long, assume the robot stopped.
    velocity_timeout_s: float = 0.5
    # How unsure we are about velocity after assuming "stopped".
    dropout_vel_std: float = 0.3     # m/s
    dropout_wz_std: float = 0.5      # rad/s

    def q_diag(self) -> np.ndarray:
        return np.array([self.q_pos, self.q_pos, self.q_yaw,
                         self.q_vel, self.q_vel, self.q_wz, self.q_bias])

    def p0_diag(self) -> np.ndarray:
        return np.array([self.p0_pos, self.p0_pos, self.p0_yaw,
                         self.p0_vel, self.p0_vel, self.p0_wz, self.p0_bias])


class UpdateResult(NamedTuple):
    accepted: bool
    nis: float      # Normalised Innovation Squared (consistency statistic)
    reason: str     # 'ok' | 'gated' | 'non_finite' | 'singular_S'


class PlanarEKF:
    """Extended Kalman Filter for the 7-state planar model above.

    The class owns (x, P). Two operations:
        predict(dt)  - advance the state through the motion model
        update(...)  - correct it with one sensor measurement
    Time bookkeeping, ordering and gating policy live one level up in
    FusionManager so this class stays small and mathematically clean.
    """

    def __init__(self, params: EKFParams | None = None):
        self.params = params or EKFParams()
        self.x = np.zeros(N)
        self.P = np.diag(self.params.p0_diag())

    def reset(self) -> None:
        self.x = np.zeros(N)
        self.P = np.diag(self.params.p0_diag())

    def healthy(self) -> bool:
        """Numerical sanity check: finite numbers, non-negative variances."""
        return bool(np.all(np.isfinite(self.x)) and np.all(np.isfinite(self.P))
                    and np.all(np.diag(self.P) > 0.0))

    # ------------------------------------------------------------------
    # PREDICT
    # ------------------------------------------------------------------
    @staticmethod
    def propagate(x: np.ndarray, P: np.ndarray, dt: float,
                  prm: EKFParams) -> Tuple[np.ndarray, np.ndarray]:
        """Constant-velocity motion model. Returns NEW (x, P); inputs untouched.

        Continuous model:   d(pos)/dt = Rot(yaw) @ [vx, vy]
                            d(yaw)/dt = wz
                            d(vx, vy, wz, bgz)/dt = 0  (+ process noise)

        Integration uses the MID-POINT heading (yaw + wz*dt/2). With plain
        Euler (start-of-step heading) a robot spinning while driving traces
        a visibly wrong arc; mid-point is exact for constant vx, vy, wz.
        """
        yaw, vx, vy, wz = x[YAW], x[VX], x[VY], x[WZ]
        ym = yaw + 0.5 * wz * dt
        c, s = math.cos(ym), math.sin(ym)
        dx = vx * c - vy * s          # world-frame velocity of the robot
        dy = vx * s + vy * c

        xn = x.copy()
        xn[X] += dx * dt
        xn[Y] += dy * dt
        xn[YAW] = wrap_angle(yaw + wz * dt)

        # Jacobian F = d f / d x. Identity everywhere except the entries
        # where one state feeds another during the step:
        F = np.eye(N)
        F[X, YAW] = -dy * dt              # heading error moves position sideways
        F[Y, YAW] = dx * dt
        F[X, VX], F[X, VY] = c * dt, -s * dt
        F[Y, VX], F[Y, VY] = s * dt, c * dt
        F[X, WZ] = -dy * 0.5 * dt * dt    # via the mid-point heading
        F[Y, WZ] = dx * 0.5 * dt * dt
        F[YAW, WZ] = dt

        Pn = F @ P @ F.T + np.diag(prm.q_diag()) * dt
        return xn, 0.5 * (Pn + Pn.T)      # re-symmetrise: kills round-off asymmetry

    @staticmethod
    def assume_stopped(x: np.ndarray, P: np.ndarray, prm: EKFParams,
                       angular: bool = False) -> Tuple[np.ndarray, np.ndarray]:
        """Fail-safe: zero the velocity states and say "we are now unsure".

        Without this, losing the lidar/camera would leave the filter happily
        integrating the LAST velocity forever - a robot that "drives away"
        in the map while sitting still. Zero + large variance is safe: the
        very next real measurement overrides it, and pose uncertainty grows.
        """
        x, P = x.copy(), P.copy()
        x[[VX, VY]] = 0.0
        P[VX, VX] += prm.dropout_vel_std ** 2
        P[VY, VY] += prm.dropout_vel_std ** 2
        if angular:
            x[WZ] = 0.0
            P[WZ, WZ] += prm.dropout_wz_std ** 2
        return x, P

    def predict(self, dt: float) -> None:
        if dt <= 0.0:
            return
        prm = self.params
        step = min(dt, prm.max_predict_dt)      # coast at most this long
        self.x, self.P = self.propagate(self.x, self.P, step, prm)
        gap = dt - step
        if gap > 0.0:
            # Total silence for longer than max_predict_dt: everything stops,
            # and we admit we do not know how far we moved during the gap.
            self.x, self.P = self.assume_stopped(self.x, self.P, prm, angular=True)
            self.P[X, X] += (prm.dropout_vel_std * gap) ** 2
            self.P[Y, Y] += (prm.dropout_vel_std * gap) ** 2
            self.P[YAW, YAW] += (prm.dropout_wz_std * gap) ** 2

    # ------------------------------------------------------------------
    # UPDATE
    # ------------------------------------------------------------------
    def update(self, z: Sequence[float], R: np.ndarray, H: np.ndarray,
               angle_rows: Sequence[int] = (), gate_chi2: float = 0.0
               ) -> UpdateResult:
        """Fuse one measurement  z = H x + noise(R).

        angle_rows : rows of the residual that are angles (wrapped to +-pi).
        gate_chi2  : reject if NIS exceeds this (<=0 disables the gate).
        """
        z = np.asarray(z, dtype=float).reshape(-1)
        if not (np.all(np.isfinite(z)) and np.all(np.isfinite(R))):
            return UpdateResult(False, float("nan"), "non_finite")

        # Innovation ("surprise"): what the sensor said minus what we expected.
        y = z - H @ self.x
        for r in angle_rows:
            y[r] = wrap_angle(y[r])

        # Innovation covariance: how big a surprise is *normal*
        #   = our own uncertainty about the prediction (H P H^T)
        #   + the sensor's noise (R)
        HP = H @ self.P
        S = HP @ H.T + R
        try:
            Sinv_y = np.linalg.solve(S, y)
            K = np.linalg.solve(S, HP).T          # K = P H^T S^-1 (P, S symmetric)
        except np.linalg.LinAlgError:
            return UpdateResult(False, float("nan"), "singular_S")

        # NIS = y^T S^-1 y  ~ chi-square(dof = len(z)) if the filter is
        # consistent. Used for (a) outlier gating and (b) tuning diagnostics.
        nis = float(y @ Sinv_y)
        if not math.isfinite(nis) or nis < 0.0:
            return UpdateResult(False, nis, "singular_S")
        if gate_chi2 > 0.0 and nis > gate_chi2:
            return UpdateResult(False, nis, "gated")

        x_new = self.x + K @ y
        x_new[YAW] = wrap_angle(x_new[YAW])

        # Joseph form: P = (I-KH) P (I-KH)^T + K R K^T.
        # Algebraically equal to (I-KH)P but stays symmetric positive
        # semi-definite despite floating-point round-off - the simple form
        # slowly loses that property over millions of updates.
        I_KH = np.eye(N) - K @ H
        P_new = I_KH @ self.P @ I_KH.T + K @ R @ K.T
        P_new = 0.5 * (P_new + P_new.T)

        self.x, self.P = x_new, P_new
        return UpdateResult(True, nis, "ok")
