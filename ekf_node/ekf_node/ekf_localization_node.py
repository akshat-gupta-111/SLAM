"""
ekf_localization_node.py - the ROS 2 wrapper around the EKF.

Rule of this file: it contains NO filter math. Its only jobs are
  (a) read parameters + calibration files,
  (b) turn each ROS message into a body-frame `Measurement`,
  (c) publish the estimate (Odometry, TF, diagnostics),
  (d) never crash on bad input.

Data flow
---------
  /imu/data   --\\
  /odom_rf2o  ---+--> callbacks --> Measurement --> FusionManager --> PlanarEKF
  /visual_odom -/                                          |
                                                           v
                        output timer (30 Hz) --> /odometry/filtered + TF odom->base_link

Everything runs in ONE thread (default single-threaded executor), so the
callbacks never run concurrently and the filter needs no locks.
"""
import dataclasses
from dataclasses import dataclass
from functools import partial
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
import yaml
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu
from std_srvs.srv import Trigger
from tf2_ros import (Buffer, TransformBroadcaster, TransformException,
                     TransformListener)

from .ekf_core import (BGZ, H_GYRO, H_TWIST, VX, VY, WZ, X, Y, YAW, EKFParams,
                       PlanarEKF)
from .fusion_manager import FusionManager, Measurement
from .geometry import quat_to_rot, twist_to_base, yaw_to_quat

_IDENTITY_EXT = (np.eye(3), np.zeros(3))
_TWIST_IDX = [0, 1, 5]            # vx, vy, wz inside a 6x6 ROS twist covariance


@dataclass
class SensorCfg:
    """Static configuration of one input sensor."""
    name: str
    topic: str
    cov: np.ndarray               # nominal noise covariance, SENSOR frame
    gate_chi2: float
    stale_timeout_s: float
    use_msg_cov: bool = False     # trust the covariance inside the message?
    max_msg_var: float = 1.0e3    # message variance above this = "tracking lost"


class EKFLocalizationNode(Node):
    # ------------------------------------------------------------------
    # 1. Construction
    # ------------------------------------------------------------------
    def __init__(self):
        super().__init__("ekf_localization_node")
        self._load_parameters()

        self.ekf = PlanarEKF(self.ekf_params)
        self.mgr = FusionManager(
            self.ekf, self.history_window_s, self.max_consecutive_rejects,
            self.recovery_inflation, on_event=self._on_manager_event)

        self._ext_cache: Dict[str, Tuple[np.ndarray, np.ndarray]] = {}
        self._last_rx: Dict[str, float] = {}      # node-clock time of last message

        # TF: listener to learn where sensors are mounted, broadcaster to
        # publish odom->base_link.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self._setup_interfaces()
        self.get_logger().info(
            "EKF ready. Frames: %s -> %s | inputs: %s"
            % (self.odom_frame, self.base_frame,
               ", ".join(f"{c.name}({c.topic})" for c in self.sensors.values()) or "NONE"))

    # ------------------------------------------------------------------
    # 2. Parameters and calibration files
    # ------------------------------------------------------------------
    def _p(self, name, default):
        """Declare a parameter with a default and return its value."""
        self.declare_parameter(name, default)
        return self.get_parameter(name).value

    @staticmethod
    def _read_yaml(path: str) -> dict:
        if not path:
            return {}
        try:
            with open(path, "r") as f:
                return yaml.safe_load(f) or {}
        except (OSError, yaml.YAMLError):
            return {}

    def _load_parameters(self):
        p = self._p
        self.odom_frame = p("odom_frame", "odom")
        self.base_frame = p("base_link_frame", "base_link")
        self.publish_tf = p("publish_tf", True)
        self.output_rate_hz = float(p("output_rate_hz", 30.0))
        self.diag_rate_hz = float(p("diagnostics_rate_hz", 1.0))
        self.max_extrap_s = float(p("max_extrapolation_s", 0.2))
        self.max_future_s = float(p("max_future_stamp_s", 0.1))
        self.history_window_s = float(p("history_window_s", 2.0))
        self.max_consecutive_rejects = int(p("gate.max_consecutive_rejects", 20))
        self.recovery_inflation = float(p("gate.recovery_inflation", 25.0))

        # Filter tuning: one parameter per EKFParams field, so adding a field
        # to the dataclass automatically makes it configurable. Defaults live
        # in exactly one place (ekf_core.EKFParams).
        kw = {f.name: float(p(f"filter.{f.name}", f.default))
              for f in dataclasses.fields(EKFParams)}
        self.ekf_params = EKFParams(**kw)

        self.sensors: Dict[str, SensorCfg] = {}
        log = self.get_logger()

        # --- IMU (gyro-z only; see guide for why not orientation/accel) ---
        if p("imu.enabled", True):
            cal = self._read_yaml(p("imu.calibration_file", "")).get("imu_calibration", {})
            try:
                var = float(cal["angular_velocity_covariance"][8])   # gz variance
                src = "calibration file"
            except (KeyError, IndexError, TypeError):
                var, src = float(p("imu.fallback_gyro_variance", 8.7e-7)), "fallback param"
            scale = float(p("imu.noise_scale", 10.0))
            self.sensors["imu"] = SensorCfg(
                "imu", p("imu.topic", "/imu/data"), np.array([[var * scale]]),
                float(p("imu.gate_chi2", 10.83)), float(p("imu.stale_timeout_s", 0.2)))
            log.info(f"IMU gyro-z variance {var:.2e} ({src}) x{scale:g} "
                     f"=> sigma {np.sqrt(var * scale) * 1e3:.2f} mrad/s")

        # --- rf2o lidar odometry (twist) ---
        if p("rf2o.enabled", True):
            cal = self._read_yaml(p("rf2o.calibration_file", "")).get("rf2o_calibration", {})
            try:
                cov = np.array(cal["velocity_covariance"], dtype=float).reshape(3, 3)
                src = "calibration file"
            except (KeyError, ValueError, TypeError):
                cov = np.diag(np.array(p("rf2o.fallback_variance",
                                         [4.9e-6, 3.1e-6, 2.4e-5]), dtype=float))
                src = "fallback param"
            scale = float(p("rf2o.noise_scale", 25.0))
            self.sensors["rf2o"] = SensorCfg(
                "rf2o", p("rf2o.topic", "/odom_rf2o"), cov * scale,
                float(p("rf2o.gate_chi2", 16.27)), float(p("rf2o.stale_timeout_s", 0.5)),
                bool(p("rf2o.use_message_covariance", False)),
                float(p("rf2o.max_message_variance", 1.0e3)))
            sd = np.sqrt(np.diag(cov * scale))
            log.info(f"rf2o sigma(vx,vy,wz) {sd[0]:.4f} {sd[1]:.4f} {sd[2]:.4f} "
                     f"({src} x{scale:g})")

        # --- visual odometry (twist) ---
        if p("visual_odom.enabled", True):
            std = np.array(p("visual_odom.noise_std", [0.05, 0.05, 0.05]), dtype=float)
            self.sensors["vo"] = SensorCfg(
                "vo", p("visual_odom.topic", "/visual_odom"), np.diag(std ** 2),
                float(p("visual_odom.gate_chi2", 16.27)),
                float(p("visual_odom.stale_timeout_s", 0.5)),
                bool(p("visual_odom.use_message_covariance", False)),
                float(p("visual_odom.max_message_variance", 1.0e3)))
            log.info(f"visual odom sigma(vx,vy,wz) = {std} (config; calibrate me!)")

    # ------------------------------------------------------------------
    # 3. ROS interfaces
    # ------------------------------------------------------------------
    def _setup_interfaces(self):
        # BEST_EFFORT for all inputs: a BEST_EFFORT subscriber can talk to
        # both BEST_EFFORT and RELIABLE publishers, whereas a RELIABLE
        # subscriber silently receives NOTHING from a BEST_EFFORT publisher.
        qos = qos_profile_sensor_data
        if "imu" in self.sensors:
            self.create_subscription(Imu, self.sensors["imu"].topic, self._imu_cb, qos)
        for key in ("rf2o", "vo"):
            if key in self.sensors:
                cfg = self.sensors[key]
                self.create_subscription(
                    Odometry, cfg.topic, partial(self._odom_cb, cfg=cfg), qos)

        self.odom_pub = self.create_publisher(Odometry, "odometry/filtered", 10)
        self.diag_pub = self.create_publisher(DiagnosticArray, "/diagnostics", 10)
        self.create_timer(1.0 / self.output_rate_hz, self._on_output_timer)
        self.create_timer(1.0 / self.diag_rate_hz, self._on_diag_timer)
        self.create_service(Trigger, "~/reset", self._on_reset)

    def _on_manager_event(self, level: str, msg: str):
        getattr(self.get_logger(), level)(msg)   # level is info|warning|error

    def _on_reset(self, _req, resp):
        self.mgr.reset()
        resp.success, resp.message = True, "EKF reset; will re-initialise on next measurement."
        return resp

    # ------------------------------------------------------------------
    # 4. Helpers: time and frames
    # ------------------------------------------------------------------
    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _resolve_stamp(self, stamp_msg, now_s: float) -> float:
        """header.stamp -> seconds, defending against broken drivers."""
        s = stamp_msg.sec + stamp_msg.nanosec * 1e-9
        if s <= 0.0:                         # unstamped message
            return now_s
        if s > now_s + self.max_future_s:    # clock skew / bad driver
            self.get_logger().warning(
                f"Message stamped {s - now_s:.2f}s in the future; using receive time.",
                throttle_duration_sec=5.0)
            return now_s
        return s

    def _extrinsics(self, frame: str):
        """(R, t): pose of `frame` expressed in base_link. Cached forever -
        sensor mounts are rigid. Returns None until TF is available."""
        frame = (frame or self.base_frame).lstrip("/")
        if frame == self.base_frame:
            return _IDENTITY_EXT
        hit = self._ext_cache.get(frame)
        if hit is not None:
            return hit
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, frame, Time())
        except TransformException as e:
            self.get_logger().warning(
                f"No TF {self.base_frame} <- {frame} yet ({e}); dropping data.",
                throttle_duration_sec=5.0)
            return None
        q, tr = tf.transform.rotation, tf.transform.translation
        ext = (quat_to_rot(q.x, q.y, q.z, q.w), np.array([tr.x, tr.y, tr.z]))
        self._ext_cache[frame] = ext
        self.get_logger().info(
            f"Cached extrinsics {self.base_frame} <- {frame}: "
            f"t=({tr.x:.3f},{tr.y:.3f},{tr.z:.3f})")
        return ext

    # ------------------------------------------------------------------
    # 5. Sensor callbacks: ROS message -> Measurement
    # ------------------------------------------------------------------
    def _imu_cb(self, msg: Imu):
        now_s = self._now_s()
        self._last_rx["imu"] = now_s
        ext = self._extrinsics(msg.header.frame_id)
        if ext is None:
            return
        cfg = self.sensors["imu"]
        w_s = np.array([msg.angular_velocity.x, msg.angular_velocity.y,
                        msg.angular_velocity.z])
        # Only the rotation matters for a gyro (angular velocity is a free
        # vector - no lever arm). Take the z-component in the base frame.
        gz = float((ext[0] @ w_s)[2])
        self.mgr.add(Measurement(
            "imu", self._resolve_stamp(msg.header.stamp, now_s),
            np.array([gz]), cfg.cov.copy(), H_GYRO, observes_velocity=False,
            gate_chi2=cfg.gate_chi2))

    def _odom_cb(self, msg: Odometry, cfg: SensorCfg):
        now_s = self._now_s()
        self._last_rx[cfg.name] = now_s
        # The twist of an Odometry message is expressed in child_frame_id.
        ext = self._extrinsics(msg.child_frame_id)
        if ext is None:
            return
        R, t = ext
        tw = msg.twist.twist
        v_b, w_b = twist_to_base(
            R, t, np.array([tw.linear.x, tw.linear.y, tw.linear.z]),
            np.array([tw.angular.x, tw.angular.y, tw.angular.z]))
        z = np.array([v_b[0], v_b[1], w_b[2]])

        cov = self._twist_covariance(msg, cfg)
        if cov is None:                       # sensor says "I'm lost" -> skip
            return
        # Rotate the (vx,vy) block of the sensor-frame covariance into base
        # frame; planar approximation (assumes the sensor is mounted ~level).
        T = np.eye(3)
        T[:2, :2] = R[:2, :2]
        T[2, 2] = R[2, 2]
        cov_b = T @ cov @ T.T

        self.mgr.add(Measurement(
            cfg.name, self._resolve_stamp(msg.header.stamp, now_s), z, cov_b,
            H_TWIST, observes_velocity=True, gate_chi2=cfg.gate_chi2))

    @staticmethod
    def _twist_covariance(msg: Odometry, cfg: SensorCfg) -> Optional[np.ndarray]:
        """Choose configured covariance, or the message's if allowed and sane."""
        if not cfg.use_msg_cov:
            return cfg.cov
        c = np.array(msg.twist.covariance, dtype=float).reshape(6, 6)
        c3 = c[np.ix_(_TWIST_IDX, _TWIST_IDX)]
        d = np.diag(c3)
        if not np.all(np.isfinite(c3)) or np.any(d <= 0.0):
            return cfg.cov                    # message has no usable covariance
        if np.any(d > cfg.max_msg_var):
            return None                       # driver flags tracking loss
        return c3

    # ------------------------------------------------------------------
    # 6. Output
    # ------------------------------------------------------------------
    def _on_output_timer(self):
        if not self.mgr.initialized:
            return
        now = self.get_clock().now()
        x, P = self.mgr.state_at(now.nanoseconds * 1e-9, self.max_extrap_s)
        self._publish_odometry(now, x, P)
        if self.publish_tf:
            self._publish_tf(now, x)

    def _publish_odometry(self, now: Time, x: np.ndarray, P: np.ndarray):
        o = Odometry()
        o.header.stamp = now.to_msg()
        o.header.frame_id = self.odom_frame
        o.child_frame_id = self.base_frame
        o.pose.pose.position.x = float(x[X])
        o.pose.pose.position.y = float(x[Y])
        qx, qy, qz, qw = yaw_to_quat(float(x[YAW]))
        o.pose.pose.orientation.x, o.pose.pose.orientation.y = qx, qy
        o.pose.pose.orientation.z, o.pose.pose.orientation.w = qz, qw
        o.twist.twist.linear.x = float(x[VX])
        o.twist.twist.linear.y = float(x[VY])
        o.twist.twist.angular.z = float(x[WZ])

        # Map our 7x7 P into ROS 6x6 covariances (order x,y,z,roll,pitch,yaw).
        # z/roll/pitch are not estimated in 2D mode: constrained to zero, so
        # a tiny variance (REP-105 style) rather than "unknown".
        pose_c = np.zeros((6, 6))
        pose_c[np.ix_(_TWIST_IDX, _TWIST_IDX)] = P[np.ix_([X, Y, YAW], [X, Y, YAW])]
        twist_c = np.zeros((6, 6))
        twist_c[np.ix_(_TWIST_IDX, _TWIST_IDX)] = P[np.ix_([VX, VY, WZ], [VX, VY, WZ])]
        for c in (pose_c, twist_c):
            for k in (2, 3, 4):
                c[k, k] = 1.0e-9
        o.pose.covariance = pose_c.flatten().tolist()
        o.twist.covariance = twist_c.flatten().tolist()
        self.odom_pub.publish(o)

    def _publish_tf(self, now: Time, x: np.ndarray):
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = float(x[X])
        t.transform.translation.y = float(x[Y])
        qx, qy, qz, qw = yaw_to_quat(float(x[YAW]))
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    # 7. Diagnostics (rate, age, accept/reject counts, NIS per sensor)
    # ------------------------------------------------------------------
    def _on_diag_timer(self):
        now_s = self._now_s()
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        for name, cfg in self.sensors.items():
            st = self.mgr.stats.get(name)
            age = now_s - self._last_rx[name] if name in self._last_rx else float("inf")
            s = DiagnosticStatus(name=f"ekf/{name}", hardware_id=cfg.topic)
            if age > cfg.stale_timeout_s:
                s.level, s.message = DiagnosticStatus.WARN, f"STALE ({age:.1f}s)"
            elif st and st.consecutive_rejects >= 5:
                s.level, s.message = DiagnosticStatus.WARN, "measurements being gated"
            else:
                s.level, s.message = DiagnosticStatus.OK, "ok"
            if st:
                for k, v in (("rate_hz", f"{st.rate_hz:.1f}"), ("age_s", f"{age:.2f}"),
                             ("accepted", st.accepted), ("rejected", st.rejected),
                             ("too_old", st.too_old),
                             ("nis_mean", f"{st.nis_mean:.2f}")):
                    s.values.append(KeyValue(key=k, value=str(v)))
            arr.status.append(s)
        if self.mgr.initialized:
            f = DiagnosticStatus(name="ekf/filter", hardware_id="planar_ekf")
            sd = np.sqrt(np.diag(self.ekf.P))
            f.level = DiagnosticStatus.WARN if self.mgr.in_dropout else DiagnosticStatus.OK
            f.message = "velocity assumed zero (sensors lost)" if self.mgr.in_dropout else "ok"
            for k, v in (("pos_std_m", f"{max(sd[X], sd[Y]):.3f}"),
                         ("yaw_std_deg", f"{np.degrees(sd[YAW]):.2f}"),
                         ("gyro_bias_rad_s", f"{self.ekf.x[BGZ]:.5f}")):
                f.values.append(KeyValue(key=k, value=v))
            arr.status.append(f)
        self.diag_pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = EKFLocalizationNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
