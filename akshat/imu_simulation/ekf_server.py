#!/usr/bin/env python3
"""
Real-Time Cylindrical Robot EKF Server with `imu_worker` Binary Reader (Port 8000)
----------------------------------------------------------------------------------
Integrates the exact binary stream reader from `data_read.py`:
  - Spawns `sudo ./imu_worker` via subprocess.Popen
  - Scales raw integers using ACCEL_FACTOR = 16384.0 / 9.80665 and GYRO_FACTOR = 131.0
  - Runs the 500-sample static offset calibration (or loads existing imu_calibration.json)
  - Feeds calibrated [ax, ay, az] (m/s^2) and [gx, gy, gz] (converted from deg/s to rad/s)
    along with nanosecond hardware timestamps into the 6-State EKF configured by your YAML file
  - Hosts the 3D wireframe cylinder visualization (Ø22cm x 66cm, IMU @ 15cm) on port 8000
  - Saves `imu_telemetry.csv` on Ctrl+C exit
"""

import argparse
import csv
import glob
import json
import math
import os
import random
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

G_ACCEL = 9.80665
# Exact scale multipliers from data_read.py
ACCEL_FACTOR = 16384.0 / 9.80665
GYRO_FACTOR = 131.0


# =====================================================================
# 1. YAML LOADER & EKF PARAMETER RESOLVER
# =====================================================================
DEFAULT_YAML_TEMPLATE = """# EKF Configuration derived from 12,000 IMU calibration samples
robot_geometry:
  diameter_m: 0.22
  height_m: 0.66
  imu_height_m: 0.15
  wheel_axle_height_m: 0.045

ekf_parameters:
  sample_rate_hz: 100.0
  gyro_angle_variance: 0.00015
  gyro_bias_variance: 0.000002
  accel_noise_variance: 0.085
  dynamic_vibration_multiplier: 8.0
"""


def _parse_scalar(val_str):
    val_str = val_str.strip()
    if not val_str:
        return ""
    if val_str.startswith("[") and val_str.endswith("]"):
        inner = val_str[1:-1].strip()
        if not inner:
            return []
        return [_parse_scalar(x) for x in inner.split(",")]
    low = val_str.lower()
    if low in ("true", "yes"):
        return True
    if low in ("false", "no"):
        return False
    try:
        if "." in val_str or "e" in low:
            return float(val_str)
        return int(val_str)
    except ValueError:
        return val_str.strip("'\"")


def load_yaml_file(path):
    try:
        import yaml  # type: ignore
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
            if isinstance(data, dict):
                return data
    except Exception:
        pass

    root = {}
    stack = [(-1, root)]
    with open(path, "r", encoding="utf-8") as f:
        for raw_line in f:
            line = raw_line.split("#", 1)[0].rstrip()
            if not line.strip():
                continue
            indent = len(line) - len(line.lstrip())
            stripped = line.strip()
            while len(stack) > 1 and indent <= stack[-1][0]:
                stack.pop()
            parent = stack[-1][1]
            if ":" in stripped:
                k, v = stripped.split(":", 1)
                k = k.strip()
                v = v.strip()
                if not v:
                    new_dict = {}
                    if isinstance(parent, dict):
                        parent[k] = new_dict
                    stack.append((indent, new_dict))
                else:
                    if isinstance(parent, dict):
                        parent[k] = _parse_scalar(v)
    return root


def flatten_dict(d, parent_key=""):
    items = {}
    if not isinstance(d, dict):
        return items
    for k, v in d.items():
        new_key = f"{parent_key}.{k}" if parent_key else str(k)
        if isinstance(v, dict):
            items.update(flatten_dict(v, new_key))
        else:
            items[str(k).lower()] = v
            items[new_key.lower()] = v
    return items


def resolve_ekf_config(yaml_path):
    if not yaml_path:
        candidates = sorted(glob.glob("*.yaml") + glob.glob("*.yml"))
        if candidates:
            yaml_path = candidates[0]
        else:
            yaml_path = "ekf_config.yaml"
            with open(yaml_path, "w", encoding="utf-8") as f:
                f.write(DEFAULT_YAML_TEMPLATE)
            print(f"[INFO] Created default '{yaml_path}' template.")

    raw_cfg = load_yaml_file(yaml_path)
    flat = flatten_dict(raw_cfg)

    def pick(keys, default):
        for k in keys:
            if k in flat and flat[k] is not None:
                return flat[k]
        return default

    hz = float(pick(["sample_rate_hz", "rate_hz", "frequency", "hz"], 100.0))
    dt = float(pick(["dt", "delta_t"], 1.0 / max(hz, 1.0)))

    q_angle = float(pick([
        "gyro_angle_variance", "q_angle", "gyro_var", "gyro_noise_var",
        "gyro_covariance", "process_noise_angle", "q"
    ], 0.00015))

    q_bias = float(pick([
        "gyro_bias_variance", "q_bias", "bias_var", "gyro_bias_cov",
        "process_noise_bias"
    ], 0.000002))

    r_accel = float(pick([
        "accel_noise_variance", "r_accel", "accel_var", "accel_covariance",
        "measurement_noise", "r"
    ], 0.085))

    vib_mult = float(pick([
        "dynamic_vibration_multiplier", "vibration_multiplier", "r_scale"
    ], 8.0))

    imu_h = float(pick(["imu_height_m", "imu_z"], 0.15))
    axle_h = float(pick(["wheel_axle_height_m", "axle_z"], 0.045))

    return {
        "yaml_path": yaml_path,
        "dt": dt,
        "q_angle": q_angle,
        "q_bias": q_bias,
        "r_accel": r_accel,
        "vib_mult": vib_mult,
        "imu_height_m": imu_h,
        "axle_height_m": axle_h,
        "raw_yaml": raw_cfg,
    }


# =====================================================================
# 2. 6-STATE EXTENDED KALMAN FILTER (WITH 15CM LEVER-ARM COMPENSATION)
# =====================================================================
class CylinderIMUEKF:
    """
    State vector: x = [roll, pitch, yaw, bg_x, bg_y, bg_z]^T
    Inputs:
      ax, ay, az : calibrated linear acceleration in m/s^2
      gx, gy, gz : calibrated angular velocity in rad/s
      dt         : hardware delta-time in seconds
    """

    def __init__(self, cfg):
        self.cfg = cfg
        self.lever_arm_z = cfg["imu_height_m"] - cfg["axle_height_m"]  # 0.105 m
        self.reset()

    def reset(self):
        self.x = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.P = [[0.0] * 6 for _ in range(6)]
        for i in range(3):
            self.P[i][i] = 0.01
        for i in range(3, 6):
            self.P[i][i] = 0.001
        self.prev_omega = [0.0, 0.0, 0.0]
        self.last_lever_accel = 0.0

    def step(self, ax, ay, az, gx_rad, gy_rad, gz_rad, dt=None):
        if dt is None or dt <= 0.0 or dt > 0.5:
            dt = self.cfg["dt"]

        phi, theta, psi, bgx, bgy, bgz = self.x
        wx = gx_rad - bgx
        wy = gy_rad - bgy
        wz = gz_rad - bgz

        # Lever-Arm Tangential & Centripetal Acceleration at z = 15cm
        alpha_x = (wx - self.prev_omega[0]) / dt
        alpha_y = (wy - self.prev_omega[1]) / dt
        self.prev_omega = [wx, wy, wz]

        r_z = self.lever_arm_z
        a_lever_x = alpha_y * r_z + wx * wz * r_z
        a_lever_y = -alpha_x * r_z + wy * wz * r_z
        a_lever_z = -(wx * wx + wy * wy) * r_z
        self.last_lever_accel = math.sqrt(
            a_lever_x ** 2 + a_lever_y ** 2 + a_lever_z ** 2
        )

        max_comp = 2.5
        a_lever_x = max(-max_comp, min(max_comp, a_lever_x))
        a_lever_y = max(-max_comp, min(max_comp, a_lever_y))

        ax_grav = ax - 0.35 * a_lever_x
        ay_grav = ay - 0.35 * a_lever_y
        az_grav = az - 0.35 * a_lever_z

        # EKF Prediction Step (Nonlinear Euler Kinematics)
        sin_p = math.sin(phi)
        cos_p = math.cos(phi)
        cos_t = math.cos(theta)
        if abs(cos_t) < 1e-4:
            cos_t = 1e-4 if cos_t >= 0 else -1e-4
        tan_t = math.sin(theta) / cos_t

        phi_dot = wx + sin_p * tan_t * wy + cos_p * tan_t * wz
        theta_dot = cos_p * wy - sin_p * wz
        psi_dot = (sin_p / cos_t) * wy + (cos_p / cos_t) * wz

        self.x[0] += phi_dot * dt
        self.x[1] += theta_dot * dt
        self.x[2] = (self.x[2] + psi_dot * dt + math.pi) % (2 * math.pi) - math.pi

        q_a = self.cfg["q_angle"] * dt
        q_b = self.cfg["q_bias"] * dt

        for i in range(3):
            b_idx = i + 3
            p00 = self.P[i][i]
            p01 = self.P[i][b_idx]
            p10 = self.P[b_idx][i]
            p11 = self.P[b_idx][b_idx]

            self.P[i][i] = p00 - dt * (p01 + p10) + (dt * dt) * p11 + q_a
            self.P[i][b_idx] = p01 - dt * p11
            self.P[b_idx][i] = p10 - dt * p11
            self.P[b_idx][b_idx] = p11 + q_b

        # EKF Measurement Update Step (Accelerometer Roll & Pitch)
        accel_norm = math.sqrt(ax_grav ** 2 + ay_grav ** 2 + az_grav ** 2)
        if accel_norm > 1e-3:
            meas_roll = math.atan2(ay_grav, az_grav)
            meas_pitch = math.atan2(
                -ax_grav, math.sqrt(ay_grav ** 2 + az_grav ** 2)
            )

            norm_dev = abs(accel_norm - G_ACCEL)
            r_eff = (
                self.cfg["r_accel"]
                * self.cfg["vib_mult"]
                * (1.0 + 4.0 * (norm_dev ** 2))
            )

            for idx, z_meas in ((0, meas_roll), (1, meas_pitch)):
                b_idx = idx + 3
                y_res = z_meas - self.x[idx]
                s_cov = self.P[idx][idx] + r_eff
                k_angle = self.P[idx][idx] / s_cov
                k_bias = self.P[b_idx][idx] / s_cov

                self.x[idx] += k_angle * y_res
                self.x[b_idx] += k_bias * y_res

                p_ii = self.P[idx][idx]
                p_ib = self.P[idx][b_idx]
                self.P[idx][idx] -= k_angle * p_ii
                self.P[idx][b_idx] -= k_angle * p_ib
                self.P[b_idx][idx] -= k_bias * p_ii
                self.P[b_idx][b_idx] -= k_bias * p_ib

        raw_roll = math.atan2(ay, max(abs(az), 1e-3) * (1 if az >= 0 else -1))
        raw_pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))

        return {
            "ekf": {
                "roll": self.x[0],
                "pitch": self.x[1],
                "yaw": self.x[2],
            },
            "raw": {
                "roll": raw_roll,
                "pitch": raw_pitch,
                "yaw": self.x[2],
            },
            "bias": [self.x[3], self.x[4], self.x[5]],
            "cov_trace": sum(self.P[i][i] for i in range(6)),
            "lever_accel": self.last_lever_accel,
            "imu_raw": {
                "ax": ax,
                "ay": ay,
                "az": az,
                "gx_deg": math.degrees(gx_rad),
                "gy_deg": math.degrees(gy_rad),
                "gz_deg": math.degrees(gz_rad),
            },
        }


# =====================================================================
# 3. IMU WORKER BINARY STREAM & CALIBRATION ENGINE (FROM data_read.py)
# =====================================================================
class IMUWorkerEngine:
    def __init__(self, cfg, worker_cmd=None, csv_path=None, skip_cal=False):
        self.cfg = cfg
        self.ekf = CylinderIMUEKF(cfg)
        self.worker_cmd = worker_cmd or ["sudo", "./imu_worker"]
        self.csv_path = csv_path

        self.lock = threading.Lock()
        self.process = None
        self.running = True
        self.source_label = "sudo ./imu_worker"

        # Exact calibration & logging structures from data_read.py
        self.cal_samples = []
        self.data_log = []
        self.cal_done = False
        self.offsets = {
            "ax": 0.0, "ay": 0.0, "az": 0.0,
            "gx": 0.0, "gy": 0.0, "gz": 0.0,
        }
        self.prev_timestamp = None

        if skip_cal and os.path.exists("imu_calibration.json"):
            try:
                with open("imu_calibration.json", "r", encoding="utf-8") as f:
                    self.offsets.update(json.load(f))
                self.cal_done = True
                print("[INFO] Loaded existing 'imu_calibration.json' offsets.")
            except Exception as e:
                print(f"[WARN] Could not read imu_calibration.json: {e}")

        self.latest_frame = self._decorate_frame(
            self.ekf.step(0.0, 0.0, G_ACCEL, 0.0, 0.0, 0.0)
        )

    def _decorate_frame(self, frame):
        frame["cal_done"] = self.cal_done
        frame["cal_count"] = len(self.cal_samples)
        frame["source"] = self.source_label
        return frame

    def apply_control(self, payload):
        with self.lock:
            if payload.get("reset"):
                self.ekf.reset()
            if payload.get("recalibrate"):
                self.cal_samples = []
                self.cal_done = False
                self.prev_timestamp = None
                print("\n⚠️ DO NOT MOVE IMU - Recalibrating static offsets (500 frames)...")

    def _compute_dt_from_timestamp(self, timestamp):
        if self.prev_timestamp is None:
            self.prev_timestamp = timestamp
            return self.cfg["dt"]
        diff = timestamp - self.prev_timestamp
        self.prev_timestamp = timestamp
        if diff <= 0:
            return self.cfg["dt"]
        # `Timestamp_ns` in data_read.py -> convert nanoseconds to seconds
        if diff > 100_000:
            dt = diff * 1e-9
        elif diff > 100:
            dt = diff * 1e-6
        else:
            dt = diff * 1e-3
        return dt if 0.0002 <= dt <= 0.5 else self.cfg["dt"]

    def _process_raw_7tuple(self, timestamp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz):
        # Exact scaling from data_read.py
        ax = raw_ax / ACCEL_FACTOR
        ay = raw_ay / ACCEL_FACTOR
        az = raw_az / ACCEL_FACTOR
        gx = raw_gx / GYRO_FACTOR  # deg/s
        gy = raw_gy / GYRO_FACTOR  # deg/s
        gz = raw_gz / GYRO_FACTOR  # deg/s

        with self.lock:
            if not self.cal_done:
                self.cal_samples.append((ax, ay, az, gx, gy, gz))
                print(f"\rCollecting calibration frames: {len(self.cal_samples)}/500", end="", flush=True)
                if len(self.cal_samples) >= 500:
                    avg = [sum(x) / len(x) for x in zip(*self.cal_samples)]
                    self.offsets["ax"] = avg[0]
                    self.offsets["ay"] = avg[1]
                    self.offsets["az"] = avg[2] - 9.80665
                    self.offsets["gx"] = avg[3]
                    self.offsets["gy"] = avg[4]
                    self.offsets["gz"] = avg[5]
                    self.cal_done = True
                    print("\n✅ Calibration complete! Saving profile and streaming EKF on :8000...\n")
                    with open("imu_calibration.json", "w", encoding="utf-8") as f:
                        json.dump(self.offsets, f, indent=4)
                self.latest_frame["cal_done"] = self.cal_done
                self.latest_frame["cal_count"] = len(self.cal_samples)
                return

            # Subtract baseline offsets (exact logic from data_read.py)
            ax_c = ax - self.offsets["ax"]
            ay_c = ay - self.offsets["ay"]
            az_c = az - self.offsets["az"]
            gx_c = gx - self.offsets["gx"]
            gy_c = gy - self.offsets["gy"]
            gz_c = gz - self.offsets["gz"]

            # Track history for SLAM/Graphing analysis (saved to imu_telemetry.csv on exit)
            self.data_log.append([timestamp, ax_c, ay_c, az_c, gx_c, gy_c, gz_c])

            dt = self._compute_dt_from_timestamp(timestamp)

            # Convert gx_c, gy_c, gz_c from deg/s to rad/s for the EKF
            frame = self.ekf.step(
                ax_c, ay_c, az_c,
                math.radians(gx_c), math.radians(gy_c), math.radians(gz_c),
                dt
            )
            self.latest_frame = self._decorate_frame(frame)

    def _run_imu_worker_loop(self):
        print(f"[INFO] Starting binary subprocess: {' '.join(self.worker_cmd)}")
        if not self.cal_done:
            print("⚠️ DO NOT MOVE IMU - Calibrating static offsets...")
        try:
            self.process = subprocess.Popen(
                self.worker_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
                bufsize=1,
            )
            assert self.process.stdout is not None
            for line in self.process.stdout:
                if not self.running:
                    break
                parts = line.strip().split(",")
                if len(parts) != 7 or parts[0].startswith("Time"):
                    continue
                try:
                    timestamp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz = map(int, parts)
                except ValueError:
                    continue
                self._process_raw_7tuple(
                    timestamp, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz
                )
        except FileNotFoundError:
            print(f"\n[WARN] Binary '{self.worker_cmd}' not found on this machine.")
            print("[INFO] Falling back to simulated `imu_worker` raw binary stream for browser testing.")
            self.source_label = "simulated ./imu_worker"
            self._run_simulated_worker_loop()
        except Exception as e:
            print(f"\n[WARN] Subprocess error ({e}). Switching to simulated stream.")
            self.source_label = "simulated ./imu_worker"
            self._run_simulated_worker_loop()

    def _run_simulated_worker_loop(self):
        """Generates synthetic raw integer 7-tuples identical to `./imu_worker` output if binary is absent."""
        dt = self.cfg["dt"]
        t_ns = int(time.time() * 1e9)
        step_ns = int(dt * 1e9)
        t = 0.0
        while self.running:
            # Simulate slight motor vibration on raw LSB integers
            vib_ax = 0.65 * math.sin(2.0 * math.pi * 29.0 * t) + random.gauss(0.0, 0.25)
            vib_ay = 0.55 * math.cos(2.0 * math.pi * 37.0 * t) + random.gauss(0.0, 0.25)
            raw_ax = int((0.04 + vib_ax) * ACCEL_FACTOR)
            raw_ay = int((-0.03 + vib_ay) * ACCEL_FACTOR)
            raw_az = int((G_ACCEL + 0.05 + random.gauss(0.0, 0.18)) * ACCEL_FACTOR)
            raw_gx = int((0.35 + random.gauss(0.0, 0.4)) * GYRO_FACTOR)
            raw_gy = int((-0.25 + random.gauss(0.0, 0.4)) * GYRO_FACTOR)
            raw_gz = int((0.10 + random.gauss(0.0, 0.3)) * GYRO_FACTOR)

            self._process_raw_7tuple(t_ns, raw_ax, raw_ay, raw_az, raw_gx, raw_gy, raw_gz)
            t_ns += step_ns
            t += dt
            time.sleep(dt)

    def _run_csv_replay_loop(self):
        self.source_label = f"CSV ({self.csv_path})"
        self.cal_done = True
        print(f"[INFO] Replaying captured CSV '{self.csv_path}'...")
        while self.running:
            with open(self.csv_path, "r", encoding="utf-8") as f:
                reader = csv.DictReader(f)
                for row in reader:
                    if not self.running:
                        break
                    norm = {k.strip().lower(): float(v) for k, v in row.items() if v}
                    ts = int(norm.get("timestamp_ns", norm.get("timestamp", 0)))
                    ax_c = norm.get("ax", 0.0)
                    ay_c = norm.get("ay", 0.0)
                    az_c = norm.get("az", G_ACCEL)
                    gx_c = norm.get("gx", 0.0)  # deg/s in imu_telemetry.csv
                    gy_c = norm.get("gy", 0.0)
                    gz_c = norm.get("gz", 0.0)
                    dt = self._compute_dt_from_timestamp(ts) if ts else self.cfg["dt"]
                    with self.lock:
                        frame = self.ekf.step(
                            ax_c, ay_c, az_c,
                            math.radians(gx_c), math.radians(gy_c), math.radians(gz_c),
                            dt
                        )
                        self.latest_frame = self._decorate_frame(frame)
                    time.sleep(self.cfg["dt"])

    def start(self):
        target = (
            self._run_csv_replay_loop
            if (self.csv_path and os.path.exists(self.csv_path))
            else self._run_imu_worker_loop
        )
        th = threading.Thread(target=target, daemon=True)
        th.start()

    def shutdown(self):
        self.running = False
        if self.process is not None:
            try:
                self.process.terminate()
            except Exception:
                pass
        # Exact CSV dump on exit from data_read.py
        if self.data_log:
            with open("imu_telemetry.csv", "w", newline="", encoding="utf-8") as f:
                writer = csv.writer(f)
                writer.writerow(["Timestamp_ns", "ax", "ay", "az", "gx", "gy", "gz"])
                writer.writerows(self.data_log)
            print(f"💾 Saved {len(self.data_log)} clean samples to 'imu_telemetry.csv'")


# =====================================================================
# 4. HTTP & SSE SERVER ON PORT 8000
# =====================================================================
ENGINE = None


class SimRequestHandler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass

    def do_GET(self):
        if self.path in ("/", "/index.html"):
            html_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "index.html")
            if not os.path.exists(html_path):
                html_path = "index.html"
            with open(html_path, "rb") as f:
                content = f.read()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(content)))
            self.end_headers()
            self.wfile.write(content)

        elif self.path == "/config":
            with ENGINE.lock:
                offsets_copy = dict(ENGINE.offsets)
            payload = json.dumps({
                "yaml_path": ENGINE.cfg["yaml_path"],
                "source": ENGINE.source_label,
                "ekf_parameters": {
                    "dt_sec": round(ENGINE.cfg["dt"], 5),
                    "gyro_angle_variance_Q": ENGINE.cfg["q_angle"],
                    "gyro_bias_variance_Q": ENGINE.cfg["q_bias"],
                    "accel_noise_variance_R": ENGINE.cfg["r_accel"],
                    "dynamic_vibration_mult": ENGINE.cfg["vib_mult"],
                    "imu_height_m": ENGINE.cfg["imu_height_m"],
                    "lever_arm_m": round(ENGINE.ekf.lever_arm_z, 4),
                },
                "calibrated_offsets": offsets_copy,
            }).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.end_headers()
            self.wfile.write(payload)

        elif self.path == "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            try:
                while True:
                    with ENGINE.lock:
                        frame_json = json.dumps(ENGINE.latest_frame)
                    self.wfile.write(f"data: {frame_json}\n\n".encode("utf-8"))
                    self.wfile.flush()
                    time.sleep(1.0 / 50.0)
            except (BrokenPipeError, ConnectionResetError):
                pass
        else:
            self.send_error(404, "Not Found")

    def do_POST(self):
        length = int(self.headers.get("Content-Length", 0))
        body = self.rfile.read(length).decode("utf-8") if length > 0 else "{}"
        data = json.loads(body)
        if self.path == "/control":
            ENGINE.apply_control(data)
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.end_headers()
        self.wfile.write(b'{"status":"ok"}')


def main():
    global ENGINE
    parser = argparse.ArgumentParser(description="Cylindrical Robot EKF Server with imu_worker Reader")
    parser.add_argument("--yaml", type=str, default=None, help="Path to EKF YAML config file")
    parser.add_argument("--worker", type=str, default="./imu_worker", help="Path to imu_worker binary")
    parser.add_argument("--no-sudo", action="store_true", help="Run imu_worker without sudo")
    parser.add_argument("--skip-cal", action="store_true", help="Reuse existing imu_calibration.json if present")
    parser.add_argument("--csv", type=str, default=None, help="Optional: replay imu_telemetry.csv instead of binary")
    parser.add_argument("--port", type=int, default=8000, help="HTTP Server Port (default: 8000)")
    args = parser.parse_args()

    cfg = resolve_ekf_config(args.yaml)
    worker_cmd = [args.worker] if args.no_sudo else ["sudo", args.worker]

    ENGINE = IMUWorkerEngine(
        cfg,
        worker_cmd=worker_cmd,
        csv_path=args.csv,
        skip_cal=args.skip_cal,
    )
    ENGINE.start()

    server = ThreadingHTTPServer(("0.0.0.0", args.port), SimRequestHandler)
    print("=" * 68)
    print(f" Cylindrical Robot EKF Server Active (Port {args.port})")
    print(f" -> Binary Cmd : {' '.join(worker_cmd)}")
    print(f" -> Geometry   : Ø22cm x 66cm H | IMU @ 15cm above ground")
    print(f" -> YAML File  : {cfg['yaml_path']}")
    print(f" -> Web URL    : http://localhost:{args.port}")
    print("=" * 68)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n\nStopping stream. Saving captured datasets...")
        ENGINE.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
