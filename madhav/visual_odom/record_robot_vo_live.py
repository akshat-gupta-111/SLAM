#!/usr/bin/env python3
import time
import csv
import os
import cv2
import numpy as np
from threading import Thread

# --- Asynchronous High-Speed Frame Capture Buffer ---
class VideoStreamAsync:
    def __init__(self, src=0, width=640, height=480):
        self.stream = cv2.VideoCapture(src)
        self.stream.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.stream.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.stream.set(cv2.CAP_PROP_FPS, 30)
        self.grabbed, self.frame = self.stream.read()
        self.started = False

    def start(self):
        if self.started: return self
        self.started = True
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()
        return self

    def update(self):
        while self.started:
            if not self.grabbed:
                self.stop()
                break
            self.grabbed, self.frame = self.stream.read()

    def read(self):
        return self.frame

    def stop(self):
        self.started = False
        if self.stream.isOpened():
            self.stream.release()

# --- Main Real-Time Visual Odometry Telemetry Node ---
def run_robot_vo_pipeline(output_csv="robot_ekf_input.csv", camera_index=0):
    vs = VideoStreamAsync(src=camera_index)
    if not vs.stream.isOpened():
        print(f"[ERROR] Could not open camera source at index {camera_index}. Check USB connection.")
        return

    vs.start()
    time.sleep(1.0) # Hardware stabilization window

    # Camera Calibration Parameters
    K = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    
    # SCALING FACTORS
    VELOCITY_SCALE = 0.005 # Scales pixel optical flow displacement into physical m/s
    ALPHA = 0.20           # Low-pass filter smoothing coefficient
    YAW_DEADZONE = 0.035   # Rotation dead-zone threshold

    feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=25, blockSize=3)
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

    last_gray = None
    last_time = time.time()
    filtered_wz = 0.0

    try:
        csv_file = open(output_csv, mode='w', newline='')
        writer = csv.writer(csv_file)
        writer.writerow(["timestamp", "dt", "vx", "vy", "wz", "inliers"])
    except IOError as e:
        print(f"[ERROR] Failed to open CSV file for writing: {e}")
        vs.stop()
        return

    print("\n" + "="*70)
    print(" LIVE VISUAL ODOMETRY TELEMETRY NODE ACTIVE (OPTICALLY SCALED)")
    print(" Push or drive your robot to verify live vx, vy, and wz updates.")
    print(" Press Ctrl+C in terminal to stop and save CSV.")
    print("="*70 + "\n")

    try:
        while True:
            frame = vs.read()
            if frame is None: 
                time.sleep(0.005)
                continue

            current_time = time.time()
            dt = current_time - last_time

            if dt < 0.05:
                time.sleep(0.002)
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if last_gray is None:
                last_gray = gray
                last_time = current_time
                continue

            p1 = cv2.goodFeaturesToTrack(last_gray, **feature_params)
            if p1 is None or len(p1) < 12:
                print("[WARNING] Low texture / Insufficient features detected. Hold steady...", end="\r")
                last_gray = gray
                last_time = current_time
                continue

            p2, st, _ = cv2.calcOpticalFlowPyrLK(last_gray, gray, p1, None, **lk_params)
            good_p1 = p1[st == 1]
            good_p2 = p2[st == 1]

            vx, vy, wz, inliers_count = 0.0, 0.0, 0.0, 0

            if len(good_p2) >= 8:
                # 1. Homography for stable rotation (wz)
                H, mask = cv2.findHomography(good_p1, good_p2, cv2.RANSAC, 3.0)
                
                if H is not None and mask is not None:
                    inliers_mask = (mask.ravel() == 1)
                    inliers_count = int(np.sum(inliers_mask))
                    
                    if inliers_count > 15:
                        # Extract rotation matrix approximation from Homography for wz
                        # H matrix top-left represents affine rotation/scale
                        # Decompose rotation angle directly from H or use standard 2D rotation extraction
                        theta = np.arctan2(H[1, 0], H[0, 0])
                        raw_wz = float(theta / dt)

                        if abs(raw_wz) < YAW_DEADZONE:
                            raw_wz = 0.0

                        filtered_wz = (ALPHA * raw_wz) + ((1.0 - ALPHA) * filtered_wz)
                        wz = filtered_wz

                        # 2. Robust Optical Flow Displacement for Linear Velocities (vx, vy)
                        valid_p1 = good_p1[inliers_mask]
                        valid_p2 = good_p2[inliers_mask]
                        
                        if len(valid_p2) > 5:
                            displacements = valid_p2 - valid_p1 # shape (N, 1, 2) or (N, 2)
                            displacements = displacements.reshape(-1, 2)
                            
                            # Median pixel shift filters out outlier moving objects or shadows
                            median_dx = np.median(displacements[:, 0])
                            median_dy = np.median(displacements[:, 1])

                            # Map pixel displacement to physical velocity (m/s)
                            # Vertical pixel flow (dy) -> Forward velocity (vx)
                            # Horizontal pixel flow (dx) -> Lateral velocity (vy)
                            vx = float(median_dy / dt) * VELOCITY_SCALE
                            vy = float(-median_dx / dt) * VELOCITY_SCALE
                    else:
                        filtered_wz *= 0.8
                        wz = filtered_wz
                else:
                    filtered_wz *= 0.8
                    wz = filtered_wz

            writer.writerow([current_time, dt, vx, vy, wz, inliers_count])

            # Live terminal feedback
            print(f"[VO LIVE] Vx: {vx:+6.3f} m/s | Vy: {vy:+6.3f} m/s | Wz: {wz:+6.3f} rad/s | Inliers: {inliers_count:2d}", end="\r")

            last_gray = gray
            last_time = current_time

    except KeyboardInterrupt:
        print("\n\n[INFO] Terminating visual pipeline safely...")
    finally:
        vs.stop()
        csv_file.close()
        print(f"[SUCCESS] Operational telemetry log saved to: {os.path.abspath(output_csv)}")

if __name__ == "__main__":
    run_robot_vo_pipeline()