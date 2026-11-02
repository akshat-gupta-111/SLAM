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

# --- Main Visual Odometry Telemetry Node (Homography-Based) ---
def run_robot_vo_pipeline(output_csv="robot_ekf_input.csv", camera_index=0):
    vs = VideoStreamAsync(src=camera_index).start()
    time.sleep(1.0) # Hardware stabilization window

    # Camera Calibration Parameters
    K = np.array([[615.0, 0.0, 320.0], [0.0, 615.0, 240.0], [0.0, 0.0, 1.0]], dtype=np.float32)
    
    # PHYSICAL REAL-WORLD SCALE FACTOR
    ROBOT_BASE_SPEED_MPS = 0.3 
    
    # LOW-PASS FILTER COEFFICIENT (Smoothing Factor: lower = smoother, higher = more responsive)
    ALPHA = 0.20
    
    # DEAD-ZONE THRESHOLD (Ignore rotations below this noise floor in rad/s)
    YAW_DEADZONE = 0.035

    feature_params = dict(maxCorners=100, qualityLevel=0.03, minDistance=25, blockSize=3)
    lk_params = dict(winSize=(15, 15), maxLevel=2, criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 20, 0.03))

    last_gray = None
    last_time = time.time()
    filtered_wz = 0.0

    csv_file = open(output_csv, mode='w', newline='')
    writer = csv.writer(csv_file)
    writer.writerow(["timestamp", "dt", "vx", "vy", "wz", "inliers"])

    print("\n[INFO] Multithreaded Homography VO Node active on Jetson Orin.")
    print("[INFO] Headless operational state running. Press Ctrl+C in terminal to stop.")

    try:
        while True:
            frame = vs.read()
            if frame is None: 
                continue

            current_time = time.time()
            dt = current_time - last_time  # Proper time delta subtraction

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
                last_gray = gray
                last_time = current_time
                continue

            p2, st, _ = cv2.calcOpticalFlowPyrLK(last_gray, gray, p1, None, **lk_params)
            good_p1 = p1[st == 1]
            good_p2 = p2[st == 1]

            vx, vy, wz, inliers_count = 0.0, 0.0, 0.0, 0

            if len(good_p2) >= 8:
                # --- HOMOGRAPHY REPLACEMENT FOR PLANAR GROUND ROBOT ---
                # Computes 3x3 transformation matrix assuming features lie on a flat floor plane (Z = 0)
                H, mask = cv2.findHomography(good_p1, good_p2, cv2.RANSAC, 3.0)
                
                if H is not None and mask is not None:
                    inliers_count = int(np.sum(mask))
                    if inliers_count > 15:  # High confidence threshold
                        # Decompose homography into rotation and translation solutions
                        num_solutions, Rs, Ts, normals = cv2.decomposeHomographyMat(H, K)
                        
                        if num_solutions > 0:
                            R_best = Rs[0]
                            t_best = Ts[0]
                            
                            # Scale Translation for linear velocities
                            vx = float(t_best[0]) * ROBOT_BASE_SPEED_MPS
                            vy = float(t_best[1]) * ROBOT_BASE_SPEED_MPS

                            # Extract stable planar yaw rotation directly from rotation matrix
                            delta_yaw = np.arctan2(R_best[1, 0], R_best[0, 0])
                            raw_wz = float(delta_yaw / dt)

                            # Apply Dead-zone: if rotation is within noise floor, force to zero
                            if abs(raw_wz) < YAW_DEADZONE:
                                raw_wz = 0.0

                            # Apply low-pass exponential smoothing
                            filtered_wz = (ALPHA * raw_wz) + ((1.0 - ALPHA) * filtered_wz)
                            wz = filtered_wz
                        else:
                            filtered_wz *= 0.8
                            wz = filtered_wz
                    else:
                        filtered_wz *= 0.8
                        wz = filtered_wz
                else:
                    filtered_wz *= 0.8
                    wz = filtered_wz

            writer.writerow([current_time, dt, vx, vy, wz, inliers_count])
            last_gray = gray
            last_time = current_time

    except KeyboardInterrupt:
        print("\n[INFO] Terminating visual pipeline safely...")
    finally:
        vs.stop()
        csv_file.close()
        print(f"[SUCCESS] Telemetry log saved to: {os.path.abspath(output_csv)}")

if __name__ == "__main__":
    run_robot_vo_pipeline()