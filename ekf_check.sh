#!/bin/bash
LABEL=${1:-run}
OUT=~/Desktop/SLAM/ekf_report_${LABEL}_$(date +%H%M%S).txt
source ~/ros2_ws/install/setup.bash
TMP=$(mktemp -d)
stats() { grep -E "average (rate|delay)|min:" "$1" | tail -n 2; }

echo "step 1/4: CPU snapshot (3 s)..."
top -b -n1 -o %CPU | head -n 14 > $TMP/top.txt
timeout 3 tegrastats > $TMP/tegra.txt 2>&1

echo "step 2/4: rates (8 s)..."
for t in /scan /odom_rf2o_corrected /odometry/filtered; do
  n=$(echo $t | tr '/' '_')
  timeout -s INT 8 ros2 topic hz $t > $TMP/hz$n.txt 2>&1 &
done
wait

echo "step 3/4: delays (8 s)..."
for t in /scan /odom_rf2o_corrected /imu/data; do
  n=$(echo $t | tr '/' '_')
  timeout -s INT 8 ros2 topic delay $t > $TMP/delay$n.txt 2>&1 &
done
wait

echo "step 4/4: snapshots (6 s)..."
timeout 6 ros2 topic echo /diagnostics --once > $TMP/diag.txt 2>&1 &
timeout 6 ros2 topic echo /odometry/filtered --once > $TMP/odom.txt 2>&1 &
timeout -s INT 5 ros2 run tf2_ros tf2_echo odom base_link > $TMP/tf.txt 2>&1 &
wait

{
echo "===== EKF CHECK [$LABEL] $(date) ====="
echo; echo "----- TOP CPU USERS (before the test added load) -----"; cat $TMP/top.txt
echo; echo "----- JETSON -----"; tail -n 1 $TMP/tegra.txt; uptime
echo; echo "----- NODES -----"; ros2 node list
echo; echo "----- /tf PUBLISHERS ONLY -----"
ros2 topic info /tf -v | grep -E "Node name|Endpoint type" | paste - - | grep PUBLISHER
echo; echo "----- rf2o publish_tf (must be False) -----"
ros2 param get /CLaserOdometry2DNode publish_tf 2>&1
for t in /scan /odom_rf2o_corrected /odometry/filtered; do
  n=$(echo $t | tr '/' '_')
  echo; echo "----- $t RATE -----"; stats $TMP/hz$n.txt
done
for t in /scan /odom_rf2o_corrected /imu/data; do
  n=$(echo $t | tr '/' '_')
  echo; echo "----- $t DELAY -----"; stats $TMP/delay$n.txt
done
echo; echo "----- DIAGNOSTICS -----"; cat $TMP/diag.txt
echo; echo "----- NaN/inf count in odometry (0 is good) -----"; grep -ciwE "nan|inf" $TMP/odom.txt
echo; echo "----- FILTERED ODOMETRY (covariance tables hidden) -----"
grep -v -E "^  - [-0-9]" $TMP/odom.txt
echo; echo "----- TF odom -> base_link -----"; head -n 12 $TMP/tf.txt
echo; echo "----- EKF LOG WARNINGS / ERRORS -----"
LOG=$(grep -rl "EKF ready" ~/.ros/log 2>/dev/null | xargs -r ls -t 2>/dev/null | head -n 1)
if [ -z "$LOG" ]; then echo "no EKF log file found"; else
  echo "log file: $LOG"
  echo "WARN: $(grep -c WARN "$LOG")   ERROR: $(grep -c ERROR "$LOG")"
  grep -E "WARN|ERROR" "$LOG" | tail -n 10
fi
echo; echo "===== END ====="
} > $OUT 2>&1
rm -rf $TMP
echo "Report saved: $OUT"
