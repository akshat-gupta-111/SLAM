#!/bin/bash

# 1. Source ROS 2 workspace environment
if [ -f "install/setup.bash" ]; then
    source install/setup.sh
else
    source /opt/ros/humble/setup.bash
fi

echo "=================================================="
echo " Starting IMU Node & Static Transform (base_link -> imu_link)"
echo "=================================================="

# 2. Launch the static transform publisher in the background (&)
# Places imu_link at x=0, y=0, z=0.115m (11.5cm above base_link)
ros2 run tf2_ros static_transform_publisher \
    --x 0 --y 0 --z 0.115 \
    --yaw 0 --pitch 0 --roll 0 \
    --frame-id base_link \
    --child-frame-id imu_link &

# Save the process ID of the TF publisher so we can kill it later easily
TF_PID=$!

# 3. Run the calibrated IMU Python node in the foreground
# (This executes imu_worker with sudo inside Python)
python3 imu_publisher_node.py

# 4. Cleanup: When you press Ctrl+C to stop the IMU node, 
# this automatically kills the background static transform publisher too.
kill $TF_PID
