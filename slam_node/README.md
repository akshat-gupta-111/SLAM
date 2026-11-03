# slam_node (Mapping, .pgm Saver, Localization & Checkpoint Navigation)

Ye ROS 2 package aapke `ekf_node` ke `/odometry/filtered` aur Lidar ke `/scan` ko use karke:
1. Live `/map` banata hai aur `.pgm` + `.yaml` file save karta hai.
2. Saved map ko load karke Localization aur `/nav` par navigation data deta hai.
3. Checkpoints mark karne aur un par navigate karne ke liye interactive menu (`checkpoint_cli`) deta hai.

## Setup & Symlink Build Instructions
```bash
# 1. Zip file ko ~/Desktop/SLAM/ me extract karein (jisse ~/Desktop/SLAM/slam_node folder ban jaye)
# 2. Apne ros2_ws/src me symlink banayein:
ln -s ~/Desktop/SLAM/slam_node ~/ros2_ws/src/slam_node

# 3. Workspace build aur source karein:
cd ~/ros2_ws
colcon build --packages-select slam_node --symlink-install
source install/setup.bash
```

## Kaise Chalayein?

### Step 1: Mapping Mode (Kamre ka Map banane ke liye)
Terminal 1:
```bash
ros2 launch slam_node mapping.launch.py
```
Terminal 2 (Interactive Control Panel kholne ke liye):
```bash
ros2 run slam_node checkpoint_cli
```
- Control Panel me **[1]** daba kar mapping start karein, kamre me ghoomein, **[4]** daba kar raste me Checkpoints save karein, aur **[3]** daba kar `.pgm` map save kar lein!
- (Ya seedha `ros2 run slam_node save_map my_room` chala kar `.pgm` save karein).

### Step 2: Navigation & Localization Mode (Checkpoints pe jane ke liye)
Mapping band karein (`Ctrl+C`) aur Navigation launch karein:
```bash
ros2 launch slam_node navigation.launch.py
```
Ab doosre terminal me `ros2 run slam_node checkpoint_cli` khol kar **[6]** (Go to Checkpoint) ya **[7]** (Patrol All Checkpoints) chunein!
