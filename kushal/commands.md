### Node to listen RPLidar /scan and then perform the mathematical calculation of Odometry and then brodcast the information to /odom_rf2o topic and don't publish the tf.

> ros2 run rf2o_laser_odometry rf2o_laser_odometry_node --ros-args -p laser_scan_topic:=/scan -p odom_topic:=/odom_rf2o -p publish_tf:=false