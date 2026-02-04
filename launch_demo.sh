#!/bin/bash
# ===========================================
# ADAPTIVE MAZE NAVIGATOR - FULL AUTO LAUNCH
# Runs EVERYTHING automatically
# ===========================================

echo "Killing any existing processes..."
pkill -f "gz sim" 2>/dev/null
pkill -f "ros_gz_bridge" 2>/dev/null
pkill -f "nav2" 2>/dev/null
pkill -f "component_container" 2>/dev/null
pkill -f "static_transform_publisher" 2>/dev/null
sleep 3

source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models

echo "[1/7] Starting Gazebo..."
gnome-terminal --title="GAZEBO" -- bash -c '
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
gz sim -s -r ~/turtlebot3_ws/src/worlds/maze_world_with_robot.sdf
exec bash'
sleep 8

echo "[2/7] Starting Bridge..."
gnome-terminal --title="BRIDGE" -- bash -c '
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
exec bash'
sleep 3

echo "[3/7] Starting TF publishers..."
gnome-terminal --title="TF_STATIC" -- bash -c '
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher -0.064 0 0.132 0 0 0 base_footprint base_scan &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint imu_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
wait
exec bash'
sleep 3

echo "[4/7] Starting Nav2 (wait ~40 seconds)..."
gnome-terminal --title="NAV2" -- bash -c '
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/liam-debono/turtlebot3_ws/src/maps/maze_map_clean.yaml params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle.yaml autostart:=true
exec bash'

echo "Waiting 40 seconds for Nav2 to fully initialize..."
sleep 40

echo "[5/7] Applying costmap fixes..."
ros2 param set /global_costmap/global_costmap track_unknown_space false
ros2 param set /global_costmap/global_costmap unknown_cost_value -1
sleep 2

echo "[6/7] Opening Gazebo GUI and Teleop..."
gnome-terminal --title="GAZEBO_GUI" -- bash -c '
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
gz sim -g
exec bash'

gnome-terminal --title="TELEOP" -- bash -c '
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
echo "=========================================="
echo "  TELEOP CONTROLS"
echo "=========================================="
echo "  W = forward"
echo "  X = backward"
echo "  A = turn left"
echo "  D = turn right"
echo "  S = stop"
echo "=========================================="
echo ""
ros2 run turtlebot3_teleop teleop_keyboard
exec bash'
sleep 2

echo "[7/7] Setting initial pose at (7, 0)..."
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 7.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' --once
sleep 2

echo ""
echo "============================================"
echo "  READY! System fully initialized."
echo "============================================"
echo ""
echo "Robot spawns at (7, 0)"
echo ""
echo "DEMO COMMAND:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: \"map\"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'"
echo ""
echo "============================================"
echo ""
echo "============================================"
echo "  READY! Starting demo in 5 seconds..."
echo "============================================"
sleep 5

# Metrics setup
METRICS_DIR=~/turtlebot3_ws/metrics
LOG_FILE=$METRICS_DIR/nav_metrics_$(date +%Y%m%d_%H%M%S).txt
mkdir -p $METRICS_DIR

echo "Starting navigation to (3.0, 0.0)..." | tee $LOG_FILE
START_TIME=$(date +%s.%N)

ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}"

END_TIME=$(date +%s.%N)
NAV_TIME=$(echo "$END_TIME - $START_TIME" | bc)

echo "" | tee -a $LOG_FILE
echo "============================================" | tee -a $LOG_FILE
echo "METRICS" | tee -a $LOG_FILE
echo "============================================" | tee -a $LOG_FILE
echo "Navigation Time: ${NAV_TIME} seconds" | tee -a $LOG_FILE
echo "Start: (7.0, 0.0)" | tee -a $LOG_FILE
echo "Goal: (3.0, 0.0)" | tee -a $LOG_FILE
echo "Log saved to: $LOG_FILE" | tee -a $LOG_FILE
echo "============================================" | tee -a $LOG_FILE
