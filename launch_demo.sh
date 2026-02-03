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

echo "[1/9] Starting Gazebo..."
gnome-terminal --title="GAZEBO" -- bash -c '
source /opt/ros/jazzy/setup.bash
export GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH:/opt/ros/jazzy/share/turtlebot3_gazebo/models
gz sim -s -r ~/turtlebot3_ws/src/worlds/maze_world_with_robot.sdf
exec bash'
sleep 8

echo "[2/9] Starting Bridge..."
gnome-terminal --title="BRIDGE" -- bash -c '
source /opt/ros/jazzy/setup.bash
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/TwistStamped]gz.msgs.Twist /scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan /odom@nav_msgs/msg/Odometry[gz.msgs.Odometry /tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V /clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock
exec bash'
sleep 3

echo "[3/9] Starting TF publishers..."
gnome-terminal --title="TF_STATIC" -- bash -c '
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher -0.064 0 0.132 0 0 0 base_footprint base_scan &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint imu_link &
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
wait
exec bash'
sleep 3

echo "[4/9] Starting Nav2 (wait ~40 seconds)..."
gnome-terminal --title="NAV2" -- bash -c '
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=true map:=/home/liam-debono/turtlebot3_ws/src/maps/maze_map_clean.yaml params_file:=/opt/ros/jazzy/share/turtlebot3_navigation2/param/waffle.yaml autostart:=true
exec bash'

echo "Waiting 40 seconds for Nav2 to fully initialize..."
sleep 40

echo "[5/9] Applying costmap fixes..."
ros2 param set /global_costmap/global_costmap track_unknown_space false
ros2 param set /global_costmap/global_costmap unknown_cost_value -1
sleep 2

echo "[6/9] Setting robot speed..."
ros2 param set /controller_server FollowPath.max_vel_x 0.5
ros2 param set /controller_server FollowPath.min_vel_x 0.15
ros2 param set /controller_server FollowPath.max_vel_theta 1.5
ros2 param set /velocity_smoother max_velocity "[0.7, 0.0, 2.0]"
sleep 1

echo "[7/9] Teleporting robot to (7, 0)..."
gz service -s /world/maze_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: "waffle", position: {x: 7, y: 0, z: 0.1}'
sleep 2

echo "[8/9] Setting initial pose at (7, 0)..."
ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: "map"}, pose: {pose: {position: {x: 7.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' --once
sleep 2

echo "[9/9] Opening Gazebo GUI and Teleop..."
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

echo ""
echo "============================================"
echo "  READY! System fully initialized."
echo "============================================"
echo ""
echo "Robot at (7, 0)"
echo ""
echo "DEMO COMMAND:"
echo "  ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose '{pose: {header: {frame_id: \"map\"}, pose: {position: {x: 3.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}'"
echo ""
echo "TELEPORT BACK TO START:"
echo "  gz service -s /world/maze_world/set_pose --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --req 'name: \"waffle\", position: {x: 7, y: 0, z: 0.1}'"
echo "  ros2 topic pub /initialpose geometry_msgs/msg/PoseWithCovarianceStamped '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 7.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}' --once"
echo ""
echo "============================================"
