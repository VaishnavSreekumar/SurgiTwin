#!/usr/bin/env bash
set -e

source /opt/ros/jazzy/setup.bash
source /home/vaish123/robotics/SurgiTwin/install/setup.bash

echo "Starting SurgiTwin Safety Controller..."
ros2 run surgical_sim surgitwin_safety_controller &
NODE_PID=$!
sleep 2

echo -e "\n========================================================"
echo "STEP 1: Publishing Valid Target Pose (0.00, 0.35, 0.20) m"
echo "========================================================"
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.0, y: 0.35, z: 0.20}, orientation: {w: 1.0}}}"
sleep 1

echo -e "\n========================================================"
echo "STEP 2: Publishing Out-Of-Bounds Target (0.80, 0.35, 0.20) m"
echo "========================================================"
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.80, y: 0.35, z: 0.20}, orientation: {w: 1.0}}}"
sleep 1

echo -e "\n========================================================"
echo "STEP 3: Triggering Emergency Stop via Service"
echo "========================================================"
ros2 service call /safety/estop std_srvs/srv/Trigger "{}"
sleep 1

echo -e "\n========================================================"
echo "STEP 4: Publishing Target During E-Stop State (Blocked)"
echo "========================================================"
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.02, y: 0.35, z: 0.20}, orientation: {w: 1.0}}}"
sleep 1

echo -e "\n========================================================"
echo "STEP 5: Resetting Emergency Stop via Service"
echo "========================================================"
ros2 service call /safety/reset std_srvs/srv/Trigger "{}"
sleep 1

echo -e "\n========================================================"
echo "STEP 6: Publishing Target After Reset (Normal Resumed)"
echo "========================================================"
ros2 topic pub --once /target_pose geometry_msgs/msg/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'base_link'}, pose: {position: {x: 0.05, y: 0.35, z: 0.20}, orientation: {w: 1.0}}}"
sleep 1

echo -e "\nStopping Safety Controller..."
kill $NODE_PID 2>/dev/null || true
wait $NODE_PID 2>/dev/null || true

echo -e "\nChecking generated safety events CSV..."
cat ~/surgitwin_safety_events.csv | tail -n 10
echo -e "\n=== Demonstration Complete ==="

