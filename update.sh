#!/bin/bash

clear
echo "=== Start update ==="

if ls office* 1> /dev/null 2>&1; then
    echo "Moving map files..."
    rm -f /home/pcip/ros2_ws/src/go2_ros2_light/go2_robot_sdk/map/*
    mv office* /home/pcip/ros2_ws/src/go2_ros2_light/go2_robot_sdk/map/
else
    echo "Only old maps will be used"
fi

echo "Step 2: Building workspace..."
cd /home/pcip/ros2_ws
colcon build

echo "Step 3: Sourcing setup..."
source install/setup.bash

echo ""
cat theta.txt
echo "=== Done ==="
