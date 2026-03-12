#!/bin/bash

clear
echo "=== Start update ==="

WS="$HOME/ros2_ws"
MAP_DIR="$WS/src/go2_ros2_light/go2_robot_sdk/map"

if ls office* 1> /dev/null 2>&1; then
    echo "Moving map files..."
    rm -f "$MAP_DIR"/*
    mv office* "$MAP_DIR/"
else
    echo "Only old maps will be used"
fi

echo "Step 2: Building workspace..."
cd "$WS"
colcon build

echo "Step 3: Sourcing setup..."
source install/setup.bash

echo ""
cat theta.txt
echo "=== Done ==="
