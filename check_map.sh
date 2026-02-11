#!/bin/bash
# Check why /map has no data

echo "=== Map Data Diagnosis ==="
echo ""

echo "1. Checking SLAM node state..."
STATE=$(ros2 lifecycle get /slam_toolbox 2>&1 | grep -oP 'active|inactive|unconfigured|shutdown')
echo "   State: $STATE"
if [ "$STATE" != "active" ]; then
    echo "   ⚠️  SLAM node is not active! Activating..."
    ros2 lifecycle set /slam_toolbox configure
    sleep 1
    ros2 lifecycle set /slam_toolbox activate
    sleep 1
    STATE=$(ros2 lifecycle get /slam_toolbox 2>&1 | grep -oP 'active|inactive|unconfigured|shutdown')
    echo "   New state: $STATE"
fi
echo ""

echo "2. Checking /scan topic..."
SCAN_PUB=$(ros2 topic info /scan 2>&1 | grep "Publisher count" | grep -oP '\d+')
if [ "$SCAN_PUB" -gt 0 ]; then
    echo "   ✓ /scan has $SCAN_PUB publisher(s)"
    RATE=$(timeout 2 ros2 topic hz /scan 2>&1 | grep "average rate" | head -1 | grep -oP '\d+\.\d+')
    if [ -n "$RATE" ]; then
        echo "   ✓ Publishing rate: $RATE Hz"
    else
        echo "   ⚠️  No data on /scan topic"
    fi
else
    echo "   ✗ /scan has no publisher"
fi
echo ""

echo "3. Checking TF transforms..."
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "   ✓ odom -> base_link transform exists"
else
    echo "   ✗ odom -> base_link transform NOT found"
    echo "   → This is required for SLAM to work!"
    echo "   → Restart the launch file to apply tf_bridge fix"
fi

if timeout 1 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation:"; then
    echo "   ✓ map -> odom transform exists (SLAM is working!)"
else
    echo "   ✗ map -> odom transform NOT found (SLAM hasn't started mapping yet)"
fi
echo ""

echo "4. Checking /map topic..."
MAP_PUB=$(ros2 topic info /map 2>&1 | grep "Publisher count" | grep -oP '\d+')
if [ "$MAP_PUB" -gt 0 ]; then
    echo "   ✓ /map has $MAP_PUB publisher(s)"
    
    # Try to get map data
    if timeout 3 ros2 topic echo /map --once 2>&1 | grep -q "header:"; then
        echo "   ✓ /map topic is publishing data"
        MAP_SIZE=$(timeout 2 ros2 topic echo /map --once 2>&1 | grep -E "(width|height)" | head -2)
        echo "   Map size: $MAP_SIZE"
    else
        echo "   ⚠️  /map topic exists but no data is being published"
        echo "   → Robot may need to move more (minimum_travel_distance: 0.05m)"
        echo "   → Or SLAM may be waiting for odom->base_link transform"
    fi
else
    echo "   ✗ /map has no publisher"
fi
echo ""

echo "=== Recommendations ==="
if ! timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "1. Restart the launch file to apply tf_bridge fix:"
    echo "   ros2 launch turtlebot4_description turtlebot4_slam.launch.py"
fi

if [ "$STATE" = "active" ] && [ "$MAP_PUB" -gt 0 ]; then
    if ! timeout 2 ros2 topic echo /map --once 2>&1 | grep -q "header:"; then
        echo "2. Move the robot to start mapping:"
        echo "   ros2 run turtlebot4_description keyboard_control.py"
        echo "   Use W/A/S/D to move the robot at least 0.05m"
    fi
fi
echo ""
