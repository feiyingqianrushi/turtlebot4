#!/bin/bash
# Check SLAM status and diagnose issues

echo "=== SLAM Status Check ==="
echo ""

# 1. Check if SLAM node is running
echo "1. Checking SLAM node..."
if ros2 node list | grep -q slam_toolbox; then
    echo "   ✓ SLAM node is running"
    STATE=$(ros2 lifecycle get /slam_toolbox 2>&1 | grep -oP 'active|inactive|unconfigured|shutdown')
    echo "   State: $STATE"
else
    echo "   ✗ SLAM node is NOT running"
    exit 1
fi
echo ""

# 2. Check /scan topic
echo "2. Checking /scan topic..."
if ros2 topic list | grep -q "^/scan$"; then
    SCAN_PUB=$(ros2 topic info /scan 2>&1 | grep "Publisher count" | grep -oP '\d+')
    if [ "$SCAN_PUB" -gt 0 ]; then
        echo "   ✓ /scan topic has $SCAN_PUB publisher(s)"
        RATE=$(timeout 2 ros2 topic hz /scan 2>&1 | grep "average rate" | head -1 | grep -oP '\d+\.\d+')
        if [ -n "$RATE" ]; then
            echo "   Publishing rate: $RATE Hz"
        fi
    else
        echo "   ✗ /scan topic has no publisher"
    fi
else
    echo "   ✗ /scan topic does not exist"
fi
echo ""

# 3. Check TF transforms
echo "3. Checking TF transforms..."
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "   ✓ odom -> base_link transform exists"
else
    echo "   ✗ odom -> base_link transform NOT found"
fi

if timeout 1 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation:"; then
    echo "   ✓ map -> odom transform exists (SLAM is working!)"
else
    echo "   ✗ map -> odom transform NOT found (SLAM hasn't started mapping yet)"
fi
echo ""

# 4. Check /map topic
echo "4. Checking /map topic..."
if ros2 topic list | grep -q "^/map$"; then
    MAP_PUB=$(ros2 topic info /map 2>&1 | grep "Publisher count" | grep -oP '\d+')
    if [ "$MAP_PUB" -gt 0 ]; then
        echo "   ✓ /map topic has $MAP_PUB publisher(s)"
        if timeout 2 ros2 topic echo /map --once 2>&1 | grep -q "header:"; then
            echo "   ✓ /map topic is publishing data"
            RATE=$(timeout 2 ros2 topic hz /map 2>&1 | grep "average rate" | head -1 | grep -oP '\d+\.\d+')
            if [ -n "$RATE" ]; then
                echo "   Publishing rate: $RATE Hz"
            fi
        else
            echo "   ✗ /map topic exists but no data is being published"
            echo "   → Robot may need to move more (minimum_travel_distance: 0.1m)"
        fi
    else
        echo "   ✗ /map topic has no publisher"
    fi
else
    echo "   ✗ /map topic does not exist"
fi
echo ""

# 5. Recommendations
echo "=== Recommendations ==="
if ! timeout 1 ros2 run tf2_ros tf2_echo map odom 2>&1 | grep -q "Translation:"; then
    echo "1. Make sure robot has moved at least 0.1m or rotated 0.1 rad"
    echo "2. Use keyboard control to move the robot:"
    echo "   ros2 run turtlebot4_description keyboard_control.py"
    echo "3. Wait a few seconds after moving"
    echo "4. Check again: ros2 run tf2_ros tf2_echo map odom"
fi
echo ""
