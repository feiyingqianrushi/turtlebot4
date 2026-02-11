#!/bin/bash
# Diagnose TF bridge issues

echo "=== TF Bridge Diagnosis ==="
echo ""

echo "1. Checking /turtlebot4/tf topic..."
if ros2 topic list | grep -q "^/turtlebot4/tf$"; then
    echo "   ✓ /turtlebot4/tf topic exists"
    PUB_COUNT=$(ros2 topic info /turtlebot4/tf 2>&1 | grep "Publisher count" | grep -oP '\d+')
    echo "   Publisher count: $PUB_COUNT"
    
    echo "   Checking for odom->base_link transform..."
    if timeout 2 ros2 topic echo /turtlebot4/tf --once 2>&1 | grep -q "turtlebot4/odom"; then
        echo "   ✓ Found turtlebot4/odom -> turtlebot4/base_link in /turtlebot4/tf"
    else
        echo "   ✗ turtlebot4/odom -> turtlebot4/base_link NOT found in /turtlebot4/tf"
    fi
else
    echo "   ✗ /turtlebot4/tf topic does not exist"
fi
echo ""

echo "2. Checking /tf topic..."
if ros2 topic list | grep -q "^/tf$"; then
    echo "   ✓ /tf topic exists"
    PUB_COUNT=$(ros2 topic info /tf 2>&1 | grep "Publisher count" | grep -oP '\d+')
    echo "   Publisher count: $PUB_COUNT"
    
    echo "   Checking for odom->base_link transform..."
    if timeout 2 ros2 topic echo /tf --once 2>&1 | grep -q "frame_id: odom"; then
        echo "   ✓ Found odom -> base_link in /tf"
    else
        echo "   ✗ odom -> base_link NOT found in /tf"
        echo "   Available transforms:"
        timeout 2 ros2 topic echo /tf --once 2>&1 | grep -E "(frame_id|child_frame_id)" | head -10
    fi
else
    echo "   ✗ /tf topic does not exist"
fi
echo ""

echo "3. Checking tf_bridge node..."
if ros2 node list | grep -q tf_bridge; then
    echo "   ✓ tf_bridge node is running"
    echo "   Node info:"
    ros2 node info /tf_bridge 2>&1 | grep -E "(Subscribers|Publishers)" | head -4
else
    echo "   ✗ tf_bridge node is NOT running"
fi
echo ""

echo "4. Testing TF lookup..."
if timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "   ✓ odom -> base_link transform is available via tf2"
else
    echo "   ✗ odom -> base_link transform is NOT available via tf2"
    echo "   Error:"
    timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -3
fi
echo ""

echo "=== Recommendations ==="
if ! timeout 1 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | grep -q "Translation:"; then
    echo "1. Restart the launch file to ensure tf_bridge is running"
    echo "2. Check tf_bridge logs for errors:"
    echo "   ros2 topic echo /rosout | grep tf_bridge"
    echo "3. Verify tf_bridge is receiving data from /turtlebot4/tf"
fi
echo ""
