#!/bin/bash
# Save map from SLAM Toolbox
# This script waits for the /map topic to be available and then saves it.

OUTPUT_FILE="${1:-$HOME/ros2_learn/L05_turtlebot4/maps/my_map}"
TIMEOUT="${2:-30}"

echo "Waiting for /map topic to be available..."
echo "Make sure SLAM is running and robot has moved to generate map data."
echo ""

# Wait for /map topic to be available
COUNTER=0
while [ $COUNTER -lt $TIMEOUT ]; do
    # Hide occasional BrokenPipeError from ros2 topic list when grep closes pipe
    if ros2 topic list 2>/dev/null | grep -q "^/map$"; then
        # Check if topic is actually publishing
        if timeout 1 ros2 topic echo /map --once > /dev/null 2>&1; then
            echo "✓ /map topic is available!"
            break
        fi
    fi
    echo -n "."
    sleep 1
    COUNTER=$((COUNTER + 1))
done

if [ $COUNTER -ge $TIMEOUT ]; then
    echo ""
    echo "✗ Timeout waiting for /map topic after ${TIMEOUT} seconds"
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure SLAM is running:"
    echo "   ros2 launch turtlebot4_description turtlebot4_slam.launch.py"
    echo ""
    echo "2. Make sure robot has moved (SLAM needs scan data to generate map):"
    echo "   ros2 run turtlebot4_description keyboard_control.py"
    echo "   # Press W/A/S/D to move the robot"
    echo ""
    echo "3. Check if /map topic exists:"
    echo "   ros2 topic list | grep map"
    echo ""
    echo "4. Try to receive map data:"
    echo "   timeout 2 ros2 topic echo /map --once"
    exit 1
fi

echo ""
echo "Saving map to: ${OUTPUT_FILE}"

# Ensure directory exists
mkdir -p "$(dirname "$OUTPUT_FILE")"

# Save map using nav2_map_server
ros2 run nav2_map_server map_saver_cli -f "$OUTPUT_FILE"

if [ $? -eq 0 ]; then
    echo ""
    echo "✓ Map saved successfully!"
    echo "  - YAML: ${OUTPUT_FILE}.yaml"
    echo "  - PGM:  ${OUTPUT_FILE}.pgm"
else
    echo ""
    echo "✗ Failed to save map"
    exit 1
fi

