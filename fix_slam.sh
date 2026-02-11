#!/bin/bash
# Fix SLAM mapping issues

echo "=== SLAM 问题诊断和修复 ==="
echo ""

echo "1. 检查当前配置..."
echo "   minimum_travel_distance: $(ros2 param get /slam_toolbox minimum_travel_distance 2>&1 | grep -oP '\d+\.\d+')"
echo "   minimum_travel_heading: $(ros2 param get /slam_toolbox minimum_travel_heading 2>&1 | grep -oP '\d+\.\d+')"
echo "   debug_logging: $(ros2 param get /slam_toolbox debug_logging 2>&1 | grep -oP 'true|false')"
echo ""

echo "2. 重要提示："
echo "   如果 SLAM 还是没有发布地图，请："
echo ""
echo "   a) 确保已经重新启动了 launch 文件（使用新配置）："
echo "      ros2 launch turtlebot4_description turtlebot4_slam.launch.py"
echo ""
echo "   b) 使用键盘控制让机器人移动："
echo "      ros2 run turtlebot4_description keyboard_control.py"
echo "      按 W 前进至少 0.05 米，或按 A/D 旋转至少 0.05 弧度"
echo ""
echo "   c) 等待几秒钟让 SLAM 处理数据"
echo ""
echo "   d) 检查地图："
echo "      ros2 topic echo /map --once"
echo "      ros2 run tf2_ros tf2_echo map odom"
echo ""

echo "3. 如果还是不行，尝试进一步降低阈值："
echo "   编辑 config/slam_toolbox.yaml，将 minimum_travel_distance 改为 0.01"
echo ""

echo "4. 检查 SLAM 日志（如果启用了 debug_logging）："
echo "   查看启动 SLAM 的终端窗口，应该能看到详细的处理信息"
echo ""
