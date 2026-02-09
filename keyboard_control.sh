#!/bin/bash
# TurtleBot4 键盘控制脚本
# 使用 W/A/S/D 控制小车前进/左转/后退/右转

source /opt/ros/jazzy/setup.bash
cd ~/ros2_learn/L05_turtlebot4
source install/setup.bash 2>/dev/null

echo "=========================================="
echo "TurtleBot4 键盘控制"
echo "=========================================="
echo ""
echo "控制说明："
echo "  w/W  - 前进"
echo "  s/S  - 后退"
echo "  a/A  - 左转"
echo "  d/D  - 右转"
echo "  x/X  - 停止"
echo "  q/Q  - 退出"
echo ""
echo "速度控制："
echo "  按一次键：低速"
echo "  按住键：持续移动"
echo ""
echo "=========================================="
echo ""

# 使用 teleop_twist_keyboard，但需要重映射话题
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args \
    -r /cmd_vel:=/turtlebot4/diffdrive_controller/cmd_vel

