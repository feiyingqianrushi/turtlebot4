#!/bin/bash
# Comprehensive SLAM diagnosis

echo "=== SLAM 完整诊断 ==="
echo ""

echo "1. SLAM 节点状态..."
STATE=$(ros2 lifecycle get /slam_toolbox 2>&1 | grep -oP 'active|inactive|unconfigured|shutdown')
echo "   状态: $STATE"
echo ""

echo "2. 检查 TF 变换..."
echo "   检查 /tf 话题中的 odom -> base_link..."
if timeout 2 ros2 topic echo /tf 2>&1 | grep -q "frame_id: odom"; then
    echo "   ✓ /tf 话题中有 odom -> base_link 变换"
    # 获取最新的变换
    LATEST_TF=$(timeout 2 ros2 topic echo /tf --once 2>&1 | grep -A 15 "frame_id: odom" | head -20)
    echo "   最新变换:"
    echo "$LATEST_TF" | grep -E "(frame_id|child_frame_id|translation|rotation)" | head -6
else
    echo "   ✗ /tf 话题中没有 odom -> base_link 变换"
fi
echo ""

echo "3. 检查 /scan 话题..."
SCAN_PUB=$(ros2 topic info /scan 2>&1 | grep "Publisher count" | grep -oP '\d+')
if [ "$SCAN_PUB" -gt 0 ]; then
    echo "   ✓ /scan 有 $SCAN_PUB 个发布者"
    
    # 检查 scan 数据质量
    SCAN_DATA=$(timeout 2 ros2 topic echo /scan --once 2>&1)
    if echo "$SCAN_DATA" | grep -q "ranges:"; then
        INF_COUNT=$(echo "$SCAN_DATA" | grep -c "\.inf" || echo "0")
        TOTAL_RANGES=$(echo "$SCAN_DATA" | grep -E "^- [0-9]" | wc -l || echo "0")
        if [ "$INF_COUNT" -gt 0 ]; then
            echo "   ⚠️  警告: /scan 中有 $INF_COUNT 个无穷大值（.inf）"
            echo "   → 这表示激光雷达没有检测到障碍物"
            echo "   → 在空环境中，SLAM 可能无法正常工作"
        else
            echo "   ✓ /scan 数据正常"
        fi
    fi
else
    echo "   ✗ /scan 没有发布者"
fi
echo ""

echo "4. 检查 /map 话题..."
MAP_PUB=$(ros2 topic info /map 2>&1 | grep "Publisher count" | grep -oP '\d+')
if [ "$MAP_PUB" -gt 0 ]; then
    echo "   ✓ /map 有 $MAP_PUB 个发布者"
    
    if timeout 3 ros2 topic echo /map --once 2>&1 | grep -q "header:"; then
        echo "   ✓ /map 正在发布数据"
        MAP_INFO=$(timeout 2 ros2 topic echo /map --once 2>&1 | grep -E "(width|height|resolution)" | head -3)
        echo "   地图信息:"
        echo "$MAP_INFO"
    else
        echo "   ⚠️  /map 话题存在但没有数据"
        echo "   → 可能原因："
        echo "     1. 机器人还没有移动（需要至少 0.05m）"
        echo "     2. /scan 数据都是 .inf（没有障碍物）"
        echo "     3. SLAM 无法找到 odom -> base_link 变换"
    fi
else
    echo "   ✗ /map 没有发布者"
fi
echo ""

echo "5. 检查 SLAM 日志（最近 10 条）..."
timeout 3 ros2 topic echo /rosout 2>&1 | grep -i "slam_toolbox" | tail -10 | while read line; do
    if echo "$line" | grep -qiE "(error|warn|fail)"; then
        echo "   ⚠️  $line"
    fi
done
echo ""

echo "=== 建议 ==="
if [ "$SCAN_PUB" -gt 0 ]; then
    SCAN_INF=$(timeout 2 ros2 topic echo /scan --once 2>&1 | grep -c "\.inf" || echo "0")
    if [ "$SCAN_INF" -gt 100 ]; then
        echo "1. /scan 数据都是 .inf，建议："
        echo "   - 在 Gazebo 中添加一些障碍物（墙壁、盒子等）"
        echo "   - 或者检查激光雷达配置"
    fi
fi

if [ "$MAP_PUB" -gt 0 ]; then
    if ! timeout 2 ros2 topic echo /map --once 2>&1 | grep -q "header:"; then
        echo "2. 地图还没有数据，建议："
        echo "   - 使用键盘控制移动机器人："
        echo "     ros2 run turtlebot4_description keyboard_control.py"
        echo "   - 移动至少 0.05 米来触发建图"
    fi
fi
echo ""
