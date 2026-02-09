#!/bin/bash
# TurtleBot4 控制脚本

# 设置 ROS2 日志环境变量
export ROS_LOG_DIR=/tmp/ros2_logs
export RCL_LOG_STDOUT_ONLY=1

if [ $# -lt 1 ]; then
    echo "用法: $0 <command> [速度] [持续时间(s)]"
    echo ""
    echo "命令:"
    echo "  forward <速度> [duration]   - 前进（默认 0.5 m/s，1s）"
    echo "  backward <速度> [duration]  - 后退（默认 0.5 m/s，1s）"
    echo "  left <角速度> [duration]    - 左转（默认 0.5 rad/s，1s）"
    echo "  right <角速度> [duration]   - 右转（默认 0.5 rad/s，1s）"
    echo "  stop               - 停止"
    echo "  move <linear> <angular> - 自定义线速度和角速度"
    echo ""
    echo "示例:"
    echo "  $0 forward 0.3     # 以 0.3 m/s 前进"
    echo "  $0 left 0.5        # 以 0.5 rad/s 左转"
    echo "  $0 move 0.5 0.3    # 线速度 0.5 m/s，角速度 0.3 rad/s"
    exit 1
fi

COMMAND=$1
LINEAR_X=0.0
ANGULAR_Z=0.0
# 控制持续时间（秒），默认 1 秒
DURATION=1.0

case "$COMMAND" in
    forward)
        # 参数始终按“速度大小”理解，方向由命令名决定
        SPEED=${2:-0.5}
        # 去掉用户误写的负号，保证 forward 一定是正向
        SPEED=${SPEED#-}
        LINEAR_X=${SPEED}
        DURATION=${3:-1.0}
        ANGULAR_Z=0.0
        echo "前进: ${LINEAR_X} m/s"
        ;;
    backward)
        SPEED=${2:-0.5}
        SPEED=${SPEED#-}
        LINEAR_X=-${SPEED}
        ANGULAR_Z=0.0
        DURATION=${3:-1.0}
        echo "后退: ${LINEAR_X#-} m/s"
        ;;
    left)
        LINEAR_X=0.0
        SPEED=${2:-0.5}
        SPEED=${SPEED#-}
        ANGULAR_Z=${SPEED}
        DURATION=${3:-1.0}
        echo "左转: ${ANGULAR_Z} rad/s"
        ;;
    right)
        LINEAR_X=0.0
        SPEED=${2:-0.5}
        SPEED=${SPEED#-}
        ANGULAR_Z=-${SPEED}
        DURATION=${3:-1.0}
        echo "右转: ${ANGULAR_Z#-} rad/s"
        ;;
    stop)
        LINEAR_X=0.0
        ANGULAR_Z=0.0
        # 停止命令短时间多发几次，确保生效
        DURATION=0.5
        echo "停止"
        ;;
    move)
        LINEAR_X=${2:-0.0}
        ANGULAR_Z=${3:-0.0}
        echo "移动: 线速度=${LINEAR_X} m/s, 角速度=${ANGULAR_Z} rad/s"
        ;;
    *)
        echo "错误: 未知命令 '$COMMAND'"
        exit 1
        ;;
esac

# TurtleBot4(Create3) 默认底盘控制器名是 diffdrive_controller（没有下划线）
# 并且订阅的是 TwistStamped，且 QoS 为 BEST_EFFORT（否则会一直 “Waiting for subscription...”）
CMD_VEL_TOPIC="/turtlebot4/diffdrive_controller/cmd_vel"

# 持续一段时间以一定频率发布，让机器人真正跑一段路
timeout "${DURATION}s" ros2 topic pub \
  --qos-reliability best_effort \
  -r 10 \
  "${CMD_VEL_TOPIC}" geometry_msgs/msg/TwistStamped "
{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: ''},
  twist: {
    linear: {x: ${LINEAR_X}, y: 0.0, z: 0.0},
    angular: {x: 0.0, y: 0.0, z: ${ANGULAR_Z}}
  }
}"

echo "命令已发送"

