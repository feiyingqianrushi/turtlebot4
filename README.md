## TurtleBot4 Gazebo Demo 使用说明

### ⚠️ 1. 必须先安装依赖

TurtleBot4 **必须**依赖 iRobot Create3 的 ROS2 包才能编译。请先执行：

```bash
# 1. 更新 apt 源
sudo apt update

# 2. 安装 iRobot Create3 相关包（ROS2 Jazzy）
sudo apt install \
  ros-jazzy-irobot-create-common \
  ros-jazzy-irobot-create-description \
  ros-jazzy-irobot-create-msgs \
  ros-jazzy-irobot-create-toolbox \
  ros-jazzy-irobot-create-ignition-toolbox
```

**验证依赖是否安装成功：**

```bash
# 检查包是否存在
ros2 pkg prefix irobot_create_description
# 应该返回类似：/opt/ros/jazzy/share/irobot_create_description
```

如果上面的命令返回路径，说明依赖已安装，可以继续编译。

---

### 2. 编译工作空间（第一次或修改代码后）

```bash
cd ~/ros2_learn/L05_turtlebot4
colcon build --symlink-install

# 编译成功后，source 环境
source install/setup.bash
```

**如果编译失败，检查错误信息：**
- 如果提示 `irobot_create_description` 找不到，说明依赖未安装，请回到步骤 1
- 如果提示其他错误，请查看编译日志

---

### 3. 一键运行 Gazebo 仿真

在一个终端中：

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_learn/L05_turtlebot4
source install/setup.bash

# 启动 Gazebo + TurtleBot4（统一的推荐入口）
ros2 launch turtlebot4_description turtlebot4_gazebo.launch.py
```

这个 launch 会自动完成：

- 启动 Gazebo Harmonic（`empty.sdf` 世界）
- 生成 TurtleBot4 的 `robot_description`
- 持续发布：
  - `/robot_description`
  - `/turtlebot4/robot_description`
- 调用 `ros_gz_sim create` 把机器人 spawn 到 Gazebo 中
- 通过 `gz_ros2_control` 创建 `/turtlebot4/controller_manager` 和 `diffdrive_controller`

> 如果 Gazebo 里暂时没有看到机器人：
> - 按键盘 `Home` 重置视角
> - 或用鼠标中键拖动、右键旋转调整到原点附近

---

### 4. 控制 TurtleBot4（推荐：控制脚本）

```bash
cd ~/ros2_learn/L05_turtlebot4
source install/setup.bash

# 前进
./control_turtlebot4.sh forward 0.5

# 后退
./control_turtlebot4.sh backward 0.3

# 左转
./control_turtlebot4.sh left 0.5

# 右转
./control_turtlebot4.sh right 0.5

# 停止
./control_turtlebot4.sh stop

# 自定义速度和角速度
./control_turtlebot4.sh move 0.5 0.3
```

**方向约定（脚本内部已经处理好正负号）：**

- `forward v`：向前，以 `v` m/s 行驶（无论你输入的是 `0.5` 还是 `-0.5`，都会按“向前 0.5 m/s”理解）
- `backward v`：向后，以 `v` m/s 行驶
- `left w`：左转，以 `w` rad/s 旋转
- `right w`：右转，以 `w` rad/s 旋转
- `stop`：停止（线速度、角速度都置 0）

底层话题：

- 控制器名：`diffdrive_controller`
- 话题：`/turtlebot4/diffdrive_controller/cmd_vel`
- 消息类型：`geometry_msgs/msg/TwistStamped`
- QoS：`BEST_EFFORT`

---

### 5. 直接用 `ros2 topic pub` 控制（不通过脚本）

```bash
# 前进（0.5 m/s）
# 注意：Create3 默认订阅 TwistStamped，并且 QoS 是 BEST_EFFORT
ros2 topic pub --once --qos-reliability best_effort /turtlebot4/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

# 左转（0.5 rad/s）
ros2 topic pub --once --qos-reliability best_effort /turtlebot4/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}}"

# 停止
ros2 topic pub --once --qos-reliability best_effort /turtlebot4/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
```
#### 持续发布命令（让机器人持续运动）

```bash
# 持续前进（按 Ctrl+C 停止）
ros2 topic pub --qos-reliability best_effort /turtlebot4/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}" -r 10
```

#### 键盘控制（如果安装了 teleop 包）

```bash
# 需要先安装 turtlebot4_navigation 包
ros2 run turtlebot4_navigation turtlebot4_teleop
```

---

### 6. 查看机器人状态 / 调试

```bash
# 查看关节状态
ros2 topic echo /joint_states

# 查看控制器
ros2 control list_controllers -c /turtlebot4/controller_manager

# 查看话题列表
ros2 topic list
```
常见现象说明：

- Gazebo 输出 `Unable to find file with URI model://turtlebot4_description/meshes/...`：
  - 这是找不到某些视觉 mesh，只影响外观，不影响物理和控制
- 如果控制器始终起不来：
  - 确认 `/turtlebot4/robot_description` 上能 `echo` 出 URDF 字符串
  - 再看 `ros2 control list_controllers -c /turtlebot4/controller_manager` 输出

---

### 7. 注意事项

- 如果编译时提示缺少 `irobot_create_description`，请确保已安装上述依赖
- TurtleBot4 使用 Gazebo Harmonic（通过 ros_gz_sim）
- 默认使用 `diffdrive_controller` 控制移动（Create3 官方配置）

## 参考资源

- 官方文档：https://github.com/turtlebot/turtlebot4
- iRobot Create3：https://github.com/iRobotEducation/create3_ros

