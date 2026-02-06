# TurtleBot4 Gazebo 调试指南

## 问题：Gazebo 中看不到机器人

### 已确认：
- ✅ robot_description 话题正常发布
- ✅ spawn 命令执行成功（日志显示 "Entity creation successful"）
- ✅ 转发节点正常工作

### 可能的原因和解决方案：

#### 1. 视角问题
- 在 Gazebo 窗口中按 `Home` 键重置视角
- 使用鼠标中键拖拽移动视角
- 使用鼠标滚轮缩放

#### 2. 模型位置问题
- 机器人可能 spawn 在地面以下
- 已设置 spawn 高度为 z=0.2，应该可见
- 尝试在 Gazebo 中手动移动视角到原点 (0, 0, 0)

#### 3. 检查模型是否真的存在
在另一个终端运行：
```bash
cd ~/ros2_learn/L05_turtlebot4
source install/setup.bash
gz model list
```

如果看到 `turtlebot4`，说明模型已加载，只是视角问题。

#### 4. 手动 spawn 测试
如果自动 spawn 有问题，可以手动测试：
```bash
# 终端1：启动 Gazebo
gz sim empty.sdf

# 终端2：发布 robot_description
ros2 topic pub --once /robot_description std_msgs/msg/String "$(xacro $(ros2 pkg prefix turtlebot4_description)/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro gazebo:=ignition namespace:=turtlebot4)"

# 终端3：手动 spawn
ros2 run ros_gz_sim create -topic robot_description -name turtlebot4 -x 0.0 -y 0.0 -z 0.2
```

#### 5. 检查 URDF 是否正确
```bash
cd ~/ros2_learn/L05_turtlebot4
source install/setup.bash
xacro $(ros2 pkg prefix turtlebot4_description)/share/turtlebot4_description/urdf/standard/turtlebot4.urdf.xacro gazebo:=ignition namespace:=turtlebot4 > /tmp/turtlebot4.urdf
# 检查文件是否有内容
head -50 /tmp/turtlebot4.urdf
```

### 下一步调试：
1. 运行 `gz model list` 检查模型是否存在
2. 在 Gazebo 中按 Home 键重置视角
3. 如果还是看不到，尝试手动 spawn 测试

## 问题：机器人出来了，但没有“diff_drive_controller 话题”，小车不动

### 结论（最关键的两个点）
- **Create3/TurtleBot4 默认底盘控制器名是 `diffdrive_controller`（没有下划线）**，不是 `diff_drive_controller`
- **它订阅的话题是** `/turtlebot4/diffdrive_controller/cmd_vel`，**消息类型是** `geometry_msgs/msg/TwistStamped`，并且订阅端 QoS 是 **BEST_EFFORT**  
  如果你用默认 `ros2 topic pub`（RELIABLE）发 `Twist`，会一直卡在 `Waiting for at least 1 matching subscription(s)...`

### 一键确认控制器是否已加载

```bash
ros2 control list_controllers -c /turtlebot4/controller_manager
```

你应该能看到：
- `diffdrive_controller ... active`
- `joint_state_broadcaster ... active`

### 发布控制命令（正确的 topic/type/QoS）

```bash
ros2 topic pub --once --qos-reliability best_effort \
  /turtlebot4/diffdrive_controller/cmd_vel geometry_msgs/msg/TwistStamped \
  "{twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

### 进一步排查：为什么 load_controller 总失败？

如果你发现 `/turtlebot4/controller_manager` **在图里重复出现两次**，会导致 service/param 请求命中“随机一个”，表现为 load_controller 反复失败：

```bash
ros2 node list | grep /turtlebot4/controller_manager -n
```

正确状态应该只有一个。我们在 `turtlebot4_gazebo.launch.py` 里已经移除了额外启动的 `ros2_control_node`，只保留 Gazebo 插件创建的 controller_manager。

