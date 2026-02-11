# 世界文件使用说明

## 可用的世界文件

### 1. `empty` - 空世界
- 描述：空世界，没有任何障碍物
- 用途：测试基本功能
- 使用：`world:=empty`

### 2. `warehouse` - 仓库世界（自定义）
- 描述：包含墙壁和障碍物的仓库环境
- 障碍物：
  - 4 面墙壁（形成 10x10 米的房间）
  - 4 个彩色盒子（红色、蓝色、绿色、黄色）
  - 2 个圆柱体（橙色、紫色）
- 用途：SLAM 建图和导航测试
- 使用：`world:=warehouse`（默认）

## 使用方法

### 启动 Gazebo 仿真（使用仓库世界）
```bash
ros2 launch turtlebot4_description turtlebot4_gazebo.launch.py world:=warehouse
```

### 启动 Gazebo 仿真（使用空世界）
```bash
ros2 launch turtlebot4_description turtlebot4_gazebo.launch.py world:=empty
```

### 启动 SLAM（使用仓库世界）
```bash
ros2 launch turtlebot4_description turtlebot4_slam.launch.py world:=warehouse
```

### 启动 SLAM（使用空世界）
```bash
ros2 launch turtlebot4_description turtlebot4_slam.launch.py world:=empty
```

## 世界文件位置

- 自定义世界文件：`src/turtlebot4/turtlebot4_description/worlds/`
- 安装后位置：`install/turtlebot4_description/share/turtlebot4_description/worlds/`

## 创建自定义世界文件

1. 在 `worlds/` 目录下创建新的 `.sdf` 文件
2. 参考 `warehouse.sdf` 的格式
3. 在 `CMakeLists.txt` 中确保 `worlds` 目录被安装
4. 使用 `world:=你的世界文件名` 启动

## 注意事项

- 世界文件名（不含 `.sdf` 扩展名）必须与 `world` 参数匹配
- 自定义世界文件必须包含 `<world name="...">` 标签，名称必须与文件名匹配
- 所有世界文件都位于 `worlds/` 目录中
