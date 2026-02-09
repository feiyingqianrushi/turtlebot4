# TurtleBot4 雷达使用说明

## 1. 查看雷达数据

### 在 RViz 中可视化

1. **启动仿真和 RViz**：
   ```bash
   ros2 launch turtlebot4_description turtlebot4_sim_rviz.launch.py
   ```

2. **在 RViz 中查看雷达扫描**：
   - 确保 **LaserScan** 显示已启用（在 Displays 面板中）
   - Topic 应该设置为 `/scan`
   - 应该能看到雷达扫描线（当前为占位符数据，显示为最大范围的圆）

### 使用命令行查看

```bash
# 查看雷达话题列表
ros2 topic list | grep scan

# 实时查看雷达数据
ros2 topic echo /scan

# 查看雷达话题信息
ros2 topic info /scan

# 查看雷达数据频率
ros2 topic hz /scan
```

## 2. 在代码中使用雷达数据

### Python 示例

创建一个 Python 节点来订阅雷达数据：

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
        self.subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        """处理雷达扫描数据"""
        # 获取最小距离（前方障碍物）
        min_range = min(msg.ranges)
        min_index = msg.ranges.index(min_range)
        min_angle = msg.angle_min + min_index * msg.angle_increment
        
        self.get_logger().info(
            f'最小距离: {min_range:.2f}m, '
            f'角度: {min_angle:.2f}rad, '
            f'索引: {min_index}'
        )
        
        # 检查前方是否有障碍物（-30度到30度）
        front_ranges = []
        for i, r in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -0.524 <= angle <= 0.524:  # -30度到30度
                if msg.range_min <= r <= msg.range_max:
                    front_ranges.append(r)
        
        if front_ranges:
            min_front = min(front_ranges)
            if min_front < 0.5:  # 0.5米内有障碍物
                self.get_logger().warn(f'前方有障碍物！距离: {min_front:.2f}m')

def main(args=None):
    rclpy.init(args=args)
    node = LidarSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### C++ 示例

```cpp
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class LidarSubscriber : public rclcpp::Node
{
public:
  LidarSubscriber() : Node("lidar_subscriber")
  {
    subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      "/scan", 10,
      std::bind(&LidarSubscriber::scan_callback, this, std::placeholders::_1));
  }

private:
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    // 找到最小距离
    float min_range = *std::min_element(msg->ranges.begin(), msg->ranges.end());
    RCLCPP_INFO(this->get_logger(), "最小距离: %.2f m", min_range);
  }
  
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LidarSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

## 3. 雷达数据格式

雷达数据发布在 `/scan` 话题上，消息类型为 `sensor_msgs/LaserScan`：

- **frame_id**: `rplidar_link` - 雷达坐标系
- **angle_min**: `-3.14` rad (-180度)
- **angle_max**: `3.14` rad (180度)
- **angle_increment**: `0.0098` rad (~0.56度)
- **range_min**: `0.164` m (16.4 cm)
- **range_max**: `12.0` m
- **ranges**: 640 个距离值（浮点数数组）
- **intensities**: 640 个强度值（浮点数数组）

## 4. 常见应用场景

### 避障导航

```python
def check_obstacle_ahead(scan_msg, distance_threshold=0.5):
    """检查前方是否有障碍物"""
    front_indices = []
    for i in range(len(scan_msg.ranges)):
        angle = scan_msg.angle_min + i * scan_msg.angle_increment
        # 检查前方 -45度到45度
        if -0.785 <= angle <= 0.785:
            if scan_msg.range_min <= scan_msg.ranges[i] <= scan_msg.range_max:
                front_indices.append(i)
    
    if front_indices:
        min_distance = min([scan_msg.ranges[i] for i in front_indices])
        return min_distance < distance_threshold, min_distance
    return False, scan_msg.range_max
```

### 跟随墙壁

```python
def follow_wall(scan_msg, side='left'):
    """跟随左侧或右侧墙壁"""
    if side == 'left':
        # 左侧90度方向
        target_angle = scan_msg.angle_min + len(scan_msg.ranges) * 0.25
    else:
        # 右侧90度方向
        target_angle = scan_msg.angle_min + len(scan_msg.ranges) * 0.75
    
    target_index = int((target_angle - scan_msg.angle_min) / scan_msg.angle_increment)
    wall_distance = scan_msg.ranges[target_index]
    
    return wall_distance
```

## 5. 注意事项

✅ **当前状态**：
- 雷达桥接节点已改进，使用 Gazebo Transport Python 绑定从 Gazebo 获取**真实数据**
- 如果 Gazebo Transport 不可用，会自动回退到占位符数据
- Gazebo 和 RViz 现在可以同步显示雷达扫描数据

🔧 **技术细节**：
- 桥接节点订阅 Gazebo 话题：`/world/empty/model/turtlebot4/link/rplidar_link/sensor/rplidar/scan`
- 将 Gazebo 的 `gz.msgs.LaserScan` 消息转换为 ROS2 的 `sensor_msgs/LaserScan`
- 自动处理 `inf` 值（超出范围的距离）

## 6. 测试雷达数据

```bash
# 1. 启动仿真
ros2 launch turtlebot4_description turtlebot4_gazebo.launch.py

# 2. 在另一个终端查看雷达数据
ros2 topic echo /scan

# 3. 在 RViz 中可视化
ros2 launch turtlebot4_description turtlebot4_sim_rviz.launch.py
```

## 7. 故障排除

如果看不到雷达数据：

1. **检查话题是否存在**：
   ```bash
   ros2 topic list | grep scan
   ```

2. **检查节点是否运行**：
   ```bash
   ros2 node list | grep lidar
   ```

3. **检查话题频率**：
   ```bash
   ros2 topic hz /scan
   ```

4. **查看节点日志**：
   ```bash
   ros2 node info /lidar_bridge
   ```

