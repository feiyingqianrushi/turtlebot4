#!/usr/bin/env python3
"""
Test script to verify SLAM is receiving and processing scan data.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from tf2_ros import TransformListener, Buffer
import time

class SLAMTester(Node):
    def __init__(self):
        super().__init__('slam_tester')
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        
        # TF buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Statistics
        self.scan_count = 0
        self.map_received = False
        self.last_scan_time = None
        
        self.get_logger().info('SLAM Tester started')
        self.get_logger().info('Waiting for scan data...')
        
        # Timer to print status
        self.timer = self.create_timer(2.0, self.print_status)
    
    def scan_callback(self, msg):
        self.scan_count += 1
        self.last_scan_time = time.time()
        
        if self.scan_count == 1:
            self.get_logger().info(f'✓ Received first scan! Frame: {msg.header.frame_id}')
            self.get_logger().info(f'  Ranges: {len(msg.ranges)}, Angle: [{msg.angle_min:.2f}, {msg.angle_max:.2f}]')
        
        # Check TF transform
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                msg.header.frame_id,
                rclpy.time.Time()
            )
            if self.scan_count % 100 == 0:
                self.get_logger().info(f'✓ TF transform OK: {msg.header.frame_id} -> base_link')
        except Exception as e:
            if self.scan_count % 100 == 0:
                self.get_logger().warn(f'✗ TF transform failed: {e}')
    
    def map_callback(self, msg):
        if not self.map_received:
            self.map_received = True
            self.get_logger().info('=' * 60)
            self.get_logger().info('✓ MAP DATA RECEIVED!')
            self.get_logger().info(f'  Resolution: {msg.info.resolution} m/pixel')
            self.get_logger().info(f'  Width: {msg.info.width}, Height: {msg.info.height}')
            self.get_logger().info(f'  Origin: [{msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f}]')
            self.get_logger().info('=' * 60)
        else:
            self.get_logger().info(f'Map updated: {msg.info.width}x{msg.info.height}')
    
    def print_status(self):
        self.get_logger().info('-' * 60)
        self.get_logger().info(f'Scan count: {self.scan_count}')
        if self.last_scan_time:
            elapsed = time.time() - self.last_scan_time
            self.get_logger().info(f'Last scan: {elapsed:.1f}s ago')
        
        self.get_logger().info(f'Map received: {"✓ YES" if self.map_received else "✗ NO"}')
        
        # Check TF transforms
        try:
            transform = self.tf_buffer.lookup_transform('odom', 'base_link', rclpy.time.Time())
            self.get_logger().info('✓ odom -> base_link transform exists')
        except Exception as e:
            self.get_logger().warn(f'✗ odom -> base_link transform: {e}')
        
        try:
            transform = self.tf_buffer.lookup_transform('map', 'odom', rclpy.time.Time())
            self.get_logger().info('✓ map -> odom transform exists (SLAM is working!)')
        except Exception as e:
            self.get_logger().warn(f'✗ map -> odom transform: {e} (SLAM not started yet)')
        
        self.get_logger().info('-' * 60)

def main(args=None):
    rclpy.init(args=args)
    node = SLAMTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
