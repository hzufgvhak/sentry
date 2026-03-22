#!/usr/bin/env python3
"""
发送测试数据到 /cmd_vel 和 /fired_info
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from auto_aim_interfaces.msg import FiredInfo
import math

class TestDataNode(Node):
    def __init__(self):
        super().__init__('test_data_sender')
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.fired_pub = self.create_publisher(FiredInfo, '/fired_info', 10)
        
        # 定时器 - 每秒触发
        self.timer = self.create_timer(1.0, self.callback)
        
        # 变量
        self.v = 1.0
        self.angle = 0.0
        self.count = 0
        self.state = 0
        self.deg_to_rad = math.pi / 180.0
        
        self.get_logger().info('测试数据发送节点已启动')
        
    def callback(self):
        # 计算弧度
        rad = self.angle * self.deg_to_rad
        
        # 发布 /cmd_vel
        twist = Twist()
        twist.linear.x = self.v
        twist.linear.y = self.v
        twist.linear.z = self.v
        self.cmd_vel_pub.publish(twist)
        
        # 发布 /fired_info
        fired = FiredInfo()
        fired.aim_pitch = rad
        fired.aim_yaw = rad
        fired.auto_fire_flag = bool(self.state)
        fired.state = bool(self.state)
        fired.pose_state = self.state
        self.fired_pub.publish(fired)
        
        # 打印
        self.get_logger().info(
            f'T: {self.count} | V: {self.v} | Deg: {self.angle} | '
            f'Rad: {rad:.6f} | State: {self.state}'
        )
        
        # 更新变量
        self.v += 0.5
        self.angle += 0.5
        self.count += 1
        
        # 每3秒切换状态
        if self.count % 3 == 0:
            self.state = 1 - self.state
            self.get_logger().info(f'>> [状态切换] -> {self.state}')

def main(args=None):
    rclpy.init(args=args)
    node = TestDataNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
