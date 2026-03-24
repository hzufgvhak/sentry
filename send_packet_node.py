#!/usr/bin/env python3
"""
发送SendPacket数据的ROS2节点
基于 rm_serial_driver::SendPacket 结构

映射关系 (参考 rm_serial_driver.cpp cmd_vel_callback):
- linear.x → nav_x
- linear.y → nav_y  
- angular.z → nav_z
- 暂时使用 angular.x/y 传递 pitch/yaw (需要C++驱动配合解析)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import math

class SendPacketNode(Node):
    def __init__(self):
        super().__init__('send_packet_node')
        
        # 创建发布者 - 发布到 /cmd_vel (C++驱动订阅的 topic)
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 定时器: 每0.5秒触发一次 (用于 yaw/pitch 更新)
        self.timer_05s = self.create_timer(0.5, self.timer_callback_05s)
        
        # 定时器: 每3秒触发一次 (用于 state/fire_flag/pose_state 切换)
        self.timer_3s = self.create_timer(3.0, self.timer_callback_3s)
        
        # 状态变量
        self.speed = 1.0  # 保留原速度递增功能
        
        # 发送包状态 (每3秒切换 0↔1)
        self.state = 0          # 0-未跟踪 1-跟踪瞄准 2-跟踪大符
        self.fire_flag = 0     # 0-发射关 1-发射开
        self.pose_state = 0    # 0-攻击 1-防御 2-移动
        
        # yaw/pitch (每0.5秒增加0.5度)
        self.yaw = 0.0         # 弧度
        self.pitch = 0.0       # 弧度
        self.DEG_TO_RAD = math.pi / 180.0
        self.delta_angle = 0.5 * self.DEG_TO_RAD  # 0.5度 = 0.0087弧度
        
        self.get_logger().info('SendPacket节点已启动')
        
    def timer_callback_05s(self):
        """每0.5秒调用: 增加yaw/pitch"""
        self.yaw += self.delta_angle
        self.pitch += self.delta_angle
        
        # 限制在 [-π, π] 范围内
        import math
        while self.yaw > math.pi:
            self.yaw -= 2 * math.pi
        while self.yaw < -math.pi:
            self.yaw += 2 * math.pi
        while self.pitch > math.pi:
            self.pitch -= 2 * math.pi
        while self.pitch < -math.pi:
            self.pitch += 2 * math.pi
            
        self.publish_packet()
        
    def timer_callback_3s(self):
        """每3秒调用: 切换 state/fire_flag/pose_state"""
        # 切换 0 ↔ 1
        self.state = 1 - self.state
        self.fire_flag = 1 - self.fire_flag
        self.pose_state = 1 - self.pose_state
        
        self.get_logger().info(
            f'状态切换: state={self.state}, fire_flag={self.fire_flag}, pose_state={self.pose_state}'
        )
        
    def publish_packet(self):
        """发布Twist消息到 /cmd_vel"""
        msg = Twist()
        
        # 导航坐标 (原速度功能 + nav_x/y/z)
        msg.linear.x = self.speed   # nav_x
        msg.linear.y = self.speed   # nav_y
        msg.linear.z = self.speed    # nav_z
        
        # 将 yaw/pitch 映射到 angular (需要C++驱动配合解析)
        # angular.x = pitch (弧度)
        # angular.y = yaw (弧度)
        # angular.z 保留用于 nav_z
        msg.angular.x = self.pitch
        msg.angular.y = self.yaw
        msg.angular.z = 0.0  # 保留
        
        # 打印日志
        self.get_logger().info(
            f'发送: speed={self.speed:.1f}, '
            f'yaw={math.degrees(self.yaw):.1f}°, pitch={math.degrees(self.pitch):.1f}°'
        )
        
        self.publisher_.publish(msg)
        
        # 速度递增 (保留原功能)
        self.speed += 0.5

def main(args=None):
    rclpy.init(args=args)
    node = SendPacketNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
