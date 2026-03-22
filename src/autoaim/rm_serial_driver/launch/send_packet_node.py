#!/usr/bin/env python3
"""
发送SendPacket数据的ROS2节点
基于 rm_serial_driver::SendPacket 结构

同时发布到两个话题:
1. /cmd_vel (geometry_msgs.msg.Twist) - 用于 nav_x/y/z (速度)
2. /fired_info (auto_aim_interfaces.msg.FiredInfo) - 用于 pitch/yaw 和状态
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from auto_aim_interfaces.msg import FiredInfo
import math

class SendPacketNode(Node):
    def __init__(self):
        super().__init__('send_packet_node')
        
        # 发布者1: /cmd_vel (导航/速度)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 发布者2: /fired_info (pitch/yaw 和状态)
        self.fired_info_pub = self.create_publisher(FiredInfo, '/fired_info', 10)
        
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
        
        self.get_logger().info('SendPacket节点已启动, 发布到 /cmd_vel 和 /fired_info')
        
    def timer_callback_05s(self):
        """每0.5秒调用: 增加yaw/pitch"""
        self.yaw += self.delta_angle
        self.pitch += self.delta_angle
        
        # 限制在 [-π, π] 范围内
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
        """同时发布到 /cmd_vel 和 /fired_info"""
        
        # 1. 发布到 /cmd_vel (Twist) - nav_x/y/z
        twist_msg = Twist()
        twist_msg.linear.x = self.speed   # nav_x
        twist_msg.linear.y = self.speed   # nav_y
        twist_msg.linear.z = self.speed   # nav_z
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(twist_msg)
        
        # 2. 发布到 /fired_info (FiredInfo) - pitch/yaw 和状态
        fired_msg = FiredInfo()
        fired_msg.aim_pitch = self.pitch
        fired_msg.aim_yaw = self.yaw
        fired_msg.auto_fire_flag = bool(self.fire_flag)
        fired_msg.state = bool(self.state)
        fired_msg.pose_state = self.pose_state
        self.fired_info_pub.publish(fired_msg)
        
        # 打印日志
        self.get_logger().info(
            f'发送: speed={self.speed:.1f}, '
            f'yaw={math.degrees(self.yaw):.1f}°, pitch={math.degrees(self.pitch):.1f}°, '
            f'state={self.state}, fire={self.fire_flag}, pose={self.pose_state}'
        )
        
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
