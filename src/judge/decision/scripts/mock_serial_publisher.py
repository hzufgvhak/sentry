#!/usr/bin/env python3
"""
模拟下位机发送数据的脚本
用于在没有实际下位机连接时测试行为树

使用方法:
  # 发送默认数据(游戏进行中, HP=500)
  ros2 run judge decision mock_serial_publisher
  
  # 发送低血量数据(触发防御姿态)
  ros2 run judge decision mock_serial_publisher --hp 50
  
  # 发送特定游戏时间
  ros2 run judge decision mock_serial_publisher --time 120
  
  # 持续发送模拟数据流
  ros2 run judge decision mock_serial_publisher --loop
  
  # 模拟阵亡
  ros2 run judge decision mock_serial_publisher --dead
"""

import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import SerialPacket
import argparse
import time
import random


class MockSerialPublisher(Node):
    def __init__(self, args):
        super().__init__('mock_serial_publisher')
        self.publisher = self.create_publisher(SerialPacket, 'serial_packet', 10)
        self.args = args
        self.timer = None
        
        # 立即发送一次数据
        self.publish_packet()
        
        # 如果是循环模式，启动定时器
        if args.loop:
            self.timer = self.create_timer(0.1, self.publish_packet)
            self.get_logger().info('开始循环发送模拟数据 (按Ctrl+C停止)')
        
    def publish_packet(self):
        msg = SerialPacket()
        
        if self.args.dead:
            # 模拟阵亡状态
            msg.header = 0x5A
            msg.detect_color = 0  # 红方
            msg.task_mode = 0
            msg.reset_tracker = False
            msg.is_play = False  # 游戏停止
            msg.change_target = False
            msg.reserved = 0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            msg.robot_hp = 0  # 死亡
            msg.game_time = self.args.time if self.args.time else 0
            msg.checksum = 0
            msg.pose_state = 1  # 防御姿态
            self.get_logger().warn('模拟下位机数据: 阵亡状态!')
            
        elif self.args.hp is not None:
            # 使用指定的HP值
            msg.header = 0x5A
            msg.detect_color = 0
            msg.task_mode = 0
            msg.reset_tracker = False
            msg.is_play = True
            msg.change_target = False
            msg.reserved = 0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            msg.robot_hp = self.args.hp
            msg.game_time = self.args.time if self.args.time else 1
            msg.checksum = 0
            msg.pose_state = 1 if self.args.hp < 100 else 0
            self.get_logger().info(f'模拟下位机数据: HP={msg.robot_hp}, pose_state={msg.pose_state}')
            
        else:
            # 默认正常游戏数据
            msg.header = 0x5A
            msg.detect_color = 0  # 红方
            msg.task_mode = 0    # 手打模式
            msg.reset_tracker = False
            msg.is_play = True   # 游戏进行中
            msg.change_target = False
            msg.reserved = 0
            msg.roll = 0.0
            msg.pitch = 0.0
            msg.yaw = 0.0
            msg.robot_hp = 500   # 满血
            msg.game_time = self.args.time if self.args.time else 1
            msg.checksum = 0
            msg.pose_state = 0   # 进攻姿态
            self.get_logger().info(
                f'模拟下位机数据: game_time={msg.game_time}, '
                f'robot_hp={msg.robot_hp}, is_play={msg.is_play}'
            )
        
        self.publisher.publish(msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='模拟下位机发送串口数据包')
    parser.add_argument('--hp', type=int, help='机器人血量 (0-2000)')
    parser.add_argument('--time', type=int, help='游戏时间 (秒)')
    parser.add_argument('--loop', action='store_true', help='循环持续发送')
    parser.add_argument('--dead', action='store_true', help='模拟阵亡状态')
    parser.add_argument('--color', type=int, default=0, choices=[0, 1], help='机器人颜色: 0=红方, 1=蓝方')
    parser.add_argument('--play', type=int, default=1, choices=[0, 1], help='游戏状态: 0=停止, 1=进行中')
    
    args = parser.parse_args()
    
    rclpy.init(args=None)
    node = MockSerialPublisher(args)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('模拟发布器已停止')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
