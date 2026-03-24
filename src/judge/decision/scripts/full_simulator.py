#!/usr/bin/env python3
"""
完全模拟上下位机数据
- 下位机数据: /serial_packet (裁判系统数据)
- 上位机/视觉数据: /target (目标检测数据)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from geometry_msgs.msg import Point, Vector3
from auto_aim_interfaces.msg import Target, SerialPacket
import argparse
import time


def main():
    parser = argparse.ArgumentParser(description='完全模拟上下位机数据')
    parser.add_argument('--time', '-t', type=int, default=180, help='比赛时间(秒)')
    parser.add_argument('--hp', type=int, default=500, help='机器人血量')
    parser.add_argument('--dead', action='store_true', help='机器人死亡')
    parser.add_argument('--tracking', action='store_true', default=False, help='是否检测到目标')
    parser.add_argument('--target-x', type=float, default=1.0, help='目标X坐标')
    parser.add_argument('--target-y', type=float, default=0.0, help='目标Y坐标')
    parser.add_argument('--target-z', type=float, default=0.5, help='目标Z坐标')
    parser.add_argument('--armors', type=int, default=1, help='装甲板数量')
    parser.add_argument('--auto-time', action='store_true', help='自动递减游戏时间')
    parser.add_argument('--loop', action='store_true', help='循环发送')
    
    args = parser.parse_args()
    
    rclpy.init()
    node = rclpy.create_node('full_simulator')
    
    # 创建发布者
    serial_pub = node.create_publisher(SerialPacket, '/serial_packet', 10)
    target_pub = node.create_publisher(Target, '/target', 10)
    
    # 打印配置
    node.get_logger().info('=== 完全模拟器配置 ===')
    node.get_logger().info(f'比赛时间: {args.time}s')
    node.get_logger().info(f'机器人血量: {args.hp}')
    node.get_logger().info(f'比赛状态: {"进行中" if not args.dead else "结束/死亡"}')
    node.get_logger().info(f'目标跟踪: {"开启" if args.tracking else "关闭"}')
    if args.tracking:
        node.get_logger().info(f'目标位置: ({args.target_x}, {args.target_y}, {args.target_z})')
        node.get_logger().info(f'装甲板数量: {args.armors}')
    node.get_logger().info('=====================')
    
    # 创建消息
    serial_msg = SerialPacket()
    serial_msg.header = 0  # uint8
    serial_msg.detect_color = 0  # 0=red, 1=blue
    serial_msg.task_mode = 0  # 0=手打
    serial_msg.reset_tracker = False
    serial_msg.is_play = not args.dead
    serial_msg.change_target = False
    serial_msg.reserved = 0
    serial_msg.roll = 0.0
    serial_msg.pitch = 0.0
    serial_msg.yaw = 0.0
    serial_msg.robot_hp = args.hp
    serial_msg.game_time = args.time
    serial_msg.checksum = 0
    serial_msg.pose_state = 2  # move
    
    target_msg = Target()
    target_msg.header = Header()
    target_msg.tracking = args.tracking
    target_msg.id = ""
    target_msg.armors_num = args.armors
    target_msg.position = Point(x=args.target_x, y=args.target_y, z=args.target_z)
    target_msg.velocity = Vector3(x=0.0, y=0.0, z=0.0)
    target_msg.yaw = 0.0
    target_msg.v_yaw = 0.0
    target_msg.radius_1 = 0.0
    target_msg.radius_2 = 0.0
    target_msg.dz = 0.0
    
    game_time = args.time
    rate = node.create_rate(10)  # 10Hz
    
    node.get_logger().info('开始发布模拟数据...')
    
    while rclpy.ok():
        # 更新时间戳
        now = node.get_clock().now().to_msg()
        target_msg.header.stamp = now
        
        # 更新游戏时间
        if args.auto_time and game_time > 0:
            game_time -= 0.1
            serial_msg.game_time = int(max(0, game_time))
        
        # 发布数据
        serial_pub.publish(serial_msg)
        target_pub.publish(target_msg)
        
        # 打印状态
        status = f"time={serial_msg.game_time}, hp={serial_msg.robot_hp}, tracking={target_msg.tracking}"
        node.get_logger().info(f'发布: {status}', throttle_duration_sec=1.0)
        
        rate.sleep()
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
