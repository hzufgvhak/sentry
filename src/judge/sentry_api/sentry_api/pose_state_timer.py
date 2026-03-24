#!/usr/bin/env python3
"""
Pose State 定时发送脚本
- 0秒: 发pose_state=2 (移动)
- 10秒: 发pose_state=0 (攻击)
- 15秒: 发pose_state=1 (防御)
"""

import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import Send


class PoseStateTimer(Node):
    def __init__(self):
        super().__init__('pose_state_timer')
        
        # 发布pose_state
        self.pose_state_publisher = self.create_publisher(
            Send,
            '/pose_state',
            10
        )
        
        # 快速发布定时器 (10Hz) - 用于频繁发布pose_state
        publish_period = 0.1  # 0.1秒发布一次 (10Hz)
        self.publish_timer = self.create_timer(publish_period, self.publish_timer_callback)
        
        # 日志定时器 (1Hz) - 每秒打印当前状态
        log_period = 0.5  # 1秒打印一次
        self.log_timer = self.create_timer(log_period, self.log_timer_callback)
        
        # 状态检测定时器 (0.5秒检查一次状态变化)
        check_period = 0.5
        self.check_timer = self.create_timer(check_period, self.check_timer_callback)
        
        # 状态变量
        self.start_time = self.get_clock().now()
        self.sent_10s = False
        self.sent_15s = False
        
        # 初始状态
        self.current_pose_state = 2
        self.previous_pose_state = 2  # 用于检测状态切换
        
        # 初始打印出家/移动姿态
        self.get_logger().info('=' * 50)
        self.get_logger().info('>>> 出家，进入移动姿态')
        self.get_logger().info('=' * 50)
        self.publish_pose_state_with_log(2)
        
    #     self.get_logger().info('=' * 50)
    #    # self.get_logger().info('Pose State Timer 启动')
    #     self.get_logger().info('pose_state=2 (移动)')
    #     self.get_logger().info('pose_state=0 (攻击)')
    #     self.get_logger().info('pose_state=1 (防御)')
    #    # self.get_logger().info('=' * 50)
    
    def publish_pose_state(self, pose_state):
        """发布pose_state消息（不带日志）"""
        msg = Send()
        msg.pose_state = pose_state
        self.pose_state_publisher.publish(msg)
    
    def publish_pose_state_with_log(self, pose_state):
        """发布pose_state消息（带日志）"""
        msg = Send()
        msg.pose_state = pose_state
        self.pose_state_publisher.publish(msg)
        
        state_str = {0: '攻击', 1: '防御', 2: '移动'}
        self.get_logger().info(f'>>> pose_state={pose_state} ({state_str.get(pose_state)})')
    
    def publish_timer_callback(self):
        """快速发布回调 - 持续发布当前pose_state"""
        self.publish_pose_state(self.current_pose_state)
    
    def log_timer_callback(self):
        """日志回调 - 每秒打印当前状态"""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        state_str = {0: '攻击', 1: '防御', 2: '移动'}
        self.get_logger().info(
            f'当前: pose_state={self.current_pose_state} ({state_str.get(self.current_pose_state)})'
        )
    
    def check_timer_callback(self):
        """状态检测回调 - 检测状态变化并打印日志"""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # 10秒后切换到攻击姿态
        if elapsed_time >= 10.0 and not self.sent_10s:
            self.sent_10s = True
            self.previous_pose_state = self.current_pose_state
            self.current_pose_state = 0
            self.get_logger().info('=' * 50)
            self.get_logger().info('>>> 进入攻击姿态')
            self.get_logger().info('=' * 50)
            self.publish_pose_state_with_log(0)
        
        # 15秒后切换到防御姿态
        if elapsed_time >= 15.0 and not self.sent_15s:
            self.sent_15s = True
            self.previous_pose_state = self.current_pose_state
            self.current_pose_state = 1
            self.get_logger().info('=' * 50)
            self.get_logger().info('>>> 血量下降，进入防御姿态')
            self.get_logger().info('=' * 50)
            self.publish_pose_state_with_log(1)


def main(args=None):
    rclpy.init(args=args)
    pose_state_timer = PoseStateTimer()
    rclpy.spin(pose_state_timer)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
