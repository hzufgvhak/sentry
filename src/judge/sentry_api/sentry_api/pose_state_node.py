#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import SerialPacket, Target, Send


class PoseStatePublisher(Node):
    def __init__(self):
        super().__init__('pose_state_publisher')
        
        # 订阅Target话题获取tracking状态
        self.target_subscription = self.create_subscription(
            Target,
            '/target',
            self.target_callback,
            10
        )
        
        # 发布模拟的serial_packet（用于模拟裁判系统数据）
        self.serial_packet_publisher = self.create_publisher(
            SerialPacket,
            '/serial_packet',
            10
        )
        
        # 发布pose_state
        self.pose_state_publisher = self.create_publisher(
            Send,
            '/pose_state',
            10
        )
        
        # 定时发布pose_state和模拟serial_packet
        timer_period = 0.01  # 10Hz
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # 状态变量
        self.tracking = False
        self.initial_hp = 500  # 初始血量
        self.current_hp = 500  # 当前血量（模拟）
        self.start_time = self.get_clock().now()
        
        # self.get_logger().info('=' * 50)
        # self.get_logger().info('Pose State Publisher 启动成功')
        # self.get_logger().info(f'初始血量: {self.initial_hp}')
        # self.get_logger().info('=' * 50)
    
    def target_callback(self, msg: Target):
        """收到Target消息时更新tracking状态"""
        old_tracking = self.tracking
        self.tracking = msg.tracking
        
        if self.tracking and not old_tracking:
            self.get_logger().info('>>> 识别到目标，进攻姿态 (pose_state=0)')
        elif not self.tracking and old_tracking:
            self.get_logger().info('>>> 进入移动姿态 (pose_state=2)')
    
    def timer_callback(self):
        """定时发布pose_state和模拟serial_packet"""
        elapsed_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        # 15秒后血量减少30
        if elapsed_time > 15.0 and self.current_hp == self.initial_hp:
            self.current_hp = self.initial_hp - 30
            self.get_logger().warn(f'血量减少')
            self.get_logger().warn(f'切换为防御姿态 (pose_state=1)')
        
        # 发布模拟的serial_packet消息
        serial_msg = SerialPacket()
        serial_msg.header = 0xA5
        serial_msg.detect_color = 1
        serial_msg.task_mode = 1
        serial_msg.reset_tracker = False
        serial_msg.is_play = True
        serial_msg.change_target = False
        serial_msg.reserved = 0
        serial_msg.roll = 0.0
        serial_msg.pitch = 0.0
        serial_msg.yaw = 0.0
        serial_msg.robot_hp = self.current_hp
        serial_msg.game_time = int(elapsed_time)
        serial_msg.checksum = 0
        serial_msg.pose_state = 2  # 默认移动
        self.serial_packet_publisher.publish(serial_msg)
        
        # 确定pose_state
        if elapsed_time > 15.0:
            # 15秒后：防御姿态
            pose_state = 1
        elif self.tracking:
            # tracking为true：进攻姿态
            pose_state = 0
        else:
            # 其他情况：移动姿态
            pose_state = 2
        
        # 发布pose_state
        msg = Send()
        msg.pose_state = pose_state
        self.pose_state_publisher.publish(msg)
        
        # 打印当前状态（每秒打印一次）
        state_str = {0: '进攻', 1: '防御', 2: '移动'}
        if int(elapsed_time * 10) % 10 == 0:
            self.get_logger().info(
                f'pose_state={pose_state}({state_str.get(pose_state)}) | '
                f'tracking={self.tracking} | '
                f'HP={self.current_hp}/{self.initial_hp} | '
                f'time={elapsed_time:.1f}s'
            )


def main(args=None):
    rclpy.init(args=args)
    pose_state_pub = PoseStatePublisher()
    rclpy.spin(pose_state_pub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
