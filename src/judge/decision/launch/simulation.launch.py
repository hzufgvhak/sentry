#!/usr/bin/env python3
"""
Simulation node for rm_decision behavior tree testing.
Publishes mock data to /serial_packet and /target topics to simulate
the upper computer (referee system) and lower computer (vision detection).
"""

import rclpy
from rclpy.node import Node
from auto_aim_interfaces.msg import SerialPacket, Target
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Header


class SimulationNode(Node):
    def __init__(self):
        super().__init__('simulation_node')
        
        # Publishers
        self.serial_packet_pub = self.create_publisher(SerialPacket, '/serial_packet', 10)
        self.target_pub = self.create_publisher(Target, '/target', 10)
        
        # Simulation parameters
        self.declare_parameter('game_time', 1)  # Start at 1 to trigger navigation
        self.declare_parameter('robot_hp', 500)  # Full HP
        self.declare_parameter('is_play', True)  # Game is playing
        self.declare_parameter('detect_color', 0)  # Red
        self.declare_parameter('tracking', False)  # Not tracking initially
        
        # Timer for publishing (10Hz)
        self.timer = self.create_timer(0.1, self.timer_callback)
        
        self.get_logger().info('Simulation node started')
        
        # State tracking
        self.tick_count = 0
        
    def timer_callback(self):
        self.tick_count += 1
        
        # Publish SerialPacket (simulating referee system)
        serial_msg = SerialPacket()
        serial_msg.header = 0xA5
        serial_msg.detect_color = 0  # Red
        serial_msg.task_mode = 1  # Auto-aim mode
        serial_msg.reset_tracker = False
        serial_msg.is_play = True
        serial_msg.change_target = False
        serial_msg.reserved = 0
        serial_msg.roll = 0.0
        serial_msg.pitch = 0.0
        serial_msg.yaw = 0.0
        serial_msg.robot_hp = 500  # Full HP
        serial_msg.game_time = 1  # Game started
        serial_msg.checksum = 0
        serial_msg.pose_state = 2  # Move state initially
        
        # Simulate different scenarios based on tick count
        if self.tick_count < 50:
            # Initial state: game start, moving
            serial_msg.robot_hp = 500
            serial_msg.pose_state = 2  # Move
            tracking = False
        elif self.tick_count < 100:
            # HP still good, tracking detected
            serial_msg.robot_hp = 400
            serial_msg.pose_state = 0  # Attack
            tracking = True
        elif self.tick_count < 150:
            # Low HP - should switch to defense
            serial_msg.robot_hp = 80  # Low HP
            serial_msg.pose_state = 1  # Defense
            tracking = True
        else:
            # Reset for loop
            self.tick_count = 0
            serial_msg.robot_hp = 500
            serial_msg.pose_state = 2
            tracking = False
            
        self.serial_packet_pub.publish(serial_msg)
        
        # Publish Target (simulating vision detection)
        target_msg = Header()
        target_msg.stamp = self.get_clock().now().to_msg()
        target_msg.frame_id = 'camera'
        
        target = Target()
        target.header = target_msg
        target.tracking = tracking
        target.id = "hero" if tracking else ""
        target.armors_num = 1 if tracking else 0
        target.position = Point(x=1.0 if tracking else 0.0, 
                                y=0.5 if tracking else 0.0, 
                                z=0.0)
        target.velocity = Vector3(x=0.0, y=0.0, z=0.0)
        target.yaw = 0.0
        target.v_yaw = 0.0
        target.radius_1 = 0.0
        target.radius_2 = 0.0
        target.dz = 0.0
        
        self.target_pub.publish(target)
        
        # Log state changes
        if tracking:
            self.get_logger().debug(f'Tick {self.tick_count}: HP={serial_msg.robot_hp}, pose_state={serial_msg.pose_state}, tracking=True', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = SimulationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
