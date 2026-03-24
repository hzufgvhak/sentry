#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from auto_aim_interfaces.msg import SerialPacket,Send
from tf2_ros import TransformListener, Buffer
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import time
class SerialPacketSubscriber(Node):
    def __init__(self):
        super().__init__('serial_packet_subscriber')
        self.subscription = self.create_subscription(
            SerialPacket,
            '/serial_packet',
            self.listener_callback,
            10  # QoS history depth
        )
        self.current_hp = 450  # 初始血量
        self.is_running_task = False # 任务锁
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav = BasicNavigator()     # 节点
        self.get_logger().info("Listening for SerialPacket messages on /serial_packet...")
    def listener_callback(self, msg: SerialPacket):
        self.get_logger().info(
            f"""
            Received SerialPacket:
            --------------------------------
            header: {hex(msg.header)}
            detect_color: {msg.detect_color}  # {'red' if msg.detect_color == 0 else 'blue'}
            task_mode: {msg.task_mode}        # {self.task_mode_to_str(msg.task_mode)}
            reset_tracker: {msg.reset_tracker}
            is_play: {msg.is_play}
            change_target: {msg.change_target}
            reserved: {msg.reserved}
            roll: {msg.roll:.2f}
            pitch: {msg.pitch:.2f}
            yaw: {msg.yaw:.2f}
            robot_hp: {msg.robot_hp}
            game_time: {msg.game_time} seconds
            checksum: {hex(msg.checksum)}
            --------------------------------
            """,
            throttle_duration_sec=0.1  # 控制日志输出频率
        )
        self.current_hp = msg.robot_hp
        # 2. 如果当前已经在处理任务（巡逻或回家），直接返回，由 set_goal_pose 内部监控中断
        if self.is_running_task:
            return
        
        #if msg.game_time == 1:
        #    if msg.robot_hp >= 100:
        #        self.set_goal_pose(1.2, 1.2)
        #        self.set_goal_pose(4.5, 1.2)
         #       self.set_goal_pose(4.5, 0.1)
        #       self.set_goal_pose(1.0, 0.0)
        #    else:
        #        self.is_running_task = True
        #        self.set_goal_pose(0.0,0.0)
         #       self.is_running_task = False
    
    
        if msg.game_time == 1:
            if self.current_hp < 100:###大于号是为了测试，实际是小于号
                self.get_logger().warn("开场血量偏低，直接启动回家流程")
                self.is_running_task = True
                self.set_goal_pose(0.0, 0.0)
                self.is_running_task = False
            else:
                self.get_logger().info("启动巡逻路径...")
                self.is_running_task = True
                waypoints = [(1.0, 1.0), (4.5, 1.2), (4.5, 0.1)]
                for pt in waypoints:
                    # 在去往下一个点前，先看一眼血量
                    if self.current_hp < 100:
                        self.get_logger().error("血量告急，取消后续巡逻点！")
                        #self.set_goal_pose(0.0, 0.0)
                        break
                    
                    # 执行移动（内部带实时中断检查）
                    reached = self.set_goal_pose(pt[0], pt[1])
                    
                    # 如果是因为血量不足导致的中断返回，立刻退出循环去执行回家
                    if not reached and self.current_hp < 100:
                        break
                
                # 巡逻结束或中断后，如果发现血量低，强制回家
                if self.current_hp < 100:
                    self.set_goal_pose(0.0, 0.0)
                
                self.is_running_task = False

    def set_goal_pose(self, x, y):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)
        #while not self.nav.isTaskComplete():
        #    feedback = self.nav.getFeedback()
        #self.nav.cancelTask()
        while not self.nav.isTaskComplete():
            # 必须 spin_once，否则无法更新 self.current_hp
            rclpy.spin_once(self, timeout_sec=0.1)
            
            # --- 实时中断检查 ---
            if (x>=0.3 and y>=0.3)and self.current_hp < 100:
                self.get_logger().error(f"警告：移动至 ({x}, {y}) 途中血量过低 ({self.current_hp})！正在紧急中断并撤回！")
                self.nav.cancelTask() # 停止当前导航
                return False # 返回失败，由外层逻辑决定去(0,0)

        # 检查最终结果
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"成功到达坐标: ({x}, {y})")
            return True
        else:
            self.get_logger().error("任务未能成功完成（可能被手动取消或路径堵塞）")
            return False




    def set_goal_pose_home(self):
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = 0.0
        goal_pose.pose.position.y = 0.0
        goal_pose.pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal_pose.pose.orientation.w = 1.0

        self.nav.goToPose(goal_pose)
        while not self.nav.isTaskComplete():
            feedback = self.nav.getFeedback()

        self.nav.cancelTask()
        result = self.nav.getResult()
        if result == TaskResult.SUCCEEDED:
             print('Goal succeeded!')
        elif result == TaskResult.CANCELED:
             print('Goal was canceled!')
        elif result == TaskResult.FAILED:
             print('Goal failed!')

    def task_mode_to_str(self, mode):
        if mode == 0:
            return "手打"
        elif mode == 1:
            return "自瞄"
        elif mode == 2:
            return "大符"


def main(args=None):
    rclpy.init(args=args)
    node = Node('judge_starter_node')

    serial_packet_sub = SerialPacketSubscriber()
    #serial_packet_sub.nav.waitUntilNav2Active()   #   `等待导航可用`
    rclpy.spin(serial_packet_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()