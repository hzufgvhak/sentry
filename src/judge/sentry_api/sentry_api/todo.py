#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator,TaskResult
from auto_aim_interfaces.msg import SerialPacket,FiredInfo,Send
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

        self.pose_state_pub = self.create_publisher(
            Send,                # 消息类型：auto_aim_interfaces/Send
            '/send',    # 话题名：自定义，和订阅者一致即可
            10                   # QoS：缓存深度10
        )
        
        self.pose_state = 2
        self.current_hp = 450  # 初始血量
        self.is_running_task = False # 任务锁
        self.current_goal = (0.0, 0.0)
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
            pose_state: {msg.pose_state}
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
            self.handle_game_start()

    def handle_game_start(self):
        """处理开场逻辑"""
        if self.current_hp < 100:
            self.get_logger().warn(f"开场血量偏低 ({self.current_hp})，直接回家防御")
            self.execute_return_home()
        else:
            self.get_logger().info(f"开场血量正常 ({self.current_hp})，执行巡逻任务")
            self.execute_patrol()

    def publish_pose_state(self, pose_state):
        """发布 FiredInfo 消息"""
        msg = FiredInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose_state = pose_state

        self.pose_state_pub.publish(msg)
        state_str = {0: "进攻(ATTACK)", 1: "防御(DEFENSE)", 2: "移动(MOVE)"}.get(state, "未知")
        self.get_logger().info(f"[PUBLISH] pose_state = {state} ({state_str})")
        

    def execute_patrol(self):
        """执行巡逻任务"""
        self.is_running_task = True
        
        # 1. 切换到移动姿态
        self.publish_pose_state(2)  # MOVE
        
        waypoints = [(1.0, 1.0), (4.5, 1.2), (4.5, 0.1)]
        
        for i, (x, y) in enumerate(waypoints):
            # 检查血量
            if self.current_hp < 100:
                self.get_logger().error(f"巡逻点{i+1}途中血量告急 ({self.current_hp})！中断并回家")
                break
            
            # 发布移动姿态（每次移动前确认）
            self.publish_pose_state(2)
            
            # 执行移动
            reached = self.navigate_to(x, y, allow_interrupt=True)
            
            if not reached:
                self.get_logger().warn(f"导航到 ({x}, {y}) 被中断")
                break
        
        # 巡逻结束或中断后处理
        self.handle_patrol_end()
        self.is_running_task = False

    def handle_patrol_end(self):
        """巡逻结束后的处理"""
        if self.current_hp < 100:
            # 血量低：回家防御
            self.get_logger().error("巡逻结束血量不足，执行回家防御")
            self.execute_return_home()
        else:
            # 血量正常：转为进攻姿态，继续在当前位置作战
            self.get_logger().info("巡逻完成，转为进攻姿态")
            self.publish_pose_state(0)  # ATTACK


    def execute_return_home(self):
        """执行回家(0,0)并防御"""
        # 切换到移动姿态回家
        self.publish_pose_state(2)  # MOVE
        self.navigate_to(0.0, 0.0, allow_interrupt=False)
        
        # 到达后切换为防御姿态
        self.publish_pose_state(1)  # DEFENSE
        self.get_logger().info("已回家，切换为防御姿态")

    
    def navigate_to(self, x, y, allow_interrupt=True):
        """
        导航到指定坐标
        :param allow_interrupt: 是否允许血量低时中断
        :return: 是否成功到达
        """
        self.current_goal = (x, y)
        
        # 构建目标
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0
        goal_pose.pose.orientation.w = 1.0  # 默认朝向

        # 发送导航指令
        self.nav.goToPose(goal_pose)
        self.get_logger().info(f"开始导航至 ({x}, {y})，当前姿态: {self.pose_state}")

        # 等待完成，期间监控血量和姿态
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
            
            # 实时中断检查（仅巡逻时允许中断）
            if allow_interrupt and self.current_hp < 100:
                if not (abs(x) < 0.1 and abs(y) < 0.1):  # 不是回家途中
                    self.get_logger().error(
                        f"导航至({x},{y})途中血量过低({self.current_hp})！紧急中断回家！"
                    )
                    self.nav.cancelTask()
                    return False
        
        # 检查结果
        result = self.nav.getResult()
        success = (result == TaskResult.SUCCEEDED)
        
        if success:
            self.get_logger().info(f"成功到达 ({x}, {y})")
        else:
            self.get_logger().warn(f"导航失败/取消，结果: {result}")
        
        return success

    def task_mode_to_str(self, mode):
        modes = {0: "手打", 1: "自瞄", 2: "大符"}
        return modes.get(mode, "未知")
        
        
    def ps_behavior(self,pose_state):
        if(pose_state==1):
            pass
            
def main(args=None):
    rclpy.init(args=args)

    #node = Node('judge_starter_node')

    serial_packet_sub = SerialPacketSubscriber()
    #serial_packet_sub.nav.waitUntilNav2Active()   #   `等待导航可用`
    rclpy.spin(serial_packet_sub)
    rclpy.shutdown()


if __name__ == '__main__':
    main()