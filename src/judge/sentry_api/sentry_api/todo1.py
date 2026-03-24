#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from auto_aim_interfaces.msg import SerialPacket, FiredInfo
from tf2_ros import TransformListener, Buffer
import time


class RobotState:
    """机器人状态枚举"""
    IDLE = 0           # 空闲/初始
    PATROLLING = 1     # 巡逻中
    RETURNING = 2      # 回家中
    DEFENDING = 3      # 在家防御
    ATTACKING = 4      # 在外进攻


class SerialPacketSubscriber(Node):
    def __init__(self):
        super().__init__('serial_packet_subscriber')
        
        # 订阅/发布
        self.subscription = self.create_subscription(
            SerialPacket, '/serial_packet', self.listener_callback, 10)
        self.fired_info_pub = self.create_publisher(FiredInfo, '/fired_info', 10)
        
        # 状态变量
        self.state = RobotState.IDLE      # 当前状态
        self.current_hp = 450
        self.is_running_task = False
        self.current_goal = (0.0, 0.0)
        self.arrive_home_time = 0.0  # ← 必须初始化！

        # 决策参数
        self.HP_THRESHOLD_LOW = 100      # 低血量阈值（回家）
        self.HP_THRESHOLD_HIGH = 300     # 高血量阈值（可出家）
        # self.PATROL_MIN_INTERVAL = 10.0  # 最小巡逻间隔（秒）
        # self.last_patrol_time = self.get_clock().now().seconds()
        self.stay_home_interval_min =10.0

        # 导航
        self.nav = BasicNavigator()
        self.get_logger().info("节点启动，等待游戏开始...")

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
        
        # 日志打印
        self.get_logger().debug(
            f"State={self.state}, HP={self.current_hp}, Time={msg.game_time}",
            throttle_duration_sec=1.0)
        
        # ========== 状态机决策 ==========
        
        # 游戏未开始
        if msg.game_time < 1:
            return
            
        # 正在执行任务，不重复决策（由任务内部处理中断）
        if self.is_running_task:
            # 但检查是否需要紧急中断（如血量极低）
            self.check_emergency_interrupt()
            return
        
        # 根据当前状态决策
        if self.state == RobotState.IDLE:
            self.decide_from_idle()
            
        elif self.state == RobotState.DEFENDING:
            self.decide_from_defending()
            
        elif self.state == RobotState.ATTACKING:
            # 在外进攻，检查是否该回家补给
            if self.current_hp < self.HP_THRESHOLD_LOW:
                self.get_logger().warn(f"进攻中血量不足({self.current_hp})，决定回家")
                self.execute_return_home()
            # 否则继续进攻，不巡逻（已在目标点）
            
        # PATROLLING/RETURNING 状态由任务内部管理

    def decide_from_idle(self):
        """从空闲状态决策"""
        if self.current_hp < self.HP_THRESHOLD_LOW:
            self.get_logger().warn(f"开场血量低({self.current_hp})，直接防御")
            self.execute_return_home()
        else:
            self.get_logger().info(f"开场血量正常({self.current_hp})，开始巡逻")
            self.execute_patrol()

    def decide_from_defending(self):
        """从防御状态决策：血量恢复后可再次出家"""
        current_time = self.get_clock().now().seconds()
        # time_since_last = current_time - self.last_patrol_time

        time_at_home = current_time - self.arrive_home_time  # 在家待了多久
        # 血量充足 + 冷却时间到 → 再次巡逻
        if self.current_hp >= self.HP_THRESHOLD_HIGH:
            if time_at_home>=self.stay_home_interval_min:
                self.get_logger().info(
                    f"血量恢复({self.current_hp})，冷却完成({time_at_home:.1f}s)，再次巡逻！")
                self.execute_patrol()
            else:
                self.get_logger().info(
                    f"血量恢复但冷却中，继续防御")
        else:
            self.get_logger().info(
                f"血量未恢复({self.current_hp} < {self.HP_THRESHOLD_HIGH})，继续防御")

    def check_emergency_interrupt(self):
        """检查是否需要紧急中断当前任务"""
        # 巡逻中血量极低，立即回家
        if self.state == RobotState.PATROLLING and self.current_hp < 100:
            self.get_logger().error(f"紧急！巡逻中血量极低({self.current_hp})，强制中断！")
            self.nav.cancelTask()
            # 注意：实际中断需要任务内部配合检查标志位
            self.is_running_task = False  # 必须解锁
            self.execute_return_home()     # 立即回家

    def execute_patrol(self):
        """执行巡逻任务"""
        self.is_running_task = True
        self.state = RobotState.PATROLLING

       # self.last_patrol_time = self.get_clock().now().seconds()
        patrol_start_time = self.get_clock().now().seconds()

        self.publish_pose_state(2)  # MOVE
        
        waypoints = [(1.0, 1.0), (4.5, 1.2), (4.5, 0.1)]
        
        for i, (x, y) in enumerate(waypoints):
            # 检查血量（动态阈值）
            if self.current_hp < self.HP_THRESHOLD_LOW:
                # self.last_patrol_time = self.get_clock().now().seconds()
                self.get_logger().warn(f"巡逻点{i+1}血量不足({self.current_hp})，中断回家")
                # self.is_running_task = False

                self.execute_return_home()
                
                return  # 提前结束

            self.publish_pose_state(2)
            reached = self.navigate_to(x, y, allow_interrupt=True)
            
            if not reached:  # 被中断
                # self.last_patrol_time = self.get_clock().now().seconds()
                self.get_logger().warn("导航被中断，结束巡逻")
                self.is_running_task = False
                self.execute_return_home()
                return

        # self.last_patrol_time = self.get_clock().now().seconds()  # ← 更新！

             # 到达点后检查（可能到达时血量刚好耗尽）
            if self.current_hp < self.HP_THRESHOLD_LOW:
                self.get_logger().warn(f"到达巡逻点{i+1}后血量不足，回家")
                self.is_running_task = False
                self.execute_return_home()
                return
        # 巡逻完成
        self.handle_patrol_end()

    def handle_patrol_end(self):
        """巡逻结束处理"""
        if self.current_hp < self.HP_THRESHOLD_LOW:
            self.get_logger().warn("巡逻完成但血量不足，回家防御")
            self.execute_return_home()
        else:
            self.get_logger().info("巡逻完成，转为进攻")
            self.state = RobotState.ATTACKING
            self.publish_pose_state(0)  # ATTACK
            self.is_running_task = False

    def execute_return_home(self):
        """执行回家"""
        self.is_running_task = True
        self.state = RobotState.RETURNING
        
        self.publish_pose_state(2)  # MOVE
        self.navigate_to(0.0, 0.0, allow_interrupt=False)

         # ========== 关键：记录到家时间 ==========
        self.arrive_home_time = self.get_clock().now().seconds()

        self.state = RobotState.DEFENDING
        self.publish_pose_state(1)  # DEFENSE
        self.get_logger().info("已回家，进入防御状态")
        
        self.is_running_task = False


    def is_near_home(self, x, y, threshold=0.3):
                return abs(x) < threshold and abs(y) < threshold


    def navigate_to(self, x, y, allow_interrupt=True):
        """导航到指定坐标"""
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.orientation.w = 1.0
        self.nav.goToPose(goal_pose)
        
        while not self.nav.isTaskComplete():
            rclpy.spin_once(self, timeout_sec=0.05)
    
            # if allow_interrupt and self.current_hp < self.HP_THRESHOLD_LOW:
            #     if not (abs(x) < 0.1 and abs(y) < 0.1):
            #         self.nav.cancelTask()
            #         return False
            # 使用
            if allow_interrupt and self.current_hp < self.HP_THRESHOLD_LOW:
                if not self.is_near_home(x, y):
                    self.nav.cancelTask()
                    return False
        
        return self.nav.getResult() == TaskResult.SUCCEEDED

    def publish_pose_state(self, pose_state):
        """发布姿态"""
        msg = FiredInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose_state = pose_state
        self.fired_info_pub.publish(msg)
        
        state_names = {0: "进攻", 1: "防御", 2: "移动"}
        self.get_logger().info(f"[PUBLISH] pose_state = {pose_state} ({state_names.get(pose_state)})")

    def task_mode_to_str(self, mode):
        return {0: "手打", 1: "自瞄", 2: "大符"}.get(mode, "未知")


def main(args=None):
    rclpy.init(args=args)
    node = SerialPacketSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()