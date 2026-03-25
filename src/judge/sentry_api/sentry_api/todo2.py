#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from auto_aim_interfaces.msg import SerialPacket, Send
import math
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class RobotState:
    IDLE = 0
    PATROLLING = 1
    RETURNING = 2
    DEFENDING = 3
    ATTACKING = 4


# 类级常量
STATE_NAMES = {
    RobotState.IDLE: "IDLE",
    RobotState.PATROLLING: "PATROL",
    RobotState.RETURNING: "RETURN",
    RobotState.DEFENDING: "DEFEND",
    RobotState.ATTACKING: "ATTACK"
}

POSE_STATE_NAMES = {
    0: "ATTACK(进攻)",
    1: "DEFENSE(防御)",
    2: "MOVE(移动)"
}

POSE_MAPPING = {
    RobotState.IDLE: 2,
    RobotState.PATROLLING: 2,
    RobotState.RETURNING: 2,
    RobotState.DEFENDING: 1,
    RobotState.ATTACKING: 0
}


class RobotDecisionNode(Node):

    def __init__(self):
        super().__init__('robot_decision_node')

        # ===================== ROS参数 =====================
        self.declare_parameter("patrol_points", [[1.0, 1.0], [4.5, 1.2], [4.5, 0.1]])
        self.declare_parameter("hp_threshold_low", 100)
        self.declare_parameter("hp_threshold_high", 300)
        self.declare_parameter("stay_home_interval", 10.0)
        self.declare_parameter("nav_timeout", 30.0)
        self.declare_parameter("home_tolerance", 0.5)
        self.declare_parameter("home_position", [0.0, 0.0])
        self.declare_parameter("pose_broadcast_interval", 0.5)
        self.declare_parameter("attack_duration", 10.0)
        self.declare_parameter("attack_position_tolerance", 1.0)
        self.declare_parameter("attack_tf_check_interval", 0.2)  # 新增：TF检查间隔

        # 读取参数
        points = self.get_parameter("patrol_points").value
        self.waypoints = [tuple(p) for p in points]
        
        if len(self.waypoints) == 0:
            self.get_logger().error("巡逻点为空！")
            raise RuntimeError("No patrol points configured")
        
        self.home = tuple(self.get_parameter("home_position").value)
        
        self.HP_THRESHOLD_LOW = self.get_parameter("hp_threshold_low").value
        self.HP_THRESHOLD_HIGH = self.get_parameter("hp_threshold_high").value
        self.stay_home_interval = self.get_parameter("stay_home_interval").value
        self.nav_timeout = self.get_parameter("nav_timeout").value
        self.home_tolerance = self.get_parameter("home_tolerance").value
        self.pose_broadcast_interval = self.get_parameter("pose_broadcast_interval").value
        self.attack_duration = self.get_parameter("attack_duration").value
        self.attack_position_tolerance = self.get_parameter("attack_position_tolerance").value
        self.attack_tf_check_interval = self.get_parameter("attack_tf_check_interval").value

        # 参数校验
        if self.HP_THRESHOLD_LOW >= self.HP_THRESHOLD_HIGH:
            self.get_logger().warn(f"阈值不合理，自动调整")
            self.HP_THRESHOLD_HIGH = self.HP_THRESHOLD_LOW + 50
        if self.stay_home_interval <= 0:
            self.stay_home_interval = 10.0
        if self.nav_timeout <= 0:
            self.nav_timeout = 30.0

        # ===================== 通信 =====================
        self.serial_sub = self.create_subscription(
            SerialPacket, '/serial_packet', self.serial_callback, 10)
        self.pose_state_pub = self.create_publisher(Send, '/send', 10)

        # ===================== TF2 =====================
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ===================== Nav2 =====================
        self.nav = BasicNavigator()
        self.get_logger().info("等待 Nav2 激活...")
        #self.nav.waitUntilNav2Active()
        self.get_logger().info("Nav2 已就绪！")

        # ===================== 状态与变量 =====================
        self.state = RobotState.IDLE
        self.current_hp = 600
        self.game_time = 0

        self.nav_busy = False
        self.current_waypoint = 0
        self.nav_start_time = 0.0
        self.nav_retry_count = 0
        self.cancel_pending = False

        self.arrive_home_time = 0.0  
        self.arrive_home_ros_time = None
        self.attack_start_time = 0.0

        # TF检查节流（ATTACK状态）
        self.last_attack_tf_check = 0.0

        self.current_pose_state = 2
        self.last_pose_broadcast_time = 0.0
        self.last_defense_log_time = 0.0
        self.last_tf_error_time = 0.0

        # ===================== 启动 =====================
        self.get_logger().info(f"加载巡逻点: {self.waypoints}")
        self.get_logger().info(f"家位置: {self.home}")
        self.get_logger().info(f"血量阈值: 低={self.HP_THRESHOLD_LOW}, 高={self.HP_THRESHOLD_HIGH}")
        
        self.publish_pose_state(2, force=True)
        
        self.timer = self.create_timer(0.1, self.state_machine_loop)
        self.get_logger().info("哨兵决策节点已完全启动")

    def change_state(self, new_state, reason=""):
        if self.state == new_state:
            return
        
        old_name = STATE_NAMES.get(self.state, f"UNKNOWN({self.state})")
        new_name = STATE_NAMES.get(new_state, f"UNKNOWN({new_state})")
        
        self.get_logger().info(f"[STATE_CHANGE] {old_name} -> {new_name} | 原因: {reason}")
        self.state = new_state
        
        new_pose_state = POSE_MAPPING.get(new_state, 2)
        self.current_pose_state = new_pose_state
        self.publish_pose_state(new_pose_state, force=True)

    def broadcast_pose_state(self, current_time):
        """持续广播姿态状态"""
        if current_time - self.last_pose_broadcast_time >= self.pose_broadcast_interval:
            self.publish_pose_state(self.current_pose_state, force=True, silent=True)
            self.last_pose_broadcast_time = current_time

    def publish_pose_state(self, pose_state, force=False, silent=False):
        """发布姿态状态"""
        if not force and hasattr(self, '_last_sent_pose_state') and \
           self._last_sent_pose_state == pose_state:
            return

        msg = FiredInfo()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose_state = pose_state
        self.pose_state_pub.publish(msg)

        if not silent and pose_state != getattr(self, "_last_sent_pose_state", None):
            self.get_logger().info(f"[POSE_STATE] -> {POSE_STATE_NAMES.get(pose_state, 'UNKNOWN')}")

        self._last_sent_pose_state = pose_state

    def serial_callback(self, msg: SerialPacket):
        self.current_hp = msg.robot_hp
        self.game_time = msg.game_time

    def get_current_distance_to(self, target_x, target_y, current_time):
        """基于TF树计算到目标的真实直线距离"""
        try:
            trans = self.tf_buffer.lookup_transform(
                'map', 
                'base_link', 
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            curr_x = trans.transform.translation.x
            curr_y = trans.transform.translation.y
            return math.hypot(curr_x - target_x, curr_y - target_y)
            
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if current_time - self.last_tf_error_time > 2.0:
                self.get_logger().warn(f"TF查询失败: {e}")
                self.last_tf_error_time = current_time
            return float('inf')

    def state_machine_loop(self):
        if self.game_time < -1:
            return

        # 修复3：使用nanoseconds（推荐写法）
        current_ros_time = self.get_clock().now().nanoseconds * 1e-9

        # 持续广播姿态状态
        self.broadcast_pose_state(current_ros_time)

        # 导航超时检查
        if self.nav_busy and not self.cancel_pending and \
           (current_ros_time - self.nav_start_time) > self.nav_timeout:
            self.get_logger().error("导航动作卡死超时！强制取消并重新规划")
            self.nav.cancelTask()
            self.cancel_pending = True
            return

        # 处理挂起的取消
        if self.cancel_pending:
            if self.nav.isTaskComplete():
                self.nav_busy = False
                self.cancel_pending = False
                self.start_return_home(reason="导航中断/超时")
            return

        # 状态分发
        handler = {
            RobotState.IDLE: self.handle_idle,
            RobotState.PATROLLING: self.handle_patrol,
            RobotState.RETURNING: self.handle_return,
            RobotState.DEFENDING: self.handle_defense,
            RobotState.ATTACKING: self.handle_attack
        }.get(self.state)
        
        if handler:
            handler(current_ros_time)

    def handle_idle(self, current_time):
        if self.current_hp < self.HP_THRESHOLD_LOW:
            self.start_return_home(reason="开局血量告急")
        else:
            self.change_state(RobotState.PATROLLING, reason="初始化完毕，启动巡逻")
            self.current_waypoint = 0

    def handle_patrol(self, current_time):
        # 血量低：启动取消流程
        if self.current_hp < self.HP_THRESHOLD_LOW:
            if self.nav_busy and not self.cancel_pending:
                self.nav.cancelTask()
                self.cancel_pending = True
                return
            if not self.nav_busy:
                self.start_return_home(reason="巡逻被击打，血量过低")
            return

        # 正常导航
        if not self.nav_busy:
            x, y = self.waypoints[self.current_waypoint]
            self.get_logger().info(f"前往巡逻点 {self.current_waypoint+1}/{len(self.waypoints)}: ({x}, {y})")
            self.send_goal(x, y)
            self.nav_busy = True
            self.nav_retry_count = 0
        else:
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                self.nav_busy = False

                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f"成功到达巡逻点 {self.current_waypoint+1}")
                    self.nav_retry_count = 0
                    
                    if self.current_waypoint == len(self.waypoints) - 1:
                        self.get_logger().info("到达最后一个巡逻点，切换至进攻姿态！")
                        self.nav_busy = False
                        self.change_state(RobotState.ATTACKING, reason="到达终点，开始进攻")
                        self.attack_start_time = current_time
                    else:
                        self.current_waypoint += 1
                        
                elif result == TaskResult.CANCELED:
                    self.get_logger().info("导航被取消")
                else:
                    self.nav_retry_count += 1
                    if self.nav_retry_count <= 3:
                        self.get_logger().warn(f"巡逻受阻，尝试原地重规划 ({self.nav_retry_count}/3)")
                        self.send_goal(self.waypoints[self.current_waypoint][0], 
                                     self.waypoints[self.current_waypoint][1])
                        self.nav_busy = True
                    else:
                        self.get_logger().error("该巡逻点死锁，跳过此点")
                        self.nav_retry_count = 0
                        if self.current_waypoint == len(self.waypoints) - 1:
                            self.nav_busy = False
                            self.change_state(RobotState.ATTACKING, reason="终点死锁，原地进攻")
                            self.attack_start_time = current_time
                        else:
                            self.current_waypoint += 1

    def handle_attack(self, current_time):
        """
        进攻状态：在最后一个点停留，发送pose_state=0
        偏离则自动回到攻击点
        """
        # 修复1：处理重导航的完成状态
        if self.nav_busy:
            if self.nav.isTaskComplete():
                self.nav_busy = False
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info("攻击点重定位成功，恢复进攻姿态")
                else:
                    self.get_logger().warn(f"攻击点重定位失败: {result}，保持原地进攻")
                # 恢复进攻姿态（FSM仍是ATTACKING）
                self.current_pose_state = 0
                self.publish_pose_state(0, force=True)
            else:
                # 重导航中，保持移动姿态
                return
        
        # 血量低：立即回家
        if self.current_hp < self.HP_THRESHOLD_LOW:
            self.get_logger().warn("进攻中血量告急，撤退回家！")
            self.start_return_home(reason="进攻中血量不足")
            return
        
        # 修复4：TF查询节流（5Hz）
        dist_to_point = float('inf')
        if current_time - self.last_attack_tf_check > self.attack_tf_check_interval:
            last_point = self.waypoints[-1]
            dist_to_point = self.get_current_distance_to(*last_point, current_time)
            self.last_attack_tf_check = current_time
            
            # 修复2：明确注释说明状态不一致是故意设计
            # ATTACK状态下临时移动姿态（重定位期间），FSM保持ATTACKING不变
            if dist_to_point > self.attack_position_tolerance:
                self.get_logger().warn(f"偏离攻击点 {dist_to_point:.2f}m > {self.attack_position_tolerance}m，重新导航")
                self.send_goal(*last_point)
                self.nav_busy = True
                # 临时切换为移动姿态，FSM仍保持ATTACKING（故意设计，见注释）
                self.current_pose_state = 2
                self.publish_pose_state(2, force=True)
                return
        
        # 进攻超时检查
        attack_time = current_time - self.attack_start_time
        if attack_time >= self.attack_duration:
            self.get_logger().info(f"进攻持续{attack_time:.1f}s，超时重新巡逻")
            self.change_state(RobotState.PATROLLING, reason="进攻超时，重新巡逻")
            self.current_waypoint = 0
            return
        
        # 确保进攻姿态（仅在需要时）
        if self.current_pose_state != 0:
            self.current_pose_state = 0
            self.publish_pose_state(0, force=True)
            
        if current_time - self.last_defense_log_time > 2.0:
            self.get_logger().info(f"[ATTACK] 进攻中... 已持续{attack_time:.1f}s/{self.attack_duration}s")
            self.last_defense_log_time = current_time

    def start_return_home(self, reason=""):
        if self.state == RobotState.RETURNING and self.nav_busy:
            self.get_logger().debug(f"已在回家途中且导航活跃，忽略重复请求: {reason}")
            return
            
        self.change_state(RobotState.RETURNING, reason=reason)
        self.send_goal(*self.home)
        self.nav_busy = True
        self.nav_retry_count = 0

    def handle_return(self, current_time):
        dist_to_home = float('inf')
        if self.nav_busy:
            dist_to_home = self.get_current_distance_to(*self.home, current_time)
        
        is_physically_home = dist_to_home < self.home_tolerance

        if self.nav_busy and self.nav.isTaskComplete():
            self.nav_busy = False
            result = self.nav.getResult()

            if result == TaskResult.SUCCEEDED or is_physically_home:
                self.change_state(RobotState.DEFENDING, 
                                reason=f"安全抵家 (距原点 {dist_to_home:.2f}m)")
                self.arrive_home_time = self.game_time
                self.arrive_home_ros_time = current_time
            else:
                self.nav_retry_count += 1
                if self.nav_retry_count <= 5:
                    self.get_logger().warn(f"回家被阻挡，重算路径 ({self.nav_retry_count}/5)")
                    self.send_goal(*self.home)
                    self.nav_busy = True
                else:
                    self.get_logger().error("彻底无法回家，就地转入防御状态！")
                    self.change_state(RobotState.DEFENDING, reason="回家死锁，原地防御")
                    self.arrive_home_time = self.game_time
                    self.arrive_home_ros_time = current_time

    def handle_defense(self, current_time):
        if self.arrive_home_ros_time is None:
            self.arrive_home_ros_time = current_time
            return

        time_at_home = max(0, current_time - self.arrive_home_ros_time)

        if self.current_hp >= self.HP_THRESHOLD_HIGH:
            if time_at_home >= self.stay_home_interval:
                self.change_state(RobotState.PATROLLING, reason="补血完毕且冷却结束，出击")
                self.current_waypoint = 0
            else:
                if current_time - self.last_defense_log_time > 2.0:
                    remaining = self.stay_home_interval - time_at_home
                    self.get_logger().info(f"[DEFENSE] 满血冷却中...剩余 {remaining:.1f}s")
                    self.last_defense_log_time = current_time
        else:
            if current_time - self.last_defense_log_time > 2.0:
                self.get_logger().info(f"[DEFENSE] 补血中: {self.current_hp}/{self.HP_THRESHOLD_HIGH}")
                self.last_defense_log_time = current_time

    def send_goal(self, x, y):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal_pose.position.z = 0.0

        goal_pose.pose.orientation.x = 0.0
        goal_pose.pose.orientation.y = 0.0
        goal_pose.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        self.nav.goToPose(goal)
        # 修复3：统一使用nanoseconds
        self.nav_start_time = self.get_clock().now().nanoseconds * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = RobotDecisionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()