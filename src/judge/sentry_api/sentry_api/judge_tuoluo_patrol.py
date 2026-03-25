#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from auto_aim_interfaces.msg import SerialPacket, Send


class TuoluoPatrolNode(Node):
    """小车按给定地点巡逻，血量低时回家点，并发布 tuoluo 状态。"""

    def __init__(self):
        """初始化节点、状态和导航接口。"""
        super().__init__('tuoluo_patrol_node')

        # 参数
        self.waypoints = [(1.0, 1.0), (4.5, 1.2), (4.5, 0.1)]
        self.home = (0.0, 0.0)
        self.hp_low_threshold = 100
        self.hp_recover_threshold = 400

        # 状态变量
        self.current_hp = 600
        self.game_time = 0
        self.task_state = 'IDLE'  # IDLE, PATROLLING, RETURNING, WAIT_RECOVER
        self.current_wp_index = 0
        self.nav = BasicNavigator()
        self.has_started_game_time_1 = False
        # 区分“目标未发送 / 已发送未完成 / 已完成待处理结果”。
        self.nav_goal_sent = False

        # 通信
        self.serial_sub = self.create_subscription(
            SerialPacket, '/serial_packet', self.serial_callback, 10
        )
        self.send_pub = self.create_publisher(Send, '/send', 10)
        self.timer = self.create_timer(0.1, self.state_machine_loop)

        self.get_logger().info('tuoluo_patrol_node 启动完成，等待 game_time==1')

    def serial_callback(self, msg: SerialPacket):
        """接收裁判数据并更新状态输入。"""
        self.current_hp = msg.robot_hp
        self.game_time = msg.game_time

        # 记录 game_time 为 1 的首次触发点
        if msg.game_time == 1 and not self.has_started_game_time_1:
            self.has_started_game_time_1 = True
            self.get_logger().info('接收到 game_time==1，准备开始巡逻')

    def state_machine_loop(self):
        """状态机主循环。"""
        # 1. IDLE -> PATROLLING 触发
        if self.task_state == 'IDLE':
            if self.has_started_game_time_1:
                self.get_logger().info('状态转换：IDLE -> PATROLLING')
                self.current_wp_index = 0
                self.task_state = 'PATROLLING'
                self._publish_tuoluo(0)
                self._start_next_patrol_goal()
            return

        # 2. PATROLLING 进行中
        if self.task_state == 'PATROLLING':
            if self.current_hp < self.hp_low_threshold:
                self.get_logger().warn(
                    f'血量低于阈值 ({self.current_hp} < {self.hp_low_threshold})，切换到 RETURNING'
                )
                self._publish_tuoluo(0)
                self._start_return_home()
                return

            # 只有任务真正结束后，才读取导航结果。
            if self.nav_goal_sent and self.nav.isTaskComplete():
                self.nav_goal_sent = False
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info(f'已到达巡逻点 {self.current_wp_index}')
                    self.current_wp_index += 1
                    if self.current_wp_index >= len(self.waypoints):
                        self.get_logger().info('完成巡逻路线，发布 tuoluo=1，开始返回家点')
                        self._publish_tuoluo(1)
                        self._start_return_home()
                    else:
                        self._start_next_patrol_goal()
                else:
                    self.get_logger().warn('巡逻点导航未成功，切换到返回家点流程')
                    self._publish_tuoluo(0)
                    self._start_return_home()
            return

        # 3. RETURNING 进行中
        if self.task_state == 'RETURNING':
            # 只有回家任务真正结束后，才读取导航结果。
            if self.nav_goal_sent and self.nav.isTaskComplete():
                self.nav_goal_sent = False
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('已返回家点，进入 WAIT_RECOVER')
                    self.task_state = 'WAIT_RECOVER'
                    self._publish_tuoluo(0)
                else:
                    self.get_logger().warn('返回家点失败，重新尝试')
                    self._start_return_home()
            return

        # 4. WAIT_RECOVER 等待血量恢复
        if self.task_state == 'WAIT_RECOVER':
            if self.current_hp >= self.hp_recover_threshold:
                self.get_logger().info(f'血量恢复到 {self.current_hp}，开始新一轮巡逻')
                self.task_state = 'PATROLLING'
                self.current_wp_index = 0
                self._publish_tuoluo(0)
                self._start_next_patrol_goal()
            return

    def _start_next_patrol_goal(self):
        """发送下一个巡逻点。"""
        if self.current_wp_index < len(self.waypoints):
            x, y = self.waypoints[self.current_wp_index]
            self.get_logger().info(f'巡逻目标 = ({x}, {y})')
            self._send_goal(x, y)
        else:
            self.get_logger().warn('巡逻点索引超出范围，返回家点')
            self._start_return_home()

    def _start_return_home(self):
        """切换到回家状态并发送回家目标。"""
        self.task_state = 'RETURNING'
        self._send_goal(self.home[0], self.home[1])

    def _send_goal(self, x: float, y: float):
        """发送单点导航目标。"""
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        goal.pose.position.z = 0.0
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0

        self.nav.goToPose(goal)
        self.nav_goal_sent = True

    def _publish_tuoluo(self, value: int):
        """发布 tuoluo 状态。"""
        msg = Send()
        msg.pose_state = 2
        msg.tuoluo = bool(value)
        self.send_pub.publish(msg)
        self.get_logger().info(f'发布 tuoluo={int(msg.tuoluo)}')


def main(args=None):
    """节点入口。"""
    rclpy.init(args=args)
    node = TuoluoPatrolNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
