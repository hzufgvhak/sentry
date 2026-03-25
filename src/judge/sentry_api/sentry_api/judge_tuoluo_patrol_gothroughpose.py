#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from auto_aim_interfaces.msg import SerialPacket, Send


class TuoluoPatrolNode(Node):
    """陀螺巡逻节点。"""

    def __init__(self):
        """初始化节点、状态和通信接口。"""
        super().__init__('tuoluo_patrol_node')

        # 巡逻参数
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

        # 通信接口
        self.serial_sub = self.create_subscription(
            SerialPacket,
            '/serial_packet',
            self.serial_callback,
            10,
        )
        self.send_pub = self.create_publisher(Send, '/send', 10)
        self.timer = self.create_timer(0.1, self.state_machine_loop)

        self.get_logger().info('tuoluo_patrol_node 启动完成，等待 game_time==1')

    def serial_callback(self, msg: SerialPacket):
        """接收串口数据并更新血量、比赛时间。"""
        self.current_hp = msg.robot_hp
        self.game_time = msg.game_time

        if msg.game_time == 1 and not self.has_started_game_time_1:
            self.has_started_game_time_1 = True
            self.get_logger().info('接收到 game_time==1，准备开始巡逻')

    def state_machine_loop(self):
        """状态机主循环。"""
        if self.task_state == 'IDLE':
            if self.has_started_game_time_1:
                self.get_logger().info('状态切换：IDLE -> PATROLLING')
                self.current_wp_index = 0
                self.task_state = 'PATROLLING'
                self._publish_tuoluo(0)
                self._start_patrol_route_gothroughposes()
            return

        if self.task_state == 'PATROLLING':
            if self.current_hp < self.hp_low_threshold:
                self.get_logger().warn(
                    f'血量低于阈值 ({self.current_hp} < {self.hp_low_threshold})，切换到 RETURNING'
                )

                # 保险起见，先取消当前巡逻任务，再发回家目标。
                if self.nav.isTaskActive():
                    self.nav.cancelTask()

                self._publish_tuoluo(0)
                self._start_return_home()
                return

            # 巡逻阶段使用 goThroughPoses，因此任务成功表示整条路线执行完成。
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('整条巡逻路线执行完成，发布 tuoluo=1，开始返回家点')
                    self._publish_tuoluo(1)
                    self._start_return_home()
                else:
                    self.get_logger().warn('多点巡逻任务未成功，转到返回家点流程')
                    self._publish_tuoluo(0)
                    self._start_return_home()
            return

        if self.task_state == 'RETURNING':
            if self.nav.isTaskComplete():
                result = self.nav.getResult()
                if result == TaskResult.SUCCEEDED:
                    self.get_logger().info('已返回家点，进入 WAIT_RECOVER')
                    self.task_state = 'WAIT_RECOVER'
                    self._publish_tuoluo(0)
                else:
                    self.get_logger().warn('返回家点失败，重新尝试')
                    self._start_return_home()
            return

        if self.task_state == 'WAIT_RECOVER':
            if self.current_hp >= self.hp_recover_threshold:
                self.get_logger().info(f'血量恢复到 {self.current_hp}，开始新一轮巡逻')
                self.task_state = 'PATROLLING'
                self.current_wp_index = 0
                self._publish_tuoluo(0)
                self._start_patrol_route_gothroughposes()
            return

    def _start_next_patrol_goal(self):
        """按当前索引发送下一个巡逻点。"""
        if self.current_wp_index < len(self.waypoints):
            x, y = self.waypoints[self.current_wp_index]
            self.get_logger().info(f'巡逻目标 = ({x}, {y})')
            self._send_goal(x, y)
        else:
            self.get_logger().warn('巡逻点索引超出范围，转到返回家点流程')
            self._start_return_home()

    def _start_return_home(self):
        """开始执行回家点导航。"""
        self.task_state = 'RETURNING'
        self._send_goal(self.home[0], self.home[1])

    def _send_goal(self, x: float, y: float):
        """使用 goToPose 发送单点导航目标。"""
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

    def _build_route_poses(self):
        """构造 goThroughPoses 所需的路径点列表。"""
        route = []
        for x, y in self.waypoints:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = float(x)
            pose.pose.position.y = float(y)
            pose.pose.position.z = 0.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
            pose.pose.orientation.w = 1.0
            route.append(pose)
        return route

    def _start_patrol_route_gothroughposes(self):
        """使用 goThroughPoses 发送整条巡逻路径。"""
        route = self._build_route_poses()
        self.get_logger().info(f'准备发送 goThroughPoses 巡逻路径，点位数量 = {len(route)}')

        # 如果存在未结束任务，先取消，避免和新的巡逻任务冲突。
        if self.nav.isTaskActive():
            self.nav.cancelTask()

        self.nav.goThroughPoses(route)

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
