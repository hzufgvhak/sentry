import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from auto_aim_interfaces.msg import SerialPacket,Send

class SerialPacketSubscriber(Node):
    def __init__(self):
        super().__init__('serial_packet_subscriber')
        self.subscription = self.create_subscription(
            SerialPacket,
            '/serial_packet',
            self.listener_callback,
            10
        )

        self.nav = BasicNavigator()
        # 记录导航状态，防止高频重复触发
        self.is_navigating = False
        self.timer = None  # 初始化定时器，避免后续调用时未定义
        self.get_logger().info("Listening for SerialPacket messages...")

    def listener_callback(self, msg: SerialPacket):
        # 1. 增加条件：如果已经在导航中，不要重复发送目标
        # 关键：先检查导航状态，再检查触发条件，避免竞态条件
        if self.is_navigating:
            self.get_logger().debug("Already navigating, ignoring message")
            return

        # 2. 只有满足特定条件且未在导航时触发
        if msg.game_time == 0:
            self.get_logger().info("Trigger condition met, starting navigation")
            waypoints = [(-1.9, 0.1), (-1.9,-1.2), (-2.1, -2.1)]
            self.start_nav(waypoints)

    def start_nav(self, points):
        # 立即设置标志位，防止重复触发
        self.is_navigating = True
        
        route = []
        for x, y in points:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.nav.get_clock().now().to_msg()
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            route.append(pose)

        self.get_logger().info(f"发送多点导航请求, 共 {len(route)} 个点...")
        
        # 检查是否有之前的任务未完成，如果有则先取消
        if not self.nav.isTaskComplete():
            self.get_logger().warn("Previous task not complete, cancelling first")
            self.nav.cancelTask()
        
        self.nav.goThroughPoses(route)
        
        # --- 关键加固 2 ---
        # 给 Action Client 一点时间去接收 Goal 请求
        import time
        time.sleep(0.1) 
        
        if self.nav.isTaskComplete():
             # 如果刚发出去就结束了，说明被拒绝了
             result = self.nav.getResult()
             if result != TaskResult.SUCCEEDED:
                 self.get_logger().error("目标在发送瞬间被拒绝！请检查地图、坐标点是否在障碍物内")
                 self.is_navigating = False
                 return
        # ------------------
        
        # 创建或重置定时器来检查状态
        if self.timer:
            self.timer.cancel()
        self.timer = self.create_timer(0.5, self.check_nav_status)

    def check_nav_status(self):
        # 检查任务是否完成
        if not self.nav.isTaskComplete():
            try:
                feedback = self.nav.getFeedback()
                if feedback and hasattr(feedback, 'distance_remaining'):
                    self.get_logger().info(f'剩余距离: {feedback.distance_remaining:.2f} m')
            except Exception as e:
                self.get_logger().debug(f'获取反馈失败: {e}')
            return

        # 任务完成后，获取结果并处理
        result = self.nav.getResult()
        
        # 添加结果为空检查，当目标被拒绝时 result 可能为 None
        if result is None:
            self.get_logger().error("导航结果为空，目标可能被拒绝或已取消")
        elif result == TaskResult.SUCCEEDED:
            self.get_logger().info("导航成功！")
        elif result == TaskResult.CANCELED:
            self.get_logger().warn("导航被取消")
        elif result == TaskResult.FAILED:
            self.get_logger().error("导航失败")
        else:
            self.get_logger().error(f"导航未知状态: {result}")
        
        # 清理状态
        self.is_navigating = False
        if self.timer:
            self.timer.cancel()
            self.timer = None

def main(args=None):
    rclpy.init(args=args)
    # 推荐使用 MultiThreadedExecutor 如果你一定要在回调里写复杂逻辑
    serial_packet_sub = SerialPacketSubscriber()
    rclpy.spin(serial_packet_sub)
    rclpy.shutdown()