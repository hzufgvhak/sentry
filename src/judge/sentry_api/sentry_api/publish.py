import rclpy
from rclpy.node import Node
# 导入自定义消息类型（修复后的 auto_aim_interfaces/Send）
from auto_aim_interfaces.msg import Send  

class SerialPacketPublisher(Node):
    def __init__(self):
        # 1. 初始化节点
        super().__init__('serial_packet_publisher')
        
        # 2. 创建发布者（核心：定义话题名+消息类型+QoS）
        self.publisher_ = self.create_publisher(
            Send,                # 消息类型：auto_aim_interfaces/Send
            '/serial_packet_twice',    # 话题名：自定义，和订阅者一致即可
            10                   # QoS：缓存深度10
        )
        
        # 3. 定时发布消息（触发话题激活）
        timer_period = 1.0  # 1秒发布一次
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        # 构造消息内容
        msg = Send()
        msg.pitch = 0.5 + self.count * 0.1
        msg.yaw = 0.2 + self.count * 0.05
        msg.state = 3  # 默认移动姿态
        
        # 4. 发布消息（关键：发布后话题才会被ROS 2注册）
        self.publisher_.publish(msg)
        self.get_logger().info(f"发布哨兵数据：pitch={msg.pitch}, state={msg.state}")
        self.count += 1

def main(args=None):
    # 初始化ROS 2
    rclpy.init(args=args)
    # 创建发布者节点
    publisher = SerialPacketPublisher()
    # 保持节点运行，持续发布消息
    rclpy.spin(publisher)
    # 销毁节点
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
