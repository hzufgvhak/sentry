// #include <rclcpp/rclcpp.hpp>
// #include <sensor_msgs/msg/imu.hpp>
// #include <tf2/LinearMath/Quaternion.h>

// class IMURotateNode : public rclcpp::Node
// {
// public:
//     IMURotateNode() : Node("imu_rotate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
//     {
//         publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data1", 10);
//         subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
//             "imu/data", 10, std::bind(&IMURotateNode::listener_callback, this, std::placeholders::_1));
//     }

// private:
//     void listener_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
//     {
//         // Create a quaternion for the rotation
//         tf2::Quaternion rotation_quaternion;
//         rotation_quaternion.setRPY(0, 3.14159, 0);  // 180 degrees in radians

//         // Rotate the orientation of the IMU data1
//         msg->orientation.x = 0.0;
//         msg->orientation.y = 0.0;
//         msg->orientation.z = 0.0;
//         msg->orientation.w = 1.0;

//         msg->angular_velocity.z = -msg->angular_velocity.z;
//         msg->linear_acceleration.y = -msg->linear_acceleration.y;
//         msg->linear_acceleration.z = -msg->linear_acceleration.z;

//         // Publish the rotated IMU data1
//         publisher_->publish(*msg);
//     }

//     rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
//     rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
// };

// int main(int argc, char * argv[])
// {
//     rclcpp::init(argc, argv);
//     rclcpp::spin(std::make_shared<IMURotateNode>());
//     rclcpp::shutdown();
//     return 0;
// }
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>

class IMURotateNode : public rclcpp::Node
{
public:
    IMURotateNode() : Node("imu_rotate_node", rclcpp::NodeOptions().use_intra_process_comms(true))
    {
        publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_fastlio", 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "imu/data", 10, std::bind(&IMURotateNode::listener_callback, this, std::placeholders::_1));
    }

private:
    /**
     * @brief IMU数据回调函数 - 处理并转换IMU数据坐标
     * 
     * 将IMU数据坐标框架绕Z轴旋转180度
     * 变换公式: x' = -x, y' = -y, z' = z
     * 用于修正IMU安装方向与期望坐标系方向的差异
     * 
     * @param msg 接收到的IMU消息
     */
    void listener_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        // 绕Z轴旋转180度的四元数
        constexpr double pi = 3.14159265358979323846;
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0.0, 0.0, pi);

        // 四元数旋转变换: 将原始姿态转换到新坐标系
        tf2::Quaternion input_orientation(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Quaternion output_orientation = rotation_quaternion * input_orientation;
        output_orientation.normalize();

        // 更新方向四元数
        msg->orientation.x = output_orientation.x();
        msg->orientation.y = output_orientation.y();
        msg->orientation.z = output_orientation.z();
        msg->orientation.w = output_orientation.w();
        
        // 角速度分量变换 (绕Z轴180度: x' = -x, y' = -y, z' = z)
        // X轴分量取反
        msg->angular_velocity.x = -msg->angular_velocity.x;
        // Y轴分量取反
        msg->angular_velocity.y = -msg->angular_velocity.y;
        // Z分量保持不变

        // 线加速度分量变换 (绕Z轴180度: x' = -x, y' = -y, z' = z)
        // X轴分量取反
        msg->linear_acceleration.x = -msg->linear_acceleration.x;
        // Y轴分量取反
        msg->linear_acceleration.y = -msg->linear_acceleration.y;
        // Z分量保持不变

        publisher_->publish(*msg);
    }

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<IMURotateNode>());
    rclcpp::shutdown();
    return 0;
}