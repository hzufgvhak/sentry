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
    void listener_callback(const sensor_msgs::msg::Imu::UniquePtr msg)
    {
        // Re-express IMU data in a frame rotated 180 deg around Y:
        // x' = -x, y' = y, z' = -z
        constexpr double pi = 3.14159265358979323846;
        tf2::Quaternion rotation_quaternion;
        rotation_quaternion.setRPY(0.0, pi, 0.0);

        tf2::Quaternion input_orientation(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Quaternion output_orientation = rotation_quaternion * input_orientation;
        output_orientation.normalize();

        msg->orientation.x = output_orientation.x();
        msg->orientation.y = output_orientation.y();
        msg->orientation.z = output_orientation.z();
        msg->orientation.w = output_orientation.w();

        msg->angular_velocity.x = -msg->angular_velocity.x;
        msg->angular_velocity.z = -msg->angular_velocity.z;

        msg->linear_acceleration.x = -msg->linear_acceleration.x;
        msg->linear_acceleration.z = -msg->linear_acceleration.z;

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
