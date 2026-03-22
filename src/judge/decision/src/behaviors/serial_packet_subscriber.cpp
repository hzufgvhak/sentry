#include "rm_decision/behaviors/serial_packet_subscriber.hpp"

using namespace BT;

namespace rm_decision
{

  SerialPacketSubscriber::SerialPacketSubscriber(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(node)
  {
    // Initialize default values
    game_time_ = 0;
    detect_color_ = 0;
    task_mode_ = 0;
    is_play_ = false;
    roll_ = 0.0f;
    pitch_ = 0.0f;
    yaw_ = 0.0f;
    robot_hp_ = 0;
    pose_state_ = 0;
    
    // Create subscription to serial_packet topic
    subscription_ = node_->create_subscription<auto_aim_interfaces::msg::SerialPacket>(
      "/serial_packet",
      rclcpp::QoS(10),
      std::bind(&SerialPacketSubscriber::serial_packet_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "SerialPacketSubscriber initialized, listening on /serial_packet");
  }

  NodeStatus SerialPacketSubscriber::tick()
  {
    // Spin node to process callbacks
    rclcpp::spin_some(node_);
    
    // Get the blackboard and write values directly
    auto blackboard = config().blackboard;
    
    // Write game state to blackboard
    blackboard->set("game_time", game_time_);
    blackboard->set("detect_color", detect_color_);
    blackboard->set("task_mode", task_mode_);
    blackboard->set("is_play", is_play_);
    blackboard->set("roll", roll_);
    blackboard->set("pitch", pitch_);
    blackboard->set("yaw", yaw_);
    blackboard->set("robot_hp", robot_hp_);
    blackboard->set("pose_state", pose_state_);
    
    RCLCPP_DEBUG(node_->get_logger(), "SerialPacketSubscriber tick: game_time=%d, is_play=%d", 
                 game_time_, is_play_);
    
    return NodeStatus::SUCCESS;
  }

  PortsList SerialPacketSubscriber::providedPorts()
  {
    return {
      OutputPort<uint16_t>("game_time"),
      OutputPort<uint8_t>("detect_color"),
      OutputPort<uint8_t>("task_mode"),
      OutputPort<bool>("is_play"),
      OutputPort<float>("roll"),
      OutputPort<float>("pitch"),
      OutputPort<float>("yaw"),
      OutputPort<uint16_t>("robot_hp")
    };
  }

  void SerialPacketSubscriber::serial_packet_callback(const auto_aim_interfaces::msg::SerialPacket::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    game_time_ = msg->game_time;
    detect_color_ = msg->detect_color;
    task_mode_ = msg->task_mode;
    is_play_ = msg->is_play;
    roll_ = msg->roll;
    pitch_ = msg->pitch;
    yaw_ = msg->yaw;
    robot_hp_ = msg->robot_hp;
    pose_state_ = msg->pose_state;
    
    
    RCLCPP_INFO(node_->get_logger(), 
                 "Received SerialPacket: game_time=%d, detect_color=%d, task_mode=%d, is_play=%d",
                 game_time_, detect_color_, task_mode_, is_play_);
  }

} // namespace rm_decision
