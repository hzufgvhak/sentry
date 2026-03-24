#include "rm_decision/behaviors/pose_state_publisher.hpp"

using namespace BT;

namespace rm_decision
{
  PoseStatePublisher::PoseStatePublisher(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
      : SyncActionNode(name, config), node_(node)
  {
    // Create publisher to /send topic
    publisher_ = node_->create_publisher<auto_aim_interfaces::msg::Send>("/send", 10);
    
    // Get default pose state from parameter - use static_cast to match types
    uint8_t default_val = 2;  // Default to 2 (移动/move)
    node_->get_parameter_or("default_pose_state", default_pose_state_, default_val);
    
    RCLCPP_INFO(node_->get_logger(), "PoseStatePublisher initialized, publishing to /send topic");
  }

  NodeStatus PoseStatePublisher::tick()
  {
    // Try to get pose_state from blackboard first
    auto blackboard = config().blackboard;
    
    uint8_t pose_state = default_pose_state_;
    
    // Check if there's a pose_state in blackboard
    if (blackboard->get("pose_state", pose_state))
    {
      RCLCPP_DEBUG(node_->get_logger(), "Using pose_state from blackboard: %d", pose_state);
    }
    else
    {
      pose_state = default_pose_state_;
      RCLCPP_DEBUG(node_->get_logger(), "Using default pose_state: %d", pose_state);
    }
    
    // Create and publish the message
    auto msg = auto_aim_interfaces::msg::Send();
    msg.pose_state = pose_state;
    
    publisher_->publish(msg);
    
    // Log the pose state
    std::string state_str;
    switch (pose_state)
    {
      case 0: state_str = "进攻(attack)"; break;
      case 1: state_str = "防御(defend)"; break;
      case 2: state_str = "移动(move)"; break;
      default: state_str = "未知"; break;
    }
    RCLCPP_INFO(node_->get_logger(), "Published pose_state: %d (%s)", pose_state, state_str.c_str());
    
    return NodeStatus::SUCCESS;
  }

  PortsList PoseStatePublisher::providedPorts()
  {
    return {
      InputPort<uint8_t>("pose_state")  // Optional input pose state
    };
  }

} // end namespace
