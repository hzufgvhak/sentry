#include "rm_decision/behaviors/target_subscriber.hpp"

using namespace BT;

namespace rm_decision
{

  TargetSubscriber::TargetSubscriber(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(node)
  {
    // Initialize default values
    tracking_ = false;
    id_ = "";
    armors_num_ = 0;
    position_x_ = 0.0f;
    position_y_ = 0.0f;
    position_z_ = 0.0f;
    
    // Create subscription to /target topic
    subscription_ = node_->create_subscription<auto_aim_interfaces::msg::Target>(
      "/target",
      rclcpp::QoS(10),
      std::bind(&TargetSubscriber::target_callback, this, std::placeholders::_1)
    );
    
    RCLCPP_INFO(node_->get_logger(), "TargetSubscriber initialized, listening on /target");
  }

  NodeStatus TargetSubscriber::tick()
  {
    // Spin node to process callbacks
    rclcpp::spin_some(node_);
    
    // Get the blackboard and write values directly
    auto blackboard = config().blackboard;
    
    // Write target data to blackboard
    blackboard->set("tracking", tracking_);
    blackboard->set("target_id", id_);
    blackboard->set("armors_num", armors_num_);
    blackboard->set("target_x", position_x_);
    blackboard->set("target_y", position_y_);
    blackboard->set("target_z", position_z_);
    
    RCLCPP_DEBUG(node_->get_logger(), "TargetSubscriber tick: tracking=%d, id=%s", 
                 tracking_, id_.c_str());
    
    return NodeStatus::SUCCESS;
  }

  PortsList TargetSubscriber::providedPorts()
  {
    return {
      OutputPort<bool>("tracking"),
      OutputPort<std::string>("target_id"),
      OutputPort<int>("armors_num"),
      OutputPort<float>("target_x"),
      OutputPort<float>("target_y"),
      OutputPort<float>("target_z")
    };
  }

  void TargetSubscriber::target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    
    tracking_ = msg->tracking;
    id_ = msg->id;
    armors_num_ = msg->armors_num;
    
    // Get position from geometry_msgs/Point
    position_x_ = msg->position.x;
    position_y_ = msg->position.y;
    position_z_ = msg->position.z;
    
    RCLCPP_INFO(node_->get_logger(), 
                "Received Target: tracking=%d, id=%s, armors_num=%d",
                tracking_, id_.c_str(), armors_num_);
  }

} // namespace rm_decision
