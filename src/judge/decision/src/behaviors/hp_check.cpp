#include "rm_decision/behaviors/hp_check.hpp"

using namespace BT;

namespace rm_decision
{
  HPCheck::HPCheck(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
      : ConditionNode(name, config), node_(node)
  {
    // Get HP threshold from parameter (default 100)
    node_->get_parameter_or("hp_low_threshold", hp_threshold_, 100);
    RCLCPP_INFO(node_->get_logger(), "HPCheck initialized with threshold: %d", hp_threshold_);
  }

  NodeStatus HPCheck::tick()
  {
    // Get blackboard to read robot_hp
    auto blackboard = config().blackboard;
    
    uint16_t robot_hp = 0;
    blackboard->get("robot_hp", robot_hp);
    
    RCLCPP_DEBUG(node_->get_logger(), "HP Check: current HP=%d, threshold=%d", robot_hp, hp_threshold_);
    
    // Return SUCCESS if HP is below threshold (needs to return home)
    if (robot_hp < hp_threshold_)
    {
      RCLCPP_INFO(node_->get_logger(), "HP too low! HP=%d < threshold=%d, need to return home", 
                   robot_hp, hp_threshold_);
      return NodeStatus::SUCCESS;
    }
    
    // HP is OK, no need to return home
    return NodeStatus::FAILURE;
  }

  PortsList HPCheck::providedPorts()
  {
    return {
      InputPort<uint16_t>("robot_hp")
    };
  }

} // end namespace
