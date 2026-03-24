#include "rm_decision/behaviors/game_time_check.hpp"

using namespace BT;

namespace rm_decision
{

  GameTimeCheck::GameTimeCheck(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : ConditionNode(name, config), node_(node)
  {
    // Get target game time from input port or use default
    node_->get_parameter_or("target_game_time", target_game_time_, 1);
  }

  BT::NodeStatus GameTimeCheck::tick()
  {
    // Get game_time from blackboard
    auto game_time = getInput<uint16_t>("game_time");
    
    if (!game_time)
    {
      RCLCPP_WARN(node_->get_logger(), "game_time not available in blackboard");
      return BT::NodeStatus::FAILURE;
    }
    
    RCLCPP_DEBUG(node_->get_logger(), "Checking game_time: current=%d, target=%d", 
                  game_time.value(), target_game_time_);
    
    // Check if game_time equals target (default 1)
    if (game_time.value() == target_game_time_)
    {
      RCLCPP_INFO(node_->get_logger(), "Game time check PASSED: %d == %d", 
                   game_time.value(), target_game_time_);
      return BT::NodeStatus::SUCCESS;
    }
    else
    {
      return BT::NodeStatus::FAILURE;
    }
  }

  PortsList GameTimeCheck::providedPorts()
  {
    const char* description = "Target game time to check against";
    return {
      InputPort<uint16_t>("game_time", description)
    };
  }

} // namespace rm_decision
