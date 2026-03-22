#include "rm_decision/behaviors/wait_for_duration.hpp"

using namespace BT;

namespace rm_decision
{
  WaitForDuration::WaitForDuration(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : SyncActionNode(name, config), node_(node)
  {
    // Get duration from input port or use default
    duration_ = 0.1;  // Default 100ms
    
    // Try to get from blackboard
    auto blackboard = config.blackboard;
    if (blackboard)
    {
      blackboard->get("duration", duration_);
    }
  }

  NodeStatus WaitForDuration::tick()
  {
    // Get duration from input port
    auto blackboard = config().blackboard;
    
    double duration = duration_;
    getInput("duration", duration);
    
    // Wait for specified duration
    std::this_thread::sleep_for(std::chrono::duration<double>(duration));
    
    RCLCPP_DEBUG(node_->get_logger(), "WaitForDuration: waited for %f seconds", duration);
    
    return NodeStatus::SUCCESS;
  }

  PortsList WaitForDuration::providedPorts()
  {
    return {
      InputPort<double>("duration")  // Duration in seconds
    };
  }

} // namespace rm_decision
