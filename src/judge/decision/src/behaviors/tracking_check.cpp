#include "rm_decision/behaviors/tracking_check.hpp"

using namespace BT;

namespace rm_decision
{
  TrackingCheck::TrackingCheck(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node)
      : ConditionNode(name, config), node_(node)
  {
  }

  NodeStatus TrackingCheck::tick()
  {
    // Get tracking status from blackboard
    auto blackboard = config().blackboard;
    
    bool tracking = false;
    
    // Try to get tracking from blackboard
    if (blackboard->get("tracking", tracking))
    {
      RCLCPP_DEBUG(node_->get_logger(), "TrackingCheck: tracking=%d", tracking);
      
      // Return SUCCESS if tracking is true, FAILURE otherwise
      if (tracking)
      {
        return NodeStatus::SUCCESS;
      }
      else
      {
        return NodeStatus::FAILURE;
      }
    }
    else
    {
      RCLCPP_WARN(node_->get_logger(), "TrackingCheck: tracking not found in blackboard");
      return NodeStatus::FAILURE;
    }
  }

  PortsList TrackingCheck::providedPorts()
  {
    return {
      // No input ports needed - reads from blackboard
    };
  }

} // namespace rm_decision
