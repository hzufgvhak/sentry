#ifndef RM_DECISION_WAIT_FOR_DURATION_HPP_
#define RM_DECISION_WAIT_FOR_DURATION_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include <chrono>

using namespace BT;

namespace rm_decision
{
  class WaitForDuration : public SyncActionNode
  {
  public:
    WaitForDuration(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // Tick function - waits for specified duration then returns SUCCESS
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    double duration_;
  };
} // namespace rm_decision

#endif
