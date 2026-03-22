#ifndef RM_DECISION_HP_CHECK_HPP_
#define RM_DECISION_HP_CHECK_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>

#include "rm_decision/custume_types.hpp"

using namespace BT;

namespace rm_decision
{
  class HPCheck : public ConditionNode
  {
  public:
    HPCheck(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // Tick function - returns SUCCESS if HP is below threshold, FAILURE otherwise
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    // Threshold for low HP - configurable via parameter
    int hp_threshold_;
  };
} // namespace rm_decision

#endif
