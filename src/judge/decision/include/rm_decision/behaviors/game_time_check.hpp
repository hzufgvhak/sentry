#ifndef RM_DECISION_GAME_TIME_CHECK_HPP_
#define RM_DECISION_GAME_TIME_CHECK_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <optional>

using namespace BT;

namespace rm_decision
{
  class GameTimeCheck : public ConditionNode
  {
  public:
    GameTimeCheck(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~GameTimeCheck() override = default;
    
    // This function is invoked once at the beginning
    BT::NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    int target_game_time_;
  };
} // namespace rm_decision

#endif
