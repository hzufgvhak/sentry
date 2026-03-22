#ifndef RM_DECISION_POSE_STATE_PUBLISHER_HPP_
#define RM_DECISION_POSE_STATE_PUBLISHER_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <auto_aim_interfaces/msg/send.hpp>

using namespace BT;

namespace rm_decision
{
  // PoseStatePublisher - publishes pose_state to /send topic
  // Used to inform other nodes about current movement state
  // pose_state: 0-进攻(attack) 1-防御(defend) 2-移动(move)
  class PoseStatePublisher : public SyncActionNode
  {
  public:
    PoseStatePublisher(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // Tick function - publishes the pose_state
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<auto_aim_interfaces::msg::Send>::SharedPtr publisher_;
    
    // Default pose state to publish
    uint8_t default_pose_state_;
  };
} // namespace rm_decision

#endif
