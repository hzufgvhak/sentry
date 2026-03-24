#ifndef RM_DECISION_TARGET_SUBSCRIBER_HPP_
#define RM_DECISION_TARGET_SUBSCRIBER_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/target.hpp"
#include <rclcpp/qos.hpp>
#include <optional>

using namespace BT;

namespace rm_decision
{
  class TargetSubscriber : public SyncActionNode
  {
  public:
    TargetSubscriber(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~TargetSubscriber() override = default;
    
    // This function is invoked once at the beginning
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    
    // Target data
    bool tracking_;
    std::string id_;
    int armors_num_;
    float position_x_;
    float position_y_;
    float position_z_;
    
    // Callback data
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr subscription_;
    std::mutex data_mutex_;
    
    void target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  };
} // namespace rm_decision

#endif
