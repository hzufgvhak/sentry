#ifndef RM_DECISION_SERIAL_PACKET_SUBSCRIBER_HPP_
#define RM_DECISION_SERIAL_PACKET_SUBSCRIBER_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include "auto_aim_interfaces/msg/serial_packet.hpp"
#include <rclcpp/qos.hpp>
#include <optional>

using namespace BT;

namespace rm_decision
{
  class SerialPacketSubscriber : public SyncActionNode
  {
  public:
    SerialPacketSubscriber(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);
    ~SerialPacketSubscriber() override = default;
    
    // This function is invoked once at the beginning
    NodeStatus tick() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    
    // Serial packet data stored in blackboard
    uint16_t game_time_;
    uint8_t detect_color_;
    uint8_t task_mode_;
    bool is_play_;
    float roll_;
    float pitch_;
    float yaw_;
    uint16_t robot_hp_;
    uint8_t pose_state_;
    
    // Callback data
    rclcpp::Subscription<auto_aim_interfaces::msg::SerialPacket>::SharedPtr subscription_;
    std::mutex data_mutex_;
    
    void serial_packet_callback(const auto_aim_interfaces::msg::SerialPacket::SharedPtr msg);
  };
} // namespace rm_decision

#endif
