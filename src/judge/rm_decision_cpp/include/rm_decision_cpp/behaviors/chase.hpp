#ifndef RM_DECISION_CHASE_HPP_
#define RM_DECISION_CHASE_HPP_

#include "behaviortree_cpp/bt_factory.h"
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

#include "rm_decision_cpp/custume_types.hpp"

using namespace BT;

namespace rm_decision
{
  // Chase behavior based on zone and target
  // Zone 0: Defense - Stay near base, limited chase
  // Zone 1: Mid - Moderate chase capability  
  // Zone 2: Attack - Aggressive chase towards enemy
  
  class Chase : public SyncActionNode
  {
  public:
    Chase(const std::string &name, const NodeConfig &config, 
          std::shared_ptr<rclcpp::Node> node,
          std::shared_ptr<tf2_ros::Buffer> tf_buffer,
          std::shared_ptr<tf2_ros::TransformListener> tf_listener);
    
    ~Chase() override = default;
    
    NodeStatus tick() override;
    
    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Chase parameters
    double max_chase_distance_;  // Maximum distance to chase from defense zone
    double mid_chase_distance_;   // Maximum distance to chase from mid zone
    double attack_chase_distance_; // Maximum distance to chase from attack zone
    
    std::string global_frame_;
    std::string base_frame_;
    
    // Current robot state
    geometry_msgs::msg::Pose robot_pose_;
    geometry_msgs::msg::Pose target_pose_;
    uint8_t robot_zone_;
    bool tracking_;
    
    void get_robot_pose_();
    bool should_chase_() const;
  };
} // namespace rm_decision

#endif
