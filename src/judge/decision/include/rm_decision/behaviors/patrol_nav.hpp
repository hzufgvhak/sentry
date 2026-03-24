#ifndef RM_DECISION_PATROL_NAV_HPP_
#define RM_DECISION_PATROL_NAV_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rm_decision/custume_types.hpp"

// Include auto_aim message for tracking detection
#include <auto_aim_interfaces/msg/target.hpp>

using namespace BT;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavigateThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

namespace rm_decision
{
  // PatrolNav - navigates through a set of waypoints in a loop
  // Used for post-waypoint patrol behavior
  // Monitors HP and time during patrol - returns to start if HP drops significantly
  // Also monitors tracking status - pauses patrol if enemy is being tracked
  class PatrolNav : public StatefulActionNode
  {
  public:
    PatrolNav(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

    // This function is invoked once at the beginning
    NodeStatus onStart() override;

    // If onStart() returned RUNNING, we will keep calling
    // this method until it return something different from RUNNING
    NodeStatus onRunning() override;

    // Callback to execute if the action was aborted by another node
    void onHalted() override;

    static PortsList providedPorts();

  private:
    rclcpp::Node::SharedPtr node_;
    // Action client
    rclcpp_action::Client<NavigateThroughPoses>::SharedPtr action_client_;
    // Goal handle
    NavigateThroughPosesGoalHandle::SharedPtr goal_handle_;
    // Send goal timeout
    int send_goal_timeout_;
    nav2_msgs::action::NavigateThroughPoses::Goal navigation_goal_;
    
    // Patrol waypoints - loop between these points
    std::vector<geometry_msgs::msg::PoseStamped> patrol_waypoints_;
    
    // Starting patrol position (for returning if attacked)
    geometry_msgs::msg::PoseStamped patrol_start_pose_;
    
    // Current patrol round
    int current_patrol_round_;
    
    // Maximum patrol rounds (0 = infinite)
    int max_patrol_rounds_;
    
    // HP monitoring during patrol
    uint16_t start_hp_;           // HP when patrol started
    rclcpp::Time hp_drop_time_;   // Time when HP drop was detected
    int hp_drop_threshold_;       // HP drop amount that triggers return
    int hp_drop_time_window_;     // Time window (seconds) to detect HP drop
    bool hp_drop_detected_;       // Flag if significant HP drop detected
    
    // Tracking detection during patrol
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_subscription_;
    bool current_tracking_;        // Current tracking status from message
    std::mutex tracking_mutex_;
    
    // Tracking detection parameters
    int tracking_check_duration_;  // Duration to check tracking (seconds)
    int tracking_threshold_;       // Number of tracking true needed to stop patrol
    int tracking_check_count_;     // Current count during checking
    bool is_checking_tracking_;    // Flag if currently checking tracking
    rclcpp::Time tracking_check_start_; // Start time of tracking check
    
    // Initialize patrol waypoints
    void init_patrol_waypoints();
    
    // Check if should return due to HP drop
    bool shouldReturnToStart();
    
    // Check if tracking is detected
    bool isTrackingDetected();
    
    // Callback for target message
    void targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
  };
} // namespace rm_decision

#endif
