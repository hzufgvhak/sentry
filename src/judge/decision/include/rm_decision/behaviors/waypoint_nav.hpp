#ifndef RM_DECISION_WAYPOINT_NAV_HPP_
#define RM_DECISION_WAYPOINT_NAV_HPP_

#include "behaviortree_cpp/bt_factory.h"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <nav2_msgs/action/navigate_through_poses.hpp>
#include <action_msgs/msg/goal_status_array.hpp>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "rm_decision/custume_types.hpp"
#include "rclcpp_action/rclcpp_action.hpp"



using namespace BT;
using NavigateThroughPoses = nav2_msgs::action::NavigateThroughPoses;
using NavigateThroughPosesGoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPoses>;

namespace rm_decision
{
  class WaypointNav : public StatefulActionNode
  {
  public:
    WaypointNav(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node);

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
    
    // Waypoints sequence
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    
    // Current waypoint index (for resuming)
    size_t current_waypoint_index_;
    
    // Helper to create waypoints based on judge.py logic
    void init_waypoints();
    
    // Set starting waypoint index from blackboard
    void load_start_index();
  };
} // namespace rm_decision

#endif
