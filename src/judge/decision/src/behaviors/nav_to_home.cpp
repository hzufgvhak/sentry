#include "rm_decision/behaviors/nav_to_home.hpp"

using namespace BT;

namespace rm_decision
{
  NavToHome::NavToHome(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node), is_waiting_(false)
  {
    // Using NavigateThroughPoses for multi-point navigation to home
    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, "navigate_through_poses");
    node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 1000);
    // Default wait time at home is 10 seconds
    node_->get_parameter_or("wait_time_at_home", wait_time_at_home_, 10);
    
    // Initialize home waypoints
    init_home_waypoints();
  }

  void NavToHome::init_home_waypoints()
  {
    // Define waypoints to return home - reverse of original waypoints
    // Original: (1.2,1.2) -> (4.5,1.2) -> (4.5,0.1) -> (1.0,0.0)
    // Return: reverse order ending at (0,0)
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    
    // Waypoint 1: Go back to (1.0, 0.0) - last original waypoint
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    home_waypoints_.push_back(pose);
    
    // Waypoint 2: (1.2, 1.2)
    pose.pose.position.x = 1.2;
    pose.pose.position.y = 1.2;
    home_waypoints_.push_back(pose);
    
    // Waypoint 3: (0.5, 0.5) - intermediate point
    pose.pose.position.x = 0.5;
    pose.pose.position.y = 0.5;
    home_waypoints_.push_back(pose);
    
    // Waypoint 4: Home position (0.0, 0.0)
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    home_waypoints_.push_back(pose);
    
    RCLCPP_INFO(node_->get_logger(), "Initialized %zu home waypoints for return navigation", 
                home_waypoints_.size());
  }

  NodeStatus NavToHome::onStart()
  {
    // Set stamp for all home waypoints
    for (auto &pose : home_waypoints_)
    {
      pose.header.stamp = node_->now();
    }
    navigation_goal_.poses = home_waypoints_;

    auto future_goal_handle = action_client_->async_send_goal(navigation_goal_);
    RCLCPP_DEBUG(node_->get_logger(), "send goal timeout ms: %d", send_goal_timeout_);
    
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send goal failed");
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "goal handle is null");
      return NodeStatus::FAILURE;
    }

    RCLCPP_INFO(node_->get_logger(), "Returning home through %zu waypoints...", home_waypoints_.size());
    is_waiting_ = false;
    return NodeStatus::RUNNING;
  }

  NodeStatus NavToHome::onRunning()
  {
    // If we are in waiting phase
    if (is_waiting_)
    {
      // Check if wait time has elapsed
      auto now = node_->now();
      auto elapsed = (now - wait_start_time_).seconds();
      
      if (elapsed >= wait_time_at_home_)
      {
        RCLCPP_INFO(node_->get_logger(), "Waited %.1f seconds at home, HP should be recovered!", elapsed);
        is_waiting_ = false;
        return NodeStatus::SUCCESS;
      }
      
      RCLCPP_DEBUG(node_->get_logger(), "Waiting at home: %.1f / %d seconds", elapsed, wait_time_at_home_);
      return NodeStatus::RUNNING;
    }
    
    // Not waiting yet, check navigation status
    if (!goal_handle_)
    {
      return NodeStatus::FAILURE;
    }

    switch (goal_handle_->get_status())
    {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      RCLCPP_DEBUG(node_->get_logger(), "goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      RCLCPP_DEBUG(node_->get_logger(), "goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      RCLCPP_DEBUG(node_->get_logger(), "goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      RCLCPP_DEBUG(node_->get_logger(), "goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      {
        RCLCPP_INFO(node_->get_logger(), "Home navigation succeeded! Now waiting for HP recovery...");
        // Start waiting phase
        is_waiting_ = true;
        wait_start_time_ = node_->now();
        RCLCPP_INFO(node_->get_logger(), "Waiting %d seconds at home for HP recovery...", wait_time_at_home_);
        return NodeStatus::RUNNING;
      }
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELED");
      return NodeStatus::FAILURE;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "goal status: STATUS_ABORTED");
      return NodeStatus::FAILURE;
    default:
      RCLCPP_DEBUG(node_->get_logger(), "goal status: ERROR CODE");
      break;
    }

    return NodeStatus::RUNNING;
  }

  void NavToHome::onHalted()
  {
    RCLCPP_INFO(node_->get_logger(), "NavToHome goal halted");
    is_waiting_ = false;
    if (goal_handle_)
    {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "cancel goal failed");
      }
      RCLCPP_INFO(node_->get_logger(), "goal canceled");
    }
  }

  PortsList NavToHome::providedPorts()
  {
    // No input ports needed - uses fixed home waypoints and wait time
    return { };
  }

} // end namespace rm_decision
