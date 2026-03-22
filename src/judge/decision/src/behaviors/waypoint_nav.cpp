#include "rm_decision/behaviors/waypoint_nav.hpp"

using namespace BT;

namespace rm_decision
{
  WaypointNav::WaypointNav(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node), current_waypoint_index_(0)
  {
    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, "navigate_through_poses");
    node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 1000);
    
    // Initialize waypoints from judge.py
    init_waypoints();
  }

  void WaypointNav::load_start_index()
  {
    // Try to get starting waypoint index from blackboard
    // This allows resuming from where we left off after returning home
    auto blackboard = config().blackboard;
    
    uint16_t start_index = 0;
    if (blackboard->get("waypoint_start_index", start_index))
    {
      current_waypoint_index_ = static_cast<size_t>(start_index);
      RCLCPP_INFO(node_->get_logger(), "Resuming waypoint navigation from index %zu", current_waypoint_index_);
    }
    else
    {
      current_waypoint_index_ = 0;
      RCLCPP_INFO(node_->get_logger(), "Starting waypoint navigation from beginning (index 0)");
    }
    
    // Clear the stored index after loading (so next time we start fresh if no new index is set)
    blackboard->set("waypoint_start_index", uint16_t(0));
  }

  void WaypointNav::init_waypoints()
  {
    // Based on judge.py waypoints:
    // (1.2, 1.2) -> (4.5, 1.2) -> (4.5, 0.1) -> (1.0, 0.0)
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    
    // Waypoint 1: (1.2, 1.2, 0.0)
    pose.pose.position.x = 1.2;
    pose.pose.position.y = 1.2;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    waypoints_.push_back(pose);
    
    // Waypoint 2: (4.5, 1.2, 0.0)
    pose.pose.position.x = 4.5;
    pose.pose.position.y = 1.2;
    waypoints_.push_back(pose);
    
    // Waypoint 3: (4.5, 0.1, 0.0)
    pose.pose.position.x = 4.5;
    pose.pose.position.y = 0.1;
    waypoints_.push_back(pose);
    
    // Waypoint 4: (1.0, 0.0, 0.0)
    pose.pose.position.x = 1.0;
    pose.pose.position.y = 0.0;
    waypoints_.push_back(pose);
    
    RCLCPP_INFO(node_->get_logger(), "Initialized %zu waypoints for navigation", waypoints_.size());
  }

  NodeStatus WaypointNav::onStart()
  {
    // Load starting waypoint index from blackboard (for resume functionality)
    load_start_index();
    
    // Set stamp for all waypoints from current index onwards
    for (size_t i = current_waypoint_index_; i < waypoints_.size(); i++)
    {
      waypoints_[i].header.stamp = node_->now();
    }
    
    // Create navigation goal from remaining waypoints
    std::vector<geometry_msgs::msg::PoseStamped> remaining_waypoints(
      waypoints_.begin() + current_waypoint_index_,
      waypoints_.end()
    );
    navigation_goal_.poses = remaining_waypoints;
    
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

    RCLCPP_INFO(node_->get_logger(), "Navigating through %zu waypoints (starting from index %zu)", 
                remaining_waypoints.size(), current_waypoint_index_);
    return NodeStatus::RUNNING;
  }

  NodeStatus WaypointNav::onRunning()
  {
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
        RCLCPP_INFO(node_->get_logger(), "All waypoints navigation succeeded!");
        // Reset waypoint index on success
        auto blackboard = config().blackboard;
        blackboard->set("waypoint_start_index", uint16_t(0));
        return NodeStatus::SUCCESS;
      }
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(node_->get_logger(), "goal status: STATUS_CANCELED");
      // Save current waypoint index for resume
      {
        auto blackboard = config().blackboard;
        blackboard->set("waypoint_start_index", static_cast<uint16_t>(current_waypoint_index_));
        RCLCPP_INFO(node_->get_logger(), "Saved waypoint index %zu for resume", current_waypoint_index_);
      }
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

  void WaypointNav::onHalted()
  {
    RCLCPP_INFO(node_->get_logger(), "WaypointNav goal halted");
    // Save current waypoint index when halted (e.g., by HP check)
    auto blackboard = config().blackboard;
    blackboard->set("waypoint_start_index", static_cast<uint16_t>(current_waypoint_index_));
    RCLCPP_INFO(node_->get_logger(), "Saved waypoint index %zu for resume after HP recovery", current_waypoint_index_);
    
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

  PortsList WaypointNav::providedPorts()
  {
    // Support input port for starting waypoint index (optional)
    return {
      InputPort<uint16_t>("start_index")
    };
  }

} // end namespace rm_decision
