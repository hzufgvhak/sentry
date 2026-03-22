#include "rm_decision/behaviors/patrol_nav.hpp"

using namespace BT;

namespace rm_decision
{
  PatrolNav::PatrolNav(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node) 
  : StatefulActionNode(name, config), node_(node), current_patrol_round_(0), hp_drop_detected_(false),
    current_tracking_(false), is_checking_tracking_(false)
  {
    action_client_ = rclcpp_action::create_client<NavigateThroughPoses>(node_, "navigate_through_poses");
    node_->get_parameter_or("send_goal_timeout_ms", send_goal_timeout_, 1000);
    node_->get_parameter_or("max_patrol_rounds", max_patrol_rounds_, 0);  // 0 = infinite
    
    // HP monitoring parameters
    node_->get_parameter_or("hp_drop_threshold", hp_drop_threshold_, 50);   // HP drop amount to trigger return
    node_->get_parameter_or("hp_drop_time_window", hp_drop_time_window_, 3);  // Time window in seconds
    
    // Tracking detection parameters
    node_->get_parameter_or("tracking_check_duration", tracking_check_duration_, 3);  // seconds to check tracking
    node_->get_parameter_or("tracking_threshold", tracking_threshold_, 3);  // number of tracking true needed
    
    // Subscribe to target topic for tracking detection
    target_subscription_ = node_->create_subscription<auto_aim_interfaces::msg::Target>(
      "/target",
      10,
      std::bind(&PatrolNav::targetCallback, this, std::placeholders::_1)
    );
    RCLCPP_INFO(node_->get_logger(), "Subscribed to /target topic for tracking detection");
    
    // Initialize patrol waypoints
    init_patrol_waypoints();
  }

  void PatrolNav::targetCallback(const auto_aim_interfaces::msg::Target::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    current_tracking_ = msg->tracking;
  }

  bool PatrolNav::isTrackingDetected()
  {
    std::lock_guard<std::mutex> lock(tracking_mutex_);
    return current_tracking_;
  }

  void PatrolNav::init_patrol_waypoints()
  {
    // Define patrol waypoints - these are the points to cycle through
    // User can customize these coordinates
    
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    
    // Patrol point 1: (1.5, 1.5)
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 1.5;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    patrol_waypoints_.push_back(pose);
    
    // Patrol point 2: (3.0, 1.5)
    pose.pose.position.x = 3.0;
    pose.pose.position.y = 1.5;
    patrol_waypoints_.push_back(pose);
    
    // Patrol point 3: (3.0, 0.5)
    pose.pose.position.x = 3.0;
    pose.pose.position.y = 0.5;
    patrol_waypoints_.push_back(pose);
    
    // Patrol point 4: (1.5, 0.5)
    pose.pose.position.x = 1.5;
    pose.pose.position.y = 0.5;
    patrol_waypoints_.push_back(pose);
    
    // Save first waypoint as patrol start position
    patrol_start_pose_ = patrol_waypoints_[0];
    
    RCLCPP_INFO(node_->get_logger(), "Initialized %zu patrol waypoints for loop navigation", patrol_waypoints_.size());
  }

  bool PatrolNav::shouldReturnToStart()
  {
    // Get current HP from blackboard
    auto blackboard = config().blackboard;
    uint16_t current_hp = 0;
    blackboard->get("robot_hp", current_hp);
    
    int hp_drop = start_hp_ - current_hp;
    
    // Check if HP dropped significantly within time window
    if (hp_drop >= hp_drop_threshold_)
    {
      if (!hp_drop_detected_)
      {
        // First time detecting HP drop
        hp_drop_detected_ = true;
        hp_drop_time_ = node_->now();
        RCLCPP_WARN(node_->get_logger(), "HP drop detected: %d (start: %d, current: %d)", 
                    hp_drop, start_hp_, current_hp);
      }
      
      // Check if HP drop occurred within the time window
      double elapsed = (node_->now() - hp_drop_time_).seconds();
      if (elapsed <= hp_drop_time_window_)
      {
        RCLCPP_WARN(node_->get_logger(), "Significant HP drop within %d seconds! Returning to patrol start...", 
                    hp_drop_time_window_);
        return true;
      }
      else
      {
        // Reset detection if time window passed
        hp_drop_detected_ = false;
      }
    }
    else
    {
      // Reset detection if HP recovered or drop is small
      hp_drop_detected_ = false;
    }
    
    return false;
  }

  NodeStatus PatrolNav::onStart()
  {
    current_patrol_round_ = 0;
    hp_drop_detected_ = false;
    
    // Get starting HP from blackboard
    auto blackboard = config().blackboard;
    blackboard->get("robot_hp", start_hp_);
    RCLCPP_INFO(node_->get_logger(), "Patrol started with HP: %d", start_hp_);
    for (auto &pose : patrol_waypoints_)
    {
      pose.header.stamp = node_->now();
    }
    navigation_goal_.poses = patrol_waypoints_;
    
    auto future_goal_handle = action_client_->async_send_goal(navigation_goal_);
    RCLCPP_DEBUG(node_->get_logger(), "send goal timeout ms: %d", send_goal_timeout_);
    
    if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(node_->get_logger(), "send patrol goal failed");
      return NodeStatus::FAILURE;
    }

    goal_handle_ = future_goal_handle.get();
    if (!goal_handle_)
    {
      RCLCPP_ERROR(node_->get_logger(), "patrol goal handle is null");
      return NodeStatus::FAILURE;
    }

    current_patrol_round_++;
    RCLCPP_INFO(node_->get_logger(), "Starting patrol round %d, navigating through %zu patrol points", 
                current_patrol_round_, patrol_waypoints_.size());
    return NodeStatus::RUNNING;
  }

  NodeStatus PatrolNav::onRunning()
  {
    // ===== 检查tracking状态 =====
    // 如果不是在检查tracking的状态，则检测tracking
    if (!is_checking_tracking_)
    {
      if (isTrackingDetected())
      {
        // 检测到tracking，停止移动并开始检查
        RCLCPP_WARN(node_->get_logger(), "Tracking detected! Stopping to check tracking status...");
        
        // 停止当前巡逻
        if (goal_handle_)
        {
          auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
          rclcpp::spin_until_future_complete(node_, cancel_future);
        }
        
        // 开始检查tracking
        is_checking_tracking_ = true;
        tracking_check_start_ = node_->now();
        tracking_check_count_ = 0;
        RCLCPP_INFO(node_->get_logger(), "Starting tracking check for %d seconds...", tracking_check_duration_);
      }
    }
    
    // 如果正在检查tracking
    if (is_checking_tracking_)
    {
      auto now = node_->now();
      double check_elapsed = (now - tracking_check_start_).seconds();
      
      // 统计tracking次数
      if (isTrackingDetected())
      {
        tracking_check_count_++;
        RCLCPP_DEBUG(node_->get_logger(), "Tracking detected! Count: %d/%d at %.1fs", 
                    tracking_check_count_, tracking_threshold_, check_elapsed);
        
        // 短暂延迟避免重复计数
        rclcpp::sleep_for(std::chrono::milliseconds(200));
      }
      
      // 检查时间到了
      if (check_elapsed >= tracking_check_duration_)
      {
        RCLCPP_INFO(node_->get_logger(), "Tracking check finished: %d times in %.1fs", 
                    tracking_check_count_, check_elapsed);
        
        if (tracking_check_count_ >= tracking_threshold_)
        {
          // Tracking次数达到阈值，认为正在攻击，返回SUCCESS结束巡逻
          RCLCPP_WARN(node_->get_logger(), "Tracking count (%d) >= threshold (%d)! Enemy detected, stopping patrol",
                      tracking_check_count_, tracking_threshold_);
          is_checking_tracking_ = false;
          return NodeStatus::SUCCESS;
        }
        else
        {
          // Tracking次数未达到阈值，继续之前的巡逻
          RCLCPP_INFO(node_->get_logger(), "Tracking count (%d) < threshold (%d), resuming patrol",
                      tracking_check_count_, tracking_threshold_);
          is_checking_tracking_ = false;
          // 继续巡逻逻辑...
        }
      }
      else
      {
        // 继续等待检查tracking，保持当前状态
        rclcpp::sleep_for(std::chrono::milliseconds(100));
        return NodeStatus::RUNNING;
      }
    }
    
    // ===== 检查HP是否需要返回 =====
    // Monitor HP and game time during patrol
    if (shouldReturnToStart())
    {
      RCLCPP_WARN(node_->get_logger(), "HP drop detected! Cancelling patrol and returning to start position...");
      
      // Cancel current patrol goal
      if (goal_handle_)
      {
        auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
        rclcpp::spin_until_future_complete(node_, cancel_future);
      }
      
      // Navigate back to patrol start position
      nav2_msgs::action::NavigateThroughPoses::Goal return_goal;
      patrol_start_pose_.header.stamp = node_->now();
      return_goal.poses.push_back(patrol_start_pose_);
      
      auto future_goal_handle = action_client_->async_send_goal(return_goal);
      if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "Failed to return to patrol start");
        return NodeStatus::FAILURE;
      }
      
      goal_handle_ = future_goal_handle.get();
      RCLCPP_INFO(node_->get_logger(), "Returning to patrol start position...");
      
      // Wait for return navigation to complete
      while (rclcpp::ok() && goal_handle_)
      {
        auto status = goal_handle_->get_status();
        if (status == action_msgs::msg::GoalStatus::STATUS_SUCCEEDED)
        {
          RCLCPP_INFO(node_->get_logger(), "Returned to patrol start position!");
          break;
        }
        else if (status == action_msgs::msg::GoalStatus::STATUS_ABORTED || 
                 status == action_msgs::msg::GoalStatus::STATUS_CANCELED)
        {
          RCLCPP_ERROR(node_->get_logger(), "Failed to return to patrol start");
          return NodeStatus::FAILURE;
        }
        rclcpp::sleep_for(std::chrono::milliseconds(100));
      }
      
      // ===== 在起始点等待并检查HP =====
      RCLCPP_INFO(node_->get_logger(), "Waiting at patrol start position to check HP stability...");
      rclcpp::Time wait_start_time = node_->now();
      int wait_time_at_patrol = 5;  // 巡逻起始点等待时间（秒）
      int stable_time_required = 3; // HP需要稳定的时间（秒）
      int safe_hp_threshold = 100;   // 安全HP阈值
      
      // 记录上次HP检测值和时间
      uint16_t last_hp = 0;
      rclcpp::Time last_hp_change_time = node_->now();
      
      while (rclcpp::ok())
      {
        auto now = node_->now();
        double elapsed = (now - wait_start_time).seconds();
        
        // 获取当前HP
        auto blackboard = config().blackboard;
        uint16_t current_hp = 0;
        blackboard->get("robot_hp", current_hp);
        
        // 检查HP是否发生变化
        if (current_hp != last_hp && last_hp != 0)
        {
          // HP变化了，记录变化时间
          last_hp_change_time = now;
          RCLCPP_DEBUG(node_->get_logger(), "HP changed from %d to %d, resetting stable timer", last_hp, current_hp);
        }
        last_hp = current_hp;
        
        // 计算HP稳定持续时间
        double hp_stable_time = (now - last_hp_change_time).seconds();
        
        // 判断是否可以继续：HP稳定一段时间 + 在安全范围内
        bool hp_stable = hp_stable_time >= stable_time_required;
        bool hp_safe = current_hp >= safe_hp_threshold;
        
        RCLCPP_DEBUG(node_->get_logger(), "At patrol start: HP=%d, stable=%.1f/%ds, safe=%d, waited=%.1f/%ds", 
                    current_hp, hp_stable_time, stable_time_required, hp_safe, elapsed, wait_time_at_patrol);
        
        // 如果HP稳定且安全，立即继续巡逻
        if (hp_stable && hp_safe)
        {
          RCLCPP_INFO(node_->get_logger(), "HP stable for %.1fs and safe (HP=%d >= %d), resuming patrol!", 
                      hp_stable_time, current_hp, safe_hp_threshold);
          break;
        }
        
        // 等待时间到了，无论HP状态如何都继续
        if (elapsed >= wait_time_at_patrol)
        {
          if (!hp_safe)
          {
            RCLCPP_WARN(node_->get_logger(), "Wait time finished but HP not safe (HP=%d < %d), will retry HP check in next cycle", 
                        current_hp, safe_hp_threshold);
            // HP不安全，返回FAILURE让上层重新检测
            return NodeStatus::FAILURE;
          }
          RCLCPP_INFO(node_->get_logger(), "Wait time finished at patrol start, HP=%d", current_hp);
          break;
        }
        
        rclcpp::sleep_for(std::chrono::milliseconds(500));
      }
      
      RCLCPP_INFO(node_->get_logger(), "Continuing patrol from start position");
      return NodeStatus::RUNNING;
    }
    
    if (!goal_handle_)
    {
      return NodeStatus::FAILURE;
    }

    switch (goal_handle_->get_status())
    {
    case action_msgs::msg::GoalStatus::STATUS_UNKNOWN:
      RCLCPP_DEBUG(node_->get_logger(), "patrol goal status: STATUS_UNKNOWN");
      break;
    case action_msgs::msg::GoalStatus::STATUS_ACCEPTED:
      RCLCPP_DEBUG(node_->get_logger(), "patrol goal status: STATUS_ACCEPTED");
      break;
    case action_msgs::msg::GoalStatus::STATUS_EXECUTING:
      RCLCPP_DEBUG(node_->get_logger(), "patrol goal status: STATUS_EXECUTING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_CANCELING:
      RCLCPP_DEBUG(node_->get_logger(), "patrol goal status: STATUS_CANCELING");
      break;
    case action_msgs::msg::GoalStatus::STATUS_SUCCEEDED:
      {
        // One patrol round completed
        RCLCPP_INFO(node_->get_logger(), "Patrol round %d completed!", current_patrol_round_);
        
        // Check if we should continue patrolling
        if (max_patrol_rounds_ > 0 && current_patrol_round_ >= max_patrol_rounds_)
        {
          RCLCPP_INFO(node_->get_logger(), "Max patrol rounds (%d) reached, stopping patrol", max_patrol_rounds_);
          return NodeStatus::SUCCESS;
        }
        
        // Continue to next patrol round
        current_patrol_round_++;
        
        // Send goal again for next round
        for (auto &pose : patrol_waypoints_)
        {
          pose.header.stamp = node_->now();
        }
        navigation_goal_.poses = patrol_waypoints_;
        
        auto future_goal_handle = action_client_->async_send_goal(navigation_goal_);
        
        if (rclcpp::spin_until_future_complete(node_, future_goal_handle, std::chrono::milliseconds(send_goal_timeout_)) != rclcpp::FutureReturnCode::SUCCESS)
        {
          RCLCPP_ERROR(node_->get_logger(), "send patrol goal failed for round %d", current_patrol_round_);
          return NodeStatus::FAILURE;
        }

        goal_handle_ = future_goal_handle.get();
        if (!goal_handle_)
        {
          RCLCPP_ERROR(node_->get_logger(), "patrol goal handle is null for round %d", current_patrol_round_);
          return NodeStatus::FAILURE;
        }
        
        RCLCPP_INFO(node_->get_logger(), "Starting patrol round %d", current_patrol_round_);
        return NodeStatus::RUNNING;
      }
    case action_msgs::msg::GoalStatus::STATUS_CANCELED:
      RCLCPP_INFO(node_->get_logger(), "patrol goal status: STATUS_CANCELED");
      return NodeStatus::FAILURE;
    case action_msgs::msg::GoalStatus::STATUS_ABORTED:
      RCLCPP_ERROR(node_->get_logger(), "patrol goal status: STATUS_ABORTED");
      return NodeStatus::FAILURE;
    default:
      RCLCPP_DEBUG(node_->get_logger(), "patrol goal status: ERROR CODE");
      break;
    }

    return NodeStatus::RUNNING;
  }

  void PatrolNav::onHalted()
  {
    RCLCPP_INFO(node_->get_logger(), "PatrolNav goal halted at round %d", current_patrol_round_);
    hp_drop_detected_ = false;
    is_checking_tracking_ = false;
    if (goal_handle_)
    {
      auto cancel_future = action_client_->async_cancel_goal(goal_handle_);
      if (rclcpp::spin_until_future_complete(node_, cancel_future) != rclcpp::FutureReturnCode::SUCCESS)
      {
        RCLCPP_ERROR(node_->get_logger(), "cancel patrol goal failed");
      }
      RCLCPP_INFO(node_->get_logger(), "patrol goal canceled");
    }
  }

  PortsList PatrolNav::providedPorts()
  {
    return {
      InputPort<int>("max_rounds"),  // Maximum patrol rounds (0 = infinite)
      InputPort<int>("hp_drop_threshold"),   // HP drop amount to trigger return
      InputPort<int>("hp_drop_time_window")  // Time window to detect HP drop
    };
  }

} // end namespace
