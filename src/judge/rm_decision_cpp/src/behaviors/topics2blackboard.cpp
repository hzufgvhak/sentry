#include "rm_decision_cpp/behaviors/topics2blackboard.hpp"

using namespace BT;
namespace rm_decision
{
  Topics2Blackboard::Topics2Blackboard(const std::string &name, const NodeConfig &config, std::shared_ptr<rclcpp::Node> node, std::shared_ptr<tf2_ros::Buffer> tf_buffer, std::shared_ptr<tf2_ros::TransformListener> tf_listener) 
  : SyncActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
  {
    bool use_costmap;
    node_->get_parameter_or<bool>("use_costmap", use_costmap, true);
    std::string global_frame;
    node_->get_parameter("global_frame", global_frame);
    if(use_costmap)
      to_frame_="map";
    else
      to_frame_=global_frame;
    
    tracking_timeout_s_ = node_->get_parameter("tracking_timeout_s").as_double();
    
    // Get team parameter (blue or red)
    node_->get_parameter_or<std::string>("team", team_, "blue");

    // msg from referee
    std::string game_state_topic_name;
    node_->get_parameter("game_state_topic_name",game_state_topic_name);
    game_state_sub_ = node_->create_subscription<rm_interfaces::msg::GameState>(
        game_state_topic_name, 10,
        std::bind(&Topics2Blackboard::game_state_callback_, this, std::placeholders::_1));

    // msg from autoaim
    std::string target_topic_name;
    node_->get_parameter("target_topic_name",target_topic_name);
    target_sub_ = node_->create_subscription<auto_aim_interfaces::msg::Target>(
      target_topic_name, rclcpp::SensorDataQoS(),
      std::bind(&Topics2Blackboard::target_callback_, this, std::placeholders::_1));
    
    // msg from odometry (robot position)
    std::string odom_topic_name;
    node_->get_parameter_or<std::string>("odom_topic_name", odom_topic_name, "odom");
    odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_name, 10,
      std::bind(&Topics2Blackboard::odom_callback_, this, std::placeholders::_1));
  }

  NodeStatus Topics2Blackboard::tick()
  {
    check_subscriber_();
    
    // Calculate and set zone based on current position
    if (odom_) {
      double robot_x = odom_->pose.pose.position.x;
      double robot_y = odom_->pose.pose.position.y;
      
      bool is_blue_team = (team_ == "blue");
      RobotZone zone = calculate_zone_(robot_x, robot_y, is_blue_team);
      setOutput<uint8_t>("robot_zone", static_cast<uint8_t>(zone));
      
      // Also set robot position directly
      geometry_msgs::msg::PoseStamped robot_pose;
      robot_pose.header.stamp = node_->now();
      robot_pose.header.frame_id = "map";
      robot_pose.pose = odom_->pose.pose;
      setOutput<geometry_msgs::msg::PoseStamped>("robot_position", robot_pose);
    }
    
    return NodeStatus::SUCCESS;
  }

  PortsList Topics2Blackboard::providedPorts()
  {
    return {
        OutputPort<std::uint16_t>("current_hp"),
        OutputPort<std::uint8_t>("game_progress"),
        OutputPort<std::uint16_t>("stage_remain_time"),
        OutputPort<std::uint8_t>("armor_id"),
        OutputPort<std::uint8_t>("hurt_type"),
        OutputPort<std::uint16_t>("my_outpost_hp"),
        OutputPort<std::uint16_t>("enemy_outpost_hp"),
        OutputPort<std::uint16_t>("my_base_hp"),
        OutputPort<std::uint16_t>("enemy_base_hp"),
        OutputPort<std::uint16_t>("projectile_allowance_17mm"),
        OutputPort<bool>("tracking"),
        OutputPort<std::string>("target_armor_id"),
        OutputPort<geometry_msgs::msg::PoseStamped>("target_position"),
        // New outputs for 25 season
        OutputPort<std::uint8_t>("robot_zone"),
        OutputPort<geometry_msgs::msg::PoseStamped>("robot_position"),
    };
  }

  void Topics2Blackboard::game_state_callback_(const rm_interfaces::msg::GameState::SharedPtr msg)
  {
    game_state_ = *msg;
    setOutput<std::uint16_t>("current_hp", game_state_->current_hp);
    setOutput<std::uint8_t>("game_progress", game_state_->game_progress);
    setOutput<std::uint16_t>("stage_remain_time", game_state_->stage_remain_time);
    setOutput<std::uint8_t>("armor_id", game_state_->armor_id);
    setOutput<std::uint8_t>("hurt_type", game_state_->hurt_type);
    setOutput<std::uint16_t>("my_outpost_hp", game_state_->my_outpost_hp);
    setOutput<std::uint16_t>("enemy_outpost_hp", game_state_->enemy_outpost_hp);
    setOutput<std::uint16_t>("my_base_hp", game_state_->my_base_hp);
    setOutput<std::uint16_t>("enemy_base_hp", game_state_->enemy_base_hp);
    setOutput<std::uint16_t>("projectile_allowance_17mm", game_state_->projectile_allowance_17mm);
  }

  void Topics2Blackboard::target_callback_(const auto_aim_interfaces::msg::Target::SharedPtr msg)
  {
    target_ = *msg;
    target_armor_id_ = target_->id;
    setOutput<bool>("tracking", target_->tracking);
    // if tracking, calculate target pose at map directly
    if (target_->tracking)
    {
      geometry_msgs::msg::TransformStamped t;
      try
      {
        t = tf_buffer_->lookupTransform(to_frame_, target_->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(target_->position, target_pose_.pose.position, t);
      }
      catch (const tf2::TransformException &ex)
      {
        RCLCPP_WARN(
            rclcpp::get_logger("Topics2Blackboard"), "Could not transform %s to %s: %s",
            to_frame_.c_str(), target_->header.frame_id.c_str(), ex.what());
        setOutput<bool>("tracking", false);
        setOutput<std::string>("target_armor_id", "None");
        return;
      }
      target_pose_.header.frame_id = to_frame_;
      target_pose_.header.stamp = node_->now();
      setOutput<geometry_msgs::msg::PoseStamped>("target_position", target_pose_);
      setOutput<std::string>("target_armor_id", target_armor_id_);
      last_tracking_time_ = node_->now();
    }else{
      // check if tracking timeout
      if(node_->now().seconds() - last_tracking_time_.seconds() > tracking_timeout_s_)
      {
        setOutput<bool>("tracking", false);
        setOutput<std::string>("target_armor_id", "None");
      }
      else
      {
        setOutput<bool>("tracking", true);
        setOutput<geometry_msgs::msg::PoseStamped>("target_position", target_pose_);
        setOutput<std::string>("target_armor_id", target_armor_id_);
      }
    }

  }

  void Topics2Blackboard::odom_callback_(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = *msg;
  }

  void Topics2Blackboard::check_subscriber_()
  {
    if(!game_state_)
    {
      RCLCPP_WARN(rclcpp::get_logger("Topics2Blackboard"), "game_state is null");
      setOutput<std::uint16_t>("current_hp", 0);
      setOutput<std::uint8_t>("game_progress", 0);
      setOutput<std::uint16_t>("stage_remain_time", 0);
      setOutput<std::uint8_t>("armor_id", 0);
      setOutput<std::uint8_t>("hurt_type", 0);
      setOutput<std::uint16_t>("my_outpost_hp", 0);
      setOutput<std::uint16_t>("enemy_outpost_hp", 0);
      setOutput<std::uint16_t>("my_base_hp", 0);
      setOutput<std::uint16_t>("enemy_base_hp", 0);
      setOutput<std::uint16_t>("projectile_allowance_17mm", 0);
    }
    if (!target_)
    {
      RCLCPP_WARN(rclcpp::get_logger("Topics2Blackboard"), "target is null");
      setOutput<bool>("tracking", false);
      setOutput<std::string>("target_armor_id", "None");
    }
    if (!odom_)
    {
      RCLCPP_WARN(rclcpp::get_logger("Topics2Blackboard"), "odom is null");
      setOutput<std::uint8_t>("robot_zone", static_cast<uint8_t>(RobotZone::DEFENSE));
    }
  }

  RobotZone Topics2Blackboard::calculate_zone_(double x, double y, bool is_blue_team)
  {
    // Blue team: positive X is towards enemy base
    // Red team: negative X is towards enemy base
    if (is_blue_team) {
      if (x > blue_attack_threshold_x_) {
        return RobotZone::ATTACK;
      } else if (x > blue_mid_threshold_x_) {
        return RobotZone::MID;
      } else {
        return RobotZone::DEFENSE;
      }
    } else {
      // Red team
      if (x < red_attack_threshold_x_) {
        return RobotZone::ATTACK;
      } else if (x < red_mid_threshold_x_) {
        return RobotZone::MID;
      } else {
        return RobotZone::DEFENSE;
      }
    }
  }
} // end namespace rm_decision