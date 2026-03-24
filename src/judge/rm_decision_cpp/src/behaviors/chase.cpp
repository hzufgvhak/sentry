#include "rm_decision_cpp/behaviors/chase.hpp"
#include <cmath>

using namespace BT;
namespace rm_decision
{
  Chase::Chase(const std::string &name, const NodeConfig &config,
               std::shared_ptr<rclcpp::Node> node,
               std::shared_ptr<tf2_ros::Buffer> tf_buffer,
               std::shared_ptr<tf2_ros::TransformListener> tf_listener)
      : SyncActionNode(name, config), node_(node), tf_buffer_(tf_buffer), tf_listener_(tf_listener)
  {
    // Get parameters
    node_->get_parameter_or<double>("max_chase_distance", max_chase_distance_, 3.0);
    node_->get_parameter_or<double>("mid_chase_distance", mid_chase_distance_, 6.0);
    node_->get_parameter_or<double>("attack_chase_distance", attack_chase_distance_, 10.0);
    node_->get_parameter_or<std::string>("global_frame", global_frame_, "map");
    node_->get_parameter_or<std::string>("base_frame", base_frame_, "base_link");
  }

  NodeStatus Chase::tick()
  {
    // Get inputs from blackboard
    auto target_position = getInput<geometry_msgs::msg::PoseStamped>("target_position");
    auto robot_position = getInput<geometry_msgs::msg::PoseStamped>("robot_position");
    auto robot_zone = getInput<uint8_t>("robot_zone");
    auto tracking = getInput<bool>("tracking");

    if (!target_position || !robot_position || !robot_zone || !tracking)
    {
      RCLCPP_WARN(node_->get_logger(), "Chase: Missing required inputs");
      return NodeStatus::FAILURE;
    }

    target_pose_ = target_position->pose;
    robot_pose_ = robot_position->pose;
    robot_zone_ = *robot_zone;
    tracking_ = *tracking;

    // If not tracking, don't chase
    if (!tracking_)
    {
      return NodeStatus::FAILURE;
    }

    // Check if should chase based on zone
    if (should_chase_())
    {
      // Set chase output - the Attack node will handle navigation to target
      setOutput<bool>("chase_active", true);
      return NodeStatus::SUCCESS;
    }
    else
    {
      // In defense zone or target too far, don't chase
      setOutput<bool>("chase_active", false);
      return NodeStatus::FAILURE;
    }
  }

  PortsList Chase::providedPorts()
  {
    return {
        InputPort<geometry_msgs::msg::PoseStamped>("target_position"),
        InputPort<geometry_msgs::msg::PoseStamped>("robot_position"),
        InputPort<uint8_t>("robot_zone"),
        InputPort<bool>("tracking"),
        OutputPort<bool>("chase_active")};
  }

  void Chase::get_robot_pose_()
  {
    try
    {
      auto t = tf_buffer_->lookupTransform(global_frame_, base_frame_, tf2::TimePointZero);
      robot_pose_.position.x = t.transform.translation.x;
      robot_pose_.position.y = t.transform.translation.y;
      robot_pose_.position.z = t.transform.translation.z;
      robot_pose_.orientation = t.transform.rotation;
    }
    catch (const tf2::TransformException &ex)
    {
      RCLCPP_WARN(node_->get_logger(), "Could not get robot pose: %s", ex.what());
    }
  }

  bool Chase::should_chase_() const
  {
    // Calculate distance to target
    double dx = target_pose_.position.x - robot_pose_.position.x;
    double dy = target_pose_.position.y - robot_pose_.position.y;
    double distance = std::sqrt(dx * dx + dy * dy);

    // Zone-based chase logic
    // Zone 0 (Defense): Limited chase - only if target is very close
    if (robot_zone_ == 0)
    {
      return distance < max_chase_distance_;
    }
    // Zone 1 (Mid): Moderate chase
    else if (robot_zone_ == 1)
    {
      return distance < mid_chase_distance_;
    }
    // Zone 2 (Attack): Aggressive chase
    else
    {
      return distance < attack_chase_distance_;
    }
  }
} // namespace rm_decision
