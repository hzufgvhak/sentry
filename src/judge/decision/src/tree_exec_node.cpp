#include "rm_decision/behaviors/serial_packet_subscriber.hpp"
#include "rm_decision/behaviors/game_time_check.hpp"
#include "rm_decision/behaviors/waypoint_nav.hpp"
#include "rm_decision/behaviors/nav_to_home.hpp"
#include "rm_decision/behaviors/hp_check.hpp"
#include "rm_decision/behaviors/patrol_nav.hpp"
#include "rm_decision/behaviors/pose_state_publisher.hpp"
#include "rm_decision/behaviors/target_subscriber.hpp"
#include "rm_decision/behaviors/tracking_check.hpp"
#include "rm_decision/behaviors/wait_for_duration.hpp"
#include <rclcpp/rclcpp.hpp>
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/loggers/groot2_publisher.h"
#include <chrono>
#include "rm_decision/custume_types.hpp"
#include "behaviortree_cpp/xml_parsing.h"
#include "behaviortree_cpp/json_export.h"
#include <fstream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("rm_decision_tree_exec");
  
  // Declare parameters
  node->declare_parameter<std::string>("tree_xml_file", "");
  node->declare_parameter("tick_period_milliseconds", 100);
  node->declare_parameter("groot_port", 5556);
  node->declare_parameter("tree_node_model_export_path", "");
  node->declare_parameter("target_game_time", 1);
  node->declare_parameter("send_goal_timeout_ms", 1000);
  node->declare_parameter("hp_low_threshold", 100);  // HP threshold for returning home
  node->declare_parameter("wait_time_at_home", 10);   // Wait time at home for HP recovery
  node->declare_parameter("max_patrol_rounds", 0);    // 0 = infinite patrol
  
  // Get parameters
  std::string tree_xml_file;
  node->get_parameter("tree_xml_file", tree_xml_file);
  
  int tick_period_milliseconds;
  node->get_parameter("tick_period_milliseconds", tick_period_milliseconds);
  
  std::chrono::system_clock::duration timeout;
  timeout = std::chrono::milliseconds(tick_period_milliseconds);
  
  unsigned int groot_port = 5556;
  node->get_parameter("groot_port", groot_port);
  
  std::string tree_node_model_export_path;
  node->get_parameter("tree_node_model_export_path", tree_node_model_export_path);

  BT::BehaviorTreeFactory factory;

  // Register the custom nodes
  factory.registerNodeType<rm_decision::SerialPacketSubscriber>("SerialPacketSubscriber", node);
  factory.registerNodeType<rm_decision::GameTimeCheck>("GameTimeCheck", node);
  factory.registerNodeType<rm_decision::WaypointNav>("WaypointNav", node);
  factory.registerNodeType<rm_decision::NavToHome>("NavToHome", node);
  factory.registerNodeType<rm_decision::HPCheck>("HPCheck", node);
  factory.registerNodeType<rm_decision::PatrolNav>("PatrolNav", node);
  factory.registerNodeType<rm_decision::PoseStatePublisher>("PoseStatePublisher", node);
  factory.registerNodeType<rm_decision::TargetSubscriber>("TargetSubscriber", node);
  factory.registerNodeType<rm_decision::TrackingCheck>("TrackingCheck", node);
  factory.registerNodeType<rm_decision::WaitForDuration>("WaitForDuration", node);
  
  RCLCPP_INFO(node->get_logger(), "Loaded all custom nodes (including PoseStatePublisher)");
  
  RCLCPP_INFO(node->get_logger(), "Loaded all custom nodes");

  // Visualize custom types in the Blackboard
  BT::RegisterJsonDefinition<geometry_msgs::msg::PoseStamped>(PoseStampedToJson);
  BT::RegisterJsonDefinition<geometry_msgs::msg::PointStamped>(PointStampedToJson);

  // Generate xml file for Groot
  std::string xml_models = BT::writeTreeNodesModelXML(factory);
  // Save to file
  if (!tree_node_model_export_path.empty())
  {
    std::ofstream file(tree_node_model_export_path);
    file << xml_models;
    file.close();
    RCLCPP_INFO(node->get_logger(), "Generated XML file: %s", tree_node_model_export_path.c_str());
  }
  
  // Create tree from file
  if (tree_xml_file.empty())
  {
    RCLCPP_ERROR(node->get_logger(), "tree_xml_file parameter is empty!");
    rclcpp::shutdown();
    return 1;
  }
  
  auto tree = factory.createTreeFromFile(tree_xml_file.c_str());
  
  std::shared_ptr<BT::Groot2Publisher> groot2publisher_ptr_;
  groot2publisher_ptr_ = std::make_unique<BT::Groot2Publisher>(tree, groot_port);

  unsigned long int tick_count = 0;
  while (rclcpp::ok())
  {
    tick_count++;
    RCLCPP_INFO(node->get_logger(), "----------Tick %lu---------", tick_count);
    rclcpp::spin_some(node);
    tree.tickOnce();
    tree.sleep(timeout);
  }

  rclcpp::shutdown();
  return 0;
}
