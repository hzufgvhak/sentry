cmake_minimum_required(VERSION 3.11)
project(rm_decision)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(auto_aim_interfaces REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_action REQUIRED)


include_directories(include)

set(ros_dependencies
  rclcpp
  auto_aim_interfaces
  nav_msgs
  nav2_msgs
  nav2_util
  geometry_msgs
  behaviortree_cpp
  rclcpp_components
  sensor_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_action
)

add_library(${PROJECT_NAME} SHARED
  src/custume_types.cpp
  src/behaviors/serial_packet_subscriber.cpp
  src/behaviors/game_time_check.cpp
  src/behaviors/waypoint_nav.cpp
  src/behaviors/nav_to_home.cpp
  src/behaviors/hp_check.cpp
  src/behaviors/patrol_nav.cpp
  src/behaviors/pose_state_publisher.cpp
  src/behaviors/target_subscriber.cpp
  src/behaviors/tracking_check.cpp
  src/behaviors/wait_for_duration.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME} ${ros_dependencies})

add_executable(tree_exec_node src/tree_exec_node.cpp
  src/custume_types.cpp
  src/behaviors/serial_packet_subscriber.cpp
  src/behaviors/game_time_check.cpp
  src/behaviors/waypoint_nav.cpp
  src/behaviors/nav_to_home.cpp
  src/behaviors/hp_check.cpp
  src/behaviors/patrol_nav.cpp
  src/behaviors/pose_state_publisher.cpp
  src/behaviors/target_subscriber.cpp
  src/behaviors/tracking_check.cpp
  src/behaviors/wait_for_duration.cpp
)
ament_target_dependencies(tree_exec_node ${ros_dependencies})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS tree_exec_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config behavior_tree
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()
