cmake_minimum_required(VERSION 3.8)
project(turtleshot_bt)

# Compiler settings
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)

# Include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${behaviortree_cpp_v3_INCLUDE_DIRS}
)

# List of dependencies
set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_components
  behaviortree_cpp_v3
  nav2_msgs
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  vision_msgs
  tf2
  tf2_geometry_msgs
  ament_index_cpp
)

# Main BT node executable
add_executable(turtleshot_bt_node
  src/turtleshot_bt_node.cpp
  src/actions/navigate_to_pose_action.cpp
  src/actions/fire_catapult_action.cpp
  src/conditions/has_target_condition.cpp
  src/conditions/target_in_range_condition.cpp
)

ament_target_dependencies(turtleshot_bt_node ${dependencies})

target_link_libraries(turtleshot_bt_node
  ${behaviortree_cpp_v3_LIBRARIES}
)

# Install executables
install(TARGETS
  turtleshot_bt_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install trees directory
install(DIRECTORY
  trees
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/mock_nav_server.py
  scripts/mock_catapult_service.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install include directory
install(DIRECTORY
  include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
