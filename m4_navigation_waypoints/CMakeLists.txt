cmake_minimum_required(VERSION 3.8)
project(m4_navigation_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(arrc REQUIRED)
find_package(arrc_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(m4_msgs REQUIRED)

# Create waypoint manager library
add_library(waypoint_manager SHARED
  src/waypoint_manager.cpp
)
target_include_directories(waypoint_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(waypoint_manager
  rclcpp
  arrc
  arrc_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
  m4_msgs
)

# Create waypoint manager node
add_executable(waypoint_manager_node
  src/waypoint_manager_node.cpp
)
target_link_libraries(waypoint_manager_node
  waypoint_manager
)
ament_target_dependencies(waypoint_manager_node
  rclcpp
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS waypoint_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS waypoint_manager_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

# Export
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp
  arrc
  arrc_interfaces
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  rclcpp_components
)

ament_package()
