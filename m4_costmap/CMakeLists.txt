cmake_minimum_required(VERSION 3.8)
project(m4_costmap)

set(CMAKE_CXX_STANDARD 20)

# lots of warnings and all warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(backward_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)

set(exec_name ${PROJECT_NAME})
set(library_name ${PROJECT_NAME}_lib)

# Create the library
add_library(${library_name}
  src/costmap.cpp
  src/costmap_layer.cpp
  src/filters/height_filter.cpp
  src/filters/footprint_filter.cpp
  src/layers/obstacle_layer.cpp
  src/layers/inflation_layer.cpp
)

target_include_directories(${library_name} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${library_name}
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_geometry_msgs
  pcl_ros
  pcl_conversions
  tf2_sensor_msgs
)

# Create the node executable
add_executable(${exec_name} src/costmap_node.cpp)
target_link_libraries(${exec_name}
  ${library_name}
)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY
    config
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${exec_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   find_package(ament_cmake_gtest REQUIRED)

#   # Add linters
#   ament_lint_auto_find_test_dependencies()

#   # Add gtest
#   ament_add_gtest(test_costmap
#     test/test_costmap.cpp
#     test/test_filters.cpp
#     test/test_layers.cpp
#   )
#   target_link_libraries(test_costmap
#     ${PROJECT_NAME}
#   )
# endif()

ament_package()
