cmake_minimum_required(VERSION 3.8)
project(m4_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/TrackPath.action"
  "action/Morph.action"
  "srv/GpsToPose.srv"
  DEPENDENCIES action_msgs geometry_msgs geographic_msgs
)

ament_package()
