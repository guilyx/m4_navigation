cmake_minimum_required(VERSION 3.5)
project(m4_navigation_waypoints)

set(CMAKE_CXX_STANDARD 20)

# lots of warnings and all warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
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

# Add the path to the CMake module
set(CMAKE_MODULE_PATH "/usr/share/cmake/geographiclib" ${CMAKE_MODULE_PATH})
find_package(GeographicLib REQUIRED)

set(exec_name ${PROJECT_NAME})
set(library_name ${PROJECT_NAME}_lib)

add_library(${library_name} SHARED
  src/waypoint_manager.cpp
)

set(dependencies
  "ament_cmake"
  "rclcpp"
  "arrc"
  "arrc_interfaces"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "tf2_geometry_msgs"
  "rclcpp_components"
  "m4_msgs"
)

ament_target_dependencies(${library_name} PUBLIC
  ${dependencies}
)

target_link_libraries(${library_name} PUBLIC
  arrc::arrc
  Geographic
)

target_include_directories(${library_name} PUBLIC include)

add_executable(${exec_name} src/main.cpp)
target_link_libraries(${exec_name} ${library_name})

install(TARGETS ${library_name}
  EXPORT export_${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${exec_name}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY DESTINATION share/${PROJECT_NAME})

ament_package()

#if(BUILD_TESTING)
  #find_package(ament_cmake_gtest REQUIRED)
  #find_package(ament_cmake_pytest REQUIRED)
  #add_subdirectory(test)
#endif()

