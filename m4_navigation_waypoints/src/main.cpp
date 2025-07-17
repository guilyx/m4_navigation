#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "m4_navigation_waypoints/waypoint_manager.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<m4_navigation_waypoints::WaypointManager>(options);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();
  return 0;
}