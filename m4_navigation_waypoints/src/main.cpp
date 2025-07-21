#include "m4_navigation_waypoints/waypoint_manager.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto options = rclcpp::NodeOptions();
  auto node = std::make_shared<m4_navigation_waypoints::WaypointManager>(options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}