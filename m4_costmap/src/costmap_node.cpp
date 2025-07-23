#include "m4_costmap/costmap.hpp"

#include <rclcpp/rclcpp.hpp>

/**
 * @brief Main entry point for the m4_costmap node.
 *
 * This version uses a MultiThreadedExecutor to allow callbacks to be processed in parallel,
 * which is recommended for nodes that may have multiple subscriptions, timers, or services.
 */
int main(int argc, char* argv[])
{
  // Initialize ROS 2
  rclcpp::init(argc, argv);

  // Create the Costmap node
  auto costmap_node = std::make_shared<m4_costmap::Costmap>();

  // Use a MultiThreadedExecutor for improved concurrency
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(costmap_node);

  // Spin the executor to process callbacks
  executor.spin();

  // Shutdown ROS 2
  rclcpp::shutdown();
  return 0;
}