#include <memory>
#include <string>

#include "m4_bt_plugins/action/navigate_to_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace m4_bt_plugins
{

  NavigateToAction::NavigateToAction(
      const std::string &name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : swarm_bt_engine::BtActionNode<m4_msgs::action::TrackPath>(name, action_name, conf),
        logger_(rclcpp::get_logger("NavigateToAction"))
  {
    RCLCPP_DEBUG(logger_, "Initialized a NavigateToAction BT node");
  }

  void NavigateToAction::on_tick()
  {
    // Get goal pose from input port
    geometry_msgs::msg::PoseStamped goal_pose;
    bool is_gps;
    if (!getInput("goal", goal_pose))
    {
      RCLCPP_ERROR(logger_, "NavigateToAction: goal port not provided");
      return;
    }

    if (!getInput("is_gps", is_gps))
    {
      RCLCPP_ERROR(logger_, "NavigateToAction: is_gps port not provided");
      return;
    }

    std::string ns = node_->get_namespace();

    // Removes leading / for tf transformation
    if (!ns.empty() && ns.front() == '/') {
      ns.erase(0, 1); // Removes the first character
    }
    goal_pose.header.frame_id = ns + goal_pose.header.frame_id;

    // Build action goal
    goal_.path.clear();
    goal_.path.push_back(goal_pose);
    goal_.is_gps = is_gps;

    RCLCPP_INFO(
        logger_,
        "Successfully built navigation goal to position (%.2f, %.2f, %.2f)",
        goal_pose.pose.position.x,
        goal_pose.pose.position.y,
        goal_pose.pose.position.z);
  }

} // namespace m4_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  auto builder = [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<m4_bt_plugins::NavigateToAction>(name, "track_path", config);
  };

  factory.registerBuilder<m4_bt_plugins::NavigateToAction>("NavigateTo", builder);
}