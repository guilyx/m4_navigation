#include <memory>
#include <string>
#include <unordered_map>
#include <algorithm>

#include "m4_bt_plugins/action/morph_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"

namespace m4_bt_plugins
{
  // Initialize the static mode map
  const std::unordered_map<std::string, bool> MorphAction::mode_map_ = {
      {FLY_MODE, true},
      {DRIVE_MODE, false}};

  MorphAction::MorphAction(
      const std::string &name,
      const std::string &action_name,
      const BT::NodeConfiguration &conf)
      : swarm_bt_engine::BtActionNode<m4_msgs::action::Morph>(name, action_name, conf),
        logger_(rclcpp::get_logger("MorphAction"))
  {
    RCLCPP_DEBUG(logger_, "Initialized a MorphAction BT node");
  }

  void MorphAction::on_tick()
  {
    // Get target mode from input port
    std::string target_mode;
    if (!getInput("target_mode", target_mode))
    {
      RCLCPP_ERROR(logger_, "MorphAction: target_mode port not provided");
      setStatus(BT::NodeStatus::FAILURE);
    }

    // Look up the mode in our map
    auto mode_it = mode_map_.find(target_mode);
    if (mode_it == mode_map_.end())
    {
      RCLCPP_ERROR(
          logger_,
          "MorphAction: invalid target_mode '%s'. Must be exactly %s or %s",
          target_mode.c_str(), FLY_MODE, DRIVE_MODE);
      setStatus(BT::NodeStatus::FAILURE);
    }

    // Set the goal using the mapped boolean value
    goal_.to_fly_mode = mode_it->second;

    RCLCPP_INFO(
        logger_,
        "Successfully built morph goal to %s mode",
        goal_.to_fly_mode ? "fly" : "drive");
  }
} // namespace m4_bt_plugins

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  auto builder = [](const std::string &name, const BT::NodeConfiguration &config)
  {
    return std::make_unique<m4_bt_plugins::MorphAction>(name, "morph", config);
  };

  factory.registerBuilder<m4_bt_plugins::MorphAction>("Morph", builder);
}