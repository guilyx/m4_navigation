#ifndef M4_BT_PLUGINS__ACTION__NAVIGATE_TO_ACTION_HPP_
#define M4_BT_PLUGINS__ACTION__NAVIGATE_TO_ACTION_HPP_

#include <string>
#include <memory>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "swarm_bt_engine/bt_action_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "m4_msgs/action/track_path.hpp"

namespace m4_bt_plugins
{
  /**
   * @brief A BT Action node that calls the TrackPath action server to navigate to a goal
   */
  class NavigateToAction : public swarm_bt_engine::BtActionNode<m4_msgs::action::TrackPath>
  {
  public:
    /**
     * @brief A constructor for m4_bt_plugins::action::NavigateToAction
     * @param name Name for the XML tag for this node
     * @param action_name The action server name
     * @param conf BT node configuration
     */
    NavigateToAction(
        const std::string &name,
        const std::string &action_name,
        const BT::NodeConfiguration &conf);

    /**
     * @brief Creates list of BT ports
     * @return BT::PortsList Containing required ports
     */
    static BT::PortsList providedPorts()
    {
      return {
          BT::InputPort<geometry_msgs::msg::PoseStamped>("goal", "Goal pose to navigate to"),
          BT::InputPort<bool>("is_gps", "Whether the goal is in GPS frame"),
      };
    }

    /**
     * @brief Called on tick, sets up the goal for the action
     */
    void on_tick() override;

  private:
    rclcpp::Logger logger_{rclcpp::get_logger("NavigateToAction")};
  };

} // namespace m4_bt_plugins

#endif // M4_BT_PLUGINS__ACTION__NAVIGATE_TO_ACTION_HPP_