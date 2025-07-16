#ifndef M4_BT_PLUGINS__ACTION__MORPH_ACTION_HPP_
#define M4_BT_PLUGINS__ACTION__MORPH_ACTION_HPP_

#include <string>
#include <memory>
#include <unordered_map>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "swarm_bt_engine/bt_action_node.hpp"

#include "m4_msgs/action/morph.hpp"

namespace m4_bt_plugins
{
  /**
   * @brief A BT Action node that calls the Morph action server to change robot morphology
   */
  class MorphAction : public swarm_bt_engine::BtActionNode<m4_msgs::action::Morph>
  {
  public:
    /**
     * @brief Constants for robot morphology modes
     */
    static constexpr const char *FLY_MODE = "FLY";
    static constexpr const char *DRIVE_MODE = "DRIVE";

    /**
     * @brief A constructor for m4_bt_plugins::action::MorphAction
     * @param name Name for the XML tag for this node
     * @param action_name The action server name
     * @param conf BT node configuration
     */
    MorphAction(
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
          BT::InputPort<std::string>(
              "target_mode",
              "Target morphology mode (" + std::string(FLY_MODE) + "/" + std::string(DRIVE_MODE) + ")"),
      };
    }

    /**
     * @brief Called on tick, sets up the goal for the action
     */
    void on_tick() override;

  private:
    rclcpp::Logger logger_{rclcpp::get_logger("MorphAction")};

    /**
     * @brief Maps mode strings to their corresponding boolean values
     */
    static const std::unordered_map<std::string, bool> mode_map_;
  };

} // namespace m4_bt_plugins

#endif // M4_BT_PLUGINS__ACTION__MORPH_ACTION_HPP_