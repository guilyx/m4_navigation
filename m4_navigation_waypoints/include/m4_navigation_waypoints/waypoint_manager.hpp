#pragma once

#include <memory>
#include <string>

#include "arrc/geo/geo.h"
#include "arrc_interfaces/srv/gps_origin.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "m4_msgs/srv/gps_to_pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace m4_navigation_waypoints
{

  /**
   * @brief Manages GPS waypoints and converts them to local coordinates
   */
  class WaypointManager : public rclcpp::Node
  {
  public:
    using GpsOriginSrv = arrc_interfaces::srv::GpsOrigin;
    using GpsOriginClientPtr = rclcpp::Client<GpsOriginSrv>::SharedPtr;
    using GpsToPoseSrv = m4_msgs::srv::GpsToPose;
    using GpsToPoseServicePtr = rclcpp::Service<GpsToPoseSrv>::SharedPtr;

    /**
     * @brief Constructor
     */
    explicit WaypointManager(const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

    /**
     * @brief Destructor
     */
    ~WaypointManager();

  private:
    /**
     * @brief Setup GPS origin
     */
    void setupGpsOrigin();

    /**
     * @brief Convert GPS coordinates to local pose
     * @param lat Latitude
     * @param lon Longitude
     * @param alt Altitude
     * @param frame_id Target frame ID
     * @return Local pose
     */
    geometry_msgs::msg::PoseStamped gpsToLocalPose(
        double lat, double lon, double alt, const std::string &frame_id = "map");

    /**
     * @brief Handle GPS to pose service requests
     */
    void handleGpsToPoseRequest(
        const std::shared_ptr<GpsToPoseSrv::Request> request,
        std::shared_ptr<GpsToPoseSrv::Response> response);

    // Service client for GPS origin
    GpsOriginClientPtr gps_origin_client_;

    // Service server for GPS to pose conversion
    GpsToPoseServicePtr gps_to_pose_server_;

    // GPS origin data
    arrc::geo::UTM utm_origin_;
    bool has_origin_{false};

    // Thread for GPS origin setup
    std::shared_ptr<std::thread> gps_thread_;
    std::atomic<bool> should_exit_{false};

    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::Node::SharedPtr srv_node_;

    GeographicLib::Geocentric earth_;
    std::unique_ptr<GeographicLib::LocalCartesian> local_cartesian_;
  };

} // namespace m4_navigation_waypoints
