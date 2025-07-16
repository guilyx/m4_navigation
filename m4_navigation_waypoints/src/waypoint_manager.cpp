#include <thread>
#include <chrono>

#include "m4_navigation_waypoints/waypoint_manager.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace m4_navigation_waypoints
{

  WaypointManager::WaypointManager(const rclcpp::NodeOptions &options)
      : Node("waypoint_manager", options)
  {
    RCLCPP_INFO(get_logger(), "Creating WaypointManager");

    // Create GPS origin service client
    gps_origin_client_ = create_client<GpsOriginSrv>("getCommonOrigin");

    // Create transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Create GPS to pose service
    gps_to_pose_server_ = create_service<GpsToPoseSrv>(
        "gps_to_pose",
        std::bind(&WaypointManager::handleGpsToPoseRequest, this,
                  std::placeholders::_1, std::placeholders::_2));

    // Start GPS origin setup thread
    should_exit_ = false;
    gps_thread_ = std::make_shared<std::thread>(&WaypointManager::setupGpsOrigin, this);
  }

  WaypointManager::~WaypointManager()
  {
    should_exit_ = true;
    if (gps_thread_ && gps_thread_->joinable())
    {
      gps_thread_->join();
    }
  }

  void WaypointManager::setupGpsOrigin()
  {
    auto request = std::make_shared<GpsOriginSrv::Request>();
    auto response = std::make_shared<GpsOriginSrv::Response>();
    float lat = NAN;
    float lon = NAN;

    RCLCPP_INFO(get_logger(), "Requesting GPS origin position");

    while (!should_exit_ && rclcpp::ok())
    {
      if (!gps_origin_client_->wait_for_service(1s))
      {
        RCLCPP_INFO(get_logger(), "Waiting for GPS origin service...");
        continue;
      }

      auto future = gps_origin_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS)
      {
        response = future.get();
        lat = response->coordinates.latitude;
        lon = response->coordinates.longitude;
        RCLCPP_INFO(
            get_logger(), "Retrieved GPS positions: %.6f / %.6f",
            lat, lon);
        if (!std::isnan(lat) && !std::isnan(lon))
        {
          break;
        }
      }
      std::this_thread::sleep_for(500ms);
    }

    if (should_exit_ || !rclcpp::ok())
    {
      return;
    }

    RCLCPP_INFO(
        get_logger(),
        "Successfully setup GPS Origin: %.6f, %.6f",
        response->coordinates.latitude,
        response->coordinates.longitude);

    arrc::geo::LatLon gps_origin;
    gps_origin.latitude = response->coordinates.latitude;
    gps_origin.longitude = response->coordinates.longitude;
    utm_origin_ = arrc::geo::toUTM(gps_origin);

    RCLCPP_INFO(
        get_logger(),
        "Zone: %d%c | UTM Northing Origin: %.2f | UTM Easting Origin: %.2f",
        utm_origin_.zone, utm_origin_.letter,
        utm_origin_.northing, utm_origin_.easting);

    has_origin_ = true;
  }

  void WaypointManager::handleGpsToPoseRequest(
      const std::shared_ptr<GpsToPoseSrv::Request> request,
      std::shared_ptr<GpsToPoseSrv::Response> response)
  {
    try
    {
      response->pose = gpsToLocalPose(
          request->latitude,
          request->longitude,
          request->altitude,
          request->frame_id);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(get_logger(), "Failed to convert GPS to pose: %s", e.what());
      throw;
    }
  }

  geometry_msgs::msg::PoseStamped WaypointManager::gpsToLocalPose(
      double lat, double lon, double alt, const std::string &frame_id)
  {
    if (!has_origin_)
    {
      throw std::runtime_error("GPS origin not set");
    }

    // Convert GPS coordinates to UTM
    arrc::geo::LatLon gps_pos;
    gps_pos.latitude = lat;
    gps_pos.longitude = lon;
    auto utm = arrc::geo::toUTM(gps_pos);

    // Calculate local position relative to origin
    double northing = utm.northing - utm_origin_.northing;
    double easting = (-1) * (utm.easting - utm_origin_.easting);

    // Create pose message
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_id;
    pose.pose.position.x = northing;
    pose.pose.position.y = easting;
    pose.pose.position.z = alt;

    // Calculate orientation to face the point from current position
    // Note: This assumes we have a way to get current position, which we'll need to add
    // For now, just set identity quaternion
    pose.pose.orientation.w = 1.0;

    return pose;
  }

} // namespace m4_navigation_waypoints

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(m4_navigation_waypoints::WaypointManager)
