#include "m4_navigation_waypoints/waypoint_manager.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;

namespace m4_navigation_waypoints
{

  WaypointManager::WaypointManager(const rclcpp::NodeOptions& options) :
      Node("m4_waypoint_manager", options), earth_(GeographicLib::Geocentric::WGS84())
  {
    RCLCPP_INFO(get_logger(), "Creating WaypointManager");
    srv_node_ = std::make_shared<rclcpp::Node>("m4_waypoint_manager_srv");

    // Create GPS origin service client
    gps_origin_client_ = srv_node_->create_client<GpsOriginSrv>("getCommonOrigin");

    // Create transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

    // Start GPS origin setup thread
    should_exit_ = false;

    // Create GPS to pose service
    gps_to_pose_server_ = create_service<GpsToPoseSrv>(
        "gps_to_pose", std::bind(&WaypointManager::handleGpsToPoseRequest, this, std::placeholders::_1, std::placeholders::_2));
    gps_thread_ = std::make_shared<std::thread>(&WaypointManager::setupGpsOrigin, this);
  }

  WaypointManager::~WaypointManager()
  {
    should_exit_ = true;
    if (gps_thread_ && gps_thread_->joinable()) {
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

    while (!should_exit_ && rclcpp::ok()) {
      if (!gps_origin_client_->wait_for_service(1s)) {
        RCLCPP_INFO(get_logger(), "Waiting for GPS origin service...");
        continue;
      }

      auto future = gps_origin_client_->async_send_request(request);
      if (rclcpp::spin_until_future_complete(srv_node_, future) == rclcpp::FutureReturnCode::SUCCESS) {
        response = future.get();
        lat = response->coordinates.latitude;
        lon = response->coordinates.longitude;
        RCLCPP_INFO(get_logger(), "Retrieved GPS positions: %.6f / %.6f", lat, lon);
        if (!std::isnan(lat) && !std::isnan(lon)) {
          break;
        }
      }
      std::this_thread::sleep_for(500ms);
    }

    if (should_exit_ || !rclcpp::ok()) {
      return;
    }

    RCLCPP_INFO(get_logger(), "Successfully setup GPS Origin: %.6f, %.6f", response->coordinates.latitude, response->coordinates.longitude);

    // Initialize LocalCartesian projector with origin
    local_cartesian_ = std::make_unique<GeographicLib::LocalCartesian>(response->coordinates.latitude, response->coordinates.longitude,
                                                                       0.0, // altitude
                                                                       earth_);

    has_origin_ = true;
  }

  void WaypointManager::handleGpsToPoseRequest(const std::shared_ptr<GpsToPoseSrv::Request> request,
                                               std::shared_ptr<GpsToPoseSrv::Response> response)
  {
    RCLCPP_INFO(get_logger(), "Received GPS to pose request");
    try {
      response->pose = gpsToLocalPose(request->latitude, request->longitude, request->altitude, request->frame_id);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to convert GPS to pose: %s", e.what());
      response->success = false;
      return;
    }

    response->success = true;
  }

  geometry_msgs::msg::PoseStamped WaypointManager::gpsToLocalPose(double lat, double lon, double alt, const std::string& frame_id)
  {
    if (!has_origin_) {
      throw std::runtime_error("GPS origin not set");
    }

    // Convert GPS coordinates to local cartesian coordinates
    double x, y, z;
    local_cartesian_->Forward(lat, lon, alt, x, y, z);

    RCLCPP_INFO(get_logger(), "Converting GPS {lat=%.6f, lon=%.6f, alt=%.2f} to local {x=%.2f, y=%.2f, z=%.2f}", lat, lon, alt, x, y, z);

    // Create pose message
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = now();
    pose.header.frame_id = frame_id;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    // Set identity quaternion for now
    pose.pose.orientation.w = 1.0;

    return pose;
  }

} // namespace m4_navigation_waypoints
