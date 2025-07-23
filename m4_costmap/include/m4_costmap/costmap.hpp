#pragma once

#include "m4_costmap/filters/footprint_filter.hpp"
#include "m4_costmap/filters/height_filter.hpp"
#include "m4_costmap/layers/inflation_layer.hpp"
#include "m4_costmap/layers/obstacle_layer.hpp"

#include <memory>
#include <mutex>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace m4_costmap
{

  class Costmap : public rclcpp::Node
  {
  public:
    explicit Costmap(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Costmap() = default;

  protected:
    // ROS parameters
    void declareParameters();
    void getParameters();

    // Pointcloud processing
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void processPointCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

    // Map management
    void updateMap();
    void resetMaps();
    void publishMap();
    void publishMarkers();
    void publishDebugInfo();

    // Services
    void clearCostmapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                              std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    // Footprint handling
    void setFootprint(const geometry_msgs::msg::Polygon& footprint);
    void setCircularFootprint(double radius);
    geometry_msgs::msg::Polygon getFootprint() const;

    // Coordinate transforms
    void worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const;
    void mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const;
    void configureLayerParameters();

  private:
    // Map properties
    double width_meters_;  // Total width of map in meters
    double height_meters_; // Total height of map in meters
    double resolution_;    // Meters per cell

    // Derived properties
    unsigned int size_x_; // Number of cells in x
    unsigned int size_y_; // Number of cells in y
    double origin_x_;     // Computed as -width_meters_/2
    double origin_y_;     // Computed as -height_meters_/2
    std::vector<unsigned char> costmap_;

    // Helper to compute grid properties
    void updateGridProperties();

    // Robot footprint
    geometry_msgs::msg::Polygon footprint_;
    double inscribed_radius_;
    double circumscribed_radius_;

    // Layers (direct instances, not plugins)
    std::unique_ptr<layers::ObstacleLayer> obstacle_layer_;
    std::unique_ptr<layers::InflationLayer> inflation_layer_;
    bool use_obstacle_layer_;
    bool use_inflation_layer_;

    // Filters
    std::unique_ptr<filters::HeightFilter> height_filter_;
    std::unique_ptr<filters::FootprintFilter> footprint_filter_;
    bool use_height_filter_;
    bool use_footprint_filter_;

    // TF
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Publishers/Subscribers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;

    // Services
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_costmap_srv_;

    // Timer
    rclcpp::TimerBase::SharedPtr update_timer_;

    // Parameters
    std::string global_frame_;
    std::string robot_base_frame_;
    double transform_tolerance_;
    double update_frequency_;
    bool pc_received_ = false;

    // Debug info
    struct LayerStats {
      double update_time;
      int cells_updated;
    };
    LayerStats obstacle_stats_;
    LayerStats inflation_stats_;
    rclcpp::Time last_update_time_;

    // Mutex for thread safety
    mutable std::mutex mutex_;
  };

} // namespace m4_costmap
