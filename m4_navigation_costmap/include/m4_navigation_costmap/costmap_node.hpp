#ifndef M4_NAVIGATION_COSTMAP__COSTMAP_NODE_HPP_
#define M4_NAVIGATION_COSTMAP__COSTMAP_NODE_HPP_

#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/polygon.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "m4_msgs/srv/check_footprint.hpp" // You'll need to create this service definition
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

#include <memory>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <string>
#include <vector>

namespace m4_navigation_costmap
{

  struct Point2D {
    double x;
    double y;
    Point2D(double x_ = 0.0, double y_ = 0.0) : x(x_), y(y_) {}
  };

  class CostmapNode : public rclcpp::Node
  {
  public:
    explicit CostmapNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    virtual ~CostmapNode() = default;

  private:
    // Parameters
    std::string pointcloud_topic_;
    std::string sensor_frame_;
    std::string global_frame_;
    std::string robot_frame_;
    double min_obstacle_height_;
    double max_obstacle_height_;
    double voxel_leaf_size_;
    double resolution_; // meters/cell
    double inflation_radius_;
    double costmap_radius_; // radius around robot to compute costmap
    int cost_scaling_factor_;
    std::vector<Point2D> robot_footprint_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

    // Publishers
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;

    // Services
    rclcpp::Service<m4_msgs::srv::CheckFootprint>::SharedPtr check_footprint_srv_;

    // TF
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // PCL objects
    pcl::PassThrough<pcl::PointXYZ> pass_filter_;
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
    pcl::CropBox<pcl::PointXYZ> radius_filter_;

    // Methods
    void initializeParameters();
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud);
    void createCostmap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, nav_msgs::msg::OccupancyGrid& costmap);
    void inflateCostmap(nav_msgs::msg::OccupancyGrid& costmap);
    bool transformPointCloud(const sensor_msgs::msg::PointCloud2::SharedPtr& cloud_in, sensor_msgs::msg::PointCloud2& cloud_out,
                             const std::string& target_frame);
    void handleCheckFootprint(const m4_msgs::srv::CheckFootprint::Request::SharedPtr request,
                              m4_msgs::srv::CheckFootprint::Response::SharedPtr response);
    bool isPointInCollision(const Point2D& point, const nav_msgs::msg::OccupancyGrid& costmap);
    bool isFootprintInCollision(const std::vector<Point2D>& footprint, const nav_msgs::msg::OccupancyGrid& costmap);
    std::vector<Point2D> transformFootprint(const geometry_msgs::msg::Polygon& footprint, const geometry_msgs::msg::Transform& transform);
    bool getRobotPose(geometry_msgs::msg::TransformStamped& transform);

    // Latest costmap cache
    nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
    std::mutex costmap_mutex_;
  };

} // namespace m4_navigation_costmap

#endif // M4_NAVIGATION_COSTMAP__COSTMAP_NODE_HPP_