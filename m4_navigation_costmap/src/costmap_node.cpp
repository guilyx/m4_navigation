#include "m4_navigation_costmap/costmap_node.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>

namespace m4_navigation_costmap
{

  CostmapNode::CostmapNode(const rclcpp::NodeOptions& options) : Node("costmap_node", options)
  {
    initializeParameters();

    // Initialize TF
    tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Create publisher
    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(10).reliable());

    // Create subscriber
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS(), std::bind(&CostmapNode::pointCloudCallback, this, std::placeholders::_1));

    // Create service
    check_footprint_srv_ = this->create_service<m4_msgs::srv::CheckFootprint>(
        "check_footprint", std::bind(&CostmapNode::handleCheckFootprint, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_INFO(this->get_logger(), "Costmap node initialized");
  }

  void CostmapNode::initializeParameters()
  {
    // Declare and get parameters
    this->declare_parameter("pointcloud_topic", "points");
    this->declare_parameter("sensor_frame", "sensor_frame");
    this->declare_parameter("global_frame", "map");
    this->declare_parameter("robot_frame", "base_link");
    this->declare_parameter("min_obstacle_height", 0.1);
    this->declare_parameter("max_obstacle_height", 2.0);
    this->declare_parameter("voxel_leaf_size", 0.05);
    this->declare_parameter("resolution", 0.05);
    this->declare_parameter("inflation_radius", 0.3);
    this->declare_parameter("cost_scaling_factor", 10);
    this->declare_parameter("costmap_radius", 5.0);

    // Default robot footprint (square)
    std::vector<double> default_footprint = {
      -0.5, -0.5, // Point 1
      -0.5, 0.5,  // Point 2
      0.5,  0.5,  // Point 3
      0.5,  -0.5  // Point 4
    };
    this->declare_parameter("robot_footprint", default_footprint);

    pointcloud_topic_ = this->get_parameter("pointcloud_topic").as_string();
    sensor_frame_ = this->get_parameter("sensor_frame").as_string();
    global_frame_ = this->get_parameter("global_frame").as_string();
    robot_frame_ = this->get_parameter("robot_frame").as_string();
    min_obstacle_height_ = this->get_parameter("min_obstacle_height").as_double();
    max_obstacle_height_ = this->get_parameter("max_obstacle_height").as_double();
    voxel_leaf_size_ = this->get_parameter("voxel_leaf_size").as_double();
    resolution_ = this->get_parameter("resolution").as_double();
    inflation_radius_ = this->get_parameter("inflation_radius").as_double();
    cost_scaling_factor_ = this->get_parameter("cost_scaling_factor").as_int();
    costmap_radius_ = this->get_parameter("costmap_radius").as_double();

    // Get footprint
    std::vector<double> footprint = this->get_parameter("robot_footprint").as_double_array();
    if (footprint.size() % 2 != 0) {
      RCLCPP_ERROR(this->get_logger(), "Robot footprint parameter must have even number of values");
      return;
    }

    robot_footprint_.clear();
    for (size_t i = 0; i < footprint.size(); i += 2) {
      robot_footprint_.emplace_back(footprint[i], footprint[i + 1]);
    }

    // Configure PCL filters
    pass_filter_.setFilterFieldName("z");
    pass_filter_.setFilterLimits(min_obstacle_height_, max_obstacle_height_);

    voxel_filter_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, voxel_leaf_size_);
  }

  void CostmapNode::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    // Get robot position
    geometry_msgs::msg::TransformStamped robot_transform;
    if (!getRobotPose(robot_transform)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
      return;
    }

    // Transform pointcloud to global frame
    sensor_msgs::msg::PointCloud2 transformed_cloud;
    if (!transformPointCloud(msg, transformed_cloud, global_frame_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to transform pointcloud");
      return;
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(transformed_cloud, *cloud);

    // Filter pointcloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    filterPointCloud(cloud, filtered_cloud);

    // Create costmap
    nav_msgs::msg::OccupancyGrid costmap;
    createCostmap(filtered_cloud, costmap);

    // Inflate costmap
    inflateCostmap(costmap);

    // Store latest costmap
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      latest_costmap_ = std::make_shared<nav_msgs::msg::OccupancyGrid>(costmap);
    }

    // Publish costmap
    costmap_pub_->publish(costmap);
  }

  void CostmapNode::filterPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr& filtered_cloud)
  {
    // Get robot position
    geometry_msgs::msg::TransformStamped robot_transform;
    if (!getRobotPose(robot_transform)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose");
      return;
    }

    // Apply radius filter around robot position
    radius_filter_.setInputCloud(cloud);
    radius_filter_.setMin(Eigen::Vector4f(-costmap_radius_, -costmap_radius_, min_obstacle_height_, 1.0));
    radius_filter_.setMax(Eigen::Vector4f(costmap_radius_, costmap_radius_, max_obstacle_height_, 1.0));
    radius_filter_.setTranslation(Eigen::Vector3f(robot_transform.transform.translation.x, robot_transform.transform.translation.y, 0.0));
    radius_filter_.setRotation(Eigen::Vector3f(0.0, 0.0, 0.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr radius_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    radius_filter_.filter(*radius_filtered);

    // Apply height filter
    pcl::PointCloud<pcl::PointXYZ>::Ptr height_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pass_filter_.setInputCloud(radius_filtered);
    pass_filter_.filter(*height_filtered);

    // Apply voxel grid filter
    voxel_filter_.setInputCloud(height_filtered);
    voxel_filter_.filter(*filtered_cloud);
  }

  bool CostmapNode::getRobotPose(geometry_msgs::msg::TransformStamped& transform)
  {
    try {
      transform = tf_buffer_->lookupTransform(global_frame_, robot_frame_, rclcpp::Time(0), rclcpp::Duration::from_seconds(0.1));
      return true;
    } catch (tf2::TransformException& ex) {
      RCLCPP_ERROR(this->get_logger(), "Failed to get robot pose: %s", ex.what());
      return false;
    }
  }

  void CostmapNode::handleCheckFootprint(const m4_msgs::srv::CheckFootprint::Request::SharedPtr request,
                                         m4_msgs::srv::CheckFootprint::Response::SharedPtr response)
  {
    // Get latest costmap
    nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
    {
      std::lock_guard<std::mutex> lock(costmap_mutex_);
      if (!latest_costmap_) {
        response->in_collision = false;
        response->closest_obstacle_dist = std::numeric_limits<double>::max();
        response->message = "No costmap available";
        return;
      }
      costmap = latest_costmap_;
    }

    // Transform footprint to costmap frame if needed
    geometry_msgs::msg::TransformStamped transform;
    std::vector<Point2D> transformed_footprint;

    if (request->frame_id != costmap->header.frame_id) {
      try {
        transform =
            tf_buffer_->lookupTransform(costmap->header.frame_id, request->frame_id, this->now(), rclcpp::Duration::from_seconds(0.1));

        transformed_footprint = transformFootprint(request->footprint, transform.transform);
      } catch (tf2::TransformException& ex) {
        response->in_collision = false;
        response->closest_obstacle_dist = std::numeric_limits<double>::max();
        response->message = std::string("Transform failed: ") + ex.what();
        return;
      }
    } else {
      // Convert polygon to Point2D vector
      transformed_footprint.reserve(request->footprint.points.size());
      for (const auto& point : request->footprint.points) {
        transformed_footprint.emplace_back(point.x, point.y);
      }
    }

    // Check collision
    response->in_collision = isFootprintInCollision(transformed_footprint, *costmap);
    response->message = response->in_collision ? "Footprint in collision" : "No collision";

    // TODO: Implement closest obstacle distance calculation
    response->closest_obstacle_dist = 0.0;
  }

  bool CostmapNode::isPointInCollision(const Point2D& point, const nav_msgs::msg::OccupancyGrid& costmap)
  {
    // Convert point to grid coordinates
    int grid_x = static_cast<int>((point.x - costmap.info.origin.position.x) / costmap.info.resolution);
    int grid_y = static_cast<int>((point.y - costmap.info.origin.position.y) / costmap.info.resolution);

    // Check if point is within grid bounds
    if (grid_x < 0 || grid_x >= static_cast<int>(costmap.info.width) || grid_y < 0 || grid_y >= static_cast<int>(costmap.info.height)) {
      return true; // Consider out-of-bounds as collision
    }

    // Get cost at point
    int index = grid_y * costmap.info.width + grid_x;
    return costmap.data[index] >= 90; // Consider high-cost cells as collision
  }

  bool CostmapNode::isFootprintInCollision(const std::vector<Point2D>& footprint, const nav_msgs::msg::OccupancyGrid& costmap)
  {
    if (footprint.empty()) {
      return false;
    }

    // Check each point of the footprint
    for (const auto& point : footprint) {
      if (isPointInCollision(point, costmap)) {
        return true;
      }
    }

    // TODO: Implement line checking between footprint points
    // This would ensure we don't miss collisions between vertices

    return false;
  }

  std::vector<Point2D> CostmapNode::transformFootprint(const geometry_msgs::msg::Polygon& footprint,
                                                       const geometry_msgs::msg::Transform& transform)
  {
    std::vector<Point2D> transformed_points;
    transformed_points.reserve(footprint.points.size());

    // Create transformation matrix
    double cos_yaw = 1.0 - 2.0 * (transform.rotation.z * transform.rotation.z);
    double sin_yaw = 2.0 * (transform.rotation.w * transform.rotation.z);

    // Transform each point
    for (const auto& point : footprint.points) {
      double x = point.x;
      double y = point.y;

      // Apply rotation
      double new_x = x * cos_yaw - y * sin_yaw;
      double new_y = x * sin_yaw + y * cos_yaw;

      // Apply translation
      new_x += transform.translation.x;
      new_y += transform.translation.y;

      transformed_points.emplace_back(new_x, new_y);
    }

    return transformed_points;
  }

} // namespace m4_navigation_costmap

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(m4_navigation_costmap::CostmapNode)
