#include "m4_costmap/costmap.hpp"

#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace m4_costmap
{

  static const unsigned char NO_INFORMATION = 255;
  static const unsigned char LETHAL_OBSTACLE = 254;
  static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
  static const unsigned char FREE_SPACE = 0;

  Costmap::Costmap(const rclcpp::NodeOptions& options) :
      Node("costmap", options),
      width_meters_(10.0),
      height_meters_(10.0),
      resolution_(0.05),
      size_x_(0),
      size_y_(0),
      origin_x_(0.0),
      origin_y_(0.0),
      inscribed_radius_(0.0),
      circumscribed_radius_(0.0),
      use_obstacle_layer_(true),
      use_inflation_layer_(true),
      use_height_filter_(true),
      use_footprint_filter_(true),
      transform_tolerance_(0.5),
      update_frequency_(5.0),
      pc_received_(false),
      last_update_time_(get_clock()->now())
  {
    // Declare parameters first
    declareParameters();

    // Initialize TF
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    // Get parameters before creating objects
    getParameters();

    // Update grid properties (this sets up the basic map structure)
    updateGridProperties();

    // Create and initialize objects with the correct dimensions
    if (use_height_filter_) {
      height_filter_ = std::make_unique<filters::HeightFilter>();
    }
    if (use_footprint_filter_) {
      footprint_filter_ = std::make_unique<filters::FootprintFilter>();
    }
    if (use_obstacle_layer_) {
      obstacle_layer_ = std::make_unique<layers::ObstacleLayer>();
      obstacle_layer_->setSize(size_x_, size_y_);
      obstacle_layer_->setResolution(resolution_);
      obstacle_layer_->setOrigin(origin_x_, origin_y_);
    }
    if (use_inflation_layer_) {
      inflation_layer_ = std::make_unique<layers::InflationLayer>();
      inflation_layer_->setSize(size_x_, size_y_);
      inflation_layer_->setResolution(resolution_);
      inflation_layer_->setOrigin(origin_x_, origin_y_);
    }

    // Configure layer-specific parameters
    configureLayerParameters();

    // Publishers
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1);
    marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("costmap_markers", 1);
    filtered_cloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>("filtered_pointcloud", 1);

    // Subscribers
    std::string pctopic = get_parameter("pointcloud_topic").as_string();
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(pctopic, 1,
                                                                    std::bind(&Costmap::pointCloudCallback, this, std::placeholders::_1));

    // Services
    clear_costmap_srv_ = create_service<std_srvs::srv::Trigger>(
        "clear_costmap", std::bind(&Costmap::clearCostmapCallback, this, std::placeholders::_1, std::placeholders::_2));

    // Update timer
    update_timer_ = create_wall_timer(std::chrono::duration<double>(1.0 / update_frequency_), std::bind(&Costmap::updateMap, this));

    RCLCPP_INFO(get_logger(), "Costmap initialized with dimensions %dx%d cells (%.2fx%.2fm)", size_x_, size_y_, width_meters_,
                height_meters_);
  }

  void Costmap::declareParameters()
  {
    // General parameters
    declare_parameter("global_frame", "map");
    declare_parameter("robot_base_frame", "base_link");
    declare_parameter("update_frequency", 5.0);
    declare_parameter("transform_tolerance", 0.5);

    // Map size parameters
    declare_parameter("width", 10.0);
    declare_parameter("height", 10.0);
    declare_parameter("resolution", 0.05);

    // Enable/disable features
    declare_parameter("use_height_filter", true);
    declare_parameter("use_footprint_filter", true);
    declare_parameter("use_obstacle_layer", true);
    declare_parameter("use_inflation_layer", true);

    // Height filter params
    declare_parameter("min_height", 0.1);
    declare_parameter("max_height", 2.0);

    // Footprint params
    declare_parameter("footprint_radius", 0.5);
    declare_parameter("is_circular_robot", true);

    // Obstacle layer params
    declare_parameter("obstacle_range", 2.5);
    declare_parameter("raytrace_range", 3.0);

    // Inflation layer params
    declare_parameter("inflation_radius", 0.55);
    declare_parameter("cost_scaling_factor", 10.0);
    declare_parameter("pointcloud_topic", "lidar/point_cloud");
  }

  void Costmap::getParameters()
  {
    // General parameters
    global_frame_ = get_parameter("global_frame").as_string();
    robot_base_frame_ = get_parameter("robot_base_frame").as_string();
    update_frequency_ = get_parameter("update_frequency").as_double();
    transform_tolerance_ = get_parameter("transform_tolerance").as_double();

    // Map size parameters
    width_meters_ = get_parameter("width").as_double();
    height_meters_ = get_parameter("height").as_double();
    resolution_ = get_parameter("resolution").as_double();

    // Feature flags
    use_height_filter_ = get_parameter("use_height_filter").as_bool();
    use_footprint_filter_ = get_parameter("use_footprint_filter").as_bool();
    use_obstacle_layer_ = get_parameter("use_obstacle_layer").as_bool();
    use_inflation_layer_ = get_parameter("use_inflation_layer").as_bool();

    // Update derived properties
    updateGridProperties();

    // Configure filters
    if (use_height_filter_ && height_filter_) {
      height_filter_->setMinHeight(get_parameter("min_height").as_double());
      height_filter_->setMaxHeight(get_parameter("max_height").as_double());
    }

    if (use_footprint_filter_ && footprint_filter_) {
      if (get_parameter("is_circular_robot").as_bool()) {
        footprint_filter_->setCircularFootprint(get_parameter("footprint_radius").as_double());
      }
    }

    // Configure obstacle layer
    if (use_obstacle_layer_ && obstacle_layer_) {
      double obstacle_range = get_parameter("obstacle_range").as_double();
      double raytrace_range = get_parameter("raytrace_range").as_double();
      obstacle_layer_->setObstacleRange(obstacle_range);
      obstacle_layer_->setRaytraceRange(raytrace_range);
    }

    // Configure inflation layer
    if (use_inflation_layer_ && inflation_layer_) {
      double inflation_radius = get_parameter("inflation_radius").as_double();
      double cost_scaling_factor = get_parameter("cost_scaling_factor").as_double();
      inflation_layer_->setInflationRadius(inflation_radius);
      inflation_layer_->setCostScalingFactor(cost_scaling_factor);
    }
  }

  void Costmap::updateGridProperties()
  {
    // Validate input parameters
    if (width_meters_ <= 0.0 || height_meters_ <= 0.0 || resolution_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "Invalid map parameters: width=%.2f, height=%.2f, resolution=%.3f", width_meters_, height_meters_,
                   resolution_);
      throw std::invalid_argument("Invalid map parameters");
    }

    // Compute grid size (round up to ensure coverage)
    size_t new_size_x = static_cast<size_t>(std::ceil(width_meters_ / resolution_));
    size_t new_size_y = static_cast<size_t>(std::ceil(height_meters_ / resolution_));

    // Check for reasonable size
    constexpr size_t MAX_GRID_SIZE = 10000; // 10000 x 10000 max
    if (new_size_x > MAX_GRID_SIZE || new_size_y > MAX_GRID_SIZE) {
      RCLCPP_ERROR(get_logger(), "Grid size too large: %zu x %zu cells (max: %zu x %zu)", new_size_x, new_size_y, MAX_GRID_SIZE,
                   MAX_GRID_SIZE);
      throw std::invalid_argument("Grid size too large");
    }

    // Update sizes
    size_x_ = static_cast<unsigned int>(new_size_x);
    size_y_ = static_cast<unsigned int>(new_size_y);

    // Compute origins to center the map on the robot
    origin_x_ = -width_meters_ / 2.0;
    origin_y_ = -height_meters_ / 2.0;

    // Update actual width/height to match grid size
    width_meters_ = size_x_ * resolution_;
    height_meters_ = size_y_ * resolution_;

    // Update grid properties in all layers
    if (use_obstacle_layer_ && obstacle_layer_) {
      obstacle_layer_->setSize(size_x_, size_y_);
      obstacle_layer_->setResolution(resolution_);
      obstacle_layer_->setOrigin(origin_x_, origin_y_);
    }

    if (use_inflation_layer_ && inflation_layer_) {
      inflation_layer_->setSize(size_x_, size_y_);
      inflation_layer_->setResolution(resolution_);
      inflation_layer_->setOrigin(origin_x_, origin_y_);
    }

    try {
      // Pre-allocate with exact size to avoid reallocation
      costmap_.clear();
      costmap_.reserve(size_x_ * size_y_);
      costmap_.resize(size_x_ * size_y_, FREE_SPACE);

      RCLCPP_INFO(get_logger(), "Costmap configured: %dx%d cells (%.2fx%.2fm), resolution: %.3fm, origin: (%.2f,%.2f)", size_x_, size_y_,
                  width_meters_, height_meters_, resolution_, origin_x_, origin_y_);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Failed to allocate costmap: %s", e.what());
      throw;
    }
  }

  void Costmap::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (!pc_received_) {
      pc_received_ = true;
      RCLCPP_INFO(get_logger(), "Pointcloud received, starting costmap update");
    }

    // Convert to PCL
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *cloud);

    // Apply filters in sequence
    if (use_height_filter_ && height_filter_) {
      cloud = height_filter_->filter(cloud);
    }
    if (use_footprint_filter_ && footprint_filter_) {
      cloud = footprint_filter_->filter(cloud);
    }

    // Publish filtered cloud
    if (filtered_cloud_pub_->get_subscription_count() > 0) {
      sensor_msgs::msg::PointCloud2 filtered_msg;
      pcl::toROSMsg(*cloud, filtered_msg);
      filtered_msg.header = msg->header;
      filtered_cloud_pub_->publish(filtered_msg);
    }

    // Process in obstacle layer if enabled
    if (use_obstacle_layer_ && obstacle_layer_) {
      obstacle_layer_->processCloud(cloud);
    }
  }

  void Costmap::updateMap()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    // Get robot pose
    geometry_msgs::msg::TransformStamped transform;
    try {
      transform =
          tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero, tf2::durationFromSec(transform_tolerance_));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(get_logger(), "Could not get robot pose: %s", ex.what());
      return;
    }

    double robot_x = transform.transform.translation.x;
    double robot_y = transform.transform.translation.y;
    double robot_yaw = tf2::getYaw(transform.transform.rotation);

    // Reset costmap
    std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);

    // Update obstacle layer if enabled
    if (use_obstacle_layer_ && obstacle_layer_) {
      double min_x = origin_x_;
      double min_y = origin_y_;
      double max_x = origin_x_ + size_x_ * resolution_;
      double max_y = origin_y_ + size_y_ * resolution_;

      obstacle_layer_->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);

      unsigned int min_i, min_j, max_i, max_j;
      worldToMap(min_x, min_y, min_i, min_j);
      worldToMap(max_x, max_y, max_i, max_j);

      obstacle_layer_->updateCosts(costmap_, min_i, min_j, max_i, max_j);
    }

    // Update inflation layer if enabled
    if (use_inflation_layer_ && inflation_layer_) {
      double min_x = origin_x_;
      double min_y = origin_y_;
      double max_x = origin_x_ + size_x_ * resolution_;
      double max_y = origin_y_ + size_y_ * resolution_;

      inflation_layer_->updateBounds(robot_x, robot_y, robot_yaw, &min_x, &min_y, &max_x, &max_y);

      unsigned int min_i, min_j, max_i, max_j;
      worldToMap(min_x, min_y, min_i, min_j);
      worldToMap(max_x, max_y, max_i, max_j);

      inflation_layer_->updateCosts(costmap_, min_i, min_j, max_i, max_j);
    }

    publishMap();
    publishMarkers();
  }

  void Costmap::publishMap()
  {
    auto msg = std::make_unique<nav_msgs::msg::OccupancyGrid>();
    msg->header.stamp = now();
    msg->header.frame_id = global_frame_;
    msg->info.resolution = resolution_;
    msg->info.width = size_x_;
    msg->info.height = size_y_;
    msg->info.origin.position.x = origin_x_;
    msg->info.origin.position.y = origin_y_;
    msg->info.origin.orientation.w = 1.0;

    msg->data.resize(size_x_ * size_y_);
    for (unsigned int i = 0; i < size_x_ * size_y_; ++i) {
      if (costmap_[i] == NO_INFORMATION) {
        msg->data[i] = -1;
      } else if (costmap_[i] == LETHAL_OBSTACLE) {
        msg->data[i] = 100;
      } else {
        msg->data[i] = static_cast<int8_t>((costmap_[i] * 100) / 253);
      }
    }

    map_pub_->publish(std::move(msg));
  }

  void Costmap::publishMarkers()
  {
    // Count markers first to pre-allocate
    size_t num_obstacle_markers = 0;
    size_t num_inflation_markers = 0;

    for (unsigned int j = 0; j < size_y_; ++j) {
      for (unsigned int i = 0; i < size_x_; ++i) {
        unsigned int index = i + j * size_x_;
        if (costmap_[index] == LETHAL_OBSTACLE) {
          ++num_obstacle_markers;
        } else if (costmap_[index] == INSCRIBED_INFLATED_OBSTACLE) {
          ++num_inflation_markers;
        }
      }
    }

    // Create markers using smart pointers
    auto obstacle_marker = std::make_unique<visualization_msgs::msg::Marker>();
    auto inflation_marker = std::make_unique<visualization_msgs::msg::Marker>();

    // Setup obstacle marker
    obstacle_marker->header.frame_id = global_frame_;
    obstacle_marker->header.stamp = get_clock()->now();
    obstacle_marker->ns = "obstacles";
    obstacle_marker->id = 0;
    obstacle_marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
    obstacle_marker->action = visualization_msgs::msg::Marker::ADD;
    obstacle_marker->scale.x = resolution_;
    obstacle_marker->scale.y = resolution_;
    obstacle_marker->scale.z = 0.1;
    obstacle_marker->color.r = 1.0;
    obstacle_marker->color.a = 0.5;
    obstacle_marker->points.reserve(num_obstacle_markers);

    // Setup inflation marker
    inflation_marker->header = obstacle_marker->header;
    inflation_marker->ns = "inflation";
    inflation_marker->id = 1;
    inflation_marker->type = visualization_msgs::msg::Marker::CUBE_LIST;
    inflation_marker->action = visualization_msgs::msg::Marker::ADD;
    inflation_marker->scale = obstacle_marker->scale;
    inflation_marker->color.b = 1.0;
    inflation_marker->color.a = 0.3;
    inflation_marker->points.reserve(num_inflation_markers);

    // Fill marker points
    for (unsigned int j = 0; j < size_y_; ++j) {
      for (unsigned int i = 0; i < size_x_; ++i) {
        unsigned int index = i + j * size_x_;
        if (costmap_[index] == LETHAL_OBSTACLE || costmap_[index] == INSCRIBED_INFLATED_OBSTACLE) {
          geometry_msgs::msg::Point p;
          mapToWorld(i, j, p.x, p.y);
          p.z = 0.05; // Half height of marker

          if (costmap_[index] == LETHAL_OBSTACLE) {
            obstacle_marker->points.push_back(p);
          } else {
            inflation_marker->points.push_back(p);
          }
        }
      }
    }

    // Create and publish marker array
    auto marker_array = std::make_unique<visualization_msgs::msg::MarkerArray>();
    marker_array->markers.reserve(2); // We know we have exactly 2 markers
    marker_array->markers.push_back(*obstacle_marker);
    marker_array->markers.push_back(*inflation_marker);

    marker_pub_->publish(std::move(marker_array));
  }

  void Costmap::worldToMap(double wx, double wy, unsigned int& mx, unsigned int& my) const
  {
    // First translate to origin
    wx -= origin_x_;
    wy -= origin_y_;

    // Convert to grid coordinates with y-axis inversion
    mx = static_cast<unsigned int>(wx / resolution_);
    my = size_y_ - 1 - static_cast<unsigned int>(wy / resolution_); // Invert y-axis
  }

  void Costmap::mapToWorld(unsigned int mx, unsigned int my, double& wx, double& wy) const
  {
    // Convert from grid coordinates with y-axis inversion
    wx = origin_x_ + (mx + 0.5) * resolution_;
    wy = origin_y_ + ((size_y_ - 1 - my) + 0.5) * resolution_; // Invert y-axis
  }

  void Costmap::clearCostmapCallback(const std::shared_ptr<std_srvs::srv::Trigger::Request>,
                                     std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::fill(costmap_.begin(), costmap_.end(), FREE_SPACE);
    response->success = true;
    response->message = "Costmap cleared";
  }

  void Costmap::configureLayerParameters()
  {
    // Configure filters
    if (use_height_filter_ && height_filter_) {
      height_filter_->setMinHeight(get_parameter("min_height").as_double());
      height_filter_->setMaxHeight(get_parameter("max_height").as_double());
    }

    if (use_footprint_filter_ && footprint_filter_) {
      if (get_parameter("is_circular_robot").as_bool()) {
        footprint_filter_->setCircularFootprint(get_parameter("footprint_radius").as_double());
      }
    }

    // Configure obstacle layer
    if (use_obstacle_layer_ && obstacle_layer_) {
      double obstacle_range = get_parameter("obstacle_range").as_double();
      double raytrace_range = get_parameter("raytrace_range").as_double();
      obstacle_layer_->setObstacleRange(obstacle_range);
      obstacle_layer_->setRaytraceRange(raytrace_range);
    }

    // Configure inflation layer
    if (use_inflation_layer_ && inflation_layer_) {
      double inflation_radius = get_parameter("inflation_radius").as_double();
      double cost_scaling_factor = get_parameter("cost_scaling_factor").as_double();
      inflation_layer_->setInflationRadius(inflation_radius);
      inflation_layer_->setCostScalingFactor(cost_scaling_factor);
    }
  }

} // namespace m4_costmap

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(m4_costmap::Costmap)