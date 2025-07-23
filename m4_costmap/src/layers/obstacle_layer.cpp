#include "m4_costmap/layers/obstacle_layer.hpp"

#include <algorithm>

namespace m4_costmap
{
  namespace layers
  {

    ObstacleLayer::ObstacleLayer() :
        obstacle_range_(2.5),
        raytrace_range_(3.0),
        track_unknown_space_(false),
        has_new_data_(false),
        last_min_x_(0.0),
        last_min_y_(0.0),
        last_max_x_(0.0),
        last_max_y_(0.0)
    {
      name_ = "obstacles";
      enabled_ = true;
    }

    void ObstacleLayer::updateBounds([[maybe_unused]] double robot_x, [[maybe_unused]] double robot_y, [[maybe_unused]] double robot_yaw,
                                     double* min_x, double* min_y, double* max_x, double* max_y)
    {
      if (!enabled_ || !has_new_data_) {
        return;
      }

      *min_x = std::min(*min_x, last_min_x_);
      *min_y = std::min(*min_y, last_min_y_);
      *max_x = std::max(*max_x, last_max_x_);
      *max_y = std::max(*max_y, last_max_y_);

      has_new_data_ = false;
    }

    void ObstacleLayer::updateCosts(std::vector<unsigned char>& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
      if (!enabled_) {
        return;
      }

      // Update master grid with obstacle costs
      unsigned int size_x = max_i - min_i;
      unsigned int size_y = max_j - min_j;

      for (unsigned int j = 0; j < size_y; ++j) {
        unsigned int y = min_j + j;
        for (unsigned int i = 0; i < size_x; ++i) {
          unsigned int x = min_i + i;
          unsigned int index = x + y * size_x_;

          if (index >= obstacle_map_.size()) {
            continue;
          }

          unsigned char cost = obstacle_map_[index];
          if (cost == LETHAL_OBSTACLE) {
            master_grid[index] = LETHAL_OBSTACLE;
          } else if (track_unknown_space_ && cost == NO_INFORMATION) {
            master_grid[index] = NO_INFORMATION;
          }
        }
      }
    }

    void ObstacleLayer::processCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
      // Reset bounds
      last_min_x_ = std::numeric_limits<double>::max();
      last_min_y_ = std::numeric_limits<double>::max();
      last_max_x_ = std::numeric_limits<double>::lowest();
      last_max_y_ = std::numeric_limits<double>::lowest();

      // Validate grid size
      if (size_x_ == 0 || size_y_ == 0) {
        RCLCPP_ERROR_THROTTLE(rclcpp::get_logger("obstacle_layer"), *rclcpp::Clock::make_shared(),
                              1000, // throttle period in ms
                              "Invalid grid size: %dx%d", size_x_, size_y_);
        return;
      }

      // Ensure obstacle map is properly sized
      try {
        if (obstacle_map_.size() != size_x_ * size_y_) {
          obstacle_map_.clear();
          obstacle_map_.reserve(size_x_ * size_y_);
          obstacle_map_.resize(size_x_ * size_y_, FREE_SPACE);
        }
      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("obstacle_layer"), "Failed to resize obstacle map: %s", e.what());
        return;
      }

      // Process each point
      for (const auto& point : cloud->points) {
        // Skip invalid points
        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
          continue;
        }

        // Check if point is within range
        double distance = std::sqrt(point.x * point.x + point.y * point.y);
        if (distance > obstacle_range_) {
          continue;
        }

        // Convert to map coordinates
        if (point.x < origin_x_ || point.y < origin_y_ || point.x > origin_x_ + size_x_ * resolution_ ||
            point.y > origin_y_ + size_y_ * resolution_) {
          continue;
        }

        // Convert to grid coordinates - note the y-axis inversion to match ROS convention
        // In ROS, x points forward, y points left, while in grid coordinates y increases downward
        double wx = point.x;
        double wy = point.y;

        // First translate to origin
        wx -= origin_x_;
        wy -= origin_y_;

        // Convert to grid coordinates with y-axis inversion
        unsigned int mx = static_cast<unsigned int>(wx / resolution_);
        unsigned int my = size_y_ - 1 - static_cast<unsigned int>(wy / resolution_); // Invert y-axis

        // Update bounds using world coordinates
        last_min_x_ = std::min(last_min_x_, static_cast<double>(point.x));
        last_min_y_ = std::min(last_min_y_, static_cast<double>(point.y));
        last_max_x_ = std::max(last_max_x_, static_cast<double>(point.x));
        last_max_y_ = std::max(last_max_y_, static_cast<double>(point.y));

        // Bounds check before marking obstacle
        if (mx < size_x_ && my < size_y_) {
          unsigned int index = mx + my * size_x_;
          if (index < obstacle_map_.size()) {
            obstacle_map_[index] = LETHAL_OBSTACLE;
          }
        }
      }

      has_new_data_ = true;
    }

  } // namespace layers
} // namespace m4_costmap