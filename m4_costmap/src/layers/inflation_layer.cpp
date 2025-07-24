#include "m4_costmap/layers/inflation_layer.hpp"

#include <algorithm>
#include <cmath>

namespace m4_costmap
{
  namespace layers
  {

    InflationLayer::InflationLayer() : inflation_radius_(0.55), cost_scaling_factor_(10.0), cache_valid_(false)
    {
      name_ = "inflation";
      enabled_ = true;
    }

    void InflationLayer::updateBounds([[maybe_unused]] double robot_x, [[maybe_unused]] double robot_y, [[maybe_unused]] double robot_yaw,
                                      double* min_x, double* min_y, double* max_x, double* max_y)
    {
      if (!enabled_) {
        return;
      }

      *min_x -= inflation_radius_;
      *min_y -= inflation_radius_;
      *max_x += inflation_radius_;
      *max_y += inflation_radius_;
    }

    void InflationLayer::updateCosts(std::vector<unsigned char>& master_grid, int min_i, int min_j, int max_i, int max_j)
    {
      if (!enabled_) {
        return;
      }

      // Validate map dimensions
      if (size_x_ == 0 || size_y_ == 0 || resolution_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Invalid map dimensions or resolution: size_x=%u, size_y=%u, resolution=%.3f",
                     size_x_, size_y_, resolution_);
        return;
      }

      // Validate grid size matches dimensions
      size_t expected_size = size_x_ * size_y_;
      if (master_grid.size() != expected_size) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Grid size mismatch: got %zu cells, expected %zu cells (%u x %u)",
                     master_grid.size(), expected_size, size_x_, size_y_);
        return;
      }

      // Compute cost decay lookup table if needed
      if (!cache_valid_) {
        computeCaches();
      }

      // Validate cache computation succeeded
      if (!cache_valid_) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Failed to compute inflation caches");
        return;
      }

      // Inflate obstacles
      inflateArea(min_i, min_j, max_i, max_j, master_grid);
    }

    void InflationLayer::computeCaches()
    {
      // Validate parameters
      if (resolution_ <= 0.0 || inflation_radius_ <= 0.0) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Invalid parameters: resolution=%.3f, inflation_radius=%.3f", resolution_,
                     inflation_radius_);
        return;
      }

      // Cache cell costs for inflation
      int cell_inflation_radius = static_cast<int>(inflation_radius_ / resolution_);

      // Check for reasonable size
      constexpr int MAX_INFLATION_CELLS = 1000; // Maximum inflation radius in cells
      if (cell_inflation_radius <= 0 || cell_inflation_radius > MAX_INFLATION_CELLS) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Invalid inflation radius in cells: %d (max: %d)", cell_inflation_radius,
                     MAX_INFLATION_CELLS);
        return;
      }

      try {
        cached_costs_.clear();
        cached_costs_.reserve(cell_inflation_radius + 2);
        cached_costs_.resize(cell_inflation_radius + 2);

        // Compute cost for each cell based on distance
        for (int i = 0; i <= cell_inflation_radius + 1; ++i) {
          double distance = i * resolution_;
          if (distance > inflation_radius_) {
            cached_costs_[i] = FREE_SPACE;
            continue;
          }

          // Cost decay formula: exp(-1.0 * cost_scaling_factor_ * (distance / inflation_radius_))
          double factor = exp(-1.0 * cost_scaling_factor_ * (distance / inflation_radius_));
          double cost = factor * (INSCRIBED_INFLATED_OBSTACLE - 1);
          cached_costs_[i] = static_cast<unsigned char>(std::min(cost, static_cast<double>(INSCRIBED_INFLATED_OBSTACLE - 1)));
        }

        cache_valid_ = true;
        RCLCPP_DEBUG(rclcpp::get_logger("inflation_layer"), "Computed inflation cache with radius %d cells (%.2fm)", cell_inflation_radius,
                     inflation_radius_);

      } catch (const std::exception& e) {
        RCLCPP_ERROR(rclcpp::get_logger("inflation_layer"), "Failed to compute inflation cache: %s", e.what());
        cache_valid_ = false;
      }
    }

    void InflationLayer::inflateArea(int min_i, int min_j, int max_i, int max_j, std::vector<unsigned char>& master_grid)
    {
      // Validate and clamp bounds to grid size
      min_i = std::max(0, min_i);
      min_j = std::max(0, min_j);
      max_i = std::min(static_cast<int>(size_x_), max_i);
      max_j = std::min(static_cast<int>(size_y_), max_j);

      if (min_i >= max_i || min_j >= max_j) {
        return; // Invalid bounds
      }

      // Get master grid dimensions
      unsigned int master_size_x = max_i - min_i;
      unsigned int master_size_y = max_j - min_j;

      // Clear inflation queue
      std::queue<CellData> empty;
      std::swap(inflation_queue_, empty);

      // Find cells to inflate
      for (int j = min_j; j < max_j; ++j) {
        for (int i = min_i; i < max_i; ++i) {
          unsigned int master_index = i + j * master_size_x;
          if (master_index < master_grid.size() && master_grid[master_index] == LETHAL_OBSTACLE) {
            enqueueCell(master_index, i, j, i, j);
          }
        }
      }

      // Process inflation
      while (!inflation_queue_.empty()) {
        const CellData& cell = inflation_queue_.front();

        // Validate cell coordinates
        if (cell.x_ >= size_x_ || cell.y_ >= size_y_) {
          inflation_queue_.pop();
          continue;
        }

        // Calculate distance to nearest obstacle
        int dx = cell.x_ - cell.src_x_;
        int dy = cell.y_ - cell.src_y_;
        double distance = hypot(dx * resolution_, dy * resolution_);

        // Get cost from lookup table
        auto cell_distance = static_cast<size_t>(distance / resolution_);
        if (cell_distance >= cached_costs_.size()) {
          inflation_queue_.pop();
          continue;
        }

        unsigned char cost = cached_costs_[cell_distance];
        if (cost == FREE_SPACE) {
          inflation_queue_.pop();
          continue;
        }

        // Update cost if higher than existing
        unsigned int master_index = cell.x_ + cell.y_ * master_size_x;
        if (master_index >= master_grid.size()) {
          inflation_queue_.pop();
          continue;
        }

        if (master_grid[master_index] < cost) {
          master_grid[master_index] = cost;

          // Propagate to neighbors (using grid coordinates)
          int mx = cell.x_;
          int my = cell.y_;

          // Check each neighbor with proper bounds checking
          if (mx > 0) {
            unsigned int left_index = (mx - 1) + my * master_size_x;
            if (left_index < master_grid.size()) {
              enqueueCell(left_index, mx - 1, my, cell.src_x_, cell.src_y_);
            }
          }
          if (mx < static_cast<int>(size_x_ - 1)) {
            unsigned int right_index = (mx + 1) + my * master_size_x;
            if (right_index < master_grid.size()) {
              enqueueCell(right_index, mx + 1, my, cell.src_x_, cell.src_y_);
            }
          }
          if (my > 0) {
            unsigned int up_index = mx + (my - 1) * master_size_x;
            if (up_index < master_grid.size()) {
              enqueueCell(up_index, mx, my - 1, cell.src_x_, cell.src_y_);
            }
          }
          if (my < static_cast<int>(size_y_ - 1)) {
            unsigned int down_index = mx + (my + 1) * master_size_x;
            if (down_index < master_grid.size()) {
              enqueueCell(down_index, mx, my + 1, cell.src_x_, cell.src_y_);
            }
          }
        }

        inflation_queue_.pop();
      }
    }

    inline void InflationLayer::enqueueCell(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y)
    {
      CellData cell = { index, mx, my, src_x, src_y };
      inflation_queue_.push(cell);
    }

  } // namespace layers
} // namespace m4_costmap