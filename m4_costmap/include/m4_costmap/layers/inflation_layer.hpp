#pragma once

#include "m4_costmap/costmap_layer.hpp"

#include <queue>

namespace m4_costmap
{
  namespace layers
  {

    struct CellData {
      unsigned int index_;
      unsigned int x_, y_;         // Current cell
      unsigned int src_x_, src_y_; // Source cell that caused inflation
    };

    class InflationLayer : public CostmapLayer
    {
    public:
      InflationLayer();
      ~InflationLayer() = default;

      void updateCosts(std::vector<unsigned char>& master_grid, int min_i, int min_j, int max_i, int max_j) override;

      void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                        double* max_y) override;

      // Parameter setters
      void setInflationRadius(double radius)
      {
        inflation_radius_ = radius;
        cache_valid_ = false; // Force cache recomputation
      }

      void setCostScalingFactor(double factor)
      {
        cost_scaling_factor_ = factor;
        cache_valid_ = false; // Force cache recomputation
      }

    private:
      void computeCaches();
      void inflateArea(int min_i, int min_j, int max_i, int max_j, std::vector<unsigned char>& master_grid);
      inline void enqueueCell(unsigned int index, unsigned int mx, unsigned int my, unsigned int src_x, unsigned int src_y);

      // Parameters
      double inflation_radius_;
      double cost_scaling_factor_;

      // Cache for cost calculations
      std::vector<unsigned char> cached_costs_;
      bool cache_valid_;

      // Queue for propagating inflation
      std::queue<CellData> inflation_queue_;
    };

  } // namespace layers
} // namespace m4_costmap