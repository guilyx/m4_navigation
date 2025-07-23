#pragma once

#include "m4_costmap/costmap_layer.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace m4_costmap
{
  namespace layers
  {

    class ObstacleLayer : public CostmapLayer
    {
    public:
      ObstacleLayer();
      ~ObstacleLayer() = default;

      void updateCosts(std::vector<unsigned char>& master_grid, int min_i, int min_j, int max_i, int max_j) override;

      void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                        double* max_y) override;

      // Process new pointcloud data
      void processCloud(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

      // Parameter setters
      void setObstacleRange(double range) { obstacle_range_ = range; }
      void setRaytraceRange(double range) { raytrace_range_ = range; }
      void setTrackUnknownSpace(bool track) { track_unknown_space_ = track; }

    private:
      // Parameters
      double obstacle_range_; // Max range to mark obstacles
      double raytrace_range_; // Range to clear obstacles
      bool track_unknown_space_;

      // Internal obstacle map
      std::vector<unsigned char> obstacle_map_;
      bool has_new_data_;

      // Bounds of last update
      double last_min_x_, last_min_y_, last_max_x_, last_max_y_;
    };

  } // namespace layers
} // namespace m4_costmap