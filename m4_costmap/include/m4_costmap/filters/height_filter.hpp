#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace m4_costmap
{
  namespace filters
  {

    class HeightFilter
    {
    public:
      HeightFilter();
      ~HeightFilter() = default;

      void setMinHeight(double min_height) { min_height_ = min_height; }
      void setMaxHeight(double max_height) { max_height_ = max_height; }

      pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

    private:
      double min_height_;
      double max_height_;
    };

  } // namespace filters
} // namespace m4_costmap