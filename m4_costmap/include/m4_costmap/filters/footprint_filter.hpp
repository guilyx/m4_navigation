#pragma once

#include "geometry_msgs/msg/polygon.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace m4_costmap
{
  namespace filters
  {

    class FootprintFilter
    {
    public:
      FootprintFilter();
      ~FootprintFilter() = default;

      void setFootprint(const geometry_msgs::msg::Polygon& footprint);
      void setCircularFootprint(double radius);

      pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud);

    private:
      bool isPointInFootprint(double x, double y) const;
      bool isPointInCircle(double x, double y) const;

      geometry_msgs::msg::Polygon footprint_;
      double radius_;
      bool is_circular_;
    };

  } // namespace filters
} // namespace m4_costmap