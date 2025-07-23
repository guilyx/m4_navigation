#include "m4_costmap/filters/footprint_filter.hpp"

#include <cmath>

namespace m4_costmap
{
  namespace filters
  {

    FootprintFilter::FootprintFilter() : radius_(0.5), is_circular_(true)
    {
    }

    void FootprintFilter::setFootprint(const geometry_msgs::msg::Polygon& footprint)
    {
      footprint_ = footprint;
      is_circular_ = false;
    }

    void FootprintFilter::setCircularFootprint(double radius)
    {
      radius_ = radius;
      is_circular_ = true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr FootprintFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
      auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      filtered_cloud->header = cloud->header;
      filtered_cloud->reserve(cloud->size());

      for (const auto& point : cloud->points) {
        if (is_circular_) {
          if (!isPointInCircle(point.x, point.y)) {
            filtered_cloud->push_back(point);
          }
        } else {
          if (!isPointInFootprint(point.x, point.y)) {
            filtered_cloud->push_back(point);
          }
        }
      }

      return filtered_cloud;
    }

    bool FootprintFilter::isPointInCircle(double x, double y) const
    {
      return (x * x + y * y) <= (radius_ * radius_);
    }

    bool FootprintFilter::isPointInFootprint(double x, double y) const
    {
      if (footprint_.points.size() < 3) {
        return false;
      }

      bool inside = false;
      size_t i, j;
      for (i = 0, j = footprint_.points.size() - 1; i < footprint_.points.size(); j = i++) {
        const auto& pi = footprint_.points[i];
        const auto& pj = footprint_.points[j];

        if (((pi.y > y) != (pj.y > y)) && (x < (pj.x - pi.x) * (y - pi.y) / (pj.y - pi.y) + pi.x)) {
          inside = !inside;
        }
      }

      return inside;
    }

  } // namespace filters
} // namespace m4_costmap