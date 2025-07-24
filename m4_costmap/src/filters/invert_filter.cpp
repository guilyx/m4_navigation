#include "m4_costmap/filters/invert_filter.hpp"

namespace m4_costmap
{
  namespace filters
  {

    InvertFilter::InvertFilter() : invert_x_(false), invert_y_(false), invert_z_(false)
    {
      name_ = "invert";
      enabled_ = true;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr InvertFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
      if (!enabled_ || (!invert_x_ && !invert_y_ && !invert_z_)) {
        return std::make_shared<pcl::PointCloud<pcl::PointXYZ>>(*cloud);
      }

      auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      filtered_cloud->header = cloud->header;
      filtered_cloud->points.reserve(cloud->points.size());

      for (const auto& point : cloud->points) {
        pcl::PointXYZ new_point;
        new_point.x = invert_x_ ? -point.x : point.x;
        new_point.y = invert_y_ ? -point.y : point.y;
        new_point.z = invert_z_ ? -point.z : point.z;
        filtered_cloud->points.push_back(new_point);
      }

      filtered_cloud->width = filtered_cloud->points.size();
      filtered_cloud->height = 1;
      filtered_cloud->is_dense = cloud->is_dense;

      return filtered_cloud;
    }

  } // namespace filters
} // namespace m4_costmap