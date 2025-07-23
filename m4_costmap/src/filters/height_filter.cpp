#include "m4_costmap/filters/height_filter.hpp"

namespace m4_costmap
{
  namespace filters
  {

    HeightFilter::HeightFilter() : min_height_(0.1), max_height_(2.0)
    {
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr HeightFilter::filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud)
    {
      auto filtered_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
      filtered_cloud->header = cloud->header;
      filtered_cloud->reserve(cloud->size());

      for (const auto& point : cloud->points) {
        if (point.z >= min_height_ && point.z <= max_height_) {
          filtered_cloud->push_back(point);
        }
      }

      return filtered_cloud;
    }

  } // namespace filters
} // namespace m4_costmap