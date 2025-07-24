#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace m4_costmap
{

  class PointCloudFilter
  {
  public:
    PointCloudFilter() : enabled_(true) {}
    virtual ~PointCloudFilter() = default;

    // Pure virtual filter function that must be implemented by derived classes
    virtual pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) = 0;

    // Enable/disable the filter
    virtual void enable() { enabled_ = true; }
    virtual void disable() { enabled_ = false; }
    virtual bool isEnabled() const { return enabled_; }

    // Get filter name
    std::string getName() const { return name_; }

  protected:
    bool enabled_;
    std::string name_;
  };

} // namespace m4_costmap