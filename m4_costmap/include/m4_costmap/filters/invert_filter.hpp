#pragma once

#include "m4_costmap/filters/point_cloud_filter.hpp"

namespace m4_costmap
{
  namespace filters
  {

    class InvertFilter : public PointCloudFilter
    {
    public:
      InvertFilter();
      ~InvertFilter() = default;

      pcl::PointCloud<pcl::PointXYZ>::Ptr filter(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud) override;

      // Setters for which axes to invert
      void setInvertX(bool invert) { invert_x_ = invert; }
      void setInvertY(bool invert) { invert_y_ = invert; }
      void setInvertZ(bool invert) { invert_z_ = invert; }

      // Getters for current inversion state
      bool getInvertX() const { return invert_x_; }
      bool getInvertY() const { return invert_y_; }
      bool getInvertZ() const { return invert_z_; }

    private:
      bool invert_x_; // Whether to invert X coordinates
      bool invert_y_; // Whether to invert Y coordinates
      bool invert_z_; // Whether to invert Z coordinates
    };

  } // namespace filters
} // namespace m4_costmap