#pragma once

#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

namespace m4_costmap
{

  class CostmapLayer
  {
  public:
    static const unsigned char NO_INFORMATION = 255;
    static const unsigned char LETHAL_OBSTACLE = 254;
    static const unsigned char INSCRIBED_INFLATED_OBSTACLE = 253;
    static const unsigned char FREE_SPACE = 0;

    CostmapLayer();
    virtual ~CostmapLayer() = default;

    // Update the layer's costs
    virtual void updateCosts(std::vector<unsigned char>& master_grid, int min_i, int min_j, int max_i, int max_j) = 0;

    // Update bounds of data that need to be updated
    virtual void updateBounds(double robot_x, double robot_y, double robot_yaw, double* min_x, double* min_y, double* max_x,
                              double* max_y) = 0;

    // Whether this layer is enabled
    virtual bool isEnabled() const { return enabled_; }
    virtual void enable() { enabled_ = true; }
    virtual void disable() { enabled_ = false; }

    // Get the layer name
    std::string getName() const { return name_; }

    // Grid property setters
    void setSize(unsigned int size_x, unsigned int size_y)
    {
      size_x_ = size_x;
      size_y_ = size_y;
    }

    void setResolution(double resolution) { resolution_ = resolution; }

    void setOrigin(double origin_x, double origin_y)
    {
      origin_x_ = origin_x;
      origin_y_ = origin_y;
    }

    // Grid property getters
    unsigned int getSizeX() const { return size_x_; }
    unsigned int getSizeY() const { return size_y_; }
    double getResolution() const { return resolution_; }
    double getOriginX() const { return origin_x_; }
    double getOriginY() const { return origin_y_; }

  protected:
    bool enabled_;
    std::string name_;

    // Map properties (shared with parent costmap)
    unsigned int size_x_;
    unsigned int size_y_;
    double resolution_;
    double origin_x_;
    double origin_y_;
  };

} // namespace m4_costmap