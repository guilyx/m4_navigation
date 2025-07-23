#include "m4_costmap/costmap_layer.hpp"

namespace m4_costmap
{

  // Define static constants
  const unsigned char CostmapLayer::NO_INFORMATION;
  const unsigned char CostmapLayer::LETHAL_OBSTACLE;
  const unsigned char CostmapLayer::INSCRIBED_INFLATED_OBSTACLE;
  const unsigned char CostmapLayer::FREE_SPACE;

  CostmapLayer::CostmapLayer() : enabled_(true), name_(""), size_x_(0), size_y_(0), resolution_(0.0), origin_x_(0.0), origin_y_(0.0)
  {
  }

} // namespace m4_costmap