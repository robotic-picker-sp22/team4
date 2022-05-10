#include "perception/feature_extraction.h"

#include <algorithm> // std::min and std::max
// #include <std_msgs/Float64.h>

#include "perception/object.h"
#include "perception_msgs/ObjectFeatures.h"
#include "ros/ros.h"

namespace perception {
void ExtractSizeFeatures(const perception::Object& object,
                         perception_msgs::ObjectFeatures* features) {
  // "x" dimension is always the smallest of x and y to account for rotations.
  // z always points up.
  
  float dim_x = object.dimensions.x;
  float dim_y = object.dimensions.y;
  float dim_z = object.dimensions.z;
  if (dim_x > dim_y) {
    dim_x = object.dimensions.y;
    dim_y = object.dimensions.x;
  }
  features->names.push_back("box_dim_x");
  features->values.push_back(dim_x);
  features->names.push_back("box_dim_y");
  features->values.push_back(dim_y);
  features->names.push_back("box_dim_z");
  features->values.push_back(dim_z);
}
}  // namespace perception