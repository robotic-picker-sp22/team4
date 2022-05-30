#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "tf/transform_listener.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"

namespace perception {
class Cropper {
 public:
  Cropper(const ros::Publisher& pub);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void CropHelper(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud);
 private:
  ros::Publisher pub_;
  tf::StampedTransform transform;
};
}  // namespace perception