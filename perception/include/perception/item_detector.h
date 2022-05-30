#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "pcl/segmentation/segment_differences.h"
#include "std_msgs/String.h"
#include "visualization_msgs/Marker.h"

namespace perception {

// Add function definitions here later

class ItemDetector {
 public:
  ItemDetector(ros::Publisher object_cloud, ros::Publisher diff_image, ros::Publisher marker_pub, ros::Publisher container_state_pub);
  void Callback(const sensor_msgs::PointCloud2& msg);
  void AddItem(const std_msgs::String item_name);
  void updateContainerState(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, std::string name);

 private:
  bool init_set;
  sensor_msgs::PointCloud2 init_shelf;
  sensor_msgs::PointCloud2 next_shelf;
  ros::Publisher object_cloud;
  ros::Publisher diff_image;
  ros::Publisher marker_pub_;
  ros::Publisher container_state_pub_;
  sensor_msgs::PointCloud2 msg_out;
};
}  // namespace perception