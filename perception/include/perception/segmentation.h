#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/segmentation/extract_clusters.h"
#include "perception/object.h"
#include "perception/object_recognizer.h"

namespace perception {

// Add function definitions here later
void SegmentObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        std::vector<Object>* objects,
                        std::string method);
void SegmentBinObjects(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        std::vector<pcl::PointIndices>* indices,
                        std::string method);

class Segmenter {
 public:
    Segmenter(const ros::Publisher& points_pub, const ros::Publisher& marker_pub, const ObjectRecognizer& recognizer, std::string algo);
    void Callback(const sensor_msgs::PointCloud2& msg);
    // Does a complete bin segmentation pipeline.
    //
    // Args:
    //  cloud: The point cloud with the bin and the objects in it.
    //  objects: The output objects.

 private:
  ros::Publisher points_pub_;
  ros::Publisher marker_pub_;
  ObjectRecognizer recognizer_;
  std::string algo_;
};
}  // namespace perception