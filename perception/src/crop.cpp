// TODO: add includes, etc.
#include "perception/crop.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/crop_box.h"
#include "pcl/common/common.h"
#include "pcl_ros/transforms.h"



// Add these typedefs after your #includes
typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
Cropper::Cropper(const ros::Publisher& pub) : pub_(pub) {
  if (!getenv("ROBOT")) {
    tf::TransformListener tf_listener;
    tf_listener.waitForTransform("base_link", "head_camera_rgb_optical_frame",                     
                                 ros::Time(0), ros::Duration(5.0));
    // TODO: Move next line to other method and check if it's fast enough
    tf_listener.lookupTransform("base_link", "head_camera_rgb_optical_frame",                    
                                    ros::Time(0), this->transform);
  }
}

void Cropper::Callback(const sensor_msgs::PointCloud2& msg) {
  PointCloudC::Ptr cloud(new PointCloudC());
  if (!getenv("ROBOT")) {
    sensor_msgs::PointCloud2 cloud_out;
    pcl_ros::transformPointCloud("base_link", transform, msg, cloud_out);
    pcl::fromROSMsg(cloud_out, *cloud);
  } else {
    pcl::fromROSMsg(msg, *cloud);
  }

  PointCloudC::Ptr cropped_cloud(new PointCloudC());
  Cropper::CropHelper(cloud, cropped_cloud);
  ROS_INFO("Cropped to %ld points", cropped_cloud->size());
  sensor_msgs::PointCloud2 msg_out;
  pcl::toROSMsg(*cropped_cloud, msg_out);
  PointC min_pcl;
  PointC max_pcl;
  pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
  ROS_INFO("min: %f, max: %f", min_pcl.x, max_pcl.x);
  pub_.publish(msg_out);
}

void Cropper::CropHelper(PointCloudC::Ptr cloud, PointCloudC::Ptr cropped_cloud) {
  double min_x, min_y, min_z, max_x, max_y, max_z, rot_x, rot_y, rot_z;
  ros::param::param("crop_min_x", min_x, 1.2);
  ros::param::param("crop_min_y", min_y, -0.5);
  ros::param::param("crop_min_z", min_z, 0.55);
  ros::param::param("crop_max_x", max_x, 1.53);
  ros::param::param("crop_max_y", max_y, 0.56);
  ros::param::param("crop_max_z", max_z, 0.76);
  ros::param::param("crop_rot_x", rot_x, 0.0);
  ros::param::param("crop_rot_y", rot_y, 0.0);
  ros::param::param("crop_rot_z", rot_z, 0.0);

  Eigen::Vector4f min_pt(min_x, min_y, min_z, 1);
  Eigen::Vector4f max_pt(max_x, max_y, max_z, 1);
  Eigen::Vector3f rot(rot_x, rot_y, rot_z);
  pcl::CropBox<PointC> crop;
  crop.setInputCloud(cloud);
  crop.setMin(min_pt);
  crop.setMax(max_pt);
  crop.setRotation(rot);
  crop.filter(*cropped_cloud);
}
}