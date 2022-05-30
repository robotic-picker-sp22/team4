#include "perception/item_detector.h"

#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl/segmentation/segment_differences.h"
#include "pcl/filters/extract_indices.h"
#include "perception/object.h"
#include <pcl/common/common.h>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl_ros/transforms.h"
#include "math.h"

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

double calculateVolume(const geometry_msgs::Vector3& v1) {
    return v1.x * v1.y * v1.z;
}



namespace perception {

void GetAxisAlignedBoundingBox(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                            geometry_msgs::Pose* pose,
                            geometry_msgs::Vector3* dimensions) {
    PointC min_pcl;
    PointC max_pcl;
    pcl::getMinMax3D<PointC>(*cloud, min_pcl, max_pcl);
    pose->position.x = (min_pcl.x + max_pcl.x) / 2.0;
    pose->position.y = (min_pcl.y + max_pcl.y) / 2.0;
    pose->position.z = (min_pcl.z + max_pcl.z) / 2.0;
    dimensions->x = std::abs(min_pcl.x - max_pcl.x);
    dimensions->y = std::abs(min_pcl.y - max_pcl.y);
    dimensions->z = std::abs(min_pcl.z - max_pcl.z);
    pose->orientation.x = 0;
    pose->orientation.y = 0;
    pose->orientation.z = 0;
    pose->orientation.w = 1;
}

void SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices) {
    double cluster_tolerance;
    int min_cluster_size, max_cluster_size;
    ros::param::param("ec_cluster_tolerance", cluster_tolerance, 0.01);
    ros::param::param("ec_min_cluster_size", min_cluster_size, 10);
    ros::param::param("ec_max_cluster_size", max_cluster_size, 10000);
    pcl::EuclideanClusterExtraction<PointC> euclid;
    euclid.setInputCloud(cloud);
    // euclid.setIndices(inside_bin_indices); // << TODO: is cloud already cropped or do we get indices for the crop?
    euclid.setClusterTolerance(cluster_tolerance);
    euclid.setMinClusterSize(min_cluster_size);
    euclid.setMaxClusterSize(max_cluster_size);
    euclid.extract(*indices);
    // Find the size of the smallest and the largest object,
    // where size = number of points in the cluster
    size_t min_size = std::numeric_limits<size_t>::max();
    size_t max_size = std::numeric_limits<size_t>::min();
    for (size_t i = 0; i < indices->size(); ++i) {
        // TODO: implement this
        size_t cluster_size = indices->at(i).indices.size();
        min_size = std::min(cluster_size, min_size);
        max_size = std::max(cluster_size, max_size);
    }

    ROS_INFO("Found %ld objects, min size: %ld, max size: %ld",
            indices->size(), min_size, max_size);
}

void SegmentObjects(PointCloudC::Ptr cloud,
                        std::vector<Object>* objects) {
// Same as callback, but with visualization code removed.
    std::vector<pcl::PointIndices> object_indices;
    SegmentBinObjects(cloud, &object_indices);

    for (size_t i = 0; i < object_indices.size(); ++i) {
        // Reify indices into a point cloud of the object.
        pcl::PointIndices::Ptr indices(new pcl::PointIndices);
        *indices = object_indices[i];
        PointCloudC::Ptr object_cloud(new PointCloudC());
        // TODO: fill in object_cloud using indices
        pcl::ExtractIndices<PointC> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.filter(*object_cloud);
        perception::Object newObject;
        newObject.cloud = object_cloud;
        GetAxisAlignedBoundingBox(object_cloud, &newObject.pose,
                                    &newObject.dimensions);
        objects->push_back(newObject);
    }
}



ItemDetector::ItemDetector(ros::Publisher object_cloud, ros::Publisher diff_image, ros::Publisher marker_pub, ros::Publisher container_state_pub) : 
    object_cloud(object_cloud), diff_image(diff_image), marker_pub_(marker_pub), container_state_pub_(container_state_pub), init_set(false) {
}

void ItemDetector::Callback(const sensor_msgs::PointCloud2& msg) {
  if (!init_set) {
      init_shelf = msg;
      init_set = true;
      ROS_INFO("Setting the shelf!");
      msg_out = init_shelf;
  } else {
      next_shelf = msg;
  }
  object_cloud.publish(msg_out);
}

void ItemDetector::AddItem(std_msgs::String item_name) {
    // PointCloudC::Ptr cloud_filtered(new PointCloudC());
    PointCloudC::Ptr transformed_init_shelf(new PointCloudC());
    PointCloudC::Ptr transformed_next_shelf(new PointCloudC());
    if (!getenv("ROBOT")) {
        tf::TransformListener tf_listener;
        tf::StampedTransform transform;
        tf_listener.waitForTransform("base_link", "head_camera_rgb_optical_frame",                     
                                    ros::Time(0), ros::Duration(5.0));
        tf_listener.lookupTransform("base_link", "head_camera_rgb_optical_frame",                    
                                        ros::Time(0), transform);
        sensor_msgs::PointCloud2 cloud_out, cloud_out_2;
        pcl_ros::transformPointCloud("base_link", transform, init_shelf, cloud_out);
        pcl::fromROSMsg(cloud_out, *transformed_init_shelf);
        pcl_ros::transformPointCloud("base_link", transform, next_shelf, cloud_out_2);
        pcl::fromROSMsg(cloud_out_2, *transformed_next_shelf);
    }
    ROS_INFO("Looking for new item!");
    double distance_threshold;
    ros::param::param("distance_threshold", distance_threshold, 0.001);
    pcl::search::Search <pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    PointCloudC::Ptr output(new PointCloudC());
    pcl::SegmentDifferences<pcl::PointXYZRGB> segDiff;
    segDiff.setInputCloud(transformed_next_shelf);
    segDiff.setTargetCloud(transformed_init_shelf);
    segDiff.setDistanceThreshold(distance_threshold);
    segDiff.setSearchMethod(tree);
    segDiff.segment(*output);
    pcl::toROSMsg(*output, msg_out);
    init_shelf = next_shelf;
    updateContainerState(output, item_name.data); 
    // sensor_msgs::Image img;
    // pcl::toROSMsg(*output, img);
    // diff_image.publish(img);
}

void ItemDetector::updateContainerState(PointCloudC::Ptr cloud, std::string name) {
    std::vector<Object> objects;
    std::string method = "euclid";

    SegmentObjects(cloud, &objects);

    Object max_object;
    double max_size_object = -1;

    for (size_t i = 0; i < objects.size(); ++i) {
        const Object& object = objects[i];
        double new_vol = calculateVolume(object.dimensions);
        if (new_vol > max_size_object) {
            max_object = object;
            max_size_object = new_vol;
        }
    }

    // Bounding box for object
    visualization_msgs::Marker object_marker;
    object_marker.ns = "objects";
    object_marker.id = 123456789;
    object_marker.header.frame_id = "base_link";
    object_marker.type = visualization_msgs::Marker::CUBE;
    object_marker.pose = max_object.pose;
    object_marker.scale = max_object.dimensions;
    object_marker.color.g = 1;
    object_marker.color.a = 0.3;
    marker_pub_.publish(object_marker);

    // name for the object
    visualization_msgs::Marker name_marker;
    name_marker.ns = "recognition";
    name_marker.id = 123456789;
    name_marker.header.frame_id = "base_link";
    name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    name_marker.pose.position = max_object.pose.position;
    name_marker.pose.position.z += 0.1;
    name_marker.pose.orientation.w = 1;
    name_marker.scale.x = 0.025;
    name_marker.scale.y = 0.025;
    name_marker.scale.z = 0.025;
    name_marker.color.r = 0;
    name_marker.color.g = 0;
    name_marker.color.b = 1.0;
    name_marker.color.a = 1.0;
    name_marker.text = name;
    marker_pub_.publish(name_marker);

    // pose of the object
    geometry_msgs::PoseStamped ps;
    ps.pose = max_object.pose;
    ps.header.frame_id = name;
    container_state_pub_.publish(ps);
}

}
  // namespace perception