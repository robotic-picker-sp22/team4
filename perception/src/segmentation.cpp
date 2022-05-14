#include "perception/segmentation.h"
#include "geometry_msgs/Pose.h"
#include "pcl/PointIndices.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "pcl/filters/extract_indices.h"
#include "visualization_msgs/Marker.h"
#include "perception/object.h"
#include <pcl/common/common.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/segmentation/region_growing_rgb.h>
#include <algorithm>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
void SegmentBinObjects(PointCloudC::Ptr cloud, std::vector<pcl::PointIndices>* indices, std::string method) {
    if (method == "euclid") {
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
    } else if (method == "region") {
        double curvature_threshold;
        int min_cluster_size, max_cluster_size;
        ros::param::param("reg_min_cluster_size", min_cluster_size, 50);
        ros::param::param("reg_max_cluster_size", max_cluster_size, 1000000);
        ros::param::param("reg_curvature_threshold", curvature_threshold, 1.0);
        pcl::PointCloud <pcl::Normal>::Ptr normals (new pcl::PointCloud <pcl::Normal>);
        pcl::RegionGrowing<PointC, pcl::Normal> reg;
        reg.setInputCloud (cloud);
        reg.setMinClusterSize (min_cluster_size);
        reg.setMaxClusterSize (max_cluster_size);
        reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI);
        reg.setCurvatureThreshold (curvature_threshold);
        reg.setInputNormals (normals); // do we need this?
        reg.extract(*indices);
    } else if (method == "color") {
        int min_cluster_size, distance_threshold, point_threshold, reg_threshold;
        ros::param::param("clr_min_cluster_size", min_cluster_size, 600);
        ros::param::param("clr_distance_threshold", distance_threshold, 10);
        ros::param::param("clr_point_threshold", point_threshold, 6);
        ros::param::param("clr_reg_threshold", reg_threshold, 5);
        pcl::RegionGrowingRGB<PointC> reg;
        reg.setInputCloud (cloud);
        reg.setDistanceThreshold (distance_threshold);
        reg.setPointColorThreshold (point_threshold);
        reg.setRegionColorThreshold (reg_threshold);
        reg.setMinClusterSize (min_cluster_size);
        reg.extract (*indices);
    } else {
        ROS_INFO("Incorrect instance of SegmentBinObjects called.");
        return;
    }

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

Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& marker_pub, const ObjectRecognizer& recognizer)
    : points_pub_(points_pub), marker_pub_(marker_pub), recognizer_(recognizer) {}

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
}

void Segmenter::Callback(const sensor_msgs::PointCloud2& msg) {
    PointCloudC::Ptr cloud_unfiltered(new PointCloudC());
    pcl::fromROSMsg(msg, *cloud_unfiltered);
    PointCloudC::Ptr cloud(new PointCloudC());
    std::vector<int> index;
    pcl::removeNaNFromPointCloud(*cloud_unfiltered, *cloud, index);

    std::vector<Object> objects;
    SegmentObjects(cloud, &objects);

    for (size_t i = 0; i < objects.size(); ++i) {
        const Object& object = objects[i];

        // Publish a bounding box around it.
        visualization_msgs::Marker object_marker;
        object_marker.ns = "objects";
        object_marker.id = i;
        object_marker.header.frame_id = "base_link";
        object_marker.type = visualization_msgs::Marker::CUBE;
        object_marker.pose = object.pose;
        object_marker.scale = object.dimensions;
        object_marker.color.g = 1;
        object_marker.color.a = 0.3;
        marker_pub_.publish(object_marker);

        // Recognize the object.
        // std::string name;
        // double confidence;
        // TODO: recognize the object with the recognizer_.
        std::string name;
        double confidence;
        recognizer_.Recognize(object, &name, &confidence);
        confidence = round(1000 * confidence) / 1000;

        std::stringstream ss;
        ss << name << " (" << confidence << ")";

        // Publish the recognition result.
        visualization_msgs::Marker name_marker;
        name_marker.ns = "recognition";
        name_marker.id = i;
        name_marker.header.frame_id = "base_link";
        name_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        name_marker.pose.position = object.pose.position;
        name_marker.pose.position.z += 0.1;
        name_marker.pose.orientation.w = 1;
        name_marker.scale.x = 0.025;
        name_marker.scale.y = 0.025;
        name_marker.scale.z = 0.025;
        name_marker.color.r = 0;
        name_marker.color.g = 0;
        name_marker.color.b = 1.0;
        name_marker.color.a = 1.0;
        name_marker.text = ss.str();
        marker_pub_.publish(name_marker);
    }
}
void SegmentObjects(PointCloudC::Ptr cloud,
                        std::vector<Object>* objects) {
// Same as callback, but with visualization code removed.
    std::vector<pcl::PointIndices> object_indices;
    SegmentBinObjects(cloud, &object_indices, "euclid");
    // SegmentBinObjects(cloud, &object_indices, "region");
    // SegmentBinObjects(cloud, &object_indices, "color");

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
}  // namespace perception