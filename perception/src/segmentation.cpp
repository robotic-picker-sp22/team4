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
#include <algorithm>

typedef pcl::PointXYZRGB PointC;
typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudC;

namespace perception {
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

    Segmenter::Segmenter(const ros::Publisher& points_pub, const ros::Publisher& marker_pub)
        : points_pub_(points_pub), marker_pub_(marker_pub) {}

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
        }
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
}  // namespace perception