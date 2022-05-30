#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "visualization_msgs/Marker.h"
#include "perception/object_recognizer.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "perception_enabled_picking_demo");
    ros::NodeHandle nh;

    if (argc < 2) {
        ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR");
        ros::spinOnce();
    }
    std::string data_dir(argv[1]);

    std::vector<perception_msgs::ObjectFeatures> dataset;
    
    perception::LoadData(data_dir, &dataset);
    perception::ObjectRecognizer recognizer(dataset);

    ros::Publisher segment_pub =
        nh.advertise<sensor_msgs::PointCloud2>("segment_cloud", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 0);
    perception::Segmenter segmenter(segment_pub, marker_pub, recognizer);
    ros::Subscriber sub_segmentation =
        nh.subscribe("smart_cropper", 1, &perception::Segmenter::Callback, &segmenter);
    
    ros::spin();
    return 0;
}