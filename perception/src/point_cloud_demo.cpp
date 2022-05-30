#include "perception/crop.h"
#include "perception/downsample.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "perception/segmentation.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/Marker.h"
#include "perception/object_recognizer.h"
#include "perception/item_detector.h"
#include "sensor_msgs/Image.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "point_cloud_demo");
    ros::NodeHandle nh;

    // Path for launch file: /home/capstone/data/objects/labels
    if (argc < 3) {
        ROS_INFO("Usage: rosrun perception point_cloud_demo DATA_DIR ALGO");
        ros::spinOnce();
    }
    std::string data_dir(argv[1]);
    std::string algo(argv[2]);

    ROS_INFO("Hello, I'm here");
    // ros::Publisher crop_pub =
    //     nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    // perception::Cropper cropper(crop_pub);
    // ros::Subscriber sub_cropper =
    //     nh.subscribe("head_camera/depth_registered/points", 1, &perception::Cropper::Callback, &cropper);
    // ros::Subscriber sub_cropper =
    //     nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);
    // ros::Publisher downsample_pub =
    //     nh.advertise<sensor_msgs::PointCloud2>("downsample_cloud", 1, true);
    // perception::Downsampler downsampler(downsample_pub);
    // ros::Subscriber sub_downsample = 
    //     nh.subscribe("cropped_cloud", 1, &perception::Downsampler::Callback, &downsampler);
    // std::vector<perception_msgs::ObjectFeatures> dataset;
    // perception::LoadData(data_dir, &dataset);
    // perception::ObjectRecognizer recognizer(dataset);

    // ros::Publisher segment_pub =
    //     nh.advertise<geometry_msgs::PoseStamped>("segment_cloud", 1, true);
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    // perception::Segmenter segmenter(segment_pub, marker_pub, recognizer, algo);
    // ros::Subscriber sub_segmentation =
    //     nh.subscribe("cropped_cloud", 1, &perception::Segmenter::Callback, &segmenter);


    ros::Publisher crop_pub =
        nh.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
    perception::Cropper cropper(crop_pub);
    ros::Subscriber sub_cropper =
        nh.subscribe("cloud_in", 1, &perception::Cropper::Callback, &cropper);


    ros::Publisher diff_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("image_cloud", 1, true); 
    ros::Publisher diff_img_pub = nh.advertise<sensor_msgs::Image>("diff_image", 1, true);
    ros::Publisher container_state_pub =
        nh.advertise<geometry_msgs::PoseStamped>("container_state", 1, true);
    ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    perception::ItemDetector item_detector(diff_cloud_pub, diff_img_pub, marker_pub, container_state_pub);
    ros::Subscriber sub_item_detector =
        nh.subscribe("cropped_cloud", 1, &perception::ItemDetector::Callback, &item_detector);
    ros::Subscriber sub_add_item =
        nh.subscribe("add_item", 1, &perception::ItemDetector::AddItem, &item_detector);
    
    // std::vector<perception_msgs::ObjectFeatures> dataset;
    // perception::LoadData(data_dir, &dataset);
    // perception::ObjectRecognizer recognizer(dataset);
    // ros::Publisher segment_pub =
    //     nh.advertise<geometry_msgs::PoseStamped>("segment_cloud", 1, true);
    // ros::Publisher marker_pub = nh.advertise<visualization_msgs::Marker>( "visualization_marker", 0);
    // perception::Segmenter segmenter(segment_pub, marker_pub, recognizer, algo);
    // ros::Subscriber sub_segmentation =
    //     nh.subscribe("image_cloud", 1, &perception::Segmenter::Callback, &segmenter);

    ros::spin();
    return 0;
}