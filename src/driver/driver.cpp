/*******************************************************************************
 * File:        driver.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: ROS wrapper for the feature detector
*******************************************************************************/
#include "driver/driver.h"

using namespace bipedlab;

Driver::Driver(ros::NodeHandle& nh)
{
    // ROS
    nh_ = nh;

    // Get Parameters
    if (!Driver::getParameters_())
    {
        debugger::debugTitleTextOutput("[Driver]", "NOTICE!!!!", 10, BR, BOLD);
        debugger::debugColorOutput("[Driver] Not enough parameters: ",
                "Using default values", 10, BR, BOLD);
        utils::pressEnterToContinue();
    }
    else
    {
        debugger::debugColorTextOutput("[Driver] Received all params", 10, BC);
    }

    // Subscriber
    point_cloud_sub_ = nh_.subscribe(velodyne_topic_, 1,
            &Driver::getPointCloud_, this);

    // Publisher
    landmark_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("landmark", 1);
    ground_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);

    // Construct Feature Detector
    FeatureDetector feature_detector;

    // Wait for Velodyne Point Cloud
    waitForData_();

    // Run Feature Dectector
    ros::Rate r(publishing_rate_);
    while (ros::ok())
    {
        ros::spinOnce(); 
        feature_detector.setInputCloud(rings_);
        feature_detector.run();

        publishPointCloud_(ground_pub_, feature_detector.getGround());
        publishPointCloud_(landmark_pub_, feature_detector.getLandmark());
        r.sleep();
    }
}

void Driver::getPointCloud_(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    for (int i = 0; i < VELODYNE_RING_NUMBER; ++i)
    {
        rings_[i].clear();
    }

    // Update rings (array of pcl::PointCloud<pcl::PointXYZ)
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);

    // Get ring number for every point cloud
    sensor_msgs::PointCloud2ConstIterator<int> iter_ring(*cloud_msg, "ring");
    for (int i = 0; iter_ring != iter_ring.end(); ++iter_ring, ++i)
    {
        rings_[*iter_ring].push_back(cloud[i]);
    }
}

void Driver::publishPointCloud_(ros::Publisher& publisher, 
        const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = "velodyne";
    publisher.publish(msg);
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParmaeters]");
    bool received_all = true;
    
    // Driver Parameters
    ros_utils::checkROSParam(nh_, "velodyne_topic", velodyne_topic_,
            getNameOf(velodyne_topic_), title_name, received_all);

    ros_utils::checkROSParam(nh_, "publishing_rate", publishing_rate_,
            getNameOf(publishing_rate_), title_name, received_all);

    // Default Parameters
    if (!received_all)
    {
        velodyne_topic_  = "/velodyne_points";
        publishing_rate_ = 2;
    }

    return received_all;
}

void Driver::waitForData_()
{
    while (ros::ok() && point_cloud_sub_.getNumPublishers() < 1)
    {
        ROS_WARN_ONCE("Wait for point cloud data");
        ros::spinOnce();
        sleep(1);
    }
}
