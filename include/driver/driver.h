/*******************************************************************************
 * File:        driver.h
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: ROS wrapper for the feature detector
*******************************************************************************/
#ifndef DRIVER_H
#define DRIVER_H

// Standard
#include <iostream>
#include <string>
#include <array>

// ROS
#include <ros/ros.h>

// Dynamic Configuration
#include <dynamic_reconfigure/server.h>
#include <feature_detection/feature_detectionConfig.h>

// PCL
#include <pcl/point_cloud.h> // PointCloud
#include <pcl/point_types.h> // PointXYZ
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg

// Messgaes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h> // PointCloud2ConstIterator
#include <geometry_msgs/PointStamped.h>

// Main Algorithm
#include "feature_detection/feature_detector.h"
#include "setting/feature_detector_setting_t.h"
// Utils
#include "utils/debugger.h"
#include "utils/ros_utils.h"
#include "utils/utils.h"

static const int VELODYNE_RING_NUMBER = 32;

class Driver
{
public:
    Driver(ros::NodeHandle& nh);

private:
    // Pthread
    pthread_mutex_t lock_;

    // ROS
    ros::NodeHandle nh_;

    // Subscriber
    ros::Subscriber point_cloud_sub_;
    ros::Subscriber clicked_point_sub_;

    // Publisher
    ros::Publisher landmark_pub_;
    ros::Publisher ground_pub_;
    ros::Publisher a_pub_;
    ros::Publisher b_pub_;

    // Paramters
    std::string velodyne_topic_;
    double publishing_rate_;
    feature_detector_setting_t feature_detector_setting_;

    // Point Clouds
    pcl::PointCloud<pcl::PointXYZI> raw_cloud_;
    std::array<pcl::PointCloud<pcl::PointXYZI>, VELODYNE_RING_NUMBER> rings_; 

    void getPointCloud_(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void publishPointCloud_(ros::Publisher& publisher, 
            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    bool getParameters_();

    void waitForData_();

    void reconfigParams_(feature_detection::feature_detectionConfig& config,
            uint32_t level);

    void getClickedPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);
};
#endif /* DRIBER_H */
