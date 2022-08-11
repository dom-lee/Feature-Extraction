/*******************************************************************************
 * File:        driver.h
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: ROS wrapper for the Feature Extractor
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
#include <feature_extraction/feature_extractionConfig.h>

// PCL
#include <pcl/point_cloud.h> // PointCloud
#include <pcl/point_types.h> // PointXYZ
#include <pcl_conversions/pcl_conversions.h> // fromROSMsg

// Messgaes
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h> // PointCloud2ConstIterator
#include <geometry_msgs/PointStamped.h>
#include <visualization_msgs/Marker.h>

// Main Algorithm
#include "feature_extraction/feature_extractor.h"
#include "setting/feature_extractor_setting_t.h"
#include "setting/lidar_setting_t.h"

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
    // ROS
    ros::NodeHandle nh_;

    // Subscriber
    std::string pointcloud_topic_; 

    ros::Subscriber point_cloud_sub_;
    ros::Subscriber clicked_point_sub_;

    // Publisher
    std::string publisher_frame_;
    double publishing_rate_;

    ros::Publisher landmark_pub_;
    ros::Publisher ground_pub_;
    ros::Publisher a_pub_;
    ros::Publisher b_pub_;
    ros::Publisher c_pub_;
    ros::Publisher beam_pub_;

    // Paramters
    lidar_setting_t lidar_setting_;
    feature_extractor_setting_t extractor_setting_;
    bool is_extractor_setting_changed_;

    std::unique_ptr<FeatureExtractor> feature_extractor_;

    // Point Clouds
    pcl::PointCloud<pcl::PointXYZ> raw_cloud_;
    std::array<pcl::PointCloud<pcl::PointXYZ>, VELODYNE_RING_NUMBER> rings_; 
    ros::Time cloud_msg_stamp_;

    void getCloudCallback_(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

    void publishPointCloud_(ros::Publisher& publisher, 
                            const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);

    void visualizeBeam_(ros::Publisher& publisher,
                        int id, std::string name,
                        std::vector<std::pair<double, double>> beam);

    bool getParameters_();

    void waitForData_();

    void reconfigParams_(feature_extraction::feature_extractionConfig& config,
                         uint32_t level);

    void getClickedPointCallBack_(const geometry_msgs::PointStamped::ConstPtr& msg);
};
#endif /* DRIVER_H */
