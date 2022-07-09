/*******************************************************************************
 * File:        driver.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: ROS wrapper for the Feature Extractor
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

    clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, 
            &Driver::getClickedPointCallBack_, this);

    // Publisher
    landmark_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("landmark", 1);
    ground_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
    a_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("a", 1);
    b_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("b", 1);

    // Dynamic Reconfiguration
    dynamic_reconfigure::Server<feature_extraction::feature_extractionConfig> server;
    server.setCallback(boost::bind(&Driver::reconfigParams_, this, _1, _2));

    // Construct Feature Extractor
    FeatureExtractor feature_extractor(lidar_setting_);

    // Run Feature Dectector
    ros::Rate r(publishing_rate_);
    while (ros::ok())
    {
        ros::spinOnce(); 
        if (!rings_[0].empty())
        {
            debugger::debugColorTextOutput("[Driver] Received Point Cloud")
            feature_extractor.setInputCloud(extractor_setting_, rings_);
            feature_extractor.run();

            publishPointCloud_(ground_pub_, feature_extractor.getGround());
            publishPointCloud_(landmark_pub_, feature_extractor.getLandmark());
            publishPointCloud_(a_pub_, feature_extractor.getA());
            publishPointCloud_(b_pub_, feature_extractor.getB());
        }
        else
        {
            ROS_WARN_ONCE("Wait for Point Cloud data");
        }
        r.sleep();
    }
}

void Driver::getPointCloud_(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    pcl::fromROSMsg(*cloud_msg, raw_cloud_);

    // Update rings (array of pcl::PointCloud<pcl::PointXYZI)
    for (int i = 0; i < VELODYNE_RING_NUMBER; ++i)
    {
        rings_[i].clear();
    }

    // Get ring number for every point cloud
    sensor_msgs::PointCloud2ConstIterator<int> iter_ring(*cloud_msg, "ring");
    for (int i = 0; iter_ring != iter_ring.end(); ++iter_ring, ++i)
    {
        rings_[*iter_ring].push_back(raw_cloud_[i]);
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

    // LiDAR Property
    ros_utils::checkROSParam(nh_, "ring_number", lidar_setting_.ring_number,
            getNameOf(lidar_setting_.ring_number), title_name, received_all);
    ros_utils::checkROSParam(nh_, "elevation_angle", 
            lidar_setting_.elevation_angles,
            getNameOf(lidar_setting_.elevation_angles), title_name, received_all);

    // Default Parameters
    if (!received_all)
    {
        velodyne_topic_  = "/velodyne_points";
        publishing_rate_ = 2;
        lidar_setting_.ring_number = 32;
        lidar_setting_.elevation_angles = {-25.0, -15.639, -11.310, -8.843,
            -7.254, -6.148, -5.333, -4.667, -4.000, -3.667, -3.333, -3.000,
            -2.667, -2.333, -2.000, -1.667, -1.333, -1.000, -0.667, -0.333,
            0.000, 0.333, 0.667, 1.000, 1.333, 1.667, 2.333, 3.333, 4.667,
            7.000, 10.333, 15.000};
    }
    
    // Convert LiDAR elevation angles from degree to radian
    for (double& angle : lidar_setting_.elevation_angles)
    {
        angle *= M_PI / 180;
    }

    return received_all;
}

void Driver::reconfigParams_(feature_extraction::feature_extractionConfig& config,
        uint32_t level)
{
    ROS_INFO_THROTTLE(1.0, "[Feature Extractor] new parameteres requested");

    extractor_setting_.RING_TO_ANALYZE       = config.ring_to_analyze;
    extractor_setting_.RING_TO_FIT_BASE      = config.ring_to_fit_base;
    extractor_setting_.FIT_PLANE_THRESHOLD   = config.fit_plane_threshold;
    extractor_setting_.LOCAL_WINDOW_SIZE     = config.local_window_size;
    extractor_setting_.ALIGNEDNESS_THRESHOLD = config.alignedness_threshold;
    extractor_setting_.ANGLE_BUFFER          = config.angle_buffer;
    extractor_setting_.DIST_DIFF_THRESHOLD   = config.dist_diff_threshold;
    extractor_setting_.ANGLE_DIFF_THRESHOLD  = config.angle_diff_threshold;
    extractor_setting_.HEIGHT_DIFF_THRESHOLD = config.height_diff_threshold;
    extractor_setting_.GROUND_THRESHOLD      = config.ground_threshold;
    extractor_setting_.ANGULAR_RESOLUTION    = config.angular_resolution;
    extractor_setting_.GROUND_DISCONTINUITY  = config.ground_discontinuity;
    extractor_setting_.CONTINUED_NUMBER      = config.continued_number;
    extractor_setting_.CURB_HEIGHT           = config.curb_height;
    extractor_setting_.DISCONTINUITY         = config.discontinuity;
    extractor_setting_.ANGLE_CURB_THRESHOLD  = config.angle_curb_threshold;
    extractor_setting_.HEIGHT_CURB_THRESHOLD = config.height_curb_threshold;
}


void Driver::getClickedPointCallBack_(const 
        geometry_msgs::PointStamped::ConstPtr& msg)
{
    pcl::PointXYZ point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;

    double elevation_angle;
    double min_distance = 10000;
    pcl::PointXYZI closest_point;
    for (int i = 0; i < rings_.size(); ++i)
    {
        for (auto& p : rings_[i])
        {
            double distance = pcl::euclideanDistance(p, point);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_point = p;
                elevation_angle = -lidar_setting_.elevation_angles[i] * M_PI/180;
            }
        }
    }
    double azimuth = std::atan2(closest_point.y, closest_point.x);
    double xy_dist = std::sqrt(std::pow(closest_point.x, 2) +
            std::pow(closest_point.y, 2));
    double delta_xy = xy_dist * extractor_setting_.ANGULAR_RESOLUTION;
    std::cout << "Intesity: " << closest_point.intensity << std::endl;
    std::cout <<  " azimuth: " << azimuth << std::endl;
    std::cout << " xy_dist: " << xy_dist << " delta_xy: " << delta_xy << std::endl;
    std::cout << "z: " << closest_point.z << " delta_z: " << delta_xy * std::sin(elevation_angle) << std::endl;
}
