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
    point_cloud_sub_ = nh_.subscribe(pointcloud_topic_, 1,
            &Driver::getCloudCallback_, this);

    clicked_point_sub_ = nh_.subscribe("/clicked_point", 1, 
            &Driver::getClickedPointCallBack_, this);

    // Publisher
    landmark_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("landmark", 1);
    ground_pub_   = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
    a_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("a", 1);
    b_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("b", 1);
    c_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("c", 1);
    beam_pub_ = nh_.advertise<visualization_msgs::Marker>("beam", 1, true);

    // Dynamic Reconfiguration
    dynamic_reconfigure::Server<feature_extraction::feature_extractionConfig> server;
    server.setCallback(boost::bind(&Driver::reconfigParams_, this, _1, _2));

    // Construct Feature Extractor
    feature_extractor_ = std::make_unique<FeatureExtractor>(lidar_setting_);

    // Run Feature Dectector
    ros::Rate r(publishing_rate_);
    while (ros::ok())
    {
        ros::spinOnce(); 
        if (!rings_[0].empty())
        {
            ROS_INFO_ONCE("[Driver] Received Point Cloud");
            if (is_extractor_setting_changed_)
            {
                feature_extractor_->changeSetting(extractor_setting_);
                is_extractor_setting_changed_ = false;
            }

            feature_extractor_->setInputCloud(rings_);
            feature_extractor_->run();

            publishPointCloud_(ground_pub_, feature_extractor_->getGround());
            publishPointCloud_(landmark_pub_, feature_extractor_->getLandmark());

            publishPointCloud_(a_pub_, feature_extractor_->getA());
            publishPointCloud_(b_pub_, feature_extractor_->getB());
            publishPointCloud_(c_pub_, feature_extractor_->getC());
            visualizeBeam_(beam_pub_, 1, "lower beam",
                           feature_extractor_->getBeamA());
        }
        else
        {
            ROS_WARN_ONCE("Wait for Point Cloud data");
        }
        r.sleep();
    }
}

void Driver::getCloudCallback_(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
    debugger::debugColorTextOutput("[Driver] PointCloud Callback", 3, BC);
    pcl::fromROSMsg(*cloud_msg, raw_cloud_);

    // Save Time Stamp for TF
    cloud_msg_stamp_ = cloud_msg->header.stamp;

    // Update rings (array of pcl::PointCloud<pcl::PointXYZ)
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
    msg.header.frame_id = publisher_frame_;
    msg.header.stamp = cloud_msg_stamp_;

    publisher.publish(msg);
}

void Driver::visualizeBeam_(ros::Publisher& publisher,
                            int id, std::string name,
                            std::vector<std::pair<double, double>> beam)
{
    visualization_msgs::Marker line_list;

    line_list.id = id;
    line_list.ns = name;
    line_list.type = visualization_msgs::Marker::LINE_LIST;
    line_list.header.frame_id = "velodyne";
    line_list.header.stamp = ros::Time::now();
    line_list.lifetime = ros::Duration();
    line_list.action = visualization_msgs::Marker::ADD;
    line_list.pose.orientation.w = 1.0;

    line_list.scale.x = 1;

    line_list.color.g = 1.0f;
    line_list.color.a = 0.5;

    geometry_msgs::Point p;

    for (const auto& point : beam)
    {
        p.x = point.first;
        p.y = point.second;
        p.z = 0;

        line_list.points.push_back(p);
    }
    
    publisher.publish(line_list);
}

bool Driver::getParameters_()
{
    std::string title_name("[Driver]/[getParmaeters]");
    bool received_all = true;
    
    // Driver Parameters
    ros_utils::checkROSParam(nh_, "pointcloud_topic", pointcloud_topic_,
            getNameOf(pointcloud_topic_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "publishing_rate", publishing_rate_,
            getNameOf(publishing_rate_), title_name, received_all);
    ros_utils::checkROSParam(nh_, "publisher_frame", publisher_frame_,
            getNameOf(publisher_frame_), title_name, received_all);

    // LiDAR Property
    ros_utils::checkROSParam(nh_, "lidar_height", lidar_setting_.height,
            getNameOf(lidar_setting_.height), title_name, received_all);
    ros_utils::checkROSParam(nh_, "ring_number", lidar_setting_.ring_number,
            getNameOf(lidar_setting_.ring_number), title_name, received_all);
    ros_utils::checkROSParam(nh_, "elevation_angle", lidar_setting_.elevation_angles,
            getNameOf(lidar_setting_.elevation_angles), title_name, received_all);

    // Default Parameters
    if (!received_all)
    {
        pointcloud_topic_  = "/velodyne_points";
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
    is_extractor_setting_changed_ = true;

    extractor_setting_.RING_TO_ANALYZE        = config.ring_to_analyze;
    extractor_setting_.FIT_PLANE_THRESHOLD    = config.fit_plane_threshold;
    extractor_setting_.ANGULAR_RESOLUTION     = config.angular_resolution;
    
    // Parameters for Obstacle Filter
    extractor_setting_.RING_TO_FIT_BASE       = config.ring_to_fit_base;
    extractor_setting_.BASE_BUFFER            = config.base_buffer;
    extractor_setting_.ANGLE_BUFFER           = config.angle_buffer;

    // Parameters for Ground Estimation
    extractor_setting_.ANGLE_DIFF_THRESHOLD   = config.angle_diff_threshold;
    extractor_setting_.HEIGHT_DIFF_THRESHOLD  = config.height_diff_threshold;
    extractor_setting_.GROUND_THRESHOLD       = config.ground_threshold;
    extractor_setting_.GROUND_DISCONTINUITY   = config.ground_discontinuity;
    extractor_setting_.CONTINUED_NUMBER       = config.continued_number;

    // Parameters for Road Model Estination
    extractor_setting_.BEAM_SECTION_NUMBER    = config.beam_section_number;
    extractor_setting_.ROAD_VIEW_RANGE        = config.road_view_range;

    // Parameters for Curb Extraction
    extractor_setting_.SMOOTH_WINDOW_SIZE     = config.smooth_window_size;
    extractor_setting_.SMOOTH_THRESHOLD_GRASS = config.smooth_threshold_grass;
    extractor_setting_.SMOOTH_THRESHOLD_PLANE = config.smooth_threshold_plane;

    extractor_setting_.CURB_HEIGHT            = config.curb_height;
    extractor_setting_.CURB_WINDOW_SIZE       = config.curb_window_size;
    extractor_setting_.CURB_HEIGHT_THRESHOLD  = config.curb_height_threshold;
    extractor_setting_.CURB_ANGLE_THRESHOLD   = config.curb_angle_threshold;
    extractor_setting_.DISCONTINUITY          = config.discontinuity;
}


void Driver::getClickedPointCallBack_(const 
        geometry_msgs::PointStamped::ConstPtr& msg)
{
    pcl::PointXYZ point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;

    std::cout << "\n Clicked Point : ("
              << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

    double azimuth = std::atan2(point.y, point.x);
    double xy_dist = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
    double delta_xy = xy_dist * extractor_setting_.ANGULAR_RESOLUTION;
    std::cout << "azimuth: " << azimuth << std::endl;
    std::cout << "xy_dist: " << xy_dist << " delta_xy: " << delta_xy << std::endl;
    std::cout << "height: " << feature_extractor_->calculateHeight(point) << std::endl;
}
