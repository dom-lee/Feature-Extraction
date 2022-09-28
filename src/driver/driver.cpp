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
    landmark_pub_  = nh_.advertise<sensor_msgs::PointCloud2>("landmark", 1);
    ground_pub_    = nh_.advertise<sensor_msgs::PointCloud2>("ground", 1);
    obstacles_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("obstacles", 1);

    bottom_beam_pub_ = nh_.advertise<visualization_msgs::Marker>("bottom_beam", 1, true);
    top_beam_pub_    = nh_.advertise<visualization_msgs::Marker>("top_beam", 1, true);

    grid_normals_pub_ = nh_.advertise<visualization_msgs::Marker>("grid_normals", 1, true);

    fitted_lines_pub_ = nh_.advertise<visualization_msgs::Marker>("fitted_lines", 1, true);
    ground_lines_pub_ = nh_.advertise<visualization_msgs::Marker>("ground_lines", 1, true);

    a_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("a", 1);
    b_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("b", 1);
    c_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("c", 1);

    cluster_pubs_.resize(100);
    for (int i = 0; i < cluster_pubs_.size(); ++i)
    {
        cluster_pubs_[i] = nh_.advertise<sensor_msgs::PointCloud2>(
                                "cluster_" + std::to_string(i), 1);
    }

    base_plane_pub_    = nh_.advertise<visualization_msgs::Marker>("base", 1, true);
    ceiling_plane_pub_ = nh_.advertise<visualization_msgs::Marker>("ceiling", 1, true);

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
            ROS_INFO_ONCE("[Driver] Received Point Cloud");
            if (is_extractor_setting_changed_)
            {
                feature_extractor.changeSetting(extractor_setting_);
                is_extractor_setting_changed_ = false;
            }

            std::chrono::steady_clock::time_point start_time = timing::getCurrentTime();

            feature_extractor.setInputCloud(rings_);
            feature_extractor.run();

            ROS_INFO("%f", timing::spendElapsedTime(start_time));


            //publishPointCloud_<pcl::PointXYZ>(ground_pub_,
                                              //feature_extractor.getGround());
            //publishPointCloud_<pcl::PointXYZ>(obstacles_pub_,
                                              //feature_extractor.getObstacles());
            publishPointCloud_<pcl::PointXYZ>(landmark_pub_,
                                              feature_extractor.getLandmark());

            publishPointCloud_<pcl::PointXYZ>(a_pub_, feature_extractor.getA());
            publishPointCloud_<pcl::PointXYZ>(b_pub_, feature_extractor.getB());
            publishPointCloud_<pcl::PointXYZ>(c_pub_, feature_extractor.getC());

            //auto cluster_ptrs = feature_extractor.getCluster();
            //for (int i = 0; i < std::min(100, (int)cluster_ptrs.size()); ++i)
            //{
                //publishPointCloud_<pcl::PointXYZ>(cluster_pubs_[i], cluster_ptrs[i]);
            //}

            visualizeLines_(grid_normals_pub_, 1, "grid_normals",
                            1.0f, 0.0f, 1.0f, feature_extractor.getGridNormals());
            //visualizeLines_(fitted_lines_pub_, 1, "fitted lines",
                            //1.0f, 1.0f, 1.0f, feature_extractor.getFittedLines());
            //visualizeLines_(ground_lines_pub_, 1, "ground lines",
                            //0.0f, 1.0f, 0.0f, feature_extractor.getGroundLines());
            //visualizeLines_(bottom_beam_pub_, 1, "bottom beam", 1.0f, 0.0f, 1.0f,
                           //feature_extractor.getBottomBeam());
            //visualizeLines_(top_beam_pub_, 1, "top beam", 0.0f, 1.0f, 1.0f,
                           //feature_extractor.getTopBeam());
            visualizePlane_(base_plane_pub_, 1, "base",
                            feature_extractor.getBasePlane());
            visualizePlane_(ceiling_plane_pub_, 1, "base",
                            feature_extractor.getCeilingPlane());
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
   
    // Convert sensor_msgs to pcl::PointCloud
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*cloud_msg, cloud);
    //// Save Time Stamp for TF
    cloud_msg_stamp_ = cloud_msg->header.stamp;

    // Update rings (array of pcl::PointCloud<pcl::PointXYZI)
    for (int i = 0; i < VELODYNE_RING_NUMBER; ++i)
    {
        rings_[i].clear();
    }

    // Get ring number for every point cloud
    sensor_msgs::PointCloud2ConstIterator<int> iter_ring(*cloud_msg, "ring");
    for (int i = 0; iter_ring != iter_ring.end(); ++iter_ring, ++i)
    {
        rings_[*iter_ring].push_back(cloud[i]);
    }


    //// PCA
    //pcl::PCA<pcl::PointXYZ> pca;
    //pca.setInputCloud(cloud.makeShared());
    //Eigen::Matrix3f aaa = pca.getEigenVectors();
    //Eigen::Vector4f mean = pca.getMean();

    //Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    //transform.block<3, 3>(0, 0) = aaa;
    //transform.block<4, 1>(0, 3) = mean;


    //pcl::PointCloud<pcl::PointXYZ> transformed_points;
    //pcl::transformPointCloud(cloud, transformed_points, transform.inverse());


    
    //// Transform PointCloud from Velodyne Frame to Odom Frame
    //tf_listener_.waitForTransform("odom", publisher_frame_,
                                  //cloud_msg_stamp_, ros::Duration(1.0));
    //pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
    //pcl_ros::transformPointCloud("odom", cloud, transformed_cloud, tf_listener_);

    //// Generalized ICP
    //if (cloud_odom_.empty())
    //{
        //cloud_odom_.push_back(std::move(transformed_cloud));
    //}
    //else
    //{
        //pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> gicp;
        //gicp.setMaxCorrespondenceDistance(5);
        //gicp.setMaximumIterations(200);
        //gicp.setMaximumOptimizerIterations(50);
        //gicp.setRANSACIterations(0);
        //gicp.setRANSACOutlierRejectionThreshold(0.05);
        //gicp.setTransformationEpsilon(0.0005);
        //gicp.setUseReciprocalCorrespondences(false);
        
        //gicp.setInputSource(transformed_cloud.makeShared());
        //gicp.setInputTarget(cloud_odom_.back().makeShared());
        
        //std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

        //pcl::PointCloud<pcl::PointXYZ> aligned_source;
        //aligned_source.header = transformed_cloud.header;
        //gicp.align(aligned_source);
        
        //std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
        //std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::seconds> (end - begin).count() << "[s]" << std::endl;

        //cloud_odom_.push_back(std::move(aligned_source));
    //}
    
    //// Save PointCloud in Odom Frame into cloud_odom_(deque)
    //if (cloud_odom_.size() > 10)
    //{
        //cloud_odom_.pop_front();
    //}
    //cloud_odom_.push_back(std::move(transformed_cloud));

    
    //// Re-Transform PointCloud from Odom Frame to Velodyne Frame
    //accumulated_cloud_.clear();
    //for (int i = 0; i < cloud_odom_.size(); ++i)
    //{
        //cloud_odom_[i].header.stamp = cloud.header.stamp; 
        //pcl::PointCloud<pcl::PointXYZ> retransformed_cloud;
        //pcl_ros::transformPointCloud(publisher_frame_, cloud_odom_[i],
                                     //retransformed_cloud, tf_listener_);
        //accumulated_cloud_ += retransformed_cloud;
    //}
    //accumulated_cloud_.header = cloud.header;


    //publishPointCloud_<pcl::PointXYZ>(a_pub_, accumulated_cloud_.makeShared());
}

template <class PointT>
void Driver::publishPointCloud_(ros::Publisher& publisher, 
                                const typename pcl::PointCloud<PointT>::Ptr cloud)
{
    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.frame_id = publisher_frame_;
    msg.header.stamp = cloud_msg_stamp_;

    publisher.publish(msg);
}

template <class PointT>
void Driver::visualizeLines_(ros::Publisher& publisher,
                             int id, std::string name,
                             double r, double g, double b,
                             const std::vector<PointT>& lines)
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

    line_list.scale.x = 0.01;
    
    line_list.color.r = r;
    line_list.color.g = g;
    line_list.color.b = b;
    line_list.color.a = 0.5;

    geometry_msgs::Point p1;
    geometry_msgs::Point p2;

    for (int i = 0; i < (int)lines.size() / 2; ++i)
    {
        p1.x = lines[2 * i].x;
        p1.y = lines[2 * i].y;
        p1.z = lines[2 * i].z;

        p2.x = lines[2 * i + 1].x;
        p2.y = lines[2 * i + 1].y;
        p2.z = lines[2 * i + 1].z;

        line_list.points.push_back(p1);
        line_list.points.push_back(p2);
    }
    publisher.publish(line_list);
}

void Driver::visualizePlane_(ros::Publisher& publisher,
                             int id, std::string name,
                             Eigen::Vector4f plane_coeff)
{
    visualization_msgs::Marker cube;

    cube.id = id;
    cube.ns = name;
    cube.type = visualization_msgs::Marker::CUBE;
    cube.header.frame_id = "velodyne";
    cube.header.stamp = ros::Time::now();
    cube.lifetime = ros::Duration();
    cube.action = visualization_msgs::Marker::ADD;
    
    cube.pose.position.x = 0;
    cube.pose.position.y = 0;
    cube.pose.position.z = -plane_coeff(3) / plane_coeff(2);

    Eigen::Vector3f basis(0, 0, 1);
    Eigen::Vector3f normal = plane_coeff.head(3);
    normal.normalize();

    Eigen::Vector3f cross_product = basis.cross(normal);
    Eigen::Vector4f quaternion(cross_product(0), cross_product(1),
                               cross_product(2), 1 + basis.dot(normal));
    quaternion.normalize();

    cube.pose.orientation.x = quaternion(0);
    cube.pose.orientation.y = quaternion(1); 
    cube.pose.orientation.z = quaternion(2); 
    cube.pose.orientation.w = quaternion(3); 

    cube.scale.x = 15.0;
    cube.scale.y = 15.0;
    cube.scale.z = 0.01;

    cube.color.r = 1.0f;
    cube.color.g = 1.0f;
    cube.color.b = 1.0f;
    cube.color.a = 0.8;
    
    publisher.publish(cube);
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

    // Parameters for Base Planar Estimation
    extractor_setting_.RING_TO_FIT_BASE        = config.ring_to_fit_base;
    extractor_setting_.GRADIENT_THRESHOLD      = config.gradient_threshold;
    extractor_setting_.BASE_FIT_THRESHOLD      = config.base_fit_threshold;
    
    // Parameters for Fitting Lines
    extractor_setting_.DISCONTINUITY_DISTANCE  = config.discontinuity_distance;
    extractor_setting_.EPSILON                 = config.epsilon;

    // Parameters for Filtering Ground Lines
    extractor_setting_.GROUND_DIST_THRESHOLD   = config.ground_dist_threshold;
    extractor_setting_.GROUND_ANGLE_THRESHOLD  = config.ground_angle_threshold;

    // Parameters for Wall Extraction
    extractor_setting_.SECTION_NUMBER          = config.section_number;
    extractor_setting_.WALL_HEIGHT_THRESHOLD   = config.wall_height_threshold;
    extractor_setting_.WALL_FIT_THRESHOLD      = config.wall_fit_threshold;
    extractor_setting_.CLUSTER_ANGLE_THRESHOLD = config.cluster_angle_threshold;
    extractor_setting_.CLUSTER_DIST_THRESHOLD  = config.cluster_dist_threshold;





    extractor_setting_.SMOOTH_WINDOW_SIZE      = config.smooth_window_size;
    extractor_setting_.SMOOTH_THRESHOLD        = config.smooth_threshold;

    // Parameters for Plane RANSAC
    extractor_setting_.FIT_PLANE_THRESHOLD     = config.fit_plane_threshold;

    // Parameters for Ground Extraction
    extractor_setting_.GRID_LENGTH             = config.grid_length;
    extractor_setting_.GRID_RANGE              = config.grid_range;
    extractor_setting_.DISCONTINUITY_HEIGHT    = config.discontinuity_height;
    extractor_setting_.OBSTACLE_THRESHOLD      = config.obstacle_threshold;
    extractor_setting_.GROUND_THRESHOLD        = config.ground_threshold;

    // Parameters for Curb Extraction
    extractor_setting_.DISCONTINUITY_AZIMUTH   = config.discontinuity_azimuth;
    extractor_setting_.SMOOTHNESS_THRESHOLD    = config.smoothness_threshold;
    extractor_setting_.SMOOTH_COUNT            = config.smooth_count;
    extractor_setting_.CONTINUITY_ANGLE        = config.continuity_angle;
    extractor_setting_.CURB_HEIGHT_THRESHOLD   = config.curb_height_threshold;
    extractor_setting_.CURB_ANGLE_THRESHOLD    = config.curb_angle_threshold;
    extractor_setting_.SIDEWALK_MIN_LENGTH     = config.sidewalk_min_length;
    extractor_setting_.SIDEWALK_MAX_LENGTH     = config.sidewalk_max_length;

    // Parameters for Road Model
    extractor_setting_.BEAM_SECTION_NUMBER     = config.beam_section_number;
    extractor_setting_.ROAD_VIEW_RANGE         = config.road_view_range;
    extractor_setting_.ROAD_WIDTH_THRESHOLD    = config.road_width_threshold;
    extractor_setting_.GROUND_COUNT_THRESHOLD  = config.ground_count_threshold;

    // Parameters for Debug
    extractor_setting_.RING_TO_ANALYZE         = config.ring_to_analyze;
}


void Driver::getClickedPointCallBack_(
    const geometry_msgs::PointStamped::ConstPtr& msg) const
{
    pcl::PointXYZ point;
    point.x = msg->point.x;
    point.y = msg->point.y;
    point.z = msg->point.z;

    std::cout << "\n Clicked Point : ("
              << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;

    pcl::PointXYZ origin(0, 0, 0);
    std::cout << "Distance : " << pcl::euclideanDistance(origin, point) << std::endl;
    double azimuth = std::atan2(point.y, point.x);
    std::cout << "Azimuth :  " << std::atan2(point.y, point.x) << std::endl;  
    
    int ring_id = -1;
    double min_distance = INFINITY;
    pcl::PointXYZ closest_point;
    for (int i = 0; i < rings_.size(); ++i)
    {
        for (int j = 0; j < rings_[i].size(); ++j)
        {
            double distance = pcl::euclideanDistance(rings_[i][j], point);
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_point = rings_[i][j];
                ring_id = i;
            }
        }
    }

    std::cout << "Ring ID: " << ring_id << std::endl;
    sleep(2);
}
