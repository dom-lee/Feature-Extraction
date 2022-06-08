/*******************************************************************************
 * File:        feature_dectector.h
 * 
 * Author:      Dongmyeong Lee (dongmyeong[at]umich.edu)
 * Created:     05/29/2022
 * 
 * Description: Dectect Curb and boundary of grass with LiDAR and Camera
*******************************************************************************/
#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

// Standard
#include <memory>
#include <array>
#include <vector>
#include <cmath>
#include <numeric>
#include <algorithm>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

// PCL
#include <pcl/point_cloud.h> // PointCloud
#include <pcl/point_types.h> // PointXYZ
#include <pcl/common/distances.h> // euclideanDistance
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// Utils
#include "utils/timing.h"
#include "utils/debugger.h"

static const int RING_NUMBER = 32;

class FeatureDetector
{
public:
    FeatureDetector();

    // Compute Base Plane & get Translation Matrix
    void setInputCloud(std::array<pcl::PointCloud<pcl::PointXYZ>, 
            RING_NUMBER>& rings);

    void run();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLandmark();

private:
    // Octant(8) x SECTION_NUMBER matrix of PointCloud
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> multi_region_; 
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> filtered_region_; 

    std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER> ground_;

    pcl::PointCloud<pcl::PointXYZ> landmark_;

    bool cloud_received_;

    // lidar location
    double lidar_height_;
    
    // Base Plane model
    pcl::ModelCoefficients base_coeff_;

    std::vector<double> section_distance_;

    // Find coefficient of plane and inlier with RANSAC
    template<class PointT>
    pcl::ModelCoefficients estimatePlane(pcl::PointCloud<PointT>& cloud);

    // Segment Point Clouds by Horizonal Angle
    void segmentMultiRegion_();

    // Filtering outlier for the ground points with the Multi-Region Plane Model 
    void filterGround_();

    // Compute Angle[rad] between two plane model
    double computeAngleTwoPlane(
            const pcl::ModelCoefficients& coeff1, 
            const pcl::ModelCoefficients& coeff2);
    
    double computeHeightDiffTwoPlane(const double distance,
        const std::pair<double, double> section_direction,
        const pcl::ModelCoefficients& coeff1, 
        const pcl::ModelCoefficients& coeff2);

    template <class PointT>
    void removeInliner(pcl::PointCloud<PointT>& cloud,
            pcl::ModelCoefficients& coeff);

    static int    RING_TO_ANALYZE;
    static double BASE_THRESHOLD;
    static double GROUND_THRESHOLD;
    static int    RING_TO_FIT_BASE;
    static double FIT_PLANE_THRESHOLD;
    static int    LOCAL_WINDOW_SIZE;
    static double NOISE_THRESHOLD;
    static double TANGENT_INCLINE;
    static double ANGLE_THRESHOLD;
    static double SECTION_START_ANGLE;
    static int    SECTION_NUMBER;
    static double SECTION_DISTANCE;
    static double HEIGHT_DIFF_THRESHOLD;
};
#endif /* FEATURE_DETECTOR_H */
