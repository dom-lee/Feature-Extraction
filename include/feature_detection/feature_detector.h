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

// Settings
#include "setting/feature_detector_setting_t.h"

static const int RING_NUMBER = 32;

class FeatureDetector
{
public:
    FeatureDetector();

    // Compute Base Plane & get Translation Matrix
    void setInputCloud(feature_detector_setting_t setting,
            std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER>& rings);

    void run();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLandmark();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getA();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getB();

private:
    // Setting Parameter
    feature_detector_setting_t setting_;

    // Octant(8) x SECTION_NUMBER matrix of PointCloud
    // label = ring_id
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> multi_region_; 
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> filtered_region_; 

    // x,y,z,diff_ground
    std::array<pcl::PointCloud<pcl::PointWithRange>, RING_NUMBER> ground_;

    pcl::PointCloud<pcl::PointXYZ> landmark_;
    pcl::PointCloud<pcl::PointXYZ> a_test_;
    pcl::PointCloud<pcl::PointXYZ> b_test_;

    bool cloud_received_;

    // lidar location
    double lidar_height_;
    
    // Base Plane model
    pcl::ModelCoefficients base_coeff_;

    std::vector<double> section_distance_;

    // Find coefficient of plane and inlier with RANSAC
    template<class PointT>
    pcl::ModelCoefficients estimatePlane_(pcl::PointCloud<PointT>& cloud);

    // Segment Point Clouds by Horizonal Angle
    void segmentMultiRegion_();

    // Filtering outlier for the ground points with the Multi-Region Plane Model 
    void filterGround_();

    void extractCurb_();

    // Compute Angle[rad] between two plane model
    double computeAngleTwoPlane_(
            const pcl::ModelCoefficients& coeff1, 
            const pcl::ModelCoefficients& coeff2);
    
    double computeHeightDiffTwoPlane_(const double distance,
        const std::pair<double, double> section_direction,
        const pcl::ModelCoefficients& coeff1, 
        const pcl::ModelCoefficients& coeff2);

    template <class PointT>
    void removeInliner_(pcl::PointCloud<PointT>& cloud,
            pcl::ModelCoefficients& coeff);
};
#endif /* FEATURE_DETECTOR_H */
