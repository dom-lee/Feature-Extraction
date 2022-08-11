/*******************************************************************************
 * File:        feature_extractor.h
 * 
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/29/2022
 * 
 * Description: Extract Curb and boundary of grass with LiDAR
*******************************************************************************/
#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

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
#include <Eigen/QR>

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
#include "setting/feature_extractor_setting_t.h"
#include "setting/lidar_setting_t.h"

static const int RING_NUMBER = 32;

class FeatureExtractor
{
public:
    FeatureExtractor(lidar_setting_t lidar_setting);

    void changeSetting(feature_extractor_setting_t setting);

    void setInputCloud(
        std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER>& rings);

    void run();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLandmark();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getA();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getB();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getC();
    std::vector<std::pair<double, double>> getBeamA();

    template <class PointT>
    double calculateHeight(const PointT& point);

private:
    // Feature Extractor Parameter
    feature_extractor_setting_t setting_;

    // Property of LiDAR
    std::vector<double> elevation_angles_;
    double initial_lidar_height_;
    pcl::PointXYZ origin_;

    // Octant(8) x SECTION_NUMBER matrix of PointCloud
    // label = ring_id
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> multi_region_; 
    std::array<std::vector<pcl::PointCloud<pcl::PointXYZL>>, 8> filtered_region_; 
    
    // Beam Modeal
    // {azimuth, distance}
    std::vector<std::pair<double, double>> max_distance_beam_zone_;
    std::vector<std::pair<double, double>> min_distance_beam_zone_;
    std::vector<std::pair<double, double>> free_beam_endpoints_;
    std::pair<double, double> intersection_;
    std::vector<double> roads_angle_;

    // section_numer = ceil(RING_TO_ANALYZE / 2)
    int section_number_;
    std::array<std::pair<double, double>, 8> section_direction_;
    std::array<std::vector<double>, 8> section_distances_;

    // x,y,z
    std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER> ground_;
    std::array<std::vector<double>, RING_NUMBER> ground_height_;

    pcl::PointCloud<pcl::PointXYZ> landmark_;
    pcl::PointCloud<pcl::PointXYZ> a_test_;
    pcl::PointCloud<pcl::PointXYZ> b_test_;
    pcl::PointCloud<pcl::PointXYZ> c_test_;

    // lidar location
    double estimated_lidar_height_;
    
    // Base Plane model
    bool base_plane_estimated_;
    pcl::ModelCoefficients base_coeff_;
    std::array<std::vector<pcl::ModelCoefficients>, 8> multi_region_plane_coeff_;

    // Estimate Base Planar to roughly estimate LiDAR pose
    template<class PointT>
    bool estimateBasePlane_(std::array<pcl::PointCloud<PointT>, RING_NUMBER>& rings);

    // Find coefficient of plane and inlier with RANSAC
    template<class PointT>
    pcl::ModelCoefficients estimatePlane_(pcl::PointCloud<PointT>& cloud);

    // Estimate Road Model with Beam Model
    void estimateRoadModel_();

    // Filtering outlier for the ground points with the Multi-Region Plane Model 
    void estimateGround_();

    void extractCurb_();

    template <class PointT>
    void processObstacleForBeamModel_(PointT& obstacle);

    // Compute Angle[rad] between two plane model
    double computeAngleTwoPlane_(
            const pcl::ModelCoefficients& coeff1, 
            const pcl::ModelCoefficients& coeff2);
    
    double computeHeightDiffTwoPlane_(
        const std::pair<double, double> boundary_point,
        const pcl::ModelCoefficients& coeff1, 
        const pcl::ModelCoefficients& coeff2);

    template <class PointT>
    void removeInliner_(pcl::PointCloud<PointT>& cloud,
            pcl::ModelCoefficients& coeff);
};
#endif /* FEATURE_extractor_H */
