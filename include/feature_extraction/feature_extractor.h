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
#include <pcl/common/pca.h>
#include <pcl/common/centroid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/model_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>

// General Functions
#include "douglasPeucker.hpp"
#include "estimatePlaneRANSAC.hpp"
#include "bresenhamLine.hpp"

// Utils
#include "utils/timing.h"
#include "utils/debugger.h"

// Settings
#include "setting/feature_extractor_setting_t.h"
#include "setting/lidar_setting_t.h"

static const int RING_NUMBER = 32;

enum Extractor_modes_t
{
    CURB,
    WALL,
};

class FeatureExtractor
{
public:
    // Constructor
    FeatureExtractor(lidar_setting_t lidar_setting,
                     int mode);

    // Change Setting Parameters
    void changeSetting(feature_extractor_setting_t setting);

    // Set PointCloud for feature extraction
    void setInputCloud(
        std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER>& rings);

    // Execute Feature Extraction
    void run();

    // Getter Functions
    pcl::PointCloud<pcl::PointXYZ>::Ptr getGround();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getLandmark();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getObstacles();

    pcl::PointCloud<pcl::PointXYZ>::Ptr getA();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getB();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getC();
    pcl::PointCloud<pcl::PointXYZ>::Ptr getD();

    // Visualize Base
    Eigen::Vector4f getBasePlane();

    // Visualize Curb Extraction Process
    std::vector<pcl::PointXYZ> getFittedLines();

    // Visualize Wall Extraction Process
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> getCluster();
    std::vector<pcl::PointXYZ> getGridNormals();

    // Visualize Glass Detection Process
    std::vector<Eigen::Vector4f> getGlassPlanes();

    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> getBottomBeam();
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> getTopBeam();

private:
    // Mode (0: Curb Extractor, 1: Wall Extractor)
    Extractor_modes_t mode_;

    // Feature Extractor Parameter
    feature_extractor_setting_t setting_;

    // Property of LiDAR
    std::vector<double> elevation_angles_;
    double estimated_lidar_height_;
    pcl::PointXYZ origin_;

    // Base Plane model
    bool base_plane_updated_;
    Eigen::Vector4f base_coeff_;
    
    // Tramsformation
    Eigen::Matrix4f transformation_;
    std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER> transformed_rings_;

    // Occupancy Grid (Transformed)
    std::vector<std::vector<int>> grid_ground_;
    // PointCloud(Ground, Ceiling Removed) for each grid cell
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZ>>> grid_cloud_;
    // Normal Vector and Mean XYZ for Grid
    std::vector<std::vector<Eigen::Vector3f>> grid_normals_;
    std::vector<std::vector<pcl::PointXYZ>> grid_centroid_;

    // Ground and Obstacles (Results from Ground Extraction)
    std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER> transformed_ground_;
    pcl::PointCloud<pcl::PointXYZ> obstacles_;

    // Fitted Lines from Douglas-Peucker Algorithm
    std::array<std::vector<pcl::PointXYZ>, RING_NUMBER> fitted_lines_;

    // Glass
    std::vector<Eigen::Vector4f> glass_planes_coeff_;


    // Beam Modeal
    // {azimuth, distance}
    std::vector<std::pair<double, double>> bottom_beam_;
    std::vector<std::pair<double, double>> top_beam_;
    std::pair<double, double> intersection_;

    // Results
    pcl::PointCloud<pcl::PointXYZ> landmark_;
    pcl::PointCloud<pcl::PointXYZ> a_test_;
    pcl::PointCloud<pcl::PointXYZ> b_test_;
    pcl::PointCloud<pcl::PointXYZ> c_test_;
    pcl::PointCloud<pcl::PointXYZ> d_test_;
    std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters_;

    // Estimate Base Planar to roughly estimate LiDAR pose
    template <class PointT>
    bool estimateBasePlane_(std::array<pcl::PointCloud<PointT>, RING_NUMBER>& rings);

    // Find Wall
    void extractWall_();

    // Cluster Wall by checking Normal Vector in DFS way
    void clusterGridDFS_(std::vector<std::vector<int>>& grid_visited,
                         int seed_m, int seed_n, int m, int n,
                         pcl::PointCloud<pcl::PointXYZ>& cluster,
                         std::vector<std::pair<int, int>>& grid_idx);
    
    // Detect Glass
    void detectGlass_();

    // Extract Ground and Find Obstacles with grid method
    void extractGround_();
    
    // Extract Curb
    void extractCurb_();

    // Estimate Road Model with Beam Model
    void estimateRoadModel_();

    // Find Min Distance for Angular Section
    double processBeamModel_(double center_x, double center_y, 
                             std::vector<std::pair<double, double>>& out_beam_model);

    // Compute Distance from point in box to intersection
    double calculateDistanceInBox_(double center_x, double center_y,
                                   double theta, double box_length);
    
    // Fitting Sorted PointCloud(same ring_id) with Douglas-Peucker Algorithm
    template <class PointT>
    void fitPointCloud_(const pcl::PointCloud<PointT>& in_ring,
                        std::vector<PointT>& out_line_list);

    // Check Whether Finite Line represenrs Ground
    template <class PointT>
    bool checkIsGroundLine_(const PointT& line_end_1, const PointT& line_end_2);
};
#endif /* FEATURE_extractor_H */
