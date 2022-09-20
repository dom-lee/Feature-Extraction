/*******************************************************************************
 * File:        feature_extractor.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: Extract Curb and boundary of grass with LiDAR
*******************************************************************************/
#include "feature_extraction/feature_extractor.h"

using namespace bipedlab;

FeatureExtractor::FeatureExtractor(lidar_setting_t lidar_setting) 
{
    // Set Property of LiDAR
    elevation_angles_ = lidar_setting.elevation_angles;
    estimated_lidar_height_ = lidar_setting.height;
    origin_ = pcl::PointXYZ(0, 0, 0);

    base_plane_updated_ = false;

    debugger::debugTitleTextOutput("[Feature Extractor]", "Constructed", 10, BC);
}

void FeatureExtractor::changeSetting(feature_extractor_setting_t setting)
{
    setting_ = setting;

    debugger::debugTitleTextOutput("[Feature Extractor]", "Setting Changed", 10, BG);
}

void FeatureExtractor::setInputCloud(
    std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER>& rings)
{
    landmark_.clear();
    a_test_.clear();
    b_test_.clear();
    c_test_.clear();

    // Estimate Base Plane
    if (!estimateBasePlane_(rings))
    {
        return;
    }

    // Sort Point Cloud and Fit Line
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        // Transform PointCloud based on Base Plane
        pcl::transformPointCloud(rings[i], transformed_rings_[i], transformation_);

        // Sort Rings by azimuth
        std::sort(transformed_rings_[i].begin(), transformed_rings_[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
    }

    debugger::debugColorTextOutput("Finish SetInputCloud", 1, BG);
}

void FeatureExtractor::run()
{
    if (base_plane_updated_)
    {
        debugger::debugColorTextOutput("[Feature Extractor] Execute", 5, BW);
        extractWall_();
    }
    else
    {
        debugger::debugColorTextOutput("[Feature Extractor] Base Error", 10, BY);
    }
}

// Getter Functions
pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getGround()
{
    pcl::PointCloud<pcl::PointXYZ> ground;
    for (int i = 0; i < setting_.RING_TO_ANALYZE; ++i)
    {
        ground += ground_[i];
    }

    return ground.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getObstacles()
{
    return obstacles_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getLandmark()
{
    
    return landmark_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getA()
{
    return a_test_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getB()
{
    return b_test_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getC()
{
    return c_test_.makeShared();
}

std::vector<pcl::PointXYZ> FeatureExtractor::getFittedLines()
{
    std::vector<pcl::PointXYZ> fitted_lines;
    for (int i = 0; i < fitted_lines_.size(); ++i)
    {
        fitted_lines.insert(fitted_lines.end(),
                            fitted_lines_[i].begin(), fitted_lines_[i].end());
    }
    return fitted_lines;
}

std::vector<pcl::PointXYZ> FeatureExtractor::getGroundLines()
{
    return ground_lines_;
}

std::vector<pcl::PointXYZ> FeatureExtractor::getGridNormals()
{
    std::vector<pcl::PointXYZ> normal_vectors;

    for (int m = 0; m < grid_normals_.size(); ++m)
    {
        for (int n = 0; n < grid_normals_[m].size(); ++n)
        {
            Eigen::Vector3f& normal_vector = grid_normals_[m][n];
            if (normal_vector.isZero(0))
            {
                continue;
            }

            normal_vectors.push_back(grid_centroid_[m][n]);

            pcl::PointXYZ point_normal = grid_centroid_[m][n];
            point_normal.x += normal_vector(0) * setting_.GRID_LENGTH;
            point_normal.y += normal_vector(1) * setting_.GRID_LENGTH;
            point_normal.z += normal_vector(2) * setting_.GRID_LENGTH;
            normal_vectors.push_back(point_normal);
        }
    }
    return normal_vectors;
}

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> FeatureExtractor::getBottomBeam()
{
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> beam_lines;
    pcl::PointXYZ point;
    for (auto& beam : bottom_beam_)
    {
        point.x = beam.first;
        point.y = beam.second;
        point.z = 0;

        beam_lines.push_back({origin_, point});
    }
    return beam_lines;
}

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> FeatureExtractor::getTopBeam()
{
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> beam_lines;

    pcl::PointXYZ intersection(intersection_.first, intersection_.second, 0);

    pcl::PointXYZ point;
    for (auto& beam : top_beam_)
    {
        point.x = beam.first;
        point.y = beam.second;
        point.z = 0;

        beam_lines.push_back({intersection, point});
    }
    return beam_lines;
}

Eigen::Vector4f FeatureExtractor::getBasePlane()
{
    return base_coeff_;
}

Eigen::Vector4f FeatureExtractor::getCeilingPlane()
{
    return ceiling_coeff_;
}

template <class PointT>
bool FeatureExtractor::estimateBasePlane_(
    std::array<pcl::PointCloud<PointT>, RING_NUMBER>& rings)
{
    // Sampling Candidates for Base Planar by filtering wall
    pcl::PointCloud<PointT> base_candidate;

    // Split Segment by azimuth
    const int section_number = 360;

    // Save {r, z} of point that have minimum Z value for each section
    std::vector<std::pair<double, double>> prev_ring_r_z(section_number, {0, 0});
    std::vector<std::pair<double, double>> curr_ring_r_z(section_number, {0, 0});

    // Save PointClouds for section
    std::vector<pcl::PointCloud<PointT>> prev_ring_cloud(section_number);
    std::vector<pcl::PointCloud<PointT>> curr_ring_cloud(section_number);

    for (int i = 0; i < setting_.RING_TO_FIT_BASE; ++i)
    {
        int ring_size = rings[i].size();
        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZ& point = rings[i][j];

            double azimuth = std::atan2(point.y, point.x);
            double r = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

            int azimuth_idx = int(azimuth * 180 / M_PI + 180) % 360;

            // Updated curr_ring_r_z when the point has a lower z value
            if (curr_ring_cloud[azimuth_idx].empty() ||
                point.z < curr_ring_r_z[azimuth_idx].second)
            {
                curr_ring_r_z[azimuth_idx] = {r, point.z};
            }
            curr_ring_cloud[azimuth_idx].push_back(point);
        }

        // Skip checking gradient for Ring 0
        if (i == 0)
        {
            std::swap(curr_ring_r_z, prev_ring_r_z);
            std::swap(curr_ring_cloud, prev_ring_cloud);
            continue;
        }

        // Calculate Gradient for each section
        for (int k = 0; k < section_number; ++k)
        {
            // Skip when section is empty
            if (curr_ring_cloud[k].empty())
            {
                continue;
            }

            double gradient = (curr_ring_r_z[k].second - prev_ring_r_z[k].second) /
                              (curr_ring_r_z[k].first  - prev_ring_r_z[k].first);

            // [* Note *] Check absolute value of gradient 
            if (std::abs(gradient) < setting_.GRADIENT_THRESHOLD)
            {
                base_candidate += prev_ring_cloud[k];
            }

            prev_ring_r_z[k] = curr_ring_r_z[k];
            prev_ring_cloud[k] = curr_ring_cloud[k];

            curr_ring_r_z[k] = {0, 0};
            curr_ring_cloud[k].clear();
        }
    } // Finish Sampling Candidates for Base

    if (base_candidate.size() < 10)
    {
        PCL_ERROR("Not Enough Candidates for Base estimation \n");
        base_plane_updated_ = false;
        return false;
    }

    // Set Base Planar Model (ax + by + cz + d = 0)
    pcl::PointIndices inliers;
    auto tmp_base_coeff = estimatePlaneRANSAC(base_candidate,
                                              setting_.BASE_FIT_THRESHOLD,
                                              inliers);
    debugger::debugColorOutput("Base Coefficients \n", tmp_base_coeff, 3, BG); 

    // Sanity Check for Vertical Base Planar
    if (tmp_base_coeff(2) < 0.8)
    {
        PCL_ERROR("Base Planar Model is Vertical! \n");
        base_plane_updated_ = false;
        return false;
    }

    base_plane_updated_ = true;
    base_coeff_ = tmp_base_coeff;

    // Calculated Transformation Matrix
    // Rotation Matrix
    // Row-Wise Normalized 
    // | c  0 -a |
    // | 0  c -b |
    // | a  b  c | 
    transformation_ = Eigen::Matrix4f::Identity(4, 4);
    transformation_.block<3, 3>(0, 0) <<
        base_coeff_(2), 0,              -base_coeff_(0),
        0,              base_coeff_(2), -base_coeff_(1),
        base_coeff_(0), base_coeff_(1), base_coeff_(2);
    transformation_.rowwise().normalize();

    debugger::debugColorOutput("Transformation \n", transformation_, 3, BB); 
    
    // Estimated LiDAR Height from ground : |d| 
    estimated_lidar_height_ = std::abs(base_coeff_(3));
    debugger::debugColorOutput("Estimated LiDAR Height : ",
                               estimated_lidar_height_, 5, BW); 

    return true;
}

void FeatureExtractor::extractWall_()
{
    // Grid Specification (Grid size is odd)
    double grid_length = setting_.GRID_LENGTH;
    int grid_size = 2 * setting_.GRID_RANGE + 1;
    double max_distance = grid_length * (setting_.GRID_RANGE + 0.5);

    // Index of Center Grid
    int center_m = setting_.GRID_RANGE;
    int center_n = setting_.GRID_RANGE;

    // {min_height, max_height} for grid
    std::vector<std::vector<double>> grid_min_z(grid_size,
        std::vector<double>(grid_size, INFINITY));
    std::vector<std::vector<double>> grid_max_z(grid_size,
        std::vector<double>(grid_size, -INFINITY));

    // Clear Transformed PointCloud in Grid
    grid_cloud_.clear();
    grid_cloud_.resize(grid_size,
                       std::vector<pcl::PointCloud<pcl::PointXYZ>>(grid_size));
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            grid_cloud_[m][n].clear();
        }
    }

    // Save PointCloud into Grid
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        for (int j = 0; j < transformed_rings_[i].size(); ++j)
        {
            pcl::PointXYZ& point = transformed_rings_[i][j];

            // Filter outside range
            if ((point.x < 0 && point.x > -2.0 && std::abs(point.y) < 0.5) ||
                std::abs(point.x) >= max_distance || 
                std::abs(point.y) >= max_distance)
            {
                continue;
            }
            

            // Visualization for Ground
            if (point.z < -estimated_lidar_height_ + 0.2 &&
                point.z > -estimated_lidar_height_ - 0.2)
            {
                c_test_.push_back(point);
            }
            // Visualization

            // Filter Ground and Ceiling
            if (point.z < -estimated_lidar_height_ + 0.2 ||
                point.z > -estimated_lidar_height_ + 1.5)
            {
                continue;
            }
            a_test_.push_back(point);

            // Index of Grid 
            int m = center_m + (point.x + grid_length / 2) / grid_length;
            int n = center_n + (point.y + grid_length / 2) / grid_length;

            // Update Min/Max Height
            grid_min_z[m][n] = std::min((double)point.z, grid_min_z[m][n]);
            grid_max_z[m][n] = std::max((double)point.z, grid_max_z[m][n]);

            grid_cloud_[m][n].push_back(point);
        }
    }

    // Resize and Clear
    grid_normals_.clear();
    grid_normals_.resize(grid_size,
                         std::vector<Eigen::Vector3f>(grid_size, {0, 0, 0}));
    grid_centroid_.clear();
    grid_centroid_.resize(grid_size,
                          std::vector<pcl::PointXYZ>(grid_size, {0, 0, 0}));

    // Compute PCA for each grid and get Normal Vector
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            // Filter Grid when not enough points
            if (grid_cloud_[m][n].size() < 3 ||
                grid_max_z[m][n] - grid_min_z[m][n] < setting_.WALL_HEIGHT_THRESHOLD)
            {
                continue;
            }

            //// Execute PCA for each grid
            //pcl::PCA<pcl::PointXYZ> pca;
            //pca.setInputCloud(grid_cloud_[m][n].makeShared());
            //Eigen::Vector3f normal_vector = pca.getEigenVectors().col(2);
            pcl::PointIndices inliers;
            auto wall_coeff = estimatePlaneRANSAC(grid_cloud_[m][n],
                                                  setting_.WALL_FIT_THRESHOLD,
                                                  inliers);
            Eigen::Vector3f normal_vector = wall_coeff.head(3);
            
            // Filter Normal Vector which is not perpendicular to the ground
            if (std::abs(normal_vector(2)) > 0.1)
            {
                continue;
            }
            grid_normals_[m][n] = normal_vector;
            pcl::computeCentroid(grid_cloud_[m][n], inliers.indices,
                                 grid_centroid_[m][n]);
        }
    }

    // Clustering by DFS (with checking normal)
    std::vector<std::vector<int>> grid_visited(grid_size,
                                               std::vector<int>(grid_size, 0));
    pcl::PointCloud<pcl::PointXYZ> transformed_cluster;
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            // Skip Grid 
            if (grid_normals_[m][n].isZero(0) ||
                grid_visited[m][n] != 0)
            {
                continue;
            }

            clusterGridDFS_(grid_visited, m, n, m, n, transformed_cluster);
            
            // Inverse Transform from transformed_cluster to cluster
            pcl::PointCloud<pcl::PointXYZ> cluster;
            pcl::transformPointCloud(transformed_cluster, cluster,
                                     transformation_.inverse());
            transformed_cluster.clear();

            // Estimate Wall Plane Coeff from Cluster
            pcl::PointIndices inliers;
            auto wall_coeff = estimatePlaneRANSAC(cluster,
                                                  setting_.WALL_FIT_THRESHOLD,
                                                  inliers);
        
            b_test_ = cluster;
            
            // Find EndPoints 
            pcl::PointXYZ wall_end_point_1;
            pcl::PointXYZ wall_end_point_2;
        }
    }

    //ground_lines_.clear();
    //for (int i = 0; i < fitted_lines_.size(); ++i)
    //{
        //for (int j = 0; j < (int)fitted_lines_[i].size() / 2; ++j)
        //{
            //if (checkIsGroundLine_(fitted_lines_[i][2 * j],
                                   //fitted_lines_[i][2 * j + 1]))
            //{
                //ground_lines_.push_back(fitted_lines_[i][2 * j]);
                //ground_lines_.push_back(fitted_lines_[i][2 * j + 1]);
            //}

        //}
        
    //}

    //// Interpolate Fitted Lines by azimuth
    //double resolution = 2.0 * M_PI / (double)setting_.SECTION_NUMBER;

    //std::vector<std::vector<pcl::PointXYZ>> interpolated_fitted_lines(
        //fitted_lines_.size(), std::vector<pcl::PointXYZ>(setting_.SECTION_NUMBER));
    //for (int i = 0; i < fitted_lines_.size(); ++i)
    //{
        //for (int j = 0; j < (int)fitted_lines_[i].size() / 2; ++j)
        //{
            //double x1 = fitted_lines_[i][2 * j].x;
            //double y1 = fitted_lines_[i][2 * j].y;
            //double z1 = fitted_lines_[i][2 * j].z;
            //double x2 = fitted_lines_[i][2 * j + 1].x;
            //double y2 = fitted_lines_[i][2 * j + 1].y;
            //double z2 = fitted_lines_[i][2 * j + 1].z;

            //double azimuth_start = std::atan2(y1, x1);
            //double azimuth_end   = std::atan2(y2, x2);
            
            //int start_idx = std::ceil((azimuth_start + M_PI) / resolution);
            //int end_idx   = std::floor((azimuth_end + M_PI) / resolution);

            //for (int k = start_idx; k < end_idx + 1; ++k)
            //{
                //double azimuth = k * resolution - M_PI;
                //double r = (y1 * x2 - x1 * y2) /
                           //((y1 - y2) * std::cos(azimuth) - 
                            //(x1 - x2) * std::sin(azimuth));
                
                //pcl::PointXYZ interpolated_point;
                //interpolated_point.x = r * std::cos(azimuth);
                //interpolated_point.y = r * std::sin(azimuth);
                //interpolated_point.z = z1 + (z1 - z2) / (x1 - x2) *
                                       //(interpolated_point.x - x1);

                //a_test_.push_back(interpolated_point);
            //}
        //}
    //}
}

void FeatureExtractor::clusterGridDFS_(std::vector<std::vector<int>>& grid_visited,
                                       int seed_m, int seed_n, int m, int n,
                                       pcl::PointCloud<pcl::PointXYZ>& cluster)
{
    // Check for Seed
    if (seed_m < 0 || seed_m >= grid_visited.size() ||
        seed_n < 0 || seed_n >= grid_visited[0].size() ||
        grid_normals_[seed_m][seed_n].isZero(0))
    {
        debugger::debugColorTextOutput("Check Seed Index", 5, BY);
        return;
    }

    // Check for current grid_idx
    if (m < 0 || m >= grid_visited.size() ||
        n < 0 || n >= grid_visited[0].size() ||
        grid_normals_[m][n].isZero(0))
    {
        return;
    }

    // Add point cloud into cluster
    cluster += grid_cloud_[m][n];

    // Exit for visited Grid
    if (grid_visited[m][n] == 1)
    {
        return;
    }

    // Filter
    Eigen::Vector3f& seed_normal = grid_normals_[seed_m][seed_n];
    Eigen::Vector3f& current_normal = grid_normals_[m][n];

    double cos_angle = seed_normal.dot(current_normal) /
                       seed_normal.norm() / current_normal.norm();
    cos_angle = (cos_angle > 1) ? 1 : cos_angle;
    cos_angle = (cos_angle < -1) ? -1 : cos_angle;

    double angle_diff = std::acos(cos_angle);

    // Filter when there's a big difference in angle between normal vectors
    if (angle_diff > setting_.CLUSTER_ANGLE_THRESHOLD)
    {
        return;
    }

    // Filter by distance from plane to centroid
    Eigen::Vector3f centroid_diff = grid_centroid_[m][n].getVector3fMap() -
                                    grid_centroid_[seed_m][seed_n].getVector3fMap();
    double distance_diff = centroid_diff.dot(seed_normal) / seed_normal.norm();
    if (distance_diff > setting_.CLUSTER_DIST_THRESHOLD)
    {
        return;
    }
   
    // Mark as Visited
    grid_visited[m][n] = 1;

    // get neighbors that line(vertical to normal vector) pass
    double start_x = grid_centroid_[m][n].x +
                     setting_.GRID_LENGTH * grid_normals_[m][n](1);
    double start_y = grid_centroid_[m][n].y - 
                     setting_.GRID_LENGTH * grid_normals_[m][n](0);
    double end_x   = grid_centroid_[m][n].x -
                     setting_.GRID_LENGTH * grid_normals_[m][n](1);
    double end_y   = grid_centroid_[m][n].y +
                     setting_.GRID_LENGTH * grid_normals_[m][n](0);
    
    std::vector<std::pair<int, int>> on_grid_idxs;
    executeBresenhamLine(start_x, start_y, end_x, end_y,
                         setting_.GRID_LENGTH, on_grid_idxs);

    // Index of Center Grid
    int center_m = setting_.GRID_RANGE;
    int center_n = setting_.GRID_RANGE;

    // DFS for Neighbors
    for (auto& idx : on_grid_idxs)
    {
        if (center_m + idx.first == m && center_n + idx.second == n)
        {
            continue;
        }

        clusterGridDFS_(grid_visited, seed_m, seed_n,
                        center_m + idx.first, center_n + idx.second, cluster);
    }
}

void FeatureExtractor::extractGround_()
{
    // Clear Old Data
    obstacles_.clear();
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        ground_[i].clear();
    }

    // Grid Specification (Grid size is odd)
    double grid_length = setting_.GRID_LENGTH;
    int grid_size = 2 * setting_.GRID_RANGE + 1;
    double max_distance = grid_length * (setting_.GRID_RANGE + 0.5);

    // Index of Center Grid
    int center_m = setting_.GRID_RANGE;
    int center_n = setting_.GRID_RANGE;

    // {min_height, max_height} for grid
    std::vector<std::vector<double>> grid_min_z(grid_size,
        std::vector<double>(grid_size, INFINITY));
    std::vector<std::vector<double>> grid_max_z(grid_size,
        std::vector<double>(grid_size, -INFINITY));

    // Transformed PointCloud in Grid
    std::vector<std::vector<pcl::PointCloud<pcl::PointXYZL>>> grid_cloud(grid_size,
        std::vector<pcl::PointCloud<pcl::PointXYZL>>(grid_size));

    // Pre-Compute Grid for Filtering Ground
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        int ring_size = transformed_rings_[i].size();

        // Pre-Compute Grid
        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZ& point = transformed_rings_[i][j];

            // Filter outside points
            if ((point.x < 2.0 && point.x > -5.0 && std::abs(point.y) < 3.0) ||
                std::abs(point.x) >= max_distance || 
                std::abs(point.y) >= max_distance)
            {
                continue;
            }

            // Index of Grid 
            int m = center_m + (point.x + grid_length / 2) / grid_length;
            int n = center_n + (point.y + grid_length / 2) / grid_length;

            // Update Only Height Difference is less than threshold
            // To avoid ground included grid (Trees Branches, Vehicle)
            if (!std::isinf(grid_max_z[m][n]) && 
                point.z - grid_max_z[m][n] > setting_.DISCONTINUITY_HEIGHT)
            {
                obstacles_.push_back(point);
                continue;
            }
           
            // Update Min/Max Height
            grid_min_z[m][n] = std::min((double)point.z, grid_min_z[m][n]);
            grid_max_z[m][n] = std::max((double)point.z, grid_max_z[m][n]);

            // Update PointCloud in Grid
            pcl::PointXYZL point_with_label;
            point_with_label.x = point.x;
            point_with_label.y = point.y;
            point_with_label.z = point.z;
            point_with_label.label = i;

            grid_cloud[m][n].push_back(point_with_label);

            // Filter Grass with Smooth Filter (ex. Grass)
            if (i < 3)
            {
                continue;
            }
        }
    } // End of Pre-Compute Grid

    // Estimate whether Cell is Ground or Not
    // -1 : Obstacle (Building, Vehicle)
    //  0 : Unknown
    //  1 : Ground
    grid_ground_ = std::vector<std::vector<int>>(grid_size,
                                                 std::vector<int>(grid_size, 0));
    std::vector<std::pair<int, int>> neighbors =
        {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    // Asign Center of Grid as Ground
    grid_ground_[center_m][center_n] = 1;
    Eigen::Vector4f transformed_base_coeff = transformation_ * base_coeff_;
    grid_min_z[center_m][center_n] = -transformed_base_coeff(3);
    grid_max_z[center_m][center_n] = -transformed_base_coeff(3);

    // Check Ground by propagation
    for (int range = 1; range <= setting_.GRID_RANGE; ++range)
    {
        std::vector<std::pair<int, int>> m_n_idx_list;
        // Top-Right to Top-Left
        for (int m = center_m + range, n = center_n - range;
             n < center_n + range ; ++n)
        {
            m_n_idx_list.push_back({m, n});
        }
        // Top-Left to Bottom-Left
        for (int m = center_m + range, n = center_n + range;
             m > center_m - range ; --m)
        {
            m_n_idx_list.push_back({m, n});
        }
        // Bottom-Left to Bottom-Right
        for (int m = center_m - range, n = center_n + range;
             n > center_n - range ; --n)
        {
            m_n_idx_list.push_back({m, n});
        }
        // Bottom-Right to Top-Right
        for (int m = center_m - range, n = center_n - range;
             m < center_m + range ; ++m)
        {
            m_n_idx_list.push_back({m, n});
        }
        
        // Iterate over Current Range of Cells
        for (auto& m_n_idx : m_n_idx_list)
        {
            int m = m_n_idx.first;
            int n = m_n_idx.second;

            // Check whether grid is ground based on neighbor ground
            double max_z_neighbor_ground = -INFINITY;
            double min_z_neighbor_ground = INFINITY;

            for (auto& neighbor : neighbors)
            {
                int tmp_m = m + neighbor.first;
                int tmp_n = n + neighbor.second;

                // Out of Grid
                if (tmp_m < 0 || tmp_m >= grid_size ||
                    tmp_n < 0 || tmp_n >= grid_size)
                {
                    continue;
                }

                // Update Min/Max Z value for neighbor ground
                // Only Consider Inner grid
                int range_neighbor = std::max(std::abs(tmp_m - center_m),
                                              std::abs(tmp_n - center_n));
                if (range_neighbor >= range)
                {
                    continue;
                }
                min_z_neighbor_ground = std::min(min_z_neighbor_ground,
                                                 grid_min_z[tmp_m][tmp_n]);
                max_z_neighbor_ground = std::max(max_z_neighbor_ground,
                                                 grid_max_z[tmp_m][tmp_n]);
            }

            // Update Min/Max Z value for Empty Cell
            if (grid_cloud[m][n].empty())
            {
                grid_ground_[m][n] = 0;

                // Reset Min/Max Z value for obstacle for ground propagation
                grid_max_z[m][n] = max_z_neighbor_ground;
                grid_min_z[m][n] = min_z_neighbor_ground;

                continue;
            }

            // Set Obstacle by checking height difference or 
            // Check Height Difference with Neighbor Ground Cells 
            // [*Important*] DownSlope has longer distance between ring
            double grid_height_diff = grid_max_z[m][n] -
                                      std::min(grid_min_z[m][n], max_z_neighbor_ground);
            if (grid_height_diff > setting_.OBSTACLE_THRESHOLD ||
                grid_max_z[m][n] > max_z_neighbor_ground + setting_.GROUND_THRESHOLD ||
                grid_min_z[m][n] < min_z_neighbor_ground - 3 * setting_.GROUND_THRESHOLD)
            {
                grid_ground_[m][n] = -1;

                // Reset Min/Max Z value for obstacle for ground propagation
                grid_max_z[m][n] = max_z_neighbor_ground;
                grid_min_z[m][n] = min_z_neighbor_ground;

                continue;
            }

            // Asign the Cell as Ground
            grid_ground_[m][n] = 1;
        }
    }

    // Save Ground and Obstacles
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            // Skip the empty Cell
            if (grid_cloud[m][n].empty())
            {
                continue;
            }

            // Save PointCloud into Ground / Obstacles
            for (auto& point_with_label : grid_cloud[m][n])
            {
                pcl::PointXYZ point;
                point.x = point_with_label.x;
                point.y = point_with_label.y;
                point.z = point_with_label.z;

                if (grid_ground_[m][n] != 1)
                {
                    obstacles_.push_back(point);
                }
                else
                {
                    ground_[point_with_label.label].push_back(point);
                }
            }
        }
    }

    // Sort Point Cloud by Couter Clock-wisely
    for (int i = 0; i < ground_.size(); ++i)
    {
        std::sort(ground_[i].begin(), ground_[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
    }
}

//void FeatureExtractor::extractCurb_()
//{
    //downsampled_lines_.resize(ground_.size());
    //pcl::PointCloud<pcl::PointXYZ> transformed_landmark;
    //for (int i = 0; i < ground_.size(); ++i)
    //{
        //downsampled_lines_[i].clear();

        //if (ground_[i].empty())
        //{
            //continue;
        //}
        //int ring_size = ground_[i].size();

        //// Precompute Line Section
        //int smooth_points_count = 1;
        //Eigen::Vector3f start_point = ground_[i][0].getVector3fMap();
        //for (int j = 0; j < ring_size; ++j)
        //{
            //Eigen::Vector3f point_curr = ground_[i][j].getVector3fMap();
            //Eigen::Vector3f point_next = ground_[i][(j + 1) % ring_size].getVector3fMap();

            //// Check Discontinuity
            //double azimuth_curr = std::atan2(point_curr(1), point_curr(0));
            //double azimuth_next = std::atan2(point_next(1), point_next(0));

            //double azimuth_diff = azimuth_next - azimuth_curr;
            //double distance_diff = (point_curr - point_next).norm();
            //bool is_next_point_discontinued = 
                //(azimuth_diff > setting_.DISCONTINUITY_AZIMUTH ||
                 //distance_diff > setting_.DISCONTINUITY_DISTANCE);

            //if (is_next_point_discontinued)
            //{
                //a_test_.push_back(ground_[i][(j + 1) % ring_size]);
            //}

            //// Skip when current_point is same with start_point
            //if (point_curr == start_point)
            //{
                //start_point = is_next_point_discontinued ? point_next : start_point;
                //smooth_points_count = is_next_point_discontinued ? 0 : 1;
                //continue;
            //}

            //// Corner Detection for next point by checking Curvature
            //Eigen::Vector3f sum_vector = Eigen::Vector3f::Zero();
            //for (int w = 0; w < setting_.SMOOTH_COUNT; ++w)
            //{
                //int idx_next = (j + w + 1) % ring_size;
                //int idx_prev = (j - w - 1 + ring_size) % ring_size;

                //sum_vector += (point_next - ground_[i][idx_next].getVector3fMap());
                //sum_vector += (point_next - ground_[i][idx_prev].getVector3fMap());
            //}
            //double smoothness = sum_vector.norm() / point_next.norm() /
                                //(2 * setting_.SMOOTH_COUNT);
            //double is_next_point_sharp = (smoothness > setting_.SMOOTHNESS_THRESHOLD);


            //if (is_next_point_sharp)
            //{
                //b_test_.push_back(ground_[i][(j + 1) % ring_size]);
            //}

            //// Check whether it needs to add downsampled lines
            //bool need_update_line = is_next_point_discontinued;
            //if (!need_update_line && is_next_point_sharp)
            //{
                //Eigen::Vector3f v_a = start_point - point_curr;
                //Eigen::Vector3f v_b = point_next - point_curr;
                //double angle = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());

                //if (angle < setting_.CONTINUITY_ANGLE)
                //{
                    //need_update_line = true;
                //}
            //}

            //// Update Downsampled Lines
            //if (need_update_line)
            //{
                //if (smooth_points_count >= 10)
                //{
                    //downsampled_lines_[i].push_back({start_point, point_curr});
                //}
                //start_point = is_next_point_discontinued ? point_next : point_curr;
                //smooth_points_count = is_next_point_discontinued ? 0 : 1;

                //pcl::PointXYZ pp(start_point(0), start_point(1), start_point(2));
            //}
            //smooth_points_count++;
        //} // End of Pre-Computing Feature Lines

        //// Execute Curb Extraction with FeatureLines
        //for (int k = 0; k < (int)downsampled_lines_[i].size() - 1; ++k)
        //{
            //// Assume current line is sidewalk
            //Eigen::Vector3f& start_curr = downsampled_lines_[i][k].first;
            //Eigen::Vector3f& end_curr   = downsampled_lines_[i][k].second;
            //pcl::PointXYZ start_point(start_curr(0), start_curr(1), start_curr(2));
            //pcl::PointXYZ end_point(end_curr(0), end_curr(1), end_curr(2));

            //// Adjacent Downsampled lines
            //Eigen::Vector3f& start_next = downsampled_lines_[i][k + 1].first;
            //Eigen::Vector3f& end_next   = downsampled_lines_[i][k + 1].second;
            //Eigen::Vector3f& start_prev = downsampled_lines_[i][k - 1].first;
            //Eigen::Vector3f& end_prev   = downsampled_lines_[i][k - 1].second;

            //// Vector for using angular threshold
            //Eigen::Vector2f v_a, v_b, v_c;
            //double angle_road_curb;

            //// Discontinued Curb
            //// Left: Sidewalk | Right: Road
            //double azimuth_diff_prev = std::atan2(start_curr(1), start_curr(0)) - 
                                       //std::atan2(end_prev(1), end_prev(0));
            //v_a = (start_prev - end_prev).head(2); // Road
            //v_b = (start_curr - end_prev).head(2); // Curb
            //angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                //start_curr(2) - end_prev(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                //end_prev.norm() > start_curr.norm() &&
                //(start_curr - end_curr).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                //(start_curr - end_curr).norm() < setting_.SIDEWALK_MAX_LENGTH)
            //{
                //transformed_landmark.push_back(start_point);
                //continue;
            //}
            
            //// Discontinued Curb
            //// Left: Road | Right: Sidewalk
            //double azimuth_diff_next = std::atan2(start_next(1), start_next(0)) -
                                       //std::atan2(end_curr(1), end_curr(0));
            //v_a = (end_next - start_next).head(2); // Road
            //v_b = (end_curr - start_next).head(2); // Curb 
            //angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (azimuth_diff_next < setting_.DISCONTINUITY_AZIMUTH &&
                //end_curr(2) - start_next(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                //start_next.norm() > end_curr.norm() &&
                //(start_curr - end_curr).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                //(start_curr - end_curr).norm() < setting_.SIDEWALK_MAX_LENGTH)
            //{
                //transformed_landmark.push_back(end_point);
                //continue;
            //}
           
            //// Continued Curb
            //// Left: Sidewalk | Center: Curb | Right: Road
            //v_a = (start_prev - end_prev).head(2); // Road
            //v_b = (end_curr - start_curr).head(2); // Curb
            //angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                //azimuth_diff_next < setting_.DISCONTINUITY_DISTANCE &&
                //end_curr(2) - start_curr(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //end_curr(2) - end_prev(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                //start_curr.norm() > end_curr.norm() &&
                //(start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                ////(end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(start_next - end_next).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                //(start_next - end_next).norm() < setting_.SIDEWALK_MAX_LENGTH)
            //{
                //transformed_landmark.push_back(start_point);
                //continue;
            //}

            //// Continued Curb
            //// Left: Road | Center: Curb | Right: Sidewalk
            //v_a = (end_next - start_next).head(2); // Road
            //v_b = (start_curr - end_curr).head(2); // Curb
            //angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (azimuth_diff_next < setting_.DISCONTINUITY_AZIMUTH &&
                //azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                //start_curr(2) - end_curr(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //start_curr(2) - start_next(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                //angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                //end_curr.norm() > start_curr.norm() &&
                //(end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(end_prev - start_prev).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                //(end_prev - start_prev).norm() < setting_.SIDEWALK_MAX_LENGTH)
            //{
                //transformed_landmark.push_back(end_point);
                //continue;
            //}
        //}
    //}

    //// Restoration
    //landmark_.clear();
    //pcl::transformPointCloud(transformed_landmark, landmark_, transformation_.transpose());
//}

void FeatureExtractor::estimateRoadModel_()
{
    bottom_beam_.clear();
    top_beam_.clear();

    // Bottom Layer Beam Model (emit beam based on LiDAR Origin)
    double max_score = processBeamModel_(0, 0, bottom_beam_);

    // Top Layer Beam Model
    for (auto& beam : bottom_beam_)
    {
        for (int i = 0; i < 30; ++i)
        {
            double theta = std::atan2(beam.second, beam.first);
            double x = (i + 1) * std::cos(theta);
            double y = (i + 1) * std::sin(theta);

            std::vector<std::pair<double, double>> new_beam;
            double score = processBeamModel_(x, y, new_beam);
            
            if (score > max_score)
            {
                max_score = score;
                intersection_ = {x, y};
                top_beam_ = new_beam;
            }
        }
    }

    std::pair<double, double> intersection = intersection_;
    std::vector<std::pair<double, double>> top_beam = top_beam_;
    for (auto& beam : top_beam)
    {
        for (int i = 0; i < 5; ++i)
        {
            double theta = std::atan2(beam.second - intersection.second,
                                      beam.first  - intersection.first);
            double x = intersection.first  + (i + 1) * std::cos(theta);
            double y = intersection.second + (i + 1) * std::sin(theta);

            std::vector<std::pair<double, double>> new_beam;
            double score = processBeamModel_(x, y, new_beam);
            if (score > max_score)
            {
                max_score = score;
                intersection_ = {x, y};
                top_beam_ = new_beam;
            }
        }
    }
}

double FeatureExtractor::processBeamModel_(double center_x, double center_y,
    std::vector<std::pair<double, double>>& out_beam_model)
{
    if (std::abs(center_x) > setting_.ROAD_VIEW_RANGE ||
        std::abs(center_y) > setting_.ROAD_VIEW_RANGE)
    {
        return INFINITY;
    }

    out_beam_model.clear();
    double score = 0.0;
    
    // Min / Max distance for angle section
    int section_number = setting_.BEAM_SECTION_NUMBER;
    std::vector<std::pair<double, double>>min_distance_beam(section_number);
    std::vector<std::pair<double, double>>max_distance_beam(section_number);

    // Grid Property
    double grid_length = setting_.GRID_LENGTH;
    int grid_size = 2 * setting_.GRID_RANGE + 1;
    int center_m = setting_.GRID_RANGE;
    int center_n = setting_.GRID_RANGE;
    double max_distance = setting_.ROAD_VIEW_RANGE;

    // Beam Model
    double resolution = 360.0 / (double)setting_.BEAM_SECTION_NUMBER;
    for (int z = 0; z < setting_.BEAM_SECTION_NUMBER; ++z)
    {
        double theta = (resolution * z - 180 + resolution / 2.0) * M_PI / 180;

        max_distance_beam[z].first = theta;
        max_distance_beam[z].second =
            calculateDistanceInBox_(center_x, center_y, theta, max_distance);
    }

    // Calculate Min Beam distance
    min_distance_beam = max_distance_beam;
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            // Skip Ground and Empty Cell
            if (grid_ground_[m][n] != -1)
            {
                continue;
            }
            
            int x = (m - center_m) * grid_length;
            int y = (n - center_n) * grid_length;
            
            if (std::abs(x) > setting_.ROAD_VIEW_RANGE ||
                std::abs(y) > setting_.ROAD_VIEW_RANGE)
            {
                continue;
            }

            double azimuth = std::atan2(y - center_y, x - center_x);
            double distance_xy = std::sqrt(std::pow(x - center_x, 2) +
                                           std::pow(y - center_y, 2));

            int idx = int((azimuth * 180 / M_PI + 180) / resolution);
            if (distance_xy < min_distance_beam[idx].second)
            {
                min_distance_beam[idx] = {azimuth, distance_xy};
            }

        }
    }

    // Count No Obstacle Section Clock-Wisely
    int max_beam_count = 0;
    for (int z = setting_.BEAM_SECTION_NUMBER - 1; z >= 0; --z)
    {
        if (min_distance_beam[z].second < max_distance_beam[z].second)
        {
            break;
        }
        max_beam_count++;
    }
    
    // Find Center of On-Road
    int last_idx = section_number - max_beam_count;
    for (int z = 0; z < last_idx; ++z)
    {
        if (min_distance_beam[z].second < max_distance_beam[z].second)
        {
            if (max_beam_count < 3)
            {
                max_beam_count = 0;
                continue;
            }

            int start_idx = (z - max_beam_count + section_number) % section_number;
            double start_x = center_x + min_distance_beam[start_idx].second *
                             std::cos(min_distance_beam[start_idx].first);
            double start_y = center_y + min_distance_beam[start_idx].second *
                             std::sin(min_distance_beam[start_idx].first);

            int end_idx = (z - 1 + section_number) % section_number;
            double end_x = center_x + min_distance_beam[end_idx].second *
                           std::cos(min_distance_beam[end_idx].first);
            double end_y = center_y + min_distance_beam[end_idx].second *
                           std::sin(min_distance_beam[end_idx].first);

            max_beam_count = 0;
            double road_width = std::sqrt(std::pow(start_x - end_x, 2) +
                                          std::pow(start_y - end_y, 2));
            if (road_width < setting_.ROAD_WIDTH_THRESHOLD)
            {
                continue;
            }

            double mean_x = (start_x + end_x) / 2;
            double mean_y = (start_y + end_y) / 2;

            // Calculate Score by counting ground cell on the beam
            std::vector<std::pair<int, int>> on_grid_idxs;
            executeBresenhamLine(center_x, center_y, mean_x, mean_y,
                                 grid_length, on_grid_idxs);
            
            double score_on_line = 0.0;
            for (auto& idx : on_grid_idxs)
            {
                if (grid_ground_[idx.first + center_m][idx.second + center_n] == 1)
                {
                    score_on_line++;
                }
            }
            
            if (score_on_line < setting_.GROUND_COUNT_THRESHOLD)
            {
                continue;
            }
            
            score += score_on_line;
            out_beam_model.push_back({mean_x, mean_y});
        }
        else
        {
            max_beam_count++;
        }
    }
    return score;
}

double FeatureExtractor::calculateDistanceInBox_(
    double center_x, double center_y, double theta, double box_length)
{
    // y = mx + b (line from center)
    double m = std::tan(theta);
    double b = -m * center_x + center_y;

    // intersection
    std::pair<double, double> intersection_vertical;
    std::pair<double, double> intersection_horizontal;
    if (theta < -M_PI / 2)
    {
        intersection_vertical.first    = -box_length;
        intersection_vertical.second   = -m * box_length + b;
        intersection_horizontal.first  = (-box_length - b) / m;
        intersection_horizontal.second = -box_length;
    }
    else if (theta < 0)
    {
        intersection_vertical.first    = box_length;
        intersection_vertical.second   = m * box_length + b;
        intersection_horizontal.first  = (-box_length - b) / m;
        intersection_horizontal.second = -box_length;
    }
    else if (theta < M_PI / 2)
    {
        intersection_vertical.first    = box_length;
        intersection_vertical.second   = m * box_length + b;
        intersection_horizontal.first  = (box_length - b) / m;
        intersection_horizontal.second = box_length;
    }
    else
    {
        intersection_vertical.first    = -box_length;
        intersection_vertical.second   = -m * box_length + b;
        intersection_horizontal.first  = (box_length - b) / m;
        intersection_horizontal.second = box_length;
    }

    double distance_to_verical = std::sqrt(
        std::pow(center_x - intersection_vertical.first, 2) + 
        std::pow(center_y - intersection_vertical.second, 2));
    double distance_to_horizontal = std::sqrt(
        std::pow(center_x - intersection_horizontal.first, 2) + 
        std::pow(center_y - intersection_horizontal.second, 2));

    return std::min(distance_to_verical, distance_to_horizontal);
}

void FeatureExtractor::executeBresenhamLine(
    double start_x, double start_y, double end_x, double end_y, double grid_length,
    std::vector<std::pair<int, int>>& out_grid_idxs)
{
    out_grid_idxs.clear();
    
    int x0 = std::floor((start_x + grid_length / 2) / grid_length);
    int y0 = std::floor((start_y + grid_length / 2) / grid_length);
    int x1 = std::floor((end_x + grid_length / 2) / grid_length);
    int y1 = std::floor((end_y + grid_length / 2) / grid_length);
    
    double dx = std::abs(x1 - x0);
    double sx = (x0 < x1) ? 1 : -1;
    double dy = -std::abs(y1 - y0);
    double sy = (y0 < y1) ? 1 : -1;
    double error = dx + dy;

    while (true)
    {
        out_grid_idxs.push_back({x0, y0});
        if (x0 == x1 && y0 == y1)
        {
            break;
        }

        double e2 = 2 * error;
        if (e2 >= dy)
        {
            if (x0 == x1)
            {
                break;
            }
            error += dy;
            x0 += sx;
        }
        if (e2 <= dx)
        {
            if (y0 == y1)
            {
                break;
            }
            error += dx;
            y0 += sy;
        }
    }
}

template <class PointT>
void FeatureExtractor::fitPointCloud_(const pcl::PointCloud<PointT>& in_ring,
                                      std::vector<PointT>& out_line_list)
{
    out_line_list.clear();

    int ring_size = in_ring.size();
    if (ring_size < 2)
    {
        return;
    }
    
    auto start_iter = in_ring.begin();
    for (auto it = in_ring.begin(); std::next(it) != in_ring.end(); ++it)
    {
        Eigen::Vector3f point_curr = (*it).getVector3fMap();
        Eigen::Vector3f point_next = (*(it + 1)).getVector3fMap();
        
        double distance_diff = (point_curr - point_next).norm();

        // Execute Douglas-Peucker for each cluster
        if (distance_diff > setting_.DISCONTINUITY_DISTANCE ||
            std::next(it, 2) == in_ring.end())
        {
            // Skip cluster less than 3 points
            if (std::distance(start_iter, it) < 2)
            {
                start_iter = it + 1;
                continue;
            }

            // Execute Douglas-Peucker Fitting Algorithm
            auto fitted_line = douglasPeucker<PointT>(start_iter, it,
                                                      setting_.EPSILON);
            
            // Save Line Segment
            for (int i = 0; i < (int)fitted_line.size() - 1; ++i)
            {
                out_line_list.push_back(fitted_line[i]);
                out_line_list.push_back(fitted_line[i + 1]);
            }

            start_iter = it + 1;
        }
    }
}

template <class PointT>
bool FeatureExtractor::checkIsGroundLine_(const PointT& line_end_1,
                                          const PointT& line_end_2)
{
    double distance = (pcl::pointToPlaneDistance(line_end_1, base_coeff_) +
                       pcl::pointToPlaneDistance(line_end_2, base_coeff_)) / 2;

    if (distance > setting_.GROUND_DIST_THRESHOLD)
    {
        return false;
    }

    // Estimated Ground Line by projecting ray on Base
    Eigen::Vector3f end_1 = line_end_1.getVector3fMap();
    Eigen::Vector3f end_2 = line_end_2.getVector3fMap();

    Eigen::Vector3f estimated_ground_end_1 = -end_1 * base_coeff_(3) /
                                             (end_1.dot(base_coeff_.head(3)));
    Eigen::Vector3f estimated_ground_end_2 = -end_2 * base_coeff_(3) /
                                             (end_2.dot(base_coeff_.head(3)));

    Eigen::Vector3f line_vec = end_1 - end_2;
    Eigen::Vector3f estimated_ground_line_vec = estimated_ground_end_1 -
                                                estimated_ground_end_2;

    double angle = std::acos(line_vec.dot(estimated_ground_line_vec) /
                             line_vec.norm() / estimated_ground_line_vec.norm());
   
    // Check Angle between Real Line and Estimated Line
    return angle < setting_.GROUND_ANGLE_THRESHOLD;
}
