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

    // Segmentation Option
    section_direction_ = {{ 
        {-1, 0}, {-0.7071, -0.7071}, {0, -1}, {0.7071, -0.7071},
        {1, 0}, {0.7071, 0.7071}, {0, 1}, {-0.7071, 0.7071} }};
   
    base_plane_updated_ = false;

    debugger::debugTitleTextOutput("[Feature Extractor]", "Constructed", 10, BC);
}

void FeatureExtractor::changeSetting(feature_extractor_setting_t setting)
{
    setting_ = setting;

    // Beam Model
    double resolution = 360.0 / (double)setting_.BEAM_SECTION_NUMBER;
    max_distance_beam_.resize(setting_.BEAM_SECTION_NUMBER);
    for (int z = 0; z < setting_.BEAM_SECTION_NUMBER; ++z)
    {
        double theta = (resolution * z - 180 + resolution / 2.0) * M_PI / 180;
        max_distance_beam_[z].first = theta;
        
        double angle = std::fmod(theta + M_PI, M_PI / 2);
        max_distance_beam_[z].second = (angle < M_PI / 4) ? 
            setting_.ROAD_VIEW_RANGE / std::cos(angle) :
            setting_.ROAD_VIEW_RANGE / std::sin(angle);
    }

    debugger::debugTitleTextOutput("[Feature Extractor]", "Setting Changed", 10, BG);
}

void FeatureExtractor::setInputCloud(
    std::array<pcl::PointCloud<pcl::PointXYZ>, RING_NUMBER>& rings)
{
    a_test_.clear();
    b_test_.clear();
    c_test_.clear();
    landmark_.clear();


    // Estimate Base Plane
    if (!estimateBasePlane_(rings))
    {
        return;
    }
    
    // Transform the Point Cloud to make base parallel to the XY plane
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        pcl::transformPointCloud(rings[i], transformed_rings_[i], transformation_);

        //c_test_ += transformed_rings_[i];

        // Sort Rings by azimuth
        std::sort(transformed_rings_[i].begin(), transformed_rings_[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
    }


    //// Resize & Clear
    //section_number_ = (setting_.RING_TO_ANALYZE + 3) / 4;
    //for (int q = 0; q < 8; ++q)
    //{
        //multi_region_[q].resize(section_number_);
        //filtered_region_[q].resize(section_number_);

        //section_distances_[q].resize(section_number_, 0.0);
        //for (int k = 0; k < section_number_; ++k)
        //{
            //multi_region_[q][k].clear();
            //filtered_region_[q][k].clear();
        //}
    //}

    // Update Grid for Filter Obstacles
    //updateGrid_(rings);
    //processOccupancyForBeamModel_();
    //estimateRoadModel_();

   

    //double a = base_coeff_.values[0];
    //double b = base_coeff_.values[1];
    //double c = base_coeff_.values[2];
    //double d = base_coeff_.values[3];

    //// LiDAR height [d]
    //debugger::debugColorOutput("Estimated LiDAR Height: ", d, 5, BG);

    //// Section Boundary Distance
    //// Approximately split section that could contains two rings
    //for (int q = 0; q < 8; ++q)
    //{
        //double alpha = std::asin(-(a * section_direction_[q].first +
                                   //b * section_direction_[q].second) / c);
        //for (int k = 1; k < section_number_; ++k)
        //{
            //double elevation_angle = (elevation_angles_[4 * k - 1] +
                                      //elevation_angles_[4 * k]) / 2;
            //double theta = alpha - elevation_angle;
            //section_distances_[q][k] = (theta > 0) ? 
                //estimated_lidar_height_ / std::sin(theta) : INFINITY;

            //debugger::debugColorOutput("Section Distance[" + 
                //std::to_string(q) + ", " + std::to_string(k) + "]: ",
                //section_distances_[q][k], 2, BW);
        //}
    //}

    //// Container for saving min_distance for each azimuth
    //const int azimuth_dividing_number = 360;
    //std::array<double, azimuth_dividing_number> prev_ring_distances;
    //std::array<double, azimuth_dividing_number> curr_ring_distances;
    //prev_ring_distances.fill(0.0);
    //curr_ring_distances.fill(0.0);

    //// Filtering out for multi-region planar model
    //for (int i = 0; i < setting_.RING_TO_ANALYZE; ++i)
    //{
        //int ring_size = rings[i].size();

        //// Elevation angle of ring
        //double elevation_angle = elevation_angles_[i];
        //double elevation_angle_prev = (i > 0) ? elevation_angles_[i - 1] : -M_PI / 2;

        //debugger::debugColorOutput("# of points in ring[" + 
            //std::to_string(i) + "] : ", ring_size, 1, BK);
        //debugger::debugColorOutput("Elevation angle of ring[" +
            //std::to_string(i) + "] : ", elevation_angle, 1, BK);

        //for (int j = 0; j < ring_size; ++j)
        //{
            //pcl::PointXYZ& point = rings[i][j];

            //// Filter outside points
            //if (std::abs(point.x > setting_.DISTANCE_TO_ANALYZE) ||
                //std::abs(point.y > setting_.DISTANCE_TO_ANALYZE))
            //{
                //continue;
            //}
            
            //// Convert to PointXYZL from PointXYZ (Label : ring_id)
            //pcl::PointXYZL point_with_label;
            //point_with_label.x = point.x;
            //point_with_label.y = point.y;
            //point_with_label.z = point.z;
            //point_with_label.label = i;
            
            //double distance = pcl::euclideanDistance(origin_, point);
            //double azimuth = std::atan2(point.y, point.x);
            //double alpha = std::asin(-(a * std::cos(azimuth) +
                                       //b * std::sin(azimuth)) / c);
            //double theta = alpha - elevation_angle;

            //// Compute Multi-Region Segment idx
            //int q = int(azimuth * 4 / M_PI + 4.5) % 8;
            //int k = 0;
            //while (k < section_distances_[q].size() && 
                   //distance >= section_distances_[q][k])
            //{
                //k++;
            //}
            //k--;

            //// Save raw points into Multi-Region
            //multi_region_[q][k].push_back(point_with_label);

            //// filter when ray head to sky
            //if (theta <= 0)
            //{
                //continue;
            //}

            //// Filter Obstacles(buildings, vehicles, and etc.) roughly
            //double distance_threshold = estimated_lidar_height_ /
                                        //std::sin(theta + setting_.ANGLE_BUFFER);
            //if (distance < distance_threshold)
            //{
                //continue;
            //}

            //// Save maximum distance for each azimuth section for next ring filtering.
            //int z = int(azimuth * 180 / M_PI + 180) % azimuth_dividing_number;
            //curr_ring_distances[z] = (curr_ring_distances[z] != 0) ?
                //std::min(curr_ring_distances[z], distance) : distance;

            //// Filter Obstacles based on previous ring distance
            //double theta_prev = alpha - elevation_angle_prev;
            //double distance_threshold_iter = prev_ring_distances[z] / 
                                             //std::sin(theta + setting_.ANGLE_BUFFER) *
                                             //std::sin(theta_prev + setting_.ANGLE_BUFFER);
            //if (distance < distance_threshold_iter)
            //{
                //continue;
            //} // End Filtering Obstacles

            //// smoothness filter (ex. grass)
            //std::vector<double> dist_diff_one(2 * setting_.SMOOTH_WINDOW_SIZE);
            //std::vector<double> dist_diff_two(2 * setting_.SMOOTH_WINDOW_SIZE - 1);
            //int dist_diff_idx = 0;
            //for (int w = -setting_.SMOOTH_WINDOW_SIZE + 1;
                 //w < setting_.SMOOTH_WINDOW_SIZE; ++w)
            //{
                //pcl::PointXYZ& p_prev = rings[i][(j + w - 1 + ring_size) % ring_size];
                //pcl::PointXYZ& p_curr = rings[i][(j + w + ring_size) % ring_size];
                //pcl::PointXYZ& p_next = rings[i][(j + w + 1 + ring_size) % ring_size];

                //dist_diff_one[dist_diff_idx] = pcl::euclideanDistance(p_prev, p_curr);
                //dist_diff_two[dist_diff_idx] = pcl::euclideanDistance(p_prev, p_next);
                //dist_diff_idx++;

                //if (w == setting_.SMOOTH_WINDOW_SIZE - 1)
                //{
                    //dist_diff_one[dist_diff_idx] =
                        //pcl::euclideanDistance(p_curr, p_next);
                //}
            //}

            //double smoothness_coeff = 0.0;
            //for (int u = 0; u < dist_diff_two.size(); ++u)
            //{
                //smoothness_coeff += dist_diff_two[u] /
                                    //(dist_diff_one[u] + dist_diff_one[u + 1]);
            //}
            //smoothness_coeff /= dist_diff_two.size();

            //if (smoothness_coeff < setting_.SMOOTH_THRESHOLD_GRASS)
            //{
                ////processobstacleforbeammodel_(point);
                //continue;
            //} // end of smoothness filter


            //// Save obstacle-filtered Points in multi-section
            //filtered_region_[q][k].push_back(point_with_label); 
        //}
        //// Change prev_ring_distance for next ring iteration
        //prev_ring_distances = curr_ring_distances;
        //curr_ring_distances.fill(0);
    //} // Finish Filter Obstacles and Save into Multi-Section


    
    //for (int z = 0; z < setting_.BEAM_SECTION_NUMBER; ++z)
    //{
        //double azimuth, distance;
        //std::tie(azimuth, distance) = min_distance_beam_[z];

        //pcl::PointXYZ point;
        //point.x = distance * std::cos(azimuth);
        //point.y = distance * std::sin(azimuth);
        //point.z = 0;

        //c_test_.push_back(point);
    //}

    debugger::debugColorTextOutput("Finish SetInputCloud", 1, BG);
}

void FeatureExtractor::run()
{
    if (base_plane_updated_)
    {
        debugger::debugColorTextOutput("[Feature Extractor] Execute", 5, BW);
        extractGround_();
        extractCurb_();
    }
    else
    {
        debugger::debugColorTextOutput("[Feature Extractor] Base Error", 10, BY);
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getGround()
{
    pcl::PointCloud<pcl::PointXYZ> ground;
    for (auto& cloud : ground_)
    {
        pcl::PointCloud<pcl::PointXYZ> tmp;
        pcl::copyPointCloud(cloud, tmp);
        ground += tmp;
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

std::shared_ptr<std::vector<std::vector<bool>>> FeatureExtractor::getOccupancyGrid()
{
    return std::make_shared<std::vector<std::vector<bool>>>(occupancy_grid_);
}

std::vector<std::pair<double, double>> FeatureExtractor::getBeamA()
{
    std::vector<std::pair<double, double>> beam;
    for (auto& end_point : free_beam_endpoints_)
    {
        beam.push_back({0, 0});
        beam.push_back(end_point);
    }
    return beam;
}

Eigen::Vector4f FeatureExtractor::getBasePlane()
{
    Eigen::Vector4f base_coeff;
    base_coeff << base_coeff_.values[0], base_coeff_.values[1],
                  base_coeff_.values[2], base_coeff_.values[3];

    return base_coeff;
}

template <class PointT>
double FeatureExtractor::calculateHeight(const PointT& point)
{
    double azimuth = std::atan2(point.y, point.x);
    double distance = pcl::euclideanDistance(point, origin_);

    int q = int(azimuth * 4 / M_PI + 4.5) % 8;
    int k = 0;
    while (k < section_distances_[q].size() &&
           distance >= section_distances_[q][k])
    {
        k++;
    }
    k--;

    Eigen::Vector4f plane_coeff = Eigen::Vector4f::Map(
        multi_region_plane_coeff_[q][k].values.data(), 4);

    return pcl::pointToPlaneDistanceSigned(point, plane_coeff);
}

template <class PointT>
bool FeatureExtractor::estimateBasePlane_(
    std::array<pcl::PointCloud<PointT>, RING_NUMBER>& rings)
{
    // Sampling Candidates for Base Planar without Obstacles
    pcl::PointCloud<PointT> base_candidate;
    for (int i = 0; i < setting_.RING_TO_FIT_BASE; ++i)
    {
        // Sort Rings by azimuth
        std::sort(rings[i].begin(), rings[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );

        int ring_size = rings[i].size();

        // Smoothness Filter (ex. Remove Grass and Obstacles)
        std::vector<double> dist_diff_ones(ring_size);
        std::vector<double> dist_diff_two(ring_size);
        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZ& p_prev = rings[i][(j - 1 + ring_size) % ring_size];
            pcl::PointXYZ& p_curr = rings[i][j];
            pcl::PointXYZ& p_next = rings[i][(j + 1) % ring_size];

            dist_diff_ones[j] = pcl::euclideanDistance(p_prev, p_curr) + 
                                pcl::euclideanDistance(p_curr, p_next);
            dist_diff_two[j]  = pcl::euclideanDistance(p_prev, p_next);
        }

        for (int j = 0; j < ring_size; ++j)
        {
            double smoothness_coeff = dist_diff_two[j] / dist_diff_ones[j];
            for (int w = 1; w < setting_.SMOOTH_WINDOW_SIZE; ++w)
            {
                int w_prev = (j - w + ring_size) % ring_size;
                int w_next = (j + w) % ring_size;

                smoothness_coeff += dist_diff_two[w_prev] / dist_diff_ones[w_prev];
                smoothness_coeff += dist_diff_two[w_next] / dist_diff_ones[w_next];
            }
            smoothness_coeff /= (2 * setting_.SMOOTH_WINDOW_SIZE - 1);

            if (smoothness_coeff > setting_.SMOOTH_THRESHOLD_PLANE)
            {
                base_candidate.push_back(rings[i][j]);
            }
        } // End of Smoothness Filte
    } // Finish Sampling Candidates for Base

    if (base_candidate.size() < 10)
    {
        PCL_ERROR("Not Enough Candidates for Base estimation \n");
        base_plane_updated_ = false;
        return false;
    }

    // Set Base Planar Model (ax + by + cz + d = 0)
    auto tmp_base_coeff = estimatePlaneRANSAC_(base_candidate);
    debugger::debugColorOutput("Base Coefficients \n", tmp_base_coeff, 3, BG); 

    // Sanity Check for Vertical Base Planar
    if (tmp_base_coeff.values[2] < 0.8)
    {
        PCL_ERROR("Base Planar Model is Vertical! \n");
        base_plane_updated_ = false;
        return false;
    }

    base_plane_updated_ = true;
    base_coeff_ = tmp_base_coeff;

    double a = base_coeff_.values[0];
    double b = base_coeff_.values[1];
    double c = base_coeff_.values[2];
    double d = base_coeff_.values[3];

    base_coeff_vec_ << a, b, c, d;

    // Calculated Transformation Matrix
    // Rotation Matrix
    // Row-Wise Normalized 
    // | c  0 -a |
    // | 0  c -b |
    // | a  b  c | 
    transformation_ = Eigen::Matrix4f::Identity(4, 4);
    transformation_.block<3, 3>(0, 0) << c, 0, -a, 0, c, -b, a, b, c;
    transformation_.rowwise().normalize();

    debugger::debugColorOutput("Transformation \n", transformation_, 3, BB); 
    
    // Estimated LiDAR Height from ground : |d| 
    estimated_lidar_height_ = std::abs(base_coeff_.values[3]);

    return true;
}

template <class PointT>
pcl::ModelCoefficients FeatureExtractor::estimatePlaneRANSAC_(
    pcl::PointCloud<PointT>& cloud)
{
    // Plane Model segmentation with RANSAC
    // https://pcl.readthedocs.io/en/latest/planar_segmentation.html
    pcl::ModelCoefficients plane_coeff; 
    if (cloud.size() < 10)
    {
        debugger::debugColorTextOutput("Not enough Data for RANSAC", 5, BY);
        return plane_coeff;
    }

    pcl::PointIndices inliers;

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;
    // Optional
    seg.setOptimizeCoefficients(true);
    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(setting_.FIT_PLANE_THRESHOLD);
    
    // Segment Planar Model
    seg.setInputCloud(cloud.makeShared());
    seg.segment(inliers, plane_coeff);

    if (inliers.indices.empty())
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    return plane_coeff;
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
    int grid_size = 2 * setting_.GRID_NUMBER + 1;
    double grid_length = setting_.GRID_LENGTH;
    double max_distance = grid_length * (setting_.GRID_NUMBER + 0.5);

    // Index of Center Grid
    int center_m = setting_.GRID_NUMBER;
    int center_n = setting_.GRID_NUMBER;

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

        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZ& point = transformed_rings_[i][j];

            // Filter outside points
            if (std::abs(point.x) >= max_distance || 
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
        }
    } // End of Pre-Compute Grid

    // Estimate whether Cell is Ground or Not
    // -1 : Obstacle
    //  0 : Unknown
    //  1 : Ground
    std::vector<std::vector<int>> grid_ground(grid_size,
        std::vector<int>(grid_size, 0));
    std::vector<std::pair<int, int>> neighbors =
        {{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};

    // Asign Center of Grid as Ground
    grid_ground[center_m][center_n] = 1;
    Eigen::Vector4f transformed_base_coeff = transformation_ * base_coeff_vec_;
    grid_min_z[center_m][center_n] = -transformed_base_coeff(3);
    grid_max_z[center_m][center_n] = -transformed_base_coeff(3);

    // Check whether Obstacle exists in the cell by cheking z_value Differnce
    std::vector<std::pair<int, int>> obstacle_grid_idx;
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            double z_diff = grid_max_z[m][n] - grid_min_z[m][n];
            if (z_diff > setting_.OBSTACLE_THRESHOLD)
            {
                grid_ground[m][n] = -1;
                obstacle_grid_idx.push_back({m, n});
            }
        }
    }

    // Propagate Obstacles with DFS
    while (!obstacle_grid_idx.empty())
    {
        int m = obstacle_grid_idx.back().first;
        int n = obstacle_grid_idx.back().second;
        obstacle_grid_idx.pop_back();

        for (auto& neighbor : neighbors)
        {
            int tmp_m = m + neighbor.first;
            int tmp_n = n + neighbor.second;

            // Skip when the neighbor is already Obstacle
            if (tmp_m < 0 || tmp_m >= grid_size ||
                tmp_n < 0 || tmp_n >= grid_size ||
                grid_cloud[tmp_m][tmp_n].empty() ||
                grid_ground[tmp_m][tmp_n] == -1)
            {
                continue;
            }

            // When Neighbor cell has higher min z_value,
            // the neighbor cell is Obstacle
            if (grid_min_z[tmp_m][tmp_n] > grid_min_z[m][n] - 0.01)
            {
                grid_ground[tmp_m][tmp_n] = -1;
                obstacle_grid_idx.push_back({tmp_m, tmp_n});
            }
        }
    }

    // Check Ground by propagation
    for (int range = 1; range <= setting_.GRID_NUMBER; ++range)
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

            // Reset min/max z value for obstacles for ground propagation
            if (grid_ground[m][n] == -1)
            {
                grid_max_z[m][n] = -INFINITY;
                grid_min_z[m][n] = INFINITY;
            }

            // Update Min/Max Z value for Cell if there is no
            // Min/Max Z value from itself
            if (std::isinf(grid_max_z[m][n]))
            {
                for (auto& neighbor : neighbors)
                {
                    int tmp_m = m + neighbor.first;
                    int tmp_n = n + neighbor.second;

                    int range_neighbor = std::max(std::abs(tmp_m - center_m),
                                                  std::abs(tmp_n - center_n));

                    // Only Consider Inner grid
                    if (range_neighbor >= range)
                    {
                        continue;
                    }

                    grid_min_z[m][n] = std::min(grid_min_z[m][n],
                                                grid_min_z[tmp_m][tmp_n]);
                    grid_max_z[m][n] = std::max(grid_max_z[m][n],
                                                grid_max_z[tmp_m][tmp_n]);
                }
                continue;
            }

            // Check whether grid is ground based on neighbor ground
            double max_z_neighbor_ground = -INFINITY;
            double min_z_neighbor_ground = INFINITY;
            {
                for (auto& neighbor : neighbors)
                {
                    int tmp_m = m + neighbor.first;
                    int tmp_n = n + neighbor.second;

                    int range_neighbor = std::max(std::abs(tmp_m - center_m),
                                                  std::abs(tmp_n - center_n));
                    
                    // Only Consider Inner grid
                    if (range_neighbor >= range)
                    {
                        continue;
                    }

                    // Update Min/Max Z value for neighbor ground
                    min_z_neighbor_ground = std::min(min_z_neighbor_ground,
                                                     grid_min_z[tmp_m][tmp_n]);
                    max_z_neighbor_ground = std::max(max_z_neighbor_ground,
                                                     grid_max_z[tmp_m][tmp_n]);
                }
            }
            
            // Check Height Difference with Neighbor Ground Cells 
            if (grid_max_z[m][n] > max_z_neighbor_ground + setting_.GROUND_THRESHOLD ||
                grid_min_z[m][n] < min_z_neighbor_ground - setting_.GROUND_THRESHOLD)
            {
                grid_ground[m][n] = -1;

                // Reset Min/Max Z value for obstacle for ground propagation
                grid_max_z[m][n] = max_z_neighbor_ground;
                grid_min_z[m][n] = min_z_neighbor_ground;
                continue;
            }
                
            // Asign the Cell as Ground
            grid_ground[m][n] = 1;
        }
    }

    // Save Obstacle Points
    for (int m = 0; m < grid_size; ++m)
    {
        for (int n = 0; n < grid_size; ++n)
        {
            for (auto& point_with_label : grid_cloud[m][n])
            {
                pcl::PointXYZ point;
                point.x = point_with_label.x;
                point.y = point_with_label.y;
                point.z = point_with_label.z;

                if (grid_ground[m][n] == 1)
                {
                    ground_[point_with_label.label].push_back(point);
                }
                else
                {
                    obstacles_.push_back(point);
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

void FeatureExtractor::extractCurb_()
{
    for (int i = 0; i < ground_.size(); ++i)
    {
        int ring_size = ground_[i].size();
        double elevation_angle = elevation_angles_[i];

        for (int j = 1; j < ring_size - 1; ++j)
        {
            pcl::PointXYZ& point      = ground_[i][j];
            pcl::PointXYZ& point_prev = ground_[i][j - 1];
            pcl::PointXYZ& point_next = ground_[i][j + 1];

            // Property(Azimuth) of Points
            double azimuth = std::atan2(point.y, point.x);
            double azimuth_prev = std::atan2(point_prev.y, point_prev.x);
            double azimuth_next = std::atan2(point_next.y, point_next.x);

            // Skip Discontinued Points
            if (azimuth_next - azimuth > setting_.DISCONTINUITY_AZIMUTH ||
                azimuth - azimuth_prev > setting_.DISCONTINUITY_AZIMUTH)
            {
                continue;
            }

            // #1. Using discontinued point
            // (When LiDAR cannot detect vertical surface of Curb)
            // Property(Distance) of Points
            double dist_origin = pcl::euclideanDistance(origin_, point);
            double dist_origin_prev = pcl::euclideanDistance(origin_, point_prev);
            double dist_origin_next = pcl::euclideanDistance(origin_, point_next);

            double theta = std::acos(estimated_lidar_height_ / dist_origin);

            double dist_diff_prev = dist_origin_prev - dist_origin;
            double dist_diff_next = dist_origin_next - dist_origin;
            double dist_diff_threshold = setting_.CURB_HEIGHT_THRESHOLD / std::cos(theta);
            if ((dist_diff_prev > dist_diff_threshold &&
                 dist_diff_prev < setting_.CURB_ANGLE_THRESHOLD * dist_diff_threshold &&
                 std::abs(dist_diff_next) < setting_.CURB_HEIGHT &&
                 point.z - point_prev.z > setting_.CURB_HEIGHT_THRESHOLD &&
                 std::abs(point.z - point_next.z) < setting_.CURB_HEIGHT) ||
                (dist_diff_next > dist_diff_threshold &&
                 dist_diff_next < setting_.CURB_ANGLE_THRESHOLD * dist_diff_threshold &&
                 std::abs(dist_diff_prev) < setting_.CURB_HEIGHT &&
                 point.z - point_next.z > setting_.CURB_HEIGHT_THRESHOLD &&
                 std::abs(point.z - point_prev.z) < setting_.CURB_HEIGHT))
            {
                landmark_.push_back(point);
            }

        }
    }
    //for (int i = 0; i < ground_.size(); ++i)
    //{
        //double elevation_angle = elevation_angles_[i];
        //int ring_size = ground_[i].size();

        //// Pre-Compute distance_xy for points
        //std::vector<double> distance_xy_points(ring_size);
        //for (int j = 0; j < ring_size; ++j)
        //{
            //pcl::PointXYZ& point = ground_[i][j];

            //distance_xy_points[j] = std::sqrt(std::pow(point.x, 2) +
                                              //std::pow(point.y, 2));
        //}

        //for (int j = 1; j < ring_size - 1; ++j)
        //{
            //pcl::PointXYZ& point      = ground_[i][j];
            //pcl::PointXYZ& point_prev = ground_[i][j - 1];
            //pcl::PointXYZ& point_next = ground_[i][j + 1];

            //// Property of point
            //double azimuth = std::atan2(point.y, point.x);
            //double distance = pcl::euclideanDistance(point, origin_);
            //double distance_xy = std::sqrt(std::pow(point.x, 2) +
                                           //std::pow(point.y, 2));

            //// #1. Using discontinued point
            //// (When LiDAR cannot detect vertical surface of Curb)
            //double gap_z_prev = point.z - point_prev.z;
            //double gap_z_next = point.z - point_next.z;
            //double gap_xy_prev = distance_xy_points[j] - distance_xy_points[j - 1];
            //double gap_xy_next = distance_xy_points[j] - distance_xy_points[j + 1];


            //// Property of point
            ////double azimuth = std::atan2(point.y, point.x);
            ////double distance = pcl::euclideanDistance(point, origin_);
            ////double distance_xy = std::sqrt(std::pow(point.x, 2) +
                                           ////std::pow(point.y, 2));
            //double theta = std::acos(distance_xy / distance);
            //double delta_xy = distance_xy * setting_.ANGULAR_RESOLUTION;
            //double delta_z = delta_xy * std::sin(theta);

            //int n_v = setting_.CURB_HEIGHT / std::sin(theta) / delta_xy;

            //// 1. XY Gap Threshold
            ////double gap_xy_prev = std::sqrt(std::pow(point_prev.x - point.x, 2) +
                                           ////std::pow(point_prev.y - point.y, 2));
            ////double gap_xy_next = std::sqrt(std::pow(point_next.x - point.x, 2) +
                                           ////std::pow(point_next.y - point.y, 2));

            //// Threshold (xy Gap)
            //if ((gap_xy_prev < delta_xy && gap_xy_next < delta_xy) || 
                //gap_xy_prev > setting_.DISCONTINUITY * delta_xy ||
                //gap_xy_next > setting_.DISCONTINUITY * delta_xy)
            //{
                //continue;
            //}
            
            ////// 2. Z Gap Threshold
            ////double gap_z_prev = std::abs(point_prev.z - point.z);
            ////double gap_z_next = std::abs(point_next.z - point.z);

            ////// Threshold (z Gap)
            ////if (gap_z_prev < delta_z && gap_z_next < delta_z)
            ////{
                ////continue;
            ////}
            ////c_test_.push_back(point);

            //// Smoothness Filter (ex. Grass)
            //std::vector<double> dist_diff_one(2 * setting_.SMOOTH_WINDOW_SIZE);
            //std::vector<double> dist_diff_two(2 * setting_.SMOOTH_WINDOW_SIZE - 1);
            //int dist_diff_idx = 0;
            //for (int w = -setting_.SMOOTH_WINDOW_SIZE + 1;
                 //w < setting_.SMOOTH_WINDOW_SIZE; ++w)
            //{
                //pcl::PointXYZ& p_prev = ground_[i][(j + w - 1 + ring_size) % ring_size];
                //pcl::PointXYZ& p_curr = ground_[i][(j + w + ring_size) % ring_size];
                //pcl::PointXYZ& p_next = ground_[i][(j + w + 1 + ring_size) % ring_size];

                //dist_diff_one[dist_diff_idx] = pcl::euclideanDistance(p_prev, p_curr);
                //dist_diff_two[dist_diff_idx] = pcl::euclideanDistance(p_prev, p_next);
                //dist_diff_idx++;

                //if (w == setting_.SMOOTH_WINDOW_SIZE - 1)
                //{
                    //dist_diff_one[dist_diff_idx] =
                        //pcl::euclideanDistance(p_curr, p_next);
                //}
            //}

            //double smoothness_coeff = 0.0;
            //for (int u = 0; u < dist_diff_two.size(); ++u)
            //{
                //smoothness_coeff += dist_diff_two[u] /
                                    //(dist_diff_one[u] + dist_diff_one[u + 1]);
            //}
            //smoothness_coeff /= dist_diff_two.size();

            //if (smoothness_coeff < setting_.SMOOTH_THRESHOLD_GRASS)
            //{
                //continue;
            //} // End of Smoothness Filter
            //if (smoothness_coeff > setting_.SMOOTH_THRESHOLD_PLANE)
            //{
            //}



            //// 1. Discontinuity (for local window size)
            //pcl::PointXYZ& point_prev_window = 
                //ground_[i][(j - setting_.CURB_WINDOW_SIZE + ring_size) % ring_size];
            //pcl::PointXYZ& point_next_window =
                //ground_[i][(j + setting_.CURB_WINDOW_SIZE + ring_size) % ring_size];

            //double dist_xy_prev_window = std::sqrt(
                //std::pow(point_prev_window.x - point.x, 2) +
                //std::pow(point_prev_window.y - point.y, 2));
            //double dist_xy_next_window = std::sqrt(
                //std::pow(point_next_window.x - point.x, 2) +
                //std::pow(point_next_window.y - point.y, 2));

            //double discontinuity_threshold = setting_.DISCONTINUITY * delta_xy;
            //if (dist_xy_prev_window > discontinuity_threshold ||
                //dist_xy_next_window > discontinuity_threshold)
            //{
                //continue;
            //}
        
            //// 1. Height & Angular Filter
            //double max_height = ground_height_[i][j];
            //double min_height = ground_height_[i][j];
            
            //Eigen::Vector2f v_a(0.0, 0.0);
            //int count_a = 1;
            //double dist_diff_a = 0.0;
            //double distance_xy_a = 0.0;
            //while (dist_diff_a < delta_xy * n_v && count_a < n_v)
            //{
                //int idx_a = (j + count_a + ring_size) % ring_size;
                //const pcl::PointXYZ& p_a = ground_[i][idx_a];

                //v_a(0) += (p_a.x - point.x);
                //v_a(1) += (p_a.y - point.y);

                //max_height = std::max(max_height, ground_height_[i][idx_a]);
                //min_height = std::min(min_height, ground_height_[i][idx_a]);

                //dist_diff_a = std::sqrt(std::pow(p_a.x - point.x, 2) +
                                        //std::pow(p_a.y - point.y, 2));
                //count_a++;
            //}
            
            //Eigen::Vector2f v_b(0.0, 0.0);
            //int count_b = 1;
            //double dist_diff_b = 0.0;
            //double distance_xy_b = 0.0;
            //while (dist_diff_b < delta_xy * n_v && count_b < n_v)
            //{
                //int idx_b = (j - count_b + ring_size) % ring_size;
                //const pcl::PointXYZ& p_b = ground_[i][idx_b];

                //v_b(0) += (p_b.x - point.x);
                //v_b(1) += (p_b.y - point.y);

                //max_height = std::max(max_height, ground_height_[i][idx_b]);
                //min_height = std::min(min_height, ground_height_[i][idx_b]);

                //dist_diff_b = std::sqrt(std::pow(p_b.x - point.x, 2) +
                                        //std::pow(p_b.y - point.y, 2));
                //count_b++;
            //}

            //double angle_window = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (angle_window > setting_.CURB_ANGLE_THRESHOLD)
            //{
                //continue;
            //}

            //if (max_height - min_height > setting_.CURB_HEIGHT_THRESHOLD)
            //{
                //landmark_.push_back(point);
            //}



            //// 4. Angular & Height Filter
            //// Compute Multi-Region Segment idx
            //int q = int(azimuth * 4 / M_PI + 4.5) % 8;
            //int k = 0;
            //while (k < section_distances_[q].size() && 
                    //distance >= section_distances_[q][k])
            //{
                //k++;
            //}
            //k--;

            //// planar model
            //pcl::ModelCoefficients& plane_coeff = multi_region_plane_coeff_[q][k];
            //double a = plane_coeff.values[0];
            //double b = plane_coeff.values[1];
            //double c = plane_coeff.values[2];
            //double d = plane_coeff.values[3];

            //// Height of point based on the planar model
            //double height_point = pcl::pointToPlaneDistanceSigned(point, a, b, c, d);

            //// estimated number of points on CURB
            //int n_v = setting_.CURB_HEIGHT / std::sin(theta) / delta_xy;

            //Eigen::Vector2f v_a(0.0, 0.0);
            //double max_height_diff_a = 0.0;
            //int count_a = 1;
            //double dist_diff_a = 0.0;
            //double distance_xy_a = 0.0;
            //while (dist_diff_a < delta_xy * n_v && count_a < n_v)
            //{
                //pcl::PointXYZ& p_a = ground_[i][(j + count_a + ring_size) % ring_size];
                
                //double x_diff = p_a.x - point.x;
                //double y_diff = p_a.y - point.y;
                //double height_diff = pcl::pointToPlaneDistanceSigned(p_a, a, b, c, d) -
                                     //height_point;

                //dist_diff_a = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
                //distance_xy_a = std::sqrt(std::pow(p_a.x, 2) + std::pow(p_a.y, 2));

                //v_a(0) += x_diff;
                //v_a(1) += y_diff;

                //max_height_diff_a = std::max(max_height_diff_a, height_diff);

                //count_a++;
            //}
            
            //Eigen::Vector2f v_b(0.0, 0.0);
            //double max_height_diff_b = 0.0;
            //int count_b = 1;
            //double dist_diff_b = 0.0;
            //double distance_xy_b = 0.0;
            //while (dist_diff_b < delta_xy * n_v && count_b < n_v)
            //{
                //pcl::PointXYZ& p_b = ground_[i][(j - count_b + ring_size) % ring_size];
                
                //double x_diff = p_b.x - point.x;
                //double y_diff = p_b.y - point.y;
                //double height_diff = pcl::pointToPlaneDistanceSigned(p_b, a, b, c, d) - 
                                     //height_point;

                //dist_diff_b = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
                //distance_xy_b = std::sqrt(std::pow(p_b.x, 2) + std::pow(p_b.y, 2));

                //v_b(0) += x_diff;
                //v_b(1) += y_diff;

                //max_height_diff_b = std::max(max_height_diff_b, height_diff);

                //count_b++;
            //}
            
            //// Threshold (Discontinuity Filter for n_v) 
            //if (dist_diff_a > setting_.DISCONTINUITY * n_v * delta_xy ||
                //dist_diff_b > setting_.DISCONTINUITY * n_v * delta_xy)
            //{
                //continue;
            //}

            //// 4. Augluar Filter
            //double angle_window = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            //if (angle_window > setting_.ANGLE_CURB_THRESHOLD)
            //{
                //continue;
            //}

            //// 5. Height Filter
            //if ((max_height_diff_a > setting_.HEIGHT_CURB_THRESHOLD &&
                 //distance_xy_a < distance_xy) ||
                //(max_height_diff_b > setting_.HEIGHT_CURB_THRESHOLD &&
                 //distance_xy_b < distance_xy))
            //{
                //landmark_.push_back(point);

                ////// Prevent continuous points
                ////j += n_v;
            //}
        //}
    //}
}



void FeatureExtractor::estimateGround_()
{
    // Clear
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        ground_[i].clear();
        ground_height_[i].clear();
    }

    //std::array<std::vector<pcl::ModelCoefficients>, 8> multi_region_plane_coeff_;
    for (int q = 0; q < 8; ++q)
    {
        multi_region_plane_coeff_[q].resize(section_number_);

        // Estimate Planar Model for Multi-Region
        for (int k = 0; k < section_number_; ++k)
        {
            multi_region_plane_coeff_[q][k] = estimatePlaneRANSAC_(filtered_region_[q][k]);

            debugger::debugColorOutput("# of Points (q: " + 
                std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                filtered_region_[q][k].size(), 1, BB);

            debugger::debugColorOutput("Plane Coeff (q: " + 
                std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                multi_region_plane_coeff_[q][k], 1, BB);
            
            pcl::ModelCoefficients& coeff_prev = (k > 0) ? 
                multi_region_plane_coeff_[q][k - 1] : base_coeff_;

            // replace with previous section planar model
            if (multi_region_plane_coeff_[q][k].values.empty())
            {
                multi_region_plane_coeff_[q][k] = coeff_prev;
            }
            // Compute Angle Difference with two plane model
            else
            {                
                double angle_diff = computeAngleTwoPlane_(coeff_prev,
                    multi_region_plane_coeff_[q][k]);
                angle_diff = std::min(angle_diff, M_PI - angle_diff);
                
                std::pair<double, double> boundary_point = 
                    {section_direction_[q].first * section_distances_[q][k],
                     section_direction_[q].second * section_distances_[q][k]};
                double height_diff = computeHeightDiffTwoPlane_(boundary_point,
                    coeff_prev, multi_region_plane_coeff_[q][k]); 

                if (M_PI / 2 - angle_diff < setting_.ANGLE_DIFF_THRESHOLD)
                {
                    debugger::debugColorTextOutput("Remove Inlier region (q: " + 
                        std::to_string(q) + ", k: " + std::to_string(k) +
                        ") : ", 2, BM);

                    // remove points that forms vertical to base
                    removeInliner_(filtered_region_[q][k],
                                   multi_region_plane_coeff_[q][k]);
                    k--;
                }
                else if (angle_diff > setting_.ANGLE_DIFF_THRESHOLD ||
                         height_diff > setting_.HEIGHT_DIFF_THRESHOLD)
                {
                    debugger::debugColorOutput("Angle Diff region (q: " + 
                        std::to_string(q) + ", k: " + std::to_string(k) +
                        ") : ", angle_diff, 2, BM);
                    debugger::debugColorOutput("Height Diff region (q: " + 
                        std::to_string(q) + ", k: " + std::to_string(k) +
                        ") : ", height_diff, 2, BM);

                    multi_region_plane_coeff_[q][k] = coeff_prev;
                }
            }
        }
    }

    // Filter with Multi-Region Plane model
    for (int q = 0; q < 8; ++q)
    {
        for (int k = 0; k < section_number_; ++k)
        {
            pcl::ModelOutlierRemoval<pcl::PointXYZL> ground_filter(true);
            ground_filter.setThreshold(setting_.GROUND_THRESHOLD);
            ground_filter.setModelType(pcl::SACMODEL_PLANE);
            ground_filter.setModelCoefficients(multi_region_plane_coeff_[q][k]);
            ground_filter.setInputCloud(multi_region_[q][k].makeShared());
            
            pcl::PointCloud<pcl::PointXYZL> ground_cloud;
            //ground_filter.setNegative(false);
            ground_filter.filter(ground_cloud);

            for (pcl::PointXYZL& p : ground_cloud)
            {
                pcl::PointXYZ point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                ground_[p.label].push_back(point);
            }
            
            //pcl::PointCloud<pcl::PointXYZL> obstacle_cloud;
    //pcl::PointIndices::ConstPtr inliers = ground_filter.getRemovedIndices();
    //pcl::ExtractIndices<PointT> extract;
    //extract.setInputCloud(multi_region_[q][k].makeShared());
    //extract.setIndices(inliers);
    //extract.setNegative(true);
    //extract.filter(obstacle_cloud);
            //pcl::ModelOutlierRemoval<pcl::PointXYZL> ground_filter;
            //ground_filter.setThreshold(setting_.GROUND_THRESHOLD);
            //ground_filter.setModelType(pcl::SACMODEL_PLANE);
            //ground_filter.setModelCoefficients(multi_region_plane_coeff_[q][k]);
            //ground_filter.setInputCloud(multi_region_[q][k].makeShared());
            //pcl::PointCloud<pcl::PointXYZL> obstacle_cloud;
            //ground_filter.setNegative(true);
            //ground_filter.filter(obstacle_cloud);
            //

            
            //for (int obstacle_idx : *ground_filter.getRemovedIndices()) 
            //{
                //pcl::PointXYZ point;
                //point.x = multi_region_[q][k][obstacle_idx].x;
                //point.y = multi_region_[q][k][obstacle_idx].y;
                //point.z = multi_region_[q][k][obstacle_idx].z;

                //processObstacleForBeamModel_(multi_region_[q][k][obstacle_idx]);
            //}
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

        // Filter Discontinued area
        pcl::PointCloud<pcl::PointXYZ> continued_ground_ring;
        pcl::PointCloud<pcl::PointXYZ> cluster;
        int count_continued = 0;
        for (int j = 1; j < ground_[i].size(); ++j)
        {
            pcl::PointXYZ& point_curr = ground_[i][j];
            pcl::PointXYZ& point_prev = ground_[i][j - 1];

            double delta_xy = setting_.ANGULAR_RESOLUTION *
                              std::sqrt(std::pow(point_curr.x, 2) +
                                        std::pow(point_curr.y, 2));
            
            double distance = std::sqrt(
                std::pow(point_curr.x - point_prev.x, 2) +
                std::pow(point_curr.y - point_prev.y, 2));
            if (distance > setting_.GROUND_DISCONTINUITY * delta_xy)
            {
                if (count_continued > setting_.CONTINUED_NUMBER)
                {
                    continued_ground_ring += cluster;
                }
                count_continued = 0;
                cluster.clear();
            }
            else
            {
                count_continued++; 
            }
            cluster.push_back(point_curr);
        }

        if (count_continued > setting_.CONTINUED_NUMBER)
        {
            continued_ground_ring += cluster;
        }

        ground_[i] = continued_ground_ring;

        // Calculate Height of points from Base Plane
        ground_height_[i].resize(ground_[i].size());
        for (int j = 0; j < ground_[i].size(); ++j)
        {
            ground_height_[i][j] = calculateHeight(ground_[i][j]);
        }
    }
}


template <class PointT>
void FeatureExtractor::updateGrid_(
    std::array<pcl::PointCloud<PointT>, RING_NUMBER>& rings)
{
    int grid_size = 2 * setting_.DISTANCE_TO_ANALYZE * setting_.GRID_RESOLUTION;
    double grid_length = 1.0 / setting_.GRID_RESOLUTION;

    // {min_height, max_height} for grid
    std::vector<std::vector<std::pair<double, double>>> grid_min_max_z(grid_size,
        std::vector<std::pair<double, double>>(grid_size, {INFINITY, -INFINITY}));

    // {{ring_id, height}} for grid
    std::vector<std::vector<std::vector<std::pair<int, double>>>> grid_ring_z(
        grid_size, std::vector<std::vector<std::pair<int, double>>>(grid_size));

    // Update {Ring_id, z value} & Min/Max Height of Grid
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        for (int j = 0; j < rings[i].size(); ++j)
        {
            pcl::PointXYZ& point = rings[i][j];

            // Index of Grid 
            int m = (point.x + setting_.DISTANCE_TO_ANALYZE) / grid_length;
            int n = (point.y + setting_.DISTANCE_TO_ANALYZE) / grid_length;

            // Filter outside points
            if (m < 0 || n < 0 || m >= grid_size || n >= grid_size)
            {
                continue;
            }

            // Pass if point with the same ring_id already saved
            if (!grid_ring_z[m][n].empty() &&
                grid_ring_z[m][n].back().first == i)
            {
                continue;
            }

            // Update {Ring_id, point.z} in Grid
            grid_ring_z[m][n].push_back({i, point.z});
            
            // Update Min/Max Height
            if (point.z < grid_min_max_z[m][n].first)
            {
                grid_min_max_z[m][n].first = point.z;
            }
            else if (point.z > grid_min_max_z[m][n].second)
            {
                grid_min_max_z[m][n].second = point.z;
            }
        }
    }
    
    std::vector<std::vector<double>> obstacle_grid(grid_size,
        std::vector<double>(grid_size, 0.0));
    occupancy_grid_.resize(grid_size);
    for (int m = 0; m < grid_size; ++m)
    {
        occupancy_grid_[m].clear();
        occupancy_grid_[m].resize(grid_size);
        for (int n = 0; n < grid_size; ++n)
        {
            // Set No obstacle when height difference is less than threshold
            double height_diff = grid_min_max_z[m][n].second - grid_min_max_z[m][n].first;
            if (height_diff < setting_.NOT_OBSTACLE_THRESHOLD)
            {
                continue;
            }

            int ring_count = 0;
            int prev_ring_id = 0;
            for (int r = 0; r < grid_ring_z[m][n].size(); ++r)
            {
                int curr_ring_id = grid_ring_z[m][n][r].first;
                if (curr_ring_id - prev_ring_id > 1)
                {
                    ring_count = 0;
                }
                ring_count++;

                if (ring_count >= 3)
                {
                    obstacle_grid[m][n] = 1.0;
                    occupancy_grid_[m][n] = true;


                    continue;
                }
                prev_ring_id = curr_ring_id;
            }
        }
    }

    //// Smooth Occupancy Grid with 3x3 filter
    //std::vector<std::pair<int, int>> neighbors =
        //{{-1, -1}, {-1, 0}, {-1, 1}, {0, -1}, {0, 1}, {1, -1}, {1, 0}, {1, 1}};
    
    //occupancy_grid_.resize(grid_size);
    //for (int m = 0; m < occupancy_grid_.size(); ++m)
    //{
        //occupancy_grid_[m].clear();
        //occupancy_grid_[m].resize(grid_size);
        //for (int n = 0; n < occupancy_grid_[m].size(); ++n)
        //{
            //double filtered_value = 2 * obstacle_grid[m][n]; 
            //for (auto neighbor : neighbors)
            //{
                //int new_m = m + neighbor.first;
                //int new_n = n + neighbor.second;
                //if (new_m >= 0 && new_n >= 0 &&
                    //new_m < grid_size && new_n < grid_size)
                //{
                    //filtered_value += obstacle_grid[new_m][new_n];
                //}
            //}
            //occupancy_grid_[m][n] = (filtered_value > 3);

            //pcl::PointXYZ point;
            //point.x = grid_length * m - setting_.DISTANCE_TO_ANALYZE + 
                      //grid_length / 2;
            //point.y = grid_length * n - setting_.DISTANCE_TO_ANALYZE +
                      //grid_length / 2;
            //point.z = 0;
            //if (occupancy_grid_[m][n])
            //{
                //b_test_.push_back(point);
            //}
        //}
    //}
}

void FeatureExtractor::processOccupancyForBeamModel_()
{
    min_distance_beam_ = max_distance_beam_;

    double grid_length = 1.0 / setting_.GRID_RESOLUTION;
    double resolution = 360.0 / (double)setting_.BEAM_SECTION_NUMBER;
    for (int i = 0; i < occupancy_grid_.size(); ++i)
    {
        for (int j = 0; j < occupancy_grid_[i].size(); ++j)
        {
            if (!occupancy_grid_[i][j])
            {
                continue;
            }

            double x = grid_length * i - setting_.DISTANCE_TO_ANALYZE + 
                       grid_length / 2;
            double y = grid_length * j - setting_.DISTANCE_TO_ANALYZE +
                       grid_length / 2;

            if (std::abs(x) > setting_.ROAD_VIEW_RANGE ||
                std::abs(y) > setting_.ROAD_VIEW_RANGE ||
                (std::abs(x) < 5.0 && std::abs(y) < 2.0))
            {
                continue;
            }

            double distance_xy = std::sqrt(std::pow(x, 2) + std::pow(y, 2));
            double azimuth = std::atan2(y, x);
            int idx = int((azimuth * 180 / M_PI + 180) / resolution);

            if (distance_xy < min_distance_beam_[idx].second)
            {
               min_distance_beam_[idx] = {azimuth, distance_xy};
            }
        }
    }
}

//void FeatureExtractor::processBeamModel_(double x, double y,
    //std::vector<std::pair<double, double>>& out_min_distance_beam,
    //std::vector<Std::pair<double, double>>& out_max_distance_beam)
//{
    //// Resize
    //out_min_distance_beam.resize(setting_.BEAM_SECTION_NUMBER);
    //out_max_distance_beam.resize(setting_.BEAM_SECTION_NUMBER);

    //// Beam Model
    //double resolution = 360.0 / (double)setting_.BEAM_SECTION_NUMBER;

    //for (int z = 0; z < setting_.BEAM_SECTION_NUMBER; ++z)
    //{
        //double theta = (resolution * z - 180 + resolution / 2.0) * M_PI / 180;
        //out_max_distance_beam_[z].first = theta;
        
    //}
    
//}

void FeatureExtractor::estimateRoadModel_()
{
    // Clear
    free_beam_endpoints_.clear();
    intersection_ = {0, 0};
    roads_angle_.clear();
    
    double resolution = 360.0 / (double)setting_.BEAM_SECTION_NUMBER;

    // Count No Obstacle Section Clock-Wisely
    std::vector<double> bottom_layer_road_angle;
    int continued_max_beam_count = 0;
    for (int z = setting_.BEAM_SECTION_NUMBER - 1; z >= 0; --z)
    {
        if (min_distance_beam_[z].second < max_distance_beam_[z].second)
        {
            break;
        }
        continued_max_beam_count++;
    }

    // Bottom Layer Beam Model (emit beam based on LiDAR Origin)
    int end_idx = setting_.BEAM_SECTION_NUMBER - continued_max_beam_count;
    for (int z = 0; z < end_idx; ++z)
    {
        if (min_distance_beam_[z].second < max_distance_beam_[z].second)
        {
            if (continued_max_beam_count >= 5)
            {
                double mean_z = (2.0 * z - continued_max_beam_count) / 2.0;
                mean_z += (mean_z < 0) ? setting_.BEAM_SECTION_NUMBER : 0;

                double theta = (resolution * mean_z - 180) * M_PI / 180;
                bottom_layer_road_angle.push_back(theta);

                double angle = std::fmod(theta + M_PI, M_PI / 2);
                double distance = (angle < M_PI / 4) ? 
                    setting_.ROAD_VIEW_RANGE / std::cos(angle) :
                    setting_.ROAD_VIEW_RANGE / std::sin(angle);

                pcl::PointXYZ point;
                point.x = distance * std::cos(theta);
                point.y = distance * std::sin(theta);
                point.z = 0.0;
                free_beam_endpoints_.push_back({point.x, point.y});
            }
            continued_max_beam_count = 0;
        }
        else
        {
            continued_max_beam_count++;
        }
    }
}

double FeatureExtractor::computeAngleTwoPlane_(
    const pcl::ModelCoefficients& coeff1, 
    const pcl::ModelCoefficients& coeff2)
{
    double a1 = coeff1.values[0];
    double b1 = coeff1.values[1];
    double c1 = coeff1.values[2];

    double a2 = coeff2.values[0];
    double b2 = coeff2.values[1];
    double c2 = coeff2.values[2];

    double angle = std::acos(a1 * a2 + b1 * b2 + c1 * c2);

    return angle;
}

double FeatureExtractor::computeHeightDiffTwoPlane_(
    const std::pair<double, double> boundary_point,
    const pcl::ModelCoefficients& coeff1, 
    const pcl::ModelCoefficients& coeff2)
{
    double a1 = coeff1.values[0];
    double b1 = coeff1.values[1];
    double d1 = coeff1.values[3];

    double a2 = coeff2.values[0];
    double b2 = coeff2.values[1];
    double d2 = coeff2.values[3];

    double x = boundary_point.first;
    double y = boundary_point.second;

    double dist1 = std::abs(a1 * x + b1 * y + d1);
    double dist2 = std::abs(a2 * x + b2 * y + d2); 

    return std::abs(dist1 - dist2);
}

template <class PointT>
void FeatureExtractor::removeInliner_(pcl::PointCloud<PointT>& cloud, 
                                      pcl::ModelCoefficients& coeff)
{
    Eigen::Vector4f plane_coeff = Eigen::Vector4f::Map(coeff.values.data(), 4);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::ExtractIndices<PointT> extract;
    for (int i = 0; i < cloud.size(); ++i)
    {
        double distance = pcl::pointToPlaneDistance(cloud.points[i], plane_coeff);
        if (distance < setting_.FIT_PLANE_THRESHOLD)
        {
            inliers->indices.push_back(i);
        }
    }
    extract.setInputCloud(cloud.makeShared());
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(cloud);
}
