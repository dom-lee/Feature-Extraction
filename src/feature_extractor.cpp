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
    a_test_.clear();
    b_test_.clear();
    c_test_.clear();

    // Estimate Base Plane
    if (!estimateBasePlane_(rings))
    {
        return;
    }
    
    // Transform the Point Cloud to make base parallel to the XY plane
    for (int i = 0; i < RING_NUMBER; ++i)
    {
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
        extractGround_();
        extractCurb_();
        //estimateRoadModel_();
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

std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> FeatureExtractor::getDownSampledLines()
{
    std::vector<std::pair<pcl::PointXYZ, pcl::PointXYZ>> downsampled_lines;
    for (int i = 0; i < setting_.RING_TO_ANALYZE; ++i)
    {
        for (int k = 0; k < downsampled_lines_[i].size(); ++k)
        {
            Eigen::Vector3f& p1 = downsampled_lines_[i][k].first;
            Eigen::Vector3f& p2 = downsampled_lines_[i][k].second;

            pcl::PointXYZ point1(p1(0), p1(1), p1(2));
            pcl::PointXYZ point2(p2(0), p2(1), p2(2));
            
            downsampled_lines.push_back({point1, point2});
        }
    }
    return downsampled_lines;
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

            if (smoothness_coeff > setting_.SMOOTH_THRESHOLD)
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
    transformation_.block<3, 3>(0, 0) << base_coeff_(2), 0, -base_coeff_(0),
                                         0, base_coeff_(2), -base_coeff_(1),
                                         base_coeff_(0), base_coeff_(1), base_coeff_(2);
    transformation_.rowwise().normalize();

    debugger::debugColorOutput("Transformation \n", transformation_, 3, BB); 
    
    // Estimated LiDAR Height from ground : |d| 
    estimated_lidar_height_ = std::abs(base_coeff_(3));
    debugger::debugColorOutput("Estimated LiDAR Height : ",
                               estimated_lidar_height_, 5, BW); 

    return true;
}

template <class PointT>
Eigen::Vector4f FeatureExtractor::estimatePlaneRANSAC_(
    pcl::PointCloud<PointT>& cloud)
{
    Eigen::Vector4f plane_coeff;
    // Plane Model segmentation with RANSAC
    // https://pcl.readthedocs.io/en/latest/planar_segmentation.html
    pcl::ModelCoefficients plane_model_coeff; 
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
    seg.segment(inliers, plane_model_coeff);

    if (inliers.indices.empty())
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    plane_coeff << plane_model_coeff.values[0], plane_model_coeff.values[1],
                   plane_model_coeff.values[2], plane_model_coeff.values[3];

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

void FeatureExtractor::extractCurb_()
{
    downsampled_lines_.resize(ground_.size());
    pcl::PointCloud<pcl::PointXYZ> transformed_landmark;
    for (int i = 0; i < ground_.size(); ++i)
    {
        downsampled_lines_[i].clear();

        if (ground_[i].empty())
        {
            continue;
        }
        int ring_size = ground_[i].size();

        // Precompute Line Section
        int smooth_points_count = 1;
        Eigen::Vector3f start_point = ground_[i][0].getVector3fMap();
        for (int j = 0; j < ring_size; ++j)
        {
            Eigen::Vector3f point_curr = ground_[i][j].getVector3fMap();
            Eigen::Vector3f point_next = ground_[i][(j + 1) % ring_size].getVector3fMap();

            // Check Discontinuity
            double azimuth_curr = std::atan2(point_curr(1), point_curr(0));
            double azimuth_next = std::atan2(point_next(1), point_next(0));

            double azimuth_diff = azimuth_next - azimuth_curr;
            double distance_diff = (point_curr - point_next).norm();
            bool is_next_point_discontinued = 
                (azimuth_diff > setting_.DISCONTINUITY_AZIMUTH ||
                 distance_diff > setting_.DISCONTINUITY_DISTANCE);

            if (is_next_point_discontinued)
            {
                a_test_.push_back(ground_[i][(j + 1) % ring_size]);
            }

            // Skip when current_point is same with start_point
            if (point_curr == start_point)
            {
                start_point = is_next_point_discontinued ? point_next : start_point;
                smooth_points_count = is_next_point_discontinued ? 0 : 1;
                continue;
            }

            // Corner Detection for next point by checking Curvature
            Eigen::Vector3f sum_vector = Eigen::Vector3f::Zero();
            for (int w = 0; w < setting_.SMOOTH_COUNT; ++w)
            {
                int idx_next = (j + w + 1) % ring_size;
                int idx_prev = (j - w - 1 + ring_size) % ring_size;

                sum_vector += (point_next - ground_[i][idx_next].getVector3fMap());
                sum_vector += (point_next - ground_[i][idx_prev].getVector3fMap());
            }
            double smoothness = sum_vector.norm() / point_next.norm() /
                                (2 * setting_.SMOOTH_COUNT);
            double is_next_point_sharp = (smoothness > setting_.SMOOTHNESS_THRESHOLD);


            if (is_next_point_sharp)
            {
                b_test_.push_back(ground_[i][(j + 1) % ring_size]);
            }

            // Check whether it needs to add downsampled lines
            bool need_update_line = is_next_point_discontinued;
            if (!need_update_line && is_next_point_sharp)
            {
                Eigen::Vector3f v_a = start_point - point_curr;
                Eigen::Vector3f v_b = point_next - point_curr;
                double angle = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());

                if (angle < setting_.CONTINUITY_ANGLE)
                {
                    need_update_line = true;
                }
            }

            // Update Downsampled Lines
            if (need_update_line)
            {
                if (smooth_points_count >= 10)
                {
                    downsampled_lines_[i].push_back({start_point, point_curr});
                }
                start_point = is_next_point_discontinued ? point_next : point_curr;
                smooth_points_count = is_next_point_discontinued ? 0 : 1;

                pcl::PointXYZ pp(start_point(0), start_point(1), start_point(2));
                c_test_.push_back(pp);
            }
            smooth_points_count++;
        } // End of Pre-Computing Feature Lines

        // Execute Curb Extraction with FeatureLines
        for (int k = 0; k < (int)downsampled_lines_[i].size() - 1; ++k)
        {
            // Assume current line is sidewalk
            Eigen::Vector3f& start_curr = downsampled_lines_[i][k].first;
            Eigen::Vector3f& end_curr   = downsampled_lines_[i][k].second;
            pcl::PointXYZ start_point(start_curr(0), start_curr(1), start_curr(2));
            pcl::PointXYZ end_point(end_curr(0), end_curr(1), end_curr(2));

            // Adjacent Downsampled lines
            Eigen::Vector3f& start_next = downsampled_lines_[i][k + 1].first;
            Eigen::Vector3f& end_next   = downsampled_lines_[i][k + 1].second;
            Eigen::Vector3f& start_prev = downsampled_lines_[i][k - 1].first;
            Eigen::Vector3f& end_prev   = downsampled_lines_[i][k - 1].second;

            // Vector for using angular threshold
            Eigen::Vector2f v_a, v_b, v_c;
            double angle_road_curb;

            // Discontinued Curb
            // Left: Sidewalk | Right: Road
            double azimuth_diff_prev = std::atan2(start_curr(1), start_curr(0)) - 
                                       std::atan2(end_prev(1), end_prev(0));
            v_a = (start_prev - end_prev).head(2); // Road
            v_b = (start_curr - end_prev).head(2); // Curb
            angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                start_curr(2) - end_prev(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                end_prev.norm() > start_curr.norm() &&
                (start_curr - end_curr).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (start_curr - end_curr).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(start_point);
                continue;
            }
            
            // Discontinued Curb
            // Left: Road | Right: Sidewalk
            double azimuth_diff_next = std::atan2(start_next(1), start_next(0)) -
                                       std::atan2(end_curr(1), end_curr(0));
            v_a = (end_next - start_next).head(2); // Road
            v_b = (end_curr - start_next).head(2); // Curb 
            angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (azimuth_diff_next < setting_.DISCONTINUITY_AZIMUTH &&
                end_curr(2) - start_next(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                start_next.norm() > end_curr.norm() &&
                (start_curr - end_curr).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (start_curr - end_curr).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(end_point);
                continue;
            }
           
            // Continued Curb
            // Left: Sidewalk | Center: Curb | Right: Road
            v_a = (start_prev - end_prev).head(2); // Road
            v_b = (end_curr - start_curr).head(2); // Curb
            angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                azimuth_diff_next < setting_.DISCONTINUITY_DISTANCE &&
                end_curr(2) - start_curr(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                end_curr(2) - end_prev(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                start_curr.norm() > end_curr.norm() &&
                (start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                (start_next - end_next).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (start_next - end_next).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(start_point);
                continue;
            }

            // Continued Curb
            // Left: Road | Center: Curb | Right: Sidewalk
            v_a = (end_next - start_next).head(2); // Road
            v_b = (start_curr - end_curr).head(2); // Curb
            angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (azimuth_diff_next < setting_.DISCONTINUITY_AZIMUTH &&
                azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                start_curr(2) - end_curr(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                start_curr(2) - start_next(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                end_curr.norm() > start_curr.norm() &&
                (end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                (start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                (end_prev - start_prev).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (end_prev - start_prev).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(end_point);
                continue;
            }
        }
    }

    // Restoration
    landmark_.clear();
    pcl::transformPointCloud(transformed_landmark, landmark_, transformation_.transpose());
}

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

void FeatureExtractor::executeDouglasPeucker(
    pcl::PointCloud<pcl::PointXYZ>& points,
    pcl::PointCloud<pcl::PointXYZ>& out_points,
    double epsilon)
{
    out_points.clear();

    double dmax = 0;
    double index = 0;

    for (int i = 1; i < (int)endpoints.size() - 1; ++i)
    {
        double distance = 
    }

    if (dmax > epsilon)
    {
        pcl::PointCloud<pcl::PointXYZ> recursive_results_1;
        pcl::PointCloud<pcl::PointXYZ> recursive_results_2;
        executeDouglasPeucker()
    }
}
