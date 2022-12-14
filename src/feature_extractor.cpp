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

FeatureExtractor::FeatureExtractor(lidar_setting_t lidar_setting, int mode) 
{
    // Feature Extractor Mode
    if (mode == 0)
    {
        mode_ = Extractor_modes_t::CURB;
    }
    else if (mode == 1)
    {
        mode_ = Extractor_modes_t::WALL;
    }
    else
    {
        debugger::debugWarningOutput("Check Mode parameter, mode: ", mode, 10);
    }

    // Set Property of LiDAR
    elevation_angles_ = lidar_setting.elevation_angles;
    estimated_lidar_height_ = lidar_setting.height;
    origin_ = pcl::PointXYZ(0, 0, 0);

    // Status
    base_plane_updated_ = false;

    debugger::debugTitleTextOutput("[FeatureExtractor]", "Constructed", 10, BC);
}

void FeatureExtractor::changeSetting(feature_extractor_setting_t setting)
{
    setting_ = setting;
    debugger::debugTitleTextOutput("[FeatureExtractor]", "Setting Changed", 10, BG);
}

void FeatureExtractor::setInputCloud(
    std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER>& rings)
{
    landmark_.clear();
    a_test_.clear();
    b_test_.clear();
    c_test_.clear();
    d_test_.clear();

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
            [](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
    }

    debugger::debugColorTextOutput("Finish SetInputCloud", 1, BG);
}

void FeatureExtractor::run()
{
    if (!base_plane_updated_)
    {
        debugger::debugColorTextOutput("[Feature Extractor] Base Error", 10, BY);
        return;
    }

    if (mode_ == Extractor_modes_t::CURB)
    {
        debugger::debugColorTextOutput("[Curb Extractor] Execute", 5, BW);

        extractGround_();
        extractCurb_();
    }
    else if (mode_ == Extractor_modes_t::WALL)
    {
        debugger::debugColorTextOutput("[Wall Extractor] Execute", 5, BW);
        
        extractWall_();
        //detectGlass_();
    }
}

// Getter Functions
pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getGround()
{
    pcl::PointCloud<pcl::PointXYZ> ground;
    for (int i = 0; i < transformed_ground_.size(); ++i)
    {
        pcl::PointCloud<pcl::PointXYZ> tmp_ground;
        pcl::transformPointCloud(transformed_ground_[i], tmp_ground,
                                 transformation_.transpose());
        //ground += tmp_ground;
        ground += transformed_ground_[i];
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

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureExtractor::getD()
{
    return d_test_.makeShared();
}

std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> FeatureExtractor::getCluster()
{
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> result;

    for (auto& cluster : clusters_)
    {
        result.push_back(cluster.makeShared());
    }

    return result;
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

std::vector<Eigen::Vector4f> FeatureExtractor::getGlassPlanes()
{
    return glass_planes_coeff_;
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
        for (int j = 0; j < rings[i].size(); ++j)
        {
            PointT& point = rings[i][j];

            double azimuth = std::atan2(point.y, point.x);
            double r = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

            int azimuth_idx = int(azimuth * 180 / M_PI + 180) % 360;

            // Updated curr_ring_r_z when the point has a lower z value
            curr_ring_r_z[azimuth_idx].first  += r;
            curr_ring_r_z[azimuth_idx].second += point.z;
            curr_ring_cloud[azimuth_idx].push_back(point);
        }

        // Compute average
        for (int k = 0; k < section_number; ++k)
        {
            curr_ring_r_z[k].first  /= curr_ring_cloud[k].size();
            curr_ring_r_z[k].second /= curr_ring_cloud[k].size();
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
    // | c   0        -a  |
    // | -ab a^2+c^2  -bc |
    // | a   b        c   |
    double a = base_coeff_(0);
    double b = base_coeff_(1);
    double c = base_coeff_(2);
    transformation_ = Eigen::Matrix4f::Identity(4, 4);
    transformation_.block<3, 3>(0, 0) << 
        c,      0,                               -a,
        -a * b, std::pow(a, 2) + std::pow(c, 2), -b * c,
        a,      b,                               c;
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
        transformed_ground_[i].clear();
        for (int j = 0; j < transformed_rings_[i].size(); ++j)
        {
            pcl::PointXYZ point(transformed_rings_[i][j].x,
                                transformed_rings_[i][j].y,
                                transformed_rings_[i][j].z);

            // Filter outside range
            if ((point.x < 0 && point.x > -2.0 && std::abs(point.y) < 0.5) ||
                 std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)) >= max_distance)
            {
                continue;
            }

            // Visualization for Ground
            if (point.z < -estimated_lidar_height_ + setting_.GROUND_DIST_THRESHOLD &&
                point.z > -estimated_lidar_height_ - setting_.GROUND_DIST_THRESHOLD)
            {
                transformed_ground_[i].push_back(point);
                continue;
            }
            
            // Filter Ground and Ceiling
            if (point.z < -estimated_lidar_height_ + setting_.GROUND_DIST_THRESHOLD ||
                point.z > -estimated_lidar_height_ + setting_.HEIGHT_THRESHOLD)
            {
                continue;
            }

            // Index of Grid 
            int m = center_m + (point.x + grid_length / 2) / grid_length;
            int n = center_n + (point.y + grid_length / 2) / grid_length;

            // Update Min/Max Height
            grid_min_z[m][n] = std::min((double)point.z, grid_min_z[m][n]);
            grid_max_z[m][n] = std::max((double)point.z, grid_max_z[m][n]);

            // Save Points in each cell
            grid_cloud_[m][n].push_back(point);

            //a_test_.push_back(point);
        }
    }
    debugger::debugColorTextOutput("Finished Saving PointCloud in Grid", 3, BK);

    //pcl::transformPointCloud(a_test_, b_test_, transformation_.transpose());
    //a_test_.clear();

    // Resize and Clear
    grid_normals_.clear();
    grid_normals_.resize(grid_size, std::vector<Eigen::Vector3f>(grid_size, {0, 0, 0}));
    grid_centroid_.clear();
    grid_centroid_.resize(grid_size, std::vector<pcl::PointXYZ>(grid_size, {0, 0, 0}));

    // Compute RANSAC for each grid and get Normal Vector
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
    debugger::debugColorTextOutput("Finished RANSAC for each Grid", 3, BK);

    // Filter Grid for candidate wall seed by checking Visibility
    std::vector<std::pair<int, int>> seed_grid_idx;
    for (int k = 0; k < 360; ++k)
    {
        std::vector<std::pair<int, int>> grid_idx_on_line;
        double end_x = max_distance * std::cos(M_PI * k / 180);
        double end_y = max_distance * std::sin(M_PI * k / 180);
        bresenhamLine(0, 0, end_x, end_y, grid_length, grid_idx_on_line);
        grid_idx_on_line.pop_back();
        
        bool is_blocked = false;
        for (auto& grid_idx : grid_idx_on_line)
        {
            int tmp_m = center_m + grid_idx.first;
            int tmp_n = center_n + grid_idx.second;

            auto& tmp_grid = grid_cloud_[tmp_m][tmp_n];
            double height_diff = grid_max_z[tmp_m][tmp_n] - grid_min_z[tmp_m][tmp_n];

            if (is_blocked || tmp_grid.size() < 3 ||
                height_diff < setting_.WALL_HEIGHT_THRESHOLD)
            {
                continue;
            } 
            
            is_blocked = true;
            seed_grid_idx.push_back({tmp_m, tmp_n});

            //a_test_ += tmp_grid;

            break;
        }
    }
    //pcl::transformPointCloud(a_test_, c_test_, transformation_.transpose());
    //a_test_.clear();

    // Clustering by DFS (with checking normal)
    std::vector<std::vector<int>> grid_visited(grid_size,
                                               std::vector<int>(grid_size, 0));
    clusters_.clear();
    for (auto& grid_idx : seed_grid_idx)
    {
        int m = grid_idx.first;
        int n = grid_idx.second;

        // Skip Grid 
        if (grid_normals_[m][n].isZero(0) ||
            grid_visited[m][n] != 0)
        {
            continue;
        }

        // Clustering
        pcl::PointCloud<pcl::PointXYZ> transformed_cluster;
        std::vector<std::pair<int, int>> index_of_cluster;
        clusterGridDFS_(grid_visited, m, n, m, n,
                        transformed_cluster, index_of_cluster);

        if (index_of_cluster.size() < 2)
        {
            continue;
        }

        // Inverse Transform from transformed_cluster to cluster
        clusters_.push_back({});
        pcl::transformPointCloud(transformed_cluster, clusters_.back(),
                                 transformation_.transpose());

        // Estimate Wall Plane Coeff from Cluster
        pcl::PointIndices inliers;
        auto wall_coeff = estimatePlaneRANSAC(clusters_.back(),
                                              setting_.WALL_FIT_THRESHOLD,
                                              inliers);
        // Intersection Point 
        // A: (-d/a, 0, 0),  B: (0, -d/b, 0)
        Eigen::Vector3f point_a(-wall_coeff(3) / wall_coeff(0), 0, 0);
        Eigen::Vector3f point_b(0, -wall_coeff(3) / wall_coeff(1), 0);
        Eigen::Vector3f vector_ab = (point_b - point_a);
        vector_ab.normalize();

        // Find Projected End point on Intersection of Wall Plane
        Eigen::Vector3f end_point1;
        Eigen::Vector3f end_point2;
        double min_projected_distance = INFINITY;
        double max_projected_distance = -INFINITY;
        for (auto& idx : index_of_cluster)
        {
            Eigen::Vector3f centroid_from_a = -point_a +
                grid_centroid_[idx.first][idx.second].getVector3fMap();
            
            double projected_dist = centroid_from_a.dot(vector_ab);
            if (projected_dist > max_projected_distance)
            {
                max_projected_distance = projected_dist;
                end_point1 = point_a + projected_dist * vector_ab;
            }
            if (projected_dist < min_projected_distance)
            {
                min_projected_distance = projected_dist;
                end_point2 = point_a + projected_dist * vector_ab;
            }
        }

        // Interpolate Endpoint
        Eigen::Vector3f end_points_vector = end_point2 - end_point1;
        double distance = end_points_vector.norm();
        end_points_vector.normalize();
        int num_interpolation = distance / setting_.INTERPOLATION_LENGTH;

        landmark_.push_back(pcl::PointXYZ(end_point1(0), end_point1(1), 0));
        for (int i = 0; i < num_interpolation; ++i)
        {
            Eigen::Vector3f interpolated_point = end_point1 +
                (i + 1) * setting_.INTERPOLATION_LENGTH * end_points_vector;
            landmark_.push_back(pcl::PointXYZ(interpolated_point(0),
                                              interpolated_point(1),
                                              0));
        }
        landmark_.push_back(pcl::PointXYZ(end_point2(0), end_point2(1), 0));
    }
    debugger::debugColorOutput("# of Clustering: ", clusters_.size(), 8, BK);
}

void FeatureExtractor::clusterGridDFS_(std::vector<std::vector<int>>& grid_visited,
                                       int seed_m, int seed_n, int m, int n,
                                       pcl::PointCloud<pcl::PointXYZ>& cluster,
                                       std::vector<std::pair<int, int>>& grid_idx)
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

    // Add point cloud into cluster
    cluster += grid_cloud_[m][n];
    grid_idx.push_back({m, n});

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
    bresenhamLine(start_x, start_y, end_x, end_y,
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
                        center_m + idx.first, center_n + idx.second,
                        cluster, grid_idx);
    }
}

void FeatureExtractor::detectGlass_()
{
    glass_planes_coeff_.clear();

    for (int i = 0; i < RING_NUMBER; ++i)
    {
        for (int j = 0; j < transformed_rings_[i].size(); ++j)
        {
            // Find start point of glass
            pcl::PointXYZI& point_curr = transformed_rings_[i][j];
            
            // Filter Point that have high intensity
            if (point_curr.intensity < setting_.GLASS_INTENSITY)
            {
                continue;
            }
            debugger::debugColorOutput("Point: ", point_curr, 8, BW);

            int ring_size = transformed_rings_[i].size();
            pcl::PointXYZI& point_prev =
                transformed_rings_[i][(j - 1 + ring_size) % ring_size];
            int intensity_delta = point_curr.intensity - point_prev.intensity;

            // Filter Point that high intensity gradient
            if (intensity_delta < setting_.GLASS_INTENSITY_DELTA)
            {
                continue;
            }
            debugger::debugColorOutput("Intensity Delta: ", intensity_delta, 8, BW);

            pcl::PointXYZ p1;
            p1.x = point_curr.x;
            p1.y = point_curr.y;
            p1.z = point_curr.z;

            // Find end point of glass
            for (int k = j + 1; k < transformed_rings_[i].size(); ++k)
            {
                double intensity_delta_tmp = transformed_rings_[i][k - 1].intensity -
                                             transformed_rings_[i][k].intensity;

                if (intensity_delta_tmp > setting_.GLASS_INTENSITY_DELTA ||
                    transformed_rings_[i][k].intensity < setting_.GLASS_INTENSITY)
                {
                    // Compute Middle Point
                    Eigen::Vector4f transformed_glass_point;
                    transformed_glass_point << 
                        (point_curr.x + transformed_rings_[i][k - 1].x) / 2,
                        (point_curr.y + transformed_rings_[i][k - 1].y) / 2,
                        (point_curr.z + transformed_rings_[i][k - 1].z) / 2,
                        1;

                    // Re-Transform
                    Eigen::Vector4f glass_point = transformation_.transpose() *
                                                  transformed_glass_point;

                    // a(x - a) + b(x - b) + c(x - c) = 0
                    double a = glass_point(0);
                    double b = glass_point(1);
                    double c = glass_point(2);
                    double d = -std::pow(a, 2) - std::pow(b, 2) - std::pow(c, 2);

                    Eigen::Vector4f glass_plane_coeff(a, b, c, d);
                    glass_planes_coeff_.push_back(glass_plane_coeff);

                    j = k - 1;

                    pcl::PointXYZ p2;
                    p2.x = glass_point(0);
                    p2.y = glass_point(1);
                    p2.z = glass_point(2);

                    pcl::PointXYZ p3;
                    p3.x = transformed_rings_[i][k - 1].x;
                    p3.y = transformed_rings_[i][k - 1].y;
                    p3.z = transformed_rings_[i][k - 1].z;



                    break;
                }
            }
        }
    }
}

void FeatureExtractor::extractGround_()
{
    // Clear Old Data
    obstacles_.clear();
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        transformed_ground_[i].clear();
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
            pcl::PointXYZ point(transformed_rings_[i][j].x,
                                transformed_rings_[i][j].y,
                                transformed_rings_[i][j].z);

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

            // Update PointCloud in Grid (Label: Ring_ID)
            pcl::PointXYZL point_with_label;
            point_with_label.x = point.x;
            point_with_label.y = point.y;
            point_with_label.z = point.z;
            point_with_label.label = i;

            grid_cloud[m][n].push_back(point_with_label);
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
                                      std::min(grid_min_z[m][n],
                                               max_z_neighbor_ground);
            if (grid_height_diff > setting_.OBSTACLE_THRESHOLD ||
                grid_max_z[m][n] > max_z_neighbor_ground +
                                   setting_.GROUND_DIST_THRESHOLD ||
                grid_min_z[m][n] < min_z_neighbor_ground - 
                                   3 * setting_.GROUND_DIST_THRESHOLD)
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
                    transformed_ground_[point_with_label.label].push_back(point);
                }
            }
        }
    }

    // Sort Point Cloud by Couter Clock-wisely
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        std::sort(transformed_ground_[i].begin(), transformed_ground_[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
    }
}

void FeatureExtractor::extractCurb_()
{
    pcl::PointCloud<pcl::PointXYZ> transformed_landmark;
    for (int i = 0; i < transformed_ground_.size(); ++i)
    {
        fitted_lines_[i].clear();
        fitPointCloud_(transformed_ground_[i], fitted_lines_[i]);
        for (int k = 1; k < (int)fitted_lines_[i].size() / 2 - 1; ++k)
        {
            // Assume current line is sidewalk
            pcl::PointXYZ start_point = fitted_lines_[i][2 * k];
            pcl::PointXYZ end_point   = fitted_lines_[i][2 * k + 1];

            Eigen::Vector3f start_curr = start_point.getVector3fMap();
            Eigen::Vector3f end_curr   = end_point.getVector3fMap();

            // Adjacent Downsampled lines
            Eigen::Vector3f start_prev = fitted_lines_[i][2 * k - 2].getVector3fMap();
            Eigen::Vector3f end_prev   = fitted_lines_[i][2 * k - 1].getVector3fMap();
            Eigen::Vector3f start_next = fitted_lines_[i][2 * k + 2].getVector3fMap();
            Eigen::Vector3f end_next   = fitted_lines_[i][2 * k + 3].getVector3fMap();

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
                a_test_.push_back(start_point);
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
                b_test_.push_back(end_point);
                continue;
            }
           
            // Continued Curb
            // Left: Sidewalk | Center: Curb | Right: Road
            v_a = (start_prev - end_prev).head(2); // Road
            v_b = (end_curr - start_curr).head(2); // Curb
            angle_road_curb = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (azimuth_diff_prev < setting_.DISCONTINUITY_AZIMUTH &&
                //azimuth_diff_next < setting_.DISCONTINUITY_DISTANCE &&
                end_curr(2) - start_curr(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                end_curr(2) - end_prev(2) > setting_.CURB_HEIGHT_THRESHOLD &&
                angle_road_curb < setting_.CURB_ANGLE_THRESHOLD &&
                start_curr.norm() > end_curr.norm() &&
                //(start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                (start_next - end_next).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (start_next - end_next).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(start_point);
                c_test_.push_back(start_point);
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
                //(end_curr - start_next).norm() < setting_.DISCONTINUITY_DISTANCE &&
                //(start_curr - end_prev).norm() < setting_.DISCONTINUITY_DISTANCE &&
                (end_prev - start_prev).norm() > setting_.SIDEWALK_MIN_LENGTH &&
                (end_prev - start_prev).norm() < setting_.SIDEWALK_MAX_LENGTH)
            {
                transformed_landmark.push_back(end_point);
                d_test_.push_back(end_point);
                continue;
            }
        }
    }
    // Transform Back
    pcl::transformPointCloud(transformed_landmark, landmark_,
                             transformation_.transpose());
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
            bresenhamLine(center_x, center_y, mean_x, mean_y,
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
        
        // Distance in XY vector space
        double distance_diff = (point_curr.head(2) - point_next.head(2)).norm();
        double distance_threshold = point_curr.head(2).norm() *
                                    setting_.AZIMUTH_RESOLUTION * 
                                    setting_.DISCONTINUITY;

        // Execute Douglas-Peucker for each cluster
        if (distance_diff > distance_threshold || std::next(it, 2) == in_ring.end())
        {
            // Skip cluster less than 3 points
            if (std::distance(start_iter, it) < 2)
            {
                start_iter = it + 1;
                continue;
            }

            // Execute Douglas-Peucker Fitting Algorithm
            std::vector<PointT> fitted_line =
                douglasPeucker<PointT>(start_iter, it, setting_.EPSILON);
            
            // Save Line Segment
            for (int i = 0; i < (int)fitted_line.size() - 1; ++i)
            {
                // Skip short(noisy) line
                double line_length = pcl::euclideanDistance(fitted_line[i],
                                                            fitted_line[i + 1]);
                if (line_length < 0.5)
                {
                    continue;
                }

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
