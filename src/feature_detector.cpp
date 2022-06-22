/*******************************************************************************
 * File:        feature_detector.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: Detect Curb and boundary of grass with LiDAR and Camera
*******************************************************************************/
#include "feature_detection/feature_detector.h"

using namespace bipedlab;

FeatureDetector::FeatureDetector(lidar_setting_t lidar_setting)
{
    debugger::debugColorTextOutput("Constructing FeatureDetector", 10, BC);
    
    // Set Property
    ring_number_ = lidar_setting.ring_number;
    elevation_angles_ = lidar_setting.elevation_angles;

    section_direction_ = {{ 
            {-1, 0}, {-0.7071, -0.7071}, {0, -1}, {0.7071, -0.7071},
            {1, 0}, {0.7071, 0.7071}, {0, 1}, {-0.7071, 0.7071} }};
   
    cloud_received_ = false;
}

void FeatureDetector::setInputCloud(feature_detector_setting_t setting,
        std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER>& rings)
{
    a_test_.clear();
    b_test_.clear();
    landmark_.clear();

    setting_ = setting;

    // Resize & Clear
    section_number_ = (setting_.RING_TO_ANALYZE + 1) / 2;
    for (int q = 0; q < 8; ++q)
    {
        multi_region_[q].resize(section_number_);
        section_distances_[q].resize(section_number_, 0.0);
        for (int k = 0; k < section_number_; ++k)
        {
            multi_region_[q][k].clear();
        }
    }

    // Lidar Origin
    pcl::PointXYZ origin(0, 0, 0);
    
    // Sampling Candidates for Base
    pcl::PointCloud<pcl::PointXYZI> base_candidate;
    for (int i = 0; i < setting_.RING_TO_FIT_BASE; ++i)
    {
        int ring_size = rings[i].size();
        if (ring_size < setting_.LOCAL_WINDOW_SIZE)
        {
            continue;
        }

        // Sort Point Cloud by Couter Clock-wisely
        std::sort(rings[i].begin(), rings[i].end(),
            [](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );
        
        // Select Candidates for fitting plane by checking alignedness
        for (int j = 0; j < ring_size; ++j)
        {
            double alignedness_coeff = 0.0;
            for (int w = -setting_.LOCAL_WINDOW_SIZE;
                        w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            {
                pcl::PointXYZI& p_curr =
                        rings[i][(j + w + ring_size) % ring_size];
                pcl::PointXYZI& p_prev =
                        rings[i][(j + w - 1 + ring_size) % ring_size];
                pcl::PointXYZI& p_next =
                        rings[i][(j + w + 1 + ring_size) % ring_size];
                
                alignedness_coeff += pcl::euclideanDistance(p_prev, p_next) /
                        (pcl::euclideanDistance(p_prev, p_curr) + 
                         pcl::euclideanDistance(p_curr, p_next));
            }
            alignedness_coeff /= (2 * setting_.LOCAL_WINDOW_SIZE + 1);

            if (alignedness_coeff > setting_.ALIGNEDNESS_THRESHOLD)
            {
                base_candidate.push_back(rings[i][j]);
            }
        }
    } // Finish Sampling Candidates for Base

    if (base_candidate.size() < 10)
    {
        PCL_ERROR("Not enough data to estimate a base planar model.\n");
        cloud_received_ = false;

        return;
    }
    cloud_received_ = true;

    // Set Base Planar Model (ax + by + cz + d = 0)
    base_coeff_ = estimatePlane_(base_candidate);
    debugger::debugColorOutput("Base Coefficients \n", base_coeff_, 3, BG); 

    double a = base_coeff_.values[0];
    double b = base_coeff_.values[1];
    double c = base_coeff_.values[2];
    double d = base_coeff_.values[3];
    // LiDAR height [d]
    lidar_height_ = d;

    // Perpendicular Base
    if (c < 0.8)
    {
        debugger::debugColorTextOutput("Check Base \n", 2, BG); 
        return;
    }

    // Section Boundary Distance
    for (int q = 0; q < 8; ++q)
    {
        double alpha = std::asin(-(a * section_direction_[q].first +
                b * section_direction_[q].second) / c);
        double dist_base_to_origin = std::abs(d / c) * std::cos(alpha);
        for (int k = 1; k < section_number_; ++k)
        {
            double elevation_angle = (elevation_angles_[2 * k + 1] +
                    elevation_angles_[2 * k + 2]) / 2;
            double theta = alpha - elevation_angle;
            section_distances_[q][k] = (theta <= 0) ? INFINITY : 
                    dist_base_to_origin / std::sin(theta);

            debugger::debugColorOutput("Section Distance[" + 
                    std::to_string(q) + ", " + std::to_string(k) + "]: ",
                    section_distances_[q][k], 2, BW);
        }

    }

    // Container for saving min_distance for each azimuth
    const int azimuth_dividing_number = 360;
    std::array<double, azimuth_dividing_number> prev_ring_distances;
    std::array<double, azimuth_dividing_number> curr_ring_distances;
    prev_ring_distances.fill(0.0);
    curr_ring_distances.fill(0.0);

    // Filtering out for multi-region planar model
    for (int i = 0; i < setting_.RING_TO_ANALYZE; ++i)
    {
        int ring_size = rings[i].size();

        // Sort Ring 
        std::sort(rings[i].begin(), rings[i].end(),
            [](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );

        // Elevation angle of ring
        double elevation_angle = elevation_angles_[i];
        double elevation_angle_prev = (i > 0) ?
                elevation_angles_[i - 1] : -M_PI / 2;

        debugger::debugColorOutput("# of points in ring[" + 
                std::to_string(i) + "] : ", ring_size, 1, BK);
        debugger::debugColorOutput("Elevation angle of ring[" +
                std::to_string(i) + "] : ", elevation_angle, 1, BK);

        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZI& point = rings[i][j];

            double distance = pcl::euclideanDistance(point, origin);
            double azimuth = std::atan2(point.y, point.x);
            double alpha = std::asin(
                    -(a * std::cos(azimuth) + b * std::sin(azimuth)) / c);
            double dist_base_to_origin = std::abs(d / c) * std::cos(alpha);
            double theta = alpha - elevation_angle;

            // filter when ray head to sky
            if (theta <= 0)
            {
                continue;
            }

            // Save minimum distance for each horizontal section
            // for next ring filtering.
            int z = int(azimuth * 180 / M_PI + 180) % 360;
            curr_ring_distances[z] = (curr_ring_distances[z] != 0) ?
                    std::min(curr_ring_distances[z], distance) : distance;

            // Two filters for obstacles(buildings, vehicles, and etc.)
            // Filter Obstacles with estimated distance
            double distance_threshold_estimate = dist_base_to_origin /
                    std::sin(theta + setting_.ANGLE_BUFFER);

            // Filter Obstacles with previous ring distance
            double theta_prev = alpha - elevation_angle_prev;
            double distance_threshold_iter = prev_ring_distances[z] / 
                    std::sin(theta + setting_.ANGLE_BUFFER) *
                    std::sin(theta_prev + setting_.ANGLE_BUFFER);

            if (distance < distance_threshold_estimate ||
                distance < distance_threshold_iter)
            {
                continue;
            }
            
            // Filter Grass
            // One of three direction should be aligned
            std::vector<double> dist_diff_one(2 * setting_.LOCAL_WINDOW_SIZE);
            std::vector<double> dist_diff_two(2 * setting_.LOCAL_WINDOW_SIZE - 1);
            for (int w = -setting_.LOCAL_WINDOW_SIZE + 1;
                    w < setting_.LOCAL_WINDOW_SIZE; ++w)
            {
                pcl::PointXYZI& p_prev = rings[i][(j + w - 1 + ring_size) % ring_size];
                pcl::PointXYZI& p_curr = rings[i][(j + w + ring_size) % ring_size];
                pcl::PointXYZI& p_next = rings[i][(j + w + 1 + ring_size) % ring_size];

                int idx = w + setting_.LOCAL_WINDOW_SIZE - 1;
                dist_diff_one[idx] = std::sqrt(
                        std::pow(p_curr.x - p_prev.x, 2) +
                        std::pow(p_curr.y - p_prev.y, 2));

                dist_diff_two[idx] = std::sqrt(
                        std::pow(p_next.x - p_prev.x, 2) +
                        std::pow(p_next.y - p_prev.y, 2));

                if (w == setting_.LOCAL_WINDOW_SIZE - 1)
                {
                    dist_diff_one[idx + 1] = std::sqrt(
                            std::pow(p_next.x - p_curr.x, 2) +
                            std::pow(p_next.y - p_curr.y, 2));
                }
            }

            double alignedness_coeff_a = 0.0;
            double alignedness_coeff_b = 0.0;
            double alignedness_coeff_c = 0.0;
            for (int w = 1; w < setting_.LOCAL_WINDOW_SIZE; ++w)
            {
                // section A
                int idx_a = w;
                alignedness_coeff_a += dist_diff_two[idx_a - 1] /
                        (dist_diff_one[idx_a - 1] + dist_diff_one[idx_a]);

                // section B
                int idx_b = setting_.LOCAL_WINDOW_SIZE / 2 + w;
                alignedness_coeff_b += dist_diff_two[idx_b - 1] /
                        (dist_diff_one[idx_b - 1] + dist_diff_one[idx_b]);

                // section C
                int idx_c = setting_.LOCAL_WINDOW_SIZE + w;
                alignedness_coeff_c += dist_diff_two[idx_c - 1] /
                        (dist_diff_one[idx_c - 1] + dist_diff_one[idx_c]);
            }
            alignedness_coeff_a /= (setting_.LOCAL_WINDOW_SIZE - 1);
            alignedness_coeff_b /= (setting_.LOCAL_WINDOW_SIZE - 1);
            alignedness_coeff_c /= (setting_.LOCAL_WINDOW_SIZE - 1);

            // Alignedness filter
            if (alignedness_coeff_a < setting_.ALIGNEDNESS_THRESHOLD &&
                    alignedness_coeff_b < setting_.ALIGNEDNESS_THRESHOLD &&
                    alignedness_coeff_c < setting_.ALIGNEDNESS_THRESHOLD)
            {
                continue;
            }

            // Compute Multi-Region Segment idx
            int q = int(azimuth * 4 / M_PI + 4.5) % 8;
            int k = 0;
            while (k < section_distances_[q].size() && 
                    distance >= section_distances_[q][k])
            {
                k++;
            }
            k--;

            // Convert to PointXYZL from PointXYZ (Label : ring_id)
            pcl::PointXYZL point_with_label;
            point_with_label.x = point.x;
            point_with_label.y = point.y;
            point_with_label.z = point.z;
            point_with_label.label = i;

            // Save obstacle-filtered Points in multi-section
            multi_region_[q][k].push_back(point_with_label); 
        }
        // Change prev_ring_distance for next ring iteration
        prev_ring_distances = curr_ring_distances;
        curr_ring_distances.fill(0);
    } // Finish Filter Obstacles and Save into Multi-Section
    
    debugger::debugColorTextOutput("Finish SetInputCloud", 1, BG);
}

void FeatureDetector::run()
{
    if (cloud_received_)
    {
        filterGround_();
        extractCurb_();
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureDetector::getGround()
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

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureDetector::getLandmark()
{
    return landmark_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureDetector::getA()
{
    return a_test_.makeShared();
}

pcl::PointCloud<pcl::PointXYZ>::Ptr FeatureDetector::getB()
{
    return b_test_.makeShared();
}

template <class PointT>
pcl::ModelCoefficients FeatureDetector::estimatePlane_(
        pcl::PointCloud<PointT>& cloud)
{
    // Plane Model segmentation with RANSAC
    // https://pcl.readthedocs.io/en/latest/planar_segmentation.html
    pcl::ModelCoefficients plane_coeff; 
    if (cloud.size() < 10)
    {
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

    if (inliers.indices.size() == 0)
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
        return plane_coeff;
    }

    return plane_coeff;
}

void FeatureDetector::filterGround_()
{
    // Clear
    for (int i = 0; i < RING_NUMBER; ++i)
    {
        ground_[i].clear();
    }

    //std::array<std::vector<pcl::ModelCoefficients>, 8> multi_region_plane_coeff_;
    for (int q = 0; q < 8; ++q)
    {
        multi_region_plane_coeff_[q].resize(section_number_);

        // Estimate Planar Model for Multi-Region
        for (int k = 0; k < section_number_; ++k)
        {
            multi_region_plane_coeff_[q][k] =
                    estimatePlane_(multi_region_[q][k]);

            debugger::debugColorOutput("# of Points (q: " + 
                    std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                    multi_region_[q][k].size(), 1, BB);

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
                    removeInliner_(multi_region_[q][k], 
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
    pcl::ModelOutlierRemoval<pcl::PointXYZL> ground_filter;
    ground_filter.setThreshold(setting_.GROUND_THRESHOLD);
    ground_filter.setModelType(pcl::SACMODEL_PLANE);
    for (int q = 0; q < 8; ++q)
    {
        for (int k = 0; k < section_number_; ++k)
        {
            ground_filter.setModelCoefficients(multi_region_plane_coeff_[q][k]);
            ground_filter.setInputCloud(multi_region_[q][k].makeShared());
            
            pcl::PointCloud<pcl::PointXYZL> filtered_ring;
            ground_filter.filter(filtered_ring);
            for (pcl::PointXYZL& p : filtered_ring)
            {
                pcl::PointXYZ point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;

                ground_[p.label].push_back(point);
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

        // Filter Discontinued area
        pcl::PointCloud<pcl::PointXYZ> continued_ground_ring;
        pcl::PointCloud<pcl::PointXYZ> cluster;
        int count_continued = 0;
        for (int j = 1; j < ground_[i].size(); ++j)
        {
            pcl::PointXYZ& point_curr = ground_[i][j];
            pcl::PointXYZ& point_prev = ground_[i][j - 1];

            double delta_xy = std::sqrt(std::pow(point_curr.x, 2) +
                    std::pow(point_curr.y, 2)) * setting_.ANGULAR_RESOLUTION;
            
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
    }
}

void FeatureDetector::extractCurb_()
{
    // Lidar Origin
    pcl::PointXYZ origin(0, 0, 0);

    for (int i = 0; i < ground_.size(); ++i)
    {
        double elevation_angle = elevation_angles_[i];
        int ring_size = ground_[i].size();
        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZ& point = ground_[i][j];

            // Property of point
            double azimuth = std::atan2(point.y, point.x);
            double distance = pcl::euclideanDistance(point, origin);
            double distance_xy = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));
            double delta_xy = distance_xy * setting_.ANGULAR_RESOLUTION;
            double theta = std::atan2(lidar_height_, distance_xy);
            double delta_z = delta_xy * std::sin(theta);

            // 1. XY Gap Threshold
            pcl::PointXYZ& point_prev = ground_[i][(j - 1 + ring_size) % ring_size];
            pcl::PointXYZ& point_next = ground_[i][(j + 1 + ring_size) % ring_size];
            
            double gap_xy_prev = std::sqrt(
                    std::pow(point_prev.x - point.x, 2) +
                    std::pow(point_prev.y - point.y, 2));
            double gap_xy_next = std::sqrt(
                    std::pow(point_next.x - point.x, 2) +
                    std::pow(point_next.y - point.y, 2));

            // Threshold (xy Gap)
            if (gap_xy_prev < delta_xy && gap_xy_next < delta_xy)
            {
                continue;
            }

            // 2. Z Gap Threshold
            double gap_z_prev = std::abs(point_prev.z - point.z);
            double gap_z_next = std::abs(point_next.z - point.z);

            // Threshold (z Gap)
            if (gap_z_prev < delta_z && gap_z_next < delta_z)
            {
                continue;
            }

            // 3. Discontinuity (for local window size)
            pcl::PointXYZ& point_prev_window = ground_[i][(j -
                    setting_.LOCAL_WINDOW_SIZE + ring_size) % ring_size];
            pcl::PointXYZ& point_next_window = ground_[i][(j + 
                    setting_.LOCAL_WINDOW_SIZE + ring_size) % ring_size];

            double dist_xy_prev_window = std::sqrt(
                    std::pow(point_prev_window.x - point.x, 2) +
                    std::pow(point_prev_window.y - point.y, 2));
            double dist_xy_next_window = std::sqrt(
                    std::pow(point_next_window.x - point.x, 2) +
                    std::pow(point_next_window.y - point.y, 2));

            if (dist_xy_prev_window > setting_.GROUND_DISCONTINUITY * delta_xy ||
                dist_xy_next_window > setting_.GROUND_DISCONTINUITY * delta_xy)
            {
                continue;
            }
            a_test_.push_back(point);

            // 4. Angular & Height Filter
            //
            // Compute Multi-Region Segment idx
            int q = int(azimuth * 4 / M_PI + 4.5) % 8;
            int k = 0;
            while (k < section_distances_[q].size() && 
                    distance >= section_distances_[q][k])
            {
                k++;
            }
            k--;

            // planar model
            pcl::ModelCoefficients plane_coeff = multi_region_plane_coeff_[q][k];
            double a = plane_coeff.values[0];
            double b = plane_coeff.values[1];
            double c = plane_coeff.values[2];
            double d = plane_coeff.values[3];

            double height_point = pcl::pointToPlaneDistanceSigned(point, a, b, c, d);
            //
            // estimated number of points on CURB
            int n_v = setting_.CURB_HEIGHT / std::sin(theta) / delta_xy;

            Eigen::Vector2f v_a(0.0, 0.0);
            double max_height_diff_a = 0.0;
            int count_a = 1;
            double dist_diff_a = 0.0;
            double distance_xy_a = 0.0;
            while (dist_diff_a < delta_xy * n_v && count_a < n_v)
            {
                pcl::PointXYZ& p_a = ground_[i][(j + count_a + ring_size) % ring_size];
                
                double x_diff = p_a.x - point.x;
                double y_diff = p_a.y - point.y;
                double height_diff = pcl::pointToPlaneDistanceSigned(p_a, a, b, c, d) - height_point;

                dist_diff_a = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
                distance_xy_a = std::sqrt(std::pow(p_a.x, 2) + std::pow(p_a.y, 2));

                v_a(0) += x_diff;
                v_a(1) += y_diff;

                max_height_diff_a = std::max(max_height_diff_a, height_diff);

                count_a++;
            }
            
            Eigen::Vector2f v_b(0.0, 0.0);
            double max_height_diff_b = 0.0;
            int count_b = 1;
            double dist_diff_b = 0.0;
            double distance_xy_b = 0.0;
            while (dist_diff_b < delta_xy * n_v && count_b < n_v)
            {
                pcl::PointXYZ& p_b = ground_[i][(j - count_b + ring_size) % ring_size];
                
                double x_diff = p_b.x - point.x;
                double y_diff = p_b.y - point.y;
                double height_diff = pcl::pointToPlaneDistanceSigned(p_b, a, b, c, d) - height_point;

                dist_diff_b = std::sqrt(std::pow(x_diff, 2) + std::pow(y_diff, 2));
                distance_xy_b = std::sqrt(std::pow(p_b.x, 2) + std::pow(p_b.y, 2));

                v_b(0) += x_diff;
                v_b(1) += y_diff;

                max_height_diff_b = std::max(max_height_diff_b, height_diff);

                count_b++;
            }
            
            // Threshold (Discontinuity Filter for n_v) 
            if (dist_diff_a > setting_.DISCONTINUITY * n_v * delta_xy ||
                dist_diff_b > setting_.DISCONTINUITY * n_v * delta_xy)
            {
                continue;
            }
            b_test_.push_back(point);

            // Threshold (Augluar Filter)
            double angle_window = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());
            if (angle_window > setting_.ANGLE_CURB_THRESHOLD)
            {
                continue;
            }

            // 5. Height Filter
            if ((max_height_diff_a > setting_.HEIGHT_CURB_THRESHOLD &&
                 distance_xy_a < distance_xy) ||
                (max_height_diff_b > setting_.HEIGHT_CURB_THRESHOLD &&
                 distance_xy_b < distance_xy))
            {
                landmark_.push_back(point);

                // Prevent continuous points
                j += n_v;
            }
        }
    }
}

double FeatureDetector::computeAngleTwoPlane_(
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

double FeatureDetector::computeHeightDiffTwoPlane_(
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
void FeatureDetector::removeInliner_(pcl::PointCloud<PointT>& cloud, 
        pcl::ModelCoefficients& coeff)
{
    Eigen::Vector4f plane_coeff;
    plane_coeff << coeff.values[0], coeff.values[1],
                   coeff.values[2], coeff.values[3];

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
