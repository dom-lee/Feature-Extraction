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
        filtered_region_[q].resize(section_number_);
        section_distances_[q].resize(section_number_, 0.0);
        for (int k = 0; k < section_number_; ++k)
        {
            multi_region_[q][k].clear();
            filtered_region_[q][k].clear();
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
        
        // Cacluate Difference of distance from origin
        std::vector<double> diff(ring_size, 0.0);
        double prev_dist = pcl::euclideanDistance(rings[i].back(), origin);
        for (int j = 0; j < ring_size; ++j)
        {
            double curr_dist = pcl::euclideanDistance(rings[i][j], origin);
            diff[j] = std::abs(curr_dist - prev_dist) / curr_dist;
            
            prev_dist = curr_dist;
        }
        
        // Select Candidates for fitting plane by checking noise
        for (int j = 0; j < ring_size; ++j)
        {
            // Calculate noise
            double sum_of_diff = 0.0;
            for (int w = -setting_.LOCAL_WINDOW_SIZE;
                    w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            {
                sum_of_diff += diff[(j + w + ring_size) % ring_size];
            }
            
            // Filtered Cloud
            if (sum_of_diff < setting_.NOISE_THRESHOLD)
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
        //// Sort Point Cloud by Couter Clock-wisely
        //std::sort(rings[i].begin(), rings[i].end(),
            //[](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
            //{
                //return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            //}
        //);
        
        //// Cacluate Distance from origin(0,0,0) 
        int ring_size = rings[i].size();
        //std::vector<double> dist_origin(ring_size, 0.0);
        //for (int j = 0; j < ring_size; ++j)
        //{
            //dist_origin[j] = pcl::euclideanDistance(rings[i][j], origin);
        //}
        
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
            pcl::PointXYZI& p = rings[i][j];

            double distance = pcl::euclideanDistance(p, origin);
            double azimuth = std::atan2(p.y, p.x);
            double alpha = std::asin(
                    -(a * std::cos(azimuth) + b * std::sin(azimuth)) / c);
            double dist_base_to_origin = std::abs(d / c) * std::cos(alpha);
            double theta = alpha - elevation_angle;

            // filter when ray head to sky
            if (theta <= 0)
            {
                continue;
            }

            // Filter Obstacles with estimated distance
            double distance_threshold_estimate = dist_base_to_origin /
                    std::sin(theta + setting_.ANGLE_BUFFER);

            if (distance < distance_threshold_estimate)
            {
                continue;
            }

            int z = int(azimuth * 180 / M_PI + 180) % 360;
            curr_ring_distances[z] = (curr_ring_distances[z] != 0) ?
                    std::min(curr_ring_distances[z], distance) : distance;

            // Filter Obstacles with previous ring distance
            double theta_prev = alpha - elevation_angle_prev;
            double distance_threshold_iter = prev_ring_distances[z] / 
                    std::sin(theta + setting_.ANGLE_BUFFER) *
                    std::sin(theta_prev + setting_.ANGLE_BUFFER);

            if (distance < distance_threshold_iter)
            {
                continue;
            }
            
            // Compute Segment idx
            int q = int(azimuth * 4 / M_PI + 4.5) % 8;
            int k = 0;
            while (k < section_distances_[q].size() && 
                    distance >= section_distances_[q][k])
            {
                k++;
            }
            k--;
            
            // Convert to PointXYZL from PointXYZ
            pcl::PointXYZL point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.label = i;

            // Save filtered Points in multi-section
            multi_region_[q][k].push_back(point); 
            filtered_region_[q][k].push_back(point);

            //TODO DELETE /////////////////////
            pcl::PointXYZ pp;
            pp.x = p.x;
            pp.y = p.y;
            pp.z = p.z;
            if ( (q + k) % 2 == 0)
            {
                a_test_.push_back(pp);
            }
            else
            {
                b_test_.push_back(pp);
            }
            ///////////////////////////////////

            //// Calculate Variance
            //double sum_dist = 0.0;
            //for (int w = -setting_.LOCAL_WINDOW_SIZE;
                    //w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            //{
                //sum_dist += dist_origin[(j + w + ring_size) % ring_size];
            //}
            //double mean_dist = sum_dist / (2 * setting_.LOCAL_WINDOW_SIZE + 1);

            //double var = 0.0;
            //for (int w = -setting_.LOCAL_WINDOW_SIZE;
                    //w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            //{
                //double dist = dist_origin[(j + w + ring_size) % ring_size];
                //var += std::pow(dist - mean_dist, 2);
            //}
            //var /= (2 * setting_.LOCAL_WINDOW_SIZE + 1);
            
            //// Save candidates for multi-region model
            //if (p.intensity < setting_.INTENSITY_THRESHOLD || 
                    //var < setting_.NOISE_THRESHOLD)
            //{
                //filtered_region_[q][k].push_back(point);
                //landmark_.push_back(pp);
            //}
        }
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
        //extractCurb_();
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

    //// Optimize Coefficients
    //typename pcl::SampleConsensusModel<PointT>::Ptr model = seg.getModel();
    //Eigen::VectorXf coeff_refined;                                             
    //Eigen::Vector4f coeff_raw(plane_coeff.values.data());                    
    //model->optimizeModelCoefficients(inliers.indices, coeff_raw, coeff_refined);
    //plane_coeff.values.resize(coeff_refined.size());                         
    //memcpy(&(plane_coeff.values[0]), &coeff_refined[0],                        
            //coeff_refined.size() * sizeof (float));                             
    //// Refine inliers                                                          
    //model.selectWithinDistance(coeff_refined, 
            //setting_.FIT_PLANE_THRESHOLD, inliers.indices);

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

    std::array<std::vector<pcl::ModelCoefficients>, 8> multi_region_plane_coeff;
    for (int q = 0; q < 8; ++q)
    {
        multi_region_plane_coeff[q].resize(section_number_);

        // Estimate Planar Model for Multi-Region
        for (int k = 0; k < section_number_; ++k)
        {
            multi_region_plane_coeff[q][k] = estimatePlane_(filtered_region_[q][k]);

            debugger::debugColorOutput("# of Points (q: " + 
                    std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                    multi_region_[q][k].size(), 1, BB);

            debugger::debugColorOutput("Plane Coeff (q: " + 
                    std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                    multi_region_plane_coeff[q][k], 1, BB);
            
            pcl::ModelCoefficients& coeff_prev = (k > 0) ? 
                    multi_region_plane_coeff[q][k - 1] : base_coeff_;

            // replace with previous section planar model
            if (multi_region_plane_coeff[q][k].values.empty())
            {
                multi_region_plane_coeff[q][k] = coeff_prev;
            }
            // Compute Angle Difference with two plane model
            else
            {                
                double angle_diff = computeAngleTwoPlane_(coeff_prev,
                        multi_region_plane_coeff[q][k]);
                angle_diff = std::min(angle_diff, M_PI - angle_diff);
                
                std::pair<double, double> boundary_point = 
                        {section_direction_[q].first * section_distances_[q][k],
                        section_direction_[q].second * section_distances_[q][k]};
                double height_diff = computeHeightDiffTwoPlane_(boundary_point,
                        coeff_prev, multi_region_plane_coeff[q][k]); 

                if (M_PI / 2 - angle_diff < setting_.ANGLE_DIFF_THRESHOLD)
                {
                    debugger::debugColorTextOutput("Remove Inlier region (q: " + 
                            std::to_string(q) + ", k: " + std::to_string(k) +
                            ") : ", 2, BM);

                    // remove points that forms vertical to base
                    removeInliner_(filtered_region_[q][k], 
                            multi_region_plane_coeff[q][k]);
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

                    multi_region_plane_coeff[q][k] = coeff_prev;
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
            double a = multi_region_plane_coeff[q][k].values[0];
            double b = multi_region_plane_coeff[q][k].values[1];
            double c = multi_region_plane_coeff[q][k].values[2];
            double d = multi_region_plane_coeff[q][k].values[3];

            ground_filter.setModelCoefficients(multi_region_plane_coeff[q][k]);
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
    }
}

void FeatureDetector::extractCurb_()
{
    //// Center of Ground
    //pcl::PointXYZ center(0, 0, -lidar_height_);

    //for (int i = 0; i < ground_.size(); ++i)
    //{
        //// Sort Point Cloud by Couter Clock-wisely
        //std::sort(ground_[i].begin(), ground_[i].end(),
            //[](const pcl::PointWithRange& lhs, const pcl::PointWithRange& rhs)
            //{
                //return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            //}
        //);

        //int ground_size = ground_[i].size();
        ////for (int j = 0; j < ground_size; ++j)
        //for (int j = 0; j < ground_size; ++j)
        //{
            //pcl::PointXYZ& point = ground_[i][j];

            //double theta = std::atan2(std::abs(point.z), 
                    //std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)));
            //double delta_xy = pcl::euclideanDistance(center, point) * 
                    //setting_.ANGULAR_RESOLUTION;
            //double delta_z = delta_xy * std::sin(theta);
            //int n_v = setting_.CURB_HEIGHT / std::sin(theta) / delta_xy;

            //// Continuity
            //int idx_prev = (j - 1 + ground_size) % ground_size;
            //int idx_next = (j + 1 + ground_size) % ground_size;
            //pcl::PointXYZ& point_prev = ground_[i][idx_prev];
            //pcl::PointXYZ& point_next = ground_[i][idx_next];
            
            //double dist_xy = std::sqrt(std::pow(point_prev.x - point_next.x, 2) +
                    //std::pow(point_prev.y - point_next.y, 2));
            //double dist_z = std::abs(point_prev.z - point_next.z);

            ////// Check Horizonral and Vertical Continuity
            ////if (dist_xy < delta_xy || dist_z < delta_z)
            ////{
                ////continue;
            ////}
            
            //bool continuity = true;
            //bool elevation_a = true;
            //bool elevation_b = true;
            //Eigen::Vector2f v_a(0, 0);
            //Eigen::Vector2f v_b(0, 0);
            //double max_height_diff = 0.0;

            //for (int w = 1; w < n_v + 1 && continuity; ++w)
            //{
                //int idx_a = (j + w + ground_size) % ground_size;
                //int idx_a_prev = (j + w - 1 + ground_size) % ground_size;
                //pcl::PointXYZ& point_a = ground_[i][idx_a];
                //pcl::PointXYZ& point_a_prev = ground_[i][idx_a_prev];

                //int idx_b = (j - w + ground_size) % ground_size;
                //int idx_b_prev = (j - w + 1 + ground_size) % ground_size;
                //pcl::PointXYZ& point_b = ground_[i][idx_b];
                //pcl::PointXYZ& point_b_prev = ground_[i][idx_b_prev];
                
                //double dist_a = pcl::euclideanDistance(point_a, point_a_prev);
                //double dist_b = pcl::euclideanDistance(point_b, point_b_prev);
                
                //// Check setting_.DISCONTINUITY
                //if (dist_a > setting_.DISCONTINUITY * delta_xy || 
                        //dist_b > setting_.DISCONTINUITY * delta_xy)
                //{
                    //continuity = false;
                //}

                //// Check elevation
                //if (point_a.range < point_a_prev.range)
                //{
                    //elevation_a = false;
                //}

                //if (point_b.range < point_b_prev.range)
                //{
                    //elevation_b = false;
                //}
                
                //// Calculating two vector
                //v_a(0) += point_a.x - point.x;
                //v_a(1) += point_a.y - point.y;

                //v_b(0) += point_b.x - point.x;
                //v_b(1) += point_b.y - point.y;

                //double height_diff = std::max(
                        //std::abs(point_a.range - point.range),
                        //std::abs(point_b.range - point.range));
                //double max_height_diff = std::max(max_height_diff, height_diff);
            //}
            
            //// setting_.DISCONTINUITY Threshold 
            //if (!continuity)
            //{
                //continue;
            //}

            //// average
            //v_a /= n_v;
            //v_b /= n_v;
            //double angle = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());

            //// Angle Threshold
            //if (angle > setting_.ANGLE_CURB_THRESHOLD)
            //{
                //continue;
            //}

            ////// Elevation Check
            ////if (!elevation_a && !elevation_b)
            ////{
                ////continue;
            ////}

            //pcl::PointXYZ a;
            //a.x = point.x;
            //a.y = point.y;
            //a.z = point.z;

            //a_test_.push_back(a);

            ////debugger::debugColorOutput("point:", point, 3, BK);
            ////debugger::debugColorOutput("n_v:", n_v, 3, BW);
            ////debugger::debugColorOutput("angle: ", angle, 3, BB);
            ////debugger::debugColorOutput("Diff: ", max_height_diff, 3, BC);
        //}
    //}
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

double FeatureDetector::computeClosestDistDiff_(pcl::PointXYZI& point,
        std::vector<pcl::PointXYZI>& prev_ring)
{
    double azimuth = std::atan2(point.y, point.x);
    double distance = std::sqrt(std::pow(point.y, 2) + std::pow(point.x, 2));
    double min_azimuth_diff = M_PI;
    double closest_dist_diff = INFINITY;
    for (int i = 0; i < prev_ring.size(); ++i)
    {
        pcl::PointXYZI& p = prev_ring[i];
        double azimuth_p = std::atan2(p.y, p.x);
        if (std::abs(azimuth_p - azimuth) < min_azimuth_diff)
        {
            min_azimuth_diff = std::abs(azimuth_p - azimuth);
            closest_dist_diff = distance - std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2)); 
        }
    }

    return closest_dist_diff;
}

    //for (int i = 0; i < rings_.size(); ++i)
    //{
        //ground_[i].clear();

        //int k = i / 2;
        //for (int j = 0; j < rings_[i].size(); ++j)
        //{
            //pcl::PointXYZ& point = rings_[i][j];
            
            //double angle = std::atan2(point.y, point.x);
            //int q = int((angle + 5 / 4 * M_PI) * 2 / M_PI) % 4; 

            //if (q != 3)
            //{
                //continue;
            //}
            
            //double distance = 
                    //pcl::pointToPlaneDistance(point, multi_region_plane_coeff[q][k]);
            //if (distance < setting_.GROUND_THRESHOLD)
            //{
                //ground_[i].push_back(point);
                ////landmark_.push_back(point);
            //}
        //}
    //}

    //// Compute Section Distance 
    //std::vector<double> region_distance(setting_.section_number_ - 1);
    //for (int i = 0; i < region_distance.size(); ++i)
    //{
        //double theta_i = setting_.SECTION_START_ANGLE + 
                //(90 - setting_.SECTION_START_ANGLE) / (setting_.section_number_ - 1) * i;
        //double distance = lidar_height_ * std::tan(theta_i * M_PI / 180);
        //region_distance[i] = distance;
    //}

    // Segment Point Cloud into multi-region
    //for (pcl::PointXYZ& point : cloud_)
    //{
        //double angle = std::atan2(point.y, point.x);
        //int quad_idx = int((angle + 5 / 4 * M_PI) * 2 / M_PI) % 4;

        //double distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

        //int k = 0;
        //while (k < region_distance.size() && distance >= region_distance[k])
        //{
            //k++;
        //}
        //multi_region_[quad_idx][k].push_back(point);
    //}
    
    //// Estimate Planar Model for Multi-Region
    //std::array<std::vector<Eigen::Vector4f>, 4> multi_region_plane_coeff;
    //static std::vector<std::pair<double, double>> direction = 
            //{ {-1, 0}, {0, -1}, {1, 0}, {0, 1} };
    //for (int i = 0; i < 4; ++i)
    //{
        //multi_region_plane_coeff[i].resize(setting_.section_number_);

        //// Find initial region
        //int k = 0;
        //while (multi_region_[i][k].size() < 3)
        //{
            //multi_region_[i][k + 1] += multi_region_[i][k];
            //k++;
        //}
        //multi_region_plane_coeff[i][k] = estimatePlane(multi_region_[i][k]);
        //saveInlinerToGround(multi_region_[i][k], multi_region_plane_coeff[i][k]);

        //for (k++; k < setting_.section_number_; ++k)
        //{
            //multi_region_plane_coeff[i][k] = estimatePlane(multi_region_[i][k]);
            
            //// Compute Height Difference at section
            //double x = direction[i].first * region_distance[k - 1];
            //double y = direction[i].second * region_distance[k - 1];
            //Eigen::Vector4f section_point(x, y, 0, 1);
            
            //// z = -(ax + by + d) / c
            //double diff_height = std::abs(
                    //multi_region_plane_coeff[i][k].dot(section_point) / 
                    //multi_region_plane_coeff[i][k](2) -
                    //multi_region_plane_coeff[i][k - 1].dot(section_point) /
                    //multi_region_plane_coeff[i][k - 1](2)); 
            
            //// cos(A) = [a1, b1, c1] * [a2, b2, c2] (normalized)
            //Eigen::Vector3f normal_tgt = 
                    //multi_region_plane_coeff[i][k].head(3).normalized();
            //Eigen::Vector3f normal_ref = 
                    //multi_region_plane_coeff[i][k - 1].head(3).normalized();
            //double diff_angle = std::acos(normal_tgt.dot(normal_ref)); 
            
            //// Replace with prev planar model when it exceeds threshold
            //if (diff_height > HEIGHT_THRESHOLD || diff_angle > setting_.ANGLE_PLANE_THRESHOLD)
            //{
                //multi_region_plane_coeff[i][k] = multi_region_plane_coeff[i][k - 1];
            //}
            //saveInlinerToGround(multi_region_[i][k], multi_region_plane_coeff[i][k]);
        //}       
    //}

    //for (pcl::PointXYZL point : ground_)
    //{
        //pcl::PointXYZ p;
        //p.x = point.x;
        //p.y = point.y;
        //p.z = point.z;
        //landmark_.push_back(p);
    //}
//}


//void FeatureDetector::saveInlinerToGround(
        //pcl::PointCloud<pcl::PointXYZL>& cloud, Eigen::Vector4f& planar_coeff)
//{
    //for (pcl::PointXYZL& point : cloud)
    //{
        //if (pcl::pointToPlaneDistance(point, planar_coeff) < setting_.GROUND_THRESHOLD)
        //{
            //ground_.push_back(point);
        //}
    //}
//}
//void FeatureDetector::filterGround_()
//{
    //for (int i = 0; i < RING_NUMBER; ++i)
    //{
        //ground_[i].clear();
    //}

    //// Inter-ring distance-based filtering
    //for (int j = 0; j < SEGMENT_ANGLE_NUMBER; ++j)
    //{   
        //std::vector<int> row_indices;
        //for (int i = 0; i < RING_NUMBER; ++i)
        //{
            //// Point exists in a segment
            //if (seg_angle_idx_[i][j] >= 0)
            //{
                //row_indices.push_back(i);
            //}
        //}
        
        //if (row_indices.size() < 2)
        //{
            //continue;
        //}
        
        //// Initialize Reference Point
        //int last_idx = row_indices.back();
        //pcl::PointXYZ& point_ref = rings_[last_idx][seg_angle_idx_[last_idx][j]];
        //double point_ref_height =
                //pcl::pointToPlaneDistance(point_ref, base_coeff_);
        //double point_ref_theta = std::atan2(lidar_height_ - point_ref_height, 
                //std::sqrt(std::pow(point_ref.x, 2) + std::pow(point_ref.y, 2)));
        
        //// Cluster
        //std::vector<int> cluster = {row_indices.back()};
        //std::vector<double> cluster_height = {point_ref_height}; 

        //// From Far Point to Close Point
        //for (auto rit = std::next(row_indices.rbegin()); 
                //rit != row_indices.rend(); ++rit)
        //{
            //// Target Point
            //pcl::PointXYZ& point_tgt = rings_[*rit][seg_angle_idx_[*rit][j]];
            //double point_tgt_height =
                    //pcl::pointToPlaneDistance(point_tgt, base_coeff_);
            //double point_tgt_theta = std::atan2(lidar_height_ - point_tgt_height, 
                    //std::sqrt(std::pow(point_tgt.x, 2) + std::pow(point_tgt.y, 2)));

            //// Compute Actual Distance and Estimated Distance
            //double diff_actual = 
                    //std::sqrt(std::pow(point_tgt.x - point_ref.x, 2) + 
                            //std::pow(point_tgt.y - point_ref.y, 2));
            //double diff_inclined = lidar_height_ * 
                    //(1.0 / (std::tan(point_ref_theta) + TANGENT_INCLINE) - 
                     //1.0 / (std::tan(point_tgt_theta) + TANGENT_INCLINE));
            //if (diff_actual < diff_inclined)
            //{
                //// Add Point into Cluster
                //cluster.push_back(*rit);
                //cluster_height.push_back(point_tgt_height);
            //}
            //else
            //{   
                //double cluster_height_avg = std::accumulate(cluster_height.begin(),
                        //cluster_height.end(), 0.0) / cluster_height.size();
                
                //// Assign Cluster as Ground
                //if (cluster_height_avg < HEIGHT_THRESHOLD || cluster.size() == 1)
                //{
                    //for (int i : cluster)
                    //{
                        //ground_[i].push_back(rings_[i][seg_angle_idx_[i][j]]);
                        //landmark_.push_back(rings_[i][seg_angle_idx_[i][j]]);
                    //}
                //}
                //// Re-initialize Cluster
                //cluster = {*rit};
                //cluster_height = {point_tgt_height};
            //}
            //// Switch for next iteration
            //point_ref = point_tgt;
            //point_ref_height = point_tgt_height;
            //point_ref_theta = point_tgt_theta;
        //}
        
        //double cluster_height_avg = std::accumulate(cluster_height.begin(), 
                //cluster_height.end(), 0.0) / cluster_height.size();
        //if (cluster_height_avg < HEIGHT_THRESHOLD || cluster.size() == 1)
        //{
            //for (int i : cluster)
            //{
                //ground_[i].push_back(rings_[i][seg_angle_idx_[i][j]]);
                //landmark_.push_back(rings_[i][seg_angle_idx_[i][j]]);
            //}
        //}
    //} // End of Inter-ring Distance-based Filtering
//}

    //// Calculate Rotation Matrix from LiDAR to Base
    //// Cross Product v = [a b c] x [0 0 1]
    //Eigen::Vector3f v(base_coeff_(1), -base_coeff_(0), 0.0);
    
    //Eigen::Matrix3f skewed_v;
    //skewed_v << 0.0, -v(2), v(1),
                //v(2), 0.0, -v(0),
                //-v(1), v(0), 0.0;

    //// Cosine Value between [a b c] [0 0 1]
    //double c = base_coeff_(2);

    //// Rotation Matrix
    //Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity() +
            //skewed_v + skewed_v * skewed_v / (1 + c);
    
    //// Transformation Matrix
    //Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    //transformation.block<3,3>(0,0) = rotation_matrix;
    //transformation(2,3) = base_coeff_(3) / base_coeff_(2);

    //// Inverse Tranformation Matrix
    //Eigen::Matrix4f inverse_transformation = transformation.inverse();

        //// transform original rings for filtering
        //pcl::PointCloud<pcl::PointXYZ> transformed_ring;
        //pcl::transformPointCloud(rings_[i], transformed_ring, transformation);

        //// Filtering setup & option
        //pass.setInputCloud(transformed_ring.makeShared());
        //pass.setFilterFieldName("z");
        //pass.setFilterLimits(-100, setting_.GROUND_THRESHOLD);

        //// Filter the transformed cloud
        //pcl::PointCloud<pcl::PointXYZ> filtered_transformed_ring;
        //pass.filter(filtered_transformed_ring);

        //// Inverse Transform and Translate the filtered cloud
        //pcl::PointCloud<pcl::PointXYZ> filtered_ring;
        //pcl::transformPointCloud(filtered_transformed_ring, filtered_ring, 
                //inverse_transformation);
                //
    //// Segmenting Point Cloud by horizontal angle
    //// RING_NUMBER x SEGMENT_ANGLE_NUMBER matrix
    //std::array<std::vector<int>, RING_NUMBER> seg_angle_idx;
    //for (int i = 0; i < RING_NUMBER; ++i)
    //{
        //seg_angle_idx[i].resize(SEGMENT_ANGLE_NUMBER, -1);

        //for (int j = 0; j < rings_[i].size(); ++j)
        //{
            //pcl::PointXYZ& point = rings_[i][j];
            //double angle = std::atan2(point.y, point.x);
            //int idx = (angle + M_PI) * SEGMENT_ANGLE_NUMBER / 2 / M_PI;
            //seg_angle_idx[i][idx] = j;
        //}
    //}

    //// Inter-ring distance-based filtering
    //for (int j = 0; j < SEGMENT_ANGLE_NUMBER; ++j)
    //{   
        //int quad_idx = int(4.0 * j / SEGMENT_ANGLE_NUMBER + 0.5) % 4;

        //// Find Ring_ids that consists point in the angle segment
        //// Determine Vertical Angle theta for each point
        //int first_idx = 0;
        //std::vector<int> row_indices;
        //std::vector<double> vertical_angles;
        //for (int i = 0; i < RING_NUMBER; ++i)
        //{
            //// Skip when there is no point in the segment
            //if (seg_angle_idx[i][j] < 0)
            //{
                //continue;
            //}
           
            //if (row_indices.empty())
            //{
                //pcl::PointXYZ& point = rings_[i][seg_angle_idx[i][j]];

                //row_indices.push_back(i);
                //first_idx = i;
                
                //double distance = std::sqrt(std::pow(point.x, 2) + 
                        //std::pow(point.y, 2));
                //double theta = std::atan2(distance, -point.z); 

                //vertical_angles.push_back(theta);
               
                //debugger::debugColorOutput("Ring ID : ", first_idx, 2, BK);
                //debugger::debugColorOutput("Angle theta : ", theta, 2, BB);

                //double ideal_distance = lidar_height_ * std::tan(theta);
                //if (distance > ideal_distance * 0.5)
                //{
                    //landmark_.push_back(point);
                //}
            //}
            //else
            //{
                //row_indices.push_back(i);
                //double theta = vertical_angles[0] + 0.006 * (i - first_idx);
                //vertical_angles.push_back(theta);

                //pcl::PointXYZ& point = rings_[i][seg_angle_idx[i][j]];

                //double distance = std::sqrt(std::pow(point.x, 2) + 
                        //std::pow(point.y, 2));
                //double ideal_distance = lidar_height_ * std::tan(theta);

                //if (distance > ideal_distance)
                //{
                    //landmark_.push_back(point);
                //}
            //}
        //}

        //// Cannot filtering obstacles
        //if (row_indices.size() < 2)
        //{
            //continue;
        //}

        //// Initialize Reference Point
        //int last_idx = row_indices.back();
        //pcl::PointXYZ& point_ref = rings_[last_idx][seg_angle_idx[last_idx][j]];
        //double point_ref_height =
                //pcl::pointToPlaneDistance(point_ref, base_coeff_);
        //double point_ref_theta = vertical_angles.back();
        
        //// Cluster
        //std::vector<int> cluster = {row_indices.back()};
        //std::vector<double> cluster_height = {point_ref_height}; 

        //// From Far Point to Close Point
        //for (int i = row_indices.size() - 2; i >= 0; --i)
        //{
            //// Target Point
            //int ring_id = row_indices[i];

            //pcl::PointXYZ& point_tgt = rings_[ring_id][seg_angle_idx[ring_id][j]];
            //double point_tgt_height =
                    //pcl::pointToPlaneDistance(point_tgt, base_coeff_);
            //double point_tgt_theta = vertical_angles[i];

            //// Compute Actual Distance and Estimated Distance
            //double diff_actual =  
                    //std::sqrt(std::pow(point_tgt.x - point_ref.x, 2) + 
                              //std::pow(point_tgt.y - point_ref.y, 2));
            //double diff_inclined = lidar_height_ * 
                    //(1.0 / (std::tan(point_ref_theta) + TANGENT_INCLINE) - 
                     //1.0 / (std::tan(point_tgt_theta) + TANGENT_INCLINE));
            
            //if (diff_actual < diff_inclined * 0.5)
            //{
                //// Add Point into Cluster
                //cluster.push_back(ring_id);
                //cluster_height.push_back(point_tgt_height);
            //}
            //else
            //{   
                //double cluster_height_avg = std::accumulate(cluster_height.begin(),
                        //cluster_height.end(), 0.0) / cluster_height.size();
                
                //// Assign Cluster as Ground
                //if (cluster_height_avg < HEIGHT_THRESHOLD || cluster.size() == 1)
                //{
                    //for (int i : cluster)
                    //{
                        //pcl::PointXYZ& point = rings_[i][seg_angle_idx[i][j]];

                        //multi_region_[quad_idx][i / 2].push_back(point);
                        //landmark_.push_back(point);
                    //}
                //}
                //// Re-initialize Cluster
                //cluster = {ring_id};
                //cluster_height = {point_tgt_height};
            //}
            //// Switch for next iteration
            //point_ref = point_tgt;
            //point_ref_height = point_tgt_height;
            //point_ref_theta = point_tgt_theta;
        //}
        
        //double cluster_height_avg = std::accumulate(cluster_height.begin(), 
                //cluster_height.end(), 0.0) / cluster_height.size();
        //if (cluster_height_avg < HEIGHT_THRESHOLD || cluster.size() == 1)
        //{
            //for (int i : cluster)
            //{
                //pcl::PointXYZ& point = rings_[i][seg_angle_idx[i][j]];
                
                //multi_region_[quad_idx][i / 2].push_back(point);
                //landmark_.push_back(point);
            //}
        //}
    //} // End of Inter-ring Distance-based Filtering
        //if (ring_size < setting_.LOCAL_WINDOW_SIZE)
        //{
            //continue;
        //}

        //// Sort Point Cloud by Couter Clock-wisely
        //std::sort(rings[i].begin(), rings[i].end(),
            //[](const pcl::PointXYZI& lhs, const pcl::PointXYZI& rhs)
            //{
                //return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            //}
        //);
        
        //// Cacluate Difference of distance from origin
        //std::vector<double> diff(ring_size, 0.0);
        ////double prev_dist = pcl::euclideanDistance(rings[i].back(), origin);
        //double prev_dist = std::sqrt(std::pow(rings[i].back().x, 2) + 
                //std::pow(rings[i].back().y, 2));
        //for (int j = 0; j < ring_size; ++j)
        //{
            ////double curr_dist = pcl::euclideanDistance(rings[i][j], origin);
            //double curr_dist = std::sqrt(std::pow(rings[i][j].x, 2) +
                    //std::pow(rings[i][j].y, 2));
            //diff[j] = std::abs(curr_dist - prev_dist) / curr_dist;
            
            //prev_dist = curr_dist;
        //}
        
        //// Select Candidates for fitting plane by checking noise
        //for (int j = 0; j < ring_size; ++j)
        //{
            //pcl::PointXYZI& p = rings[i][j];

            //if (p.intensity > setting_.INTENSITY_THRESHOLD)
            //{
                //continue;
            //}

            //// Convert to PointXYZL from PointXYZ
            //pcl::PointXYZL point;
            //point.x = p.x;
            //point.y = p.y;
            //point.z = p.z;
            //point.label = i;

            //// Calculate and save Segment
            //double angle = std::atan2(point.y, point.x);
            //int q = int(angle * 4 / M_PI + 4.5) % 8;

            ////double distance = std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2));

            ////int k = 0;
            ////while (k < setting_.section_number_ && distance >= section_distance_[k])
            ////{
                ////k++;
            ////}
            ////k--;
            //multi_region_[q][i/2].push_back(point);

            //// Calculate noise
            //double sum_of_diff = 0.0;
            //for (int w = -setting_.LOCAL_WINDOW_SIZE;
                    //w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            //{
                //sum_of_diff += diff[(j + w + ring_size) % ring_size];
            //}
            
            //// Filtered Cloud
            //if (sum_of_diff < setting_.NOISE_THRESHOLD)
            //{
                //pcl::PointXYZ pp;
                //pp.x = p.x;
                //pp.y = p.y;
                //pp.z = p.z;
                //filtered_region_[q][i/2].push_back(point);
                //b_test_.push_back(pp);
                //if (i < setting_.RING_TO_FIT_BASE)
                //{
                    //candidate_base.push_back(point);
                //}
            //}
        //}
    //} // Finish Sampling Candidates

    //cloud_received_ = true;

    //landmark_.clear();
