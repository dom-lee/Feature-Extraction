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

FeatureDetector::FeatureDetector()
{
    debugger::debugColorTextOutput("Constructing FeatureDetector", 10, BC);
   
    lidar_height_ = 1.5;
    
    cloud_received_ = false;
}

void FeatureDetector::setInputCloud(feature_detector_setting_t setting,
        std::array<pcl::PointCloud<pcl::PointXYZI>, RING_NUMBER>& rings)
{
    setting_ = setting;

    // Resize & Clear
    for (int q = 0; q < 8; ++q)
    {
        multi_region_[q].resize(setting_.SECTION_NUMBER);
        filtered_region_[q].resize(setting_.SECTION_NUMBER);
        for (int k = 0; k < setting_.SECTION_NUMBER; ++k)
        {
            multi_region_[q][k].clear();
            filtered_region_[q][k].clear();
        }
    }
    a_test_.clear();
    b_test_.clear();

    // Lidar Origin
    pcl::PointXYZ origin(0, 0, 0);

    // Section Divider
    section_distance_.resize(setting_.SECTION_NUMBER, 0.0);
    //double theta = setting_.SECTION_START_ANGLE;
    //double distance = lidar_height_ * std::tan(theta);
    for (int k = 1; k < setting_.SECTION_NUMBER; ++k)
    {
        section_distance_[k] = setting_.SECTION_DISTANCE * k;
        //setting_.SECTION_DISTANCE_[k] = lidar_height_ * std::tan(theta);
        //theta += (M_PI / 2 - setting_.SECTION_START_ANGLE) / setting_.SECTION_NUMBER;
        debugger::debugColorOutput("Section Distance (" +
                std::to_string(k + 1) + "/" + std::to_string(setting_.SECTION_NUMBER) +
                ") : ", section_distance_[k], 1, BC);
    }

    // Sampling candidate points to fit base planar model
    pcl::PointCloud<pcl::PointXYZL> candidate_base;
    for (int i = 0; i < setting_.RING_TO_ANALYZE; ++i)
    {
        int ring_size = rings[i].size();
        debugger::debugColorOutput("# of points in ring" + 
                std::to_string(i) + " : ", rings_size, 1, BK);

        if (ring_size < setting_.LOCAL_WINDOW_SIZE
        {
            continue;
        }

        // Sort Point Cloud by Couter Clock-wisely
        std::sort(rings[i].begin(), rings[i].end(),
            [](const pcl::PointXYZ& lhs, const pcl::PointXYZ& rhs)
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
            diff[j] = (curr_dist - prev_dist) / curr_dist;
            
            prev_dist = curr_dist;
        }
        
        // Select Candidates for fitting plane by checking noise
        for (int j = 0; j < ring_size; ++j)
        {
            pcl::PointXYZI& p = rings[i][j];

            // Convert to PointXYZL from PointXYZ
            pcl::PointXYZIL point;
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.label = i;

            // Calculate and save Segment
            double angle = std::atan2(point.y, point.x);
            int q = int(angle * 4 / M_PI + 4.5) % 8;

            double distance = std::sqrt(std::pow(p.x, 2) + std::pow(p.y, 2));

            int k = 0;
            while (k < setting_.SECTION_NUMBER && distance >= section_distance_[k])
            {
                k++;
            }
            k--;
            multi_region_[q][k].push_back(point);

            // Calculate noise
            double sum_of_diff = 0.0;
            for (int w = -setting_.LOCAL_WINDOW_SIZE;
                    w <= setting_.LOCAL_WINDOW_SIZE; ++w)
            n{
                sum_of_diff += diff[(j + w + ring_size) % ring_size];
            }
            
            // Filtered Cloud
            if (sum_of_diff < setting_.NOISE_THRESHOLD)
            {
                if (i < setting_.RING_TO_FIT_BASE)
                {
                    candidate_base.push_back(point);
                }
                filtered_region_[q][k].push_back(point);
                b_test_.push_back(p);
            }
        }
    } // Finish Sampling Candidates

    if (candidate_base.size() < 3)
    {
        PCL_ERROR("Not enough data to estimate a base planar model.\n");
        cloud_received_ = false;

        return;
    }
    
    // Set Base Plane
    base_coeff_ = estimatePlane_(candidate_base);
    lidar_height_ = pcl::pointToPlaneDistance(origin, base_coeff_.values[0],
            base_coeff_.values[1], base_coeff_.values[2], base_coeff_.values[3]);
   
    // Update Base Plane Coefficient
    //base_coeff_.values[3] -= translation(2,3);
    debugger::debugColorOutput("Base Coefficients \n", base_coeff_, 2, BG); 
    cloud_received_ = true;

    landmark_.clear();
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
    seg.setOptimizeCoefficients(true); // To turn off waning message
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

    // Estimate Planar Model for Multi-Region
    std::array<std::pair<double, double>, 8> section_direction =
            { { {-1, 0}, {-0.7071, -0.7071}, {0, -1}, {0.7071, -0.7071},
                {1, 0}, {0.7071, 0.7071}, {0, 1}, {-0.7071, 0.7071} } };
    std::array<std::vector<pcl::ModelCoefficients>, 8> multi_region_plane_coeff;
    for (int q = 0; q < 8; ++q)
    {
        multi_region_plane_coeff[q].resize(setting_.SECTION_NUMBER);

        // Estimate Plane Coefficients for closer one
        for (int k = 0; k < setting_.SECTION_NUMBER; ++k)
        {
            multi_region_plane_coeff[q][k] = 
                    estimatePlane_(filtered_region_[q][k]);

            debugger::debugColorOutput("# of Points (q: " + 
                    std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                    filtered_region_[q][k].size(), 2, BB);

            debugger::debugColorOutput("Plane Coeff (q: " + 
                    std::to_string(q) + ", k: " + std::to_string(k) + ")\n", 
                    multi_region_plane_coeff[q][k], 2, BB);

            if (multi_region_plane_coeff[q][k].values.empty())
            {
                if (k == 0)
                {
                    multi_region_plane_coeff[q][k] = base_coeff_;
                }
                else
                {
                    multi_region_plane_coeff[q][k] = 
                            multi_region_plane_coeff[q][k - 1];
                }
            }
            // Compute Angle Difference with two plane model
            else
            {                
                pcl::ModelCoefficients& coeff_prev = (k > 0) ? 
                        multi_region_plane_coeff[q][k - 1] : base_coeff_;

                double angle = computeAngleTwoPlane_(coeff_prev,
                        multi_region_plane_coeff[q][k]);
                angle = std::min(angle, M_PI - angle);

                double height_diff = computeHeightDiffTwoPlane_(
                        section_distance_[k], section_direction[q], 
                        coeff_prev, multi_region_plane_coeff[q][k]);
                
                if (std::abs(M_PI / 2 - angle) < setting_.ANGLE_PLANE_THRESHOLD)
                {
                    removeInliner_(filtered_region_[q][k], 
                            multi_region_plane_coeff[q][k]);
                    k--;
                }
                else if (angle > setting_.ANGLE_PLANE_THRESHOLD || 
                        height_diff > setting_.HEIGHT_DIFF_THRESHOLD)
                {
                    debugger::debugColorOutput("Angle between region (q: " + 
                            std::to_string(q) + ", k: " + std::to_string(k) +
                            ") : ", angle, 2, BM);
                    
                    debugger::debugColorOutput("Distance between region (q: " + 
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
        for (int k = 0; k < setting_.SECTION_NUMBER; ++k)
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
                pcl::PointWithRange point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.range = a * p.x + b * p.y + c * p.z + d;

                ground_[p.label].push_back(point);
            }
        }
    }
}

void FeatureDetector::extractCurb_()
{
    // Center of Ground
    pcl::PointXYZ center(0, 0, -lidar_height_);

    for (int i = 0; i < ground_.size(); ++i)
    {
        // Sort Point Cloud by Couter Clock-wisely
        std::sort(ground_[i].begin(), ground_[i].end(),
            [](const pcl::PointWithRange& lhs, const pcl::PointWithRange& rhs)
            {
                return std::atan2(lhs.y, lhs.x) < std::atan2(rhs.y, rhs.x);
            }
        );

        int ground_size = ground_[i].size();
        //for (int j = 0; j < ground_size; ++j)
        for (int j = 0; j < ground_size; ++j)
        {
            pcl::PointWithRange& point = ground_[i][j];

            double theta = std::atan2(std::abs(point.z), 
                    std::sqrt(std::pow(point.x, 2) + std::pow(point.y, 2)));
            double delta_xy = pcl::euclideanDistance(center, point) * 
                    setting_.ANGULAR_RESOLUTION;
            double delta_z = delta_xy * std::sin(theta);
            int n_v = setting_.CURB_HEIGHT / std::sin(theta) / delta_xy;

            // Continuity
            int idx_prev = (j - 1 + ground_size) % ground_size;
            int idx_next = (j + 1 + ground_size) % ground_size;
            pcl::PointWithRange& point_prev = ground_[i][idx_prev];
            pcl::PointWithRange& point_next = ground_[i][idx_next];
            
            double dist_xy = std::sqrt(std::pow(point_prev.x - point_next.x, 2) +
                    std::pow(point_prev.y - point_next.y, 2));
            double dist_z = std::abs(point_prev.z - point_next.z);

            //// Check Horizonral and Vertical Continuity
            //if (dist_xy < delta_xy || dist_z < delta_z)
            //{
                //continue;
            //}
            
            bool continuity = true;
            bool elevation_a = true;
            bool elevation_b = true;
            Eigen::Vector2f v_a(0, 0);
            Eigen::Vector2f v_b(0, 0);
            double max_height_diff = 0.0;

            for (int w = 1; w < n_v + 1 && continuity; ++w)
            {
                int idx_a = (j + w + ground_size) % ground_size;
                int idx_a_prev = (j + w - 1 + ground_size) % ground_size;
                pcl::PointWithRange& point_a = ground_[i][idx_a];
                pcl::PointWithRange& point_a_prev = ground_[i][idx_a_prev];

                int idx_b = (j - w + ground_size) % ground_size;
                int idx_b_prev = (j - w + 1 + ground_size) % ground_size;
                pcl::PointWithRange& point_b = ground_[i][idx_b];
                pcl::PointWithRange& point_b_prev = ground_[i][idx_b_prev];
                
                double dist_a = pcl::euclideanDistance(point_a, point_a_prev);
                double dist_b = pcl::euclideanDistance(point_b, point_b_prev);
                
                // Check setting_.DISCONTINUITY
                if (dist_a > setting_.DISCONTINUITY * delta_xy || 
                        dist_b > setting_.DISCONTINUITY * delta_xy)
                {
                    continuity = false;
                }

                // Check elevation
                if (point_a.range < point_a_prev.range)
                {
                    elevation_a = false;
                }

                if (point_b.range < point_b_prev.range)
                {
                    elevation_b = false;
                }
                
                // Calculating two vector
                v_a(0) += point_a.x - point.x;
                v_a(1) += point_a.y - point.y;

                v_b(0) += point_b.x - point.x;
                v_b(1) += point_b.y - point.y;

                double height_diff = std::max(
                        std::abs(point_a.range - point.range),
                        std::abs(point_b.range - point.range));
                double max_height_diff = std::max(max_height_diff, height_diff);
            }
            
            // setting_.DISCONTINUITY Threshold 
            if (!continuity)
            {
                continue;
            }

            // average
            v_a /= n_v;
            v_b /= n_v;
            double angle = std::acos(v_a.dot(v_b) / v_a.norm() / v_b.norm());

            // Angle Threshold
            if (angle > setting_.ANGLE_CURB_THRESHOLD)
            {
                continue;
            }

            //// Elevation Check
            //if (!elevation_a && !elevation_b)
            //{
                //continue;
            //}

            pcl::PointXYZ a;
            a.x = point.x;
            a.y = point.y;
            a.z = point.z;

            a_test_.push_back(a);

            //debugger::debugColorOutput("point:", point, 3, BK);
            //debugger::debugColorOutput("n_v:", n_v, 3, BW);
            //debugger::debugColorOutput("angle: ", angle, 3, BB);
            //debugger::debugColorOutput("Diff: ", max_height_diff, 3, BC);
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

double FeatureDetector::computeHeightDiffTwoPlane_(const double distance,
        const std::pair<double, double> section_direction,
        const pcl::ModelCoefficients& coeff1, 
        const pcl::ModelCoefficients& coeff2)
{
    double a1 = coeff1.values[0];
    double b1 = coeff1.values[1];
    double c1 = coeff1.values[2];
    double d1 = coeff1.values[3];

    double a2 = coeff2.values[0];
    double b2 = coeff2.values[1];
    double c2 = coeff2.values[2];
    double d2 = coeff2.values[3];

    double x = section_direction.first * distance;
    double y = section_direction.second * distance;

    double dist1 = std::abs(a1 * x + b1 * y + d1) / 
            std::sqrt(std::pow(a1, 2) + std::pow(b1, 2) + std::pow(c1, 2));
    double dist2 = std::abs(a2 * x + b2 * y + d2) / 
            std::sqrt(std::pow(a2, 2) + std::pow(b2, 2) + std::pow(c2, 2));

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
    //std::vector<double> region_distance(setting_.SECTION_NUMBER - 1);
    //for (int i = 0; i < region_distance.size(); ++i)
    //{
        //double theta_i = setting_.SECTION_START_ANGLE + 
                //(90 - setting_.SECTION_START_ANGLE) / (setting_.SECTION_NUMBER - 1) * i;
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
        //multi_region_plane_coeff[i].resize(setting_.SECTION_NUMBER);

        //// Find initial region
        //int k = 0;
        //while (multi_region_[i][k].size() < 3)
        //{
            //multi_region_[i][k + 1] += multi_region_[i][k];
            //k++;
        //}
        //multi_region_plane_coeff[i][k] = estimatePlane(multi_region_[i][k]);
        //saveInlinerToGround(multi_region_[i][k], multi_region_plane_coeff[i][k]);

        //for (k++; k < setting_.SECTION_NUMBER; ++k)
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
