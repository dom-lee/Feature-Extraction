/*******************************************************************************
 * File:        douglasPeucker.hpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     09/07/2022
 *
 * Description: Ramer-Douglas-Peucker Algorithm that decimates a list of points 
 *              similar curve with fewer points
*******************************************************************************/
#ifndef DOUGLAS_PEUCKER_HPP
#define DOUGLAS_PEUCKER_HPP

// STD Library
#include <vector>
#include <algorithm>
#include <iterator>
// Eigen Library
#include <Eigen/Dense>
#include <Eigen/Geometry>
// PCL
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

// calculate perpendicular distance from point to line
inline double getPointToLineDistance(const Eigen::Vector3f& point,
                                     const Eigen::Vector3f& line_point_1,
                                     const Eigen::Vector3f& line_point_2)
{
    // d = ||PA x PB|| / ||AB||
    Eigen::Vector3f pa = line_point_1 - point;
    Eigen::Vector3f pb = line_point_2 - point;
    Eigen::Vector3f ab = line_point_1 - line_point_2;

    return pa.cross(pb).norm() / ab.norm();
}

// Execute Ramer-Douglas-Peucker Algorithm
template <class PointT>
inline std::vector<PointT> douglasPeucker(
    typename pcl::PointCloud<PointT>::const_iterator points_start_iter,
    typename pcl::PointCloud<PointT>::const_iterator points_end_iter,
    double epsilon)
{
    if (std::distance(points_start_iter, points_end_iter) == 0)
    {
        return {};
    }

    // Find the point with the maximum distance
    double distance_max = 0;
    double index = 0;
    for (auto it = points_start_iter + 1; it != points_end_iter; ++it)
    {
        double distance = getPointToLineDistance((*it).getVector3fMap(),
                                                 (*points_start_iter).getVector3fMap(),
                                                 (*points_end_iter).getVector3fMap());
        if (distance > distance_max)
        {
            distance_max = distance;
            index = std::distance(points_start_iter, it);
        }
    }

    std::vector<PointT> result_points;

    // If distance_max is greater than epsilon, recursively simplify
    if (distance_max > epsilon)
    {
        // Recursive by spliting Line
        auto recursive_result1 = douglasPeucker<PointT>(points_start_iter,
                                                        points_start_iter + index,
                                                        epsilon);
        auto recursive_result2 = douglasPeucker<PointT>(points_start_iter + index,
                                                        points_end_iter,
                                                        epsilon);
        
        // Concatenate Recursive Results
        result_points.insert(result_points.end(), 
                             recursive_result1.begin(),
                             recursive_result1.end());
        result_points.insert(result_points.end(),
                             recursive_result2.begin() + 1,
                             recursive_result2.end());
    }
    else
    {
        result_points = {*points_start_iter, *points_end_iter};
    }
    
    return result_points;
}
#endif /* DOUGLAS_PEUCKER_HPP */ 
