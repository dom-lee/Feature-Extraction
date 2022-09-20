/*******************************************************************************
 * File:        estimatePlaneRANSAC.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     09/10/2022
 *
 * Description: Estimate Plane Coefficient with RANSAC Method
*******************************************************************************/
#ifndef ESTIMATE_PLANE_RANSAC_HPP
#define ESTIMATE_PLANE_RANSAC_HPP

// Eigen
#include <Eigen/Dense>

// PCL
#include <pcl/point_cloud.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/sac_model_plane.h>

// Utils
#include "utils/debugger.h"

using namespace bipedlab;

template <class PointT>
Eigen::Vector4f estimatePlaneRANSAC(const pcl::PointCloud<PointT>& cloud,
                                    double plane_threshold,
                                    pcl::PointIndices& inliers)
{
    Eigen::Vector4f plane_coeff;

    if (cloud.size() < 3)
    {
        debugger::debugColorTextOutput("Not enough Data for RANSAC", 5, BY);
        return plane_coeff;
    }

    // Plane Model segmentation with RANSAC
    // https://pcl.readthedocs.io/en/latest/planar_segmentation.html
    pcl::ModelCoefficients plane_model_coeff; 

    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg;

    // Optional
    seg.setOptimizeCoefficients(false);

    // Mandatory
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(plane_threshold);
    
    // Segment Planar Model
    seg.setInputCloud(cloud.makeShared());
    seg.segment(inliers, plane_model_coeff);

    if (inliers.indices.empty())
    {
        PCL_ERROR("Could not estimate a planar model for the given dataset.\n");
    }

    plane_coeff << plane_model_coeff.values[0], plane_model_coeff.values[1],
                   plane_model_coeff.values[2], plane_model_coeff.values[3];

    // Optimization with suprressing Warning
    if (inliers.indices.size() > 4)
    {
        typename pcl::SampleConsensusModel<PointT>::Ptr model = seg.getModel();

        Eigen::VectorXf refined_plane_coeff; 
        model->optimizeModelCoefficients(inliers.indices,
                                         plane_coeff,
                                         refined_plane_coeff);
        plane_coeff = refined_plane_coeff.head(4);

        model->selectWithinDistance(plane_coeff, plane_threshold, inliers.indices);
    }

    return plane_coeff;
}
#endif /* ESTIMATE_PLANE_RANSAC_HPP */
