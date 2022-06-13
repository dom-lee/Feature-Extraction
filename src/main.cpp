/*******************************************************************************
 * File:        main.cpp
 *
 * Author:      Dongmyeong Lee (domlee[at]umich.edu)
 * Created:     05/30/2022
 *
 * Description: Run main ROS node of feature detector
*******************************************************************************/
#include <ros/ros.h>

#include "driver/driver.h"

int DEBUG_LEVEL = 10;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "feature_detection");
    ros::NodeHandle nh("~");

    Driver driver(nh);
    ros::spin();
    
    return 0;
}
