#ifndef LIDAR_SETTING_H
#define LIDAR_SETTING_H

#include <vector>

typedef struct lidar_setting
{
    int ring_number;
    std::vector<double> elevation_angles;
} lidar_setting_t;
#endif /* LIDAR_SETTING_H */
