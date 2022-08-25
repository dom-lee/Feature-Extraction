#ifndef FEATURE_EXTRACTOR_SETTING_H
#define FEATURE_EXTRACTOR_SETTING_H

typedef struct feature_extractor_setting
{
    // Parameters for Base Plane Estimation
    int    RING_TO_FIT_BASE;
    int    SMOOTH_WINDOW_SIZE;
    double SMOOTH_THRESHOLD;

    // Parameters for Plane RANSAC
    double FIT_PLANE_THRESHOLD;

    // Parameters for Ground Extraction
    double GRID_LENGTH;
    int    GRID_RANGE;
    double DISCONTINUITY_HEIGHT;
    double OBSTACLE_THRESHOLD;
    double GROUND_THRESHOLD;
    int    GRASS_COUNT_THRESHOLD;
    double GRASS_RATIO_THRESHOLD;

    // Parameters for Road Modeling
    int    BEAM_SECTION_NUMBER;
    double ROAD_VIEW_RANGE;
    double ROAD_WIDTH_THRESHOLD;
    int    GROUND_COUNT_THRESHOLD;

    // Parameters for Curb Extraction
    double DISCONTINUITY_RATIO;
    double DISCONTINUITY_AZIMUTH;
    double ANGULAR_RESOLUTION;
    double CONTINUITY_ANGLE;
    double CONTINUITY_DISTANCE;
    double SMOOTHNESS_THRESHOLD;
    int    SMOOTH_COUNT;
    double SMOOTH_DIST_THRESHOLD;
    double CURB_HEIGHT_THRESHOLD;
    double CURB_ANGLE_THRESHOLD;
    double SIDEWALK_LENGTH;


    int    RING_TO_ANALYZE;
    double DISTANCE_TO_ANALYZE;
    double GRID_RESOLUTION;
    double NOT_OBSTACLE_THRESHOLD;


    // Parameter for Filtering Obstacles
    double BASE_BUFFER;
    double ANGLE_BUFFER;
    
    // Parameter for Estimate Ground
    double ANGLE_DIFF_THRESHOLD;
    double HEIGHT_DIFF_THRESHOLD;
    double GROUND_DISCONTINUITY;
    int    CONTINUED_NUMBER;

    // Parameter for Road Model Estimation

    double DISCONTINUITY_DISTANCE;

    double SMOOTH_THRESHOLD_GRASS;
    double CURB_HEIGHT;
    int    CURB_WINDOW_SIZE;
    int    DISCONTINUITY;
} feature_extractor_setting_t;
#endif /* FEATURE_EXTRACTOR_SETTING_H */
