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

    // Parameters for Curb Extraction
    double DISCONTINUITY_AZIMUTH;
    double DISCONTINUITY_DISTANCE;
    double SMOOTHNESS_THRESHOLD;
    int    SMOOTH_COUNT;
    double CONTINUITY_ANGLE;
    double CURB_HEIGHT_THRESHOLD;
    double CURB_ANGLE_THRESHOLD;
    double SIDEWALK_LENGTH;

    // Parameters for Road Modeling
    int    BEAM_SECTION_NUMBER;
    double ROAD_VIEW_RANGE;
    double ROAD_WIDTH_THRESHOLD;
    int    GROUND_COUNT_THRESHOLD;

    // Parameters for Debug
    int    RING_TO_ANALYZE;
} feature_extractor_setting_t;
#endif /* FEATURE_EXTRACTOR_SETTING_H */
