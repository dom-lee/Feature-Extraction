#ifndef FEATURE_EXTRACTOR_SETTING_H
#define FEATURE_EXTRACTOR_SETTING_H

typedef struct feature_extractor_setting
{
    int    RING_TO_ANALYZE;
    double FIT_PLANE_THRESHOLD;
    double ANGULAR_RESOLUTION;

    // Parameter for Filtering Obstacles
    int    RING_TO_FIT_BASE;
    double BASE_BUFFER;
    double ANGLE_BUFFER;
    
    // Parameter for Estimate Ground
    double ANGLE_DIFF_THRESHOLD;
    double HEIGHT_DIFF_THRESHOLD;
    double GROUND_THRESHOLD;
    double GROUND_DISCONTINUITY;
    int    CONTINUED_NUMBER;

    // Parameter for Road Model Estimation
    int    BEAM_SECTION_NUMBER;
    double ROAD_VIEW_RANGE;

    // Parameter for Curb Extraction
    int    SMOOTH_WINDOW_SIZE;
    double SMOOTH_THRESHOLD_GRASS;
    double SMOOTH_THRESHOLD_PLANE;
    double CURB_HEIGHT;
    int    CURB_WINDOW_SIZE;
    double CURB_HEIGHT_THRESHOLD;
    double CURB_ANGLE_THRESHOLD;
    int    DISCONTINUITY;
} feature_extractor_setting_t;
#endif /* FEATURE_EXTRACTOR_SETTING_H */
