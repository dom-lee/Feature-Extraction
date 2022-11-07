#ifndef FEATURE_EXTRACTOR_SETTING_H
#define FEATURE_EXTRACTOR_SETTING_H

typedef struct feature_extractor_setting
{
    // Parameters for filtering raw Point Cloud
    double HEIGHT_THRESHOLD;

    // Parameters for Grid
    double GRID_LENGTH;
    int    GRID_RANGE;

    // Parameters for Base Plane Estimation
    int    RING_TO_FIT_BASE;
    double GRADIENT_THRESHOLD;
    double BASE_FIT_THRESHOLD;

    // Parameters for Fitting Lines
    double AZIMUTH_RESOLUTION;
    double DISCONTINUITY;
    double EPSILON;

    // Parameters for Filtering Ground Lines
    double GROUND_DIST_THRESHOLD;
    double GROUND_ANGLE_THRESHOLD;

    // Parameters for Wall Extraction
    double INTERPOLATION_LENGTH;
    double SECTION_NUMBER;
    double WALL_HEIGHT_THRESHOLD;
    double WALL_FIT_THRESHOLD;
    double CLUSTER_ANGLE_THRESHOLD;
    double CLUSTER_DIST_THRESHOLD;

    // Parameters for Glass Detection
    int    GLASS_INTENSITY;
    int    GLASS_INTENSITY_DELTA;


    // Parameters for Ground Extraction
    double DISCONTINUITY_HEIGHT;
    double OBSTACLE_THRESHOLD;
    double GROUND_THRESHOLD;

    // Parameters for Curb Extraction
    double DISCONTINUITY_AZIMUTH;
    double CURB_HEIGHT_THRESHOLD;
    double CURB_ANGLE_THRESHOLD;
    double SIDEWALK_MIN_LENGTH;
    double SIDEWALK_MAX_LENGTH;

    // Parameters for Road Modeling
    int    BEAM_SECTION_NUMBER;
    double ROAD_VIEW_RANGE;
    double ROAD_WIDTH_THRESHOLD;
    int    GROUND_COUNT_THRESHOLD;

    // Parameters for Debug
    int    RING_TO_ANALYZE;
} feature_extractor_setting_t;
#endif /* FEATURE_EXTRACTOR_SETTING_H */
