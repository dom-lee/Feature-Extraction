#ifndef FEATURE_DETECTOR_SETTING_H
#define FEATURE_DETECTOR_SETTING_H

typedef struct feature_detector_setting
{
    int    RING_TO_ANALYZE;
    int    RING_TO_FIT_BASE;
    double FIT_PLANE_THRESHOLD;
    int    LOCAL_WINDOW_SIZE;
    double NOISE_THRESHOLD;
    double ANGLE_BUFFER;
    double DIST_DIFF_THRESHOLD;
    double ANGLE_DIFF_THRESHOLD;
    double HEIGHT_DIFF_THRESHOLD;
    double GROUND_THRESHOLD;
    double CURB_HEIGHT;
    double ANGULAR_RESOLUTION;
    double ANGLE_CURB_THRESHOLD;
    double DISCONTINUITY;
    int    INTENSITY_THRESHOLD;
} feature_detector_setting_t;
#endif /* FEATURE_DETECTOR_SETTING_H */
