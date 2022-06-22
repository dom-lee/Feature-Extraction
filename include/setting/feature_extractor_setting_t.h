#ifndef FEATURE_EXTRACTOR_SETTING_H
#define FEATURE_EXTRACTOR_SETTING_H

typedef struct feature_extractor_setting
{
    int    RING_TO_ANALYZE;
    int    RING_TO_FIT_BASE;
    double FIT_PLANE_THRESHOLD;
    int    LOCAL_WINDOW_SIZE;
    double ALIGNEDNESS_THRESHOLD;
    double ANGLE_BUFFER;
    double DIST_DIFF_THRESHOLD;
    double ANGLE_DIFF_THRESHOLD;
    double HEIGHT_DIFF_THRESHOLD;
    double GROUND_THRESHOLD;
    double ANGULAR_RESOLUTION;
    double GROUND_DISCONTINUITY;
    int    CONTINUED_NUMBER;
    double CURB_HEIGHT;
    double DISCONTINUITY;
    double ANGLE_CURB_THRESHOLD;
    double HEIGHT_CURB_THRESHOLD;
} feature_extractor_setting_t;
#endif /* FEATURE_EXTRACTOR_SETTING_H */
