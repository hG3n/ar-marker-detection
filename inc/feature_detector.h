#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <opencv2/core.hpp>

class FeatureDetector
{
    public:
        FeatureDetector();
        void detectFeatures(const cv::Mat &image, std::vector<cv::Point2f> &features);

    private:
        int max_corners_;
        double quality_level_;
        double min_distance_;
        int block_size_;
        int gradient_size_;
        bool use_harris_detector_;
        double k_;
};

#endif // FEATURE_DETECTOR_H
