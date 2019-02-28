#include <iostream>

#include "feature_detector.h"
#include <opencv2/imgproc.hpp>

FeatureDetector::FeatureDetector()
    : max_corners_(23)
    , quality_level_(0.01)
    , min_distance_(10)
    , block_size_(3)
    , gradient_size_(3)
    , use_harris_detector_(false)
    , k_(0.04)
{}

void FeatureDetector::detectFeatures(const cv::Mat &image, std::vector<cv::Point2f> &features)
{;
    std::vector<cv::Point2f> corners;
    cv::Mat copy = image.clone();
    cv::goodFeaturesToTrack(image,
                            corners,
                            max_corners_,
                            quality_level_,
                            min_distance_,
                            cv::Mat(),
                            block_size_,
                            gradient_size_,
                            use_harris_detector_,
                            k_);
    std::cout << "** Number of corners detected: " << corners.size() << std::endl;

    features = corners;
}
