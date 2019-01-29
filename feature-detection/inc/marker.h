#ifndef MARKER_H
#define MARKER_H

#include <vector>
#include <opencv2/core.hpp>

class Marker
{
    public:
        Marker(const cv::Mat& image);
        int decodeId();

    private:
        cv::Mat image_;

        // borders ordered clockwise starting with top left
        std::vector<cv::Point2f> borders_;
};

#endif // MARKER_H
