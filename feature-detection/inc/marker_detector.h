//
// Created by Hagen Hiller on 2019-02-07.
//

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "marker.h"

class MarkerDetector
{

public:
    
    MarkerDetector();

    MarkerDetector(int pattern_size, int pattern_segements, bool verbose = false);

    void findMarkers(const cv::Mat &binarized_image, std::vector<Marker> *active_markers);


private:
    void findShapeCorners(const cv::Mat &binarized_image, std::vector<std::vector<cv::Point2f>> &filtered_corners);


    void createMarkerMap(const cv::Mat &warped_shape, cv::Mat &marker_map, bool use_area_mean = false);

    int validateMarker(const cv::Mat &marker_map);

    std::vector<cv::Point2f> sortVertices(std::vector<cv::Point2f> point_list) const;


    /// member
    int pattern_size;
    int pattern_segment_size;

    std::vector<cv::Point2f> transform_matrix_;
    bool verbose_;
};


#endif //MARKER_DETECTOR_H
