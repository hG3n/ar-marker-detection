//
// Created by Hagen Hiller on 2019-02-07.
//

#ifndef MARKER_DETECTOR_H
#define MARKER_DETECTOR_H

#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "marker.h"

class MarkerDetector {

public:

    /**
     * c'tor
     */
    MarkerDetector();

    /**
     * custom c'tor
     */
    MarkerDetector(int pattern_size, int pattern_segements, bool verbose = false);

    /**
     * Function to find markers in a given image
     * @param binarized_image
     * @param active_markers
     */
    void findMarkers(const cv::Mat &binarized_image, std::vector <Marker> *active_markers);


private:
    /**
     * Finds shapes in an image and their respective corners
     */
    void
    findShapeCorners(const cv::Mat &binarized_image, std::vector <std::vector<cv::Point2f>> &filtered_corners) const;

    /**
     * Creates a map containing the current marker layout in binary data
     */
    void createMarkerMap(const cv::Mat &warped_shape, cv::Mat &marker_map, bool use_area_mean = false);

    /**
     * Validates a found marker
     * @param marker_map
     * @return
     */
    int validateMarker(const cv::Mat &marker_map);

    /**
     * Sorts pointlist clockwise starting with the top left element
     * @param point_list
     * @return
     */
    static std::vector <cv::Point2f> sortVertices(std::vector <cv::Point2f> point_list);

    /**
     * Finds the mean centroid of a point list
     * @param point_list
     * @return
     */
    static cv::Point2f findCentroid(const std::vector <cv::Point2f> &point_list);

    /**
     * Converts radians to degrees
     * @param value
     * @return
     */
    static double rad2deg(double value);


    /// private members
    int pattern_size;
    int pattern_segment_size;

    std::vector <cv::Point2f> transform_matrix_;
    bool verbose_;
};


#endif //MARKER_DETECTOR_H
