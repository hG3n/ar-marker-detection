#ifndef MARKER_H
#define MARKER_H


#include <opencv2/core.hpp>
#include <ostream>


class Marker
{

public:
    enum Orientation
    {
        UP, DOWN, LEFT, RIGHT
    };

    Marker();

    friend std::ostream &operator<<(std::ostream &os, const Marker &marker);

    Marker(int id,
           const std::vector<cv::Point2f> &corners,
           const cv::Mat &marker_map);


    int getId() const;

    cv::Point2f getCentroid() const;

    cv::Mat getMarkerMap() const;


private:
    std::vector<cv::Point2f> corners_;
    cv::Point2f centroid_;
    cv::Mat marker_map_;
    int marker_id_;


};


#endif //MARKER_H
