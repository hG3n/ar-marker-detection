//
// Created by Hagen Hiller on 2019-02-07.
//

#include "marker.h"

#include "misc.h"

Marker::Marker()
        : corners_()
        , centroid_()
        , marker_map_()
        , marker_id_()
{}

Marker::Marker(int id,
               const std::vector<cv::Point2f> &corners,
               const cv::Mat &marker_map)
        : corners_(corners)
        , centroid_()
        , marker_map_(marker_map)
        , marker_id_(id)
{}

int Marker::getId() const
{
    return marker_id_;
}

cv::Point2f Marker::getCentroid() const
{
    return Misc::findCentroid(corners_);
};

cv::Mat Marker::getMarkerMap() const
{
    return marker_map_;
}


std::vector<cv::Point2f> Marker::getCorners() const
{
    return corners_;
}

std::ostream &operator<<(std::ostream &os, const Marker &marker)
{
    os << "Marker: " << "marker_id_: " << marker.marker_id_ << " corners_: " << marker.corners_ << " centroid_: "
       << marker.centroid_;
    return os;
}




