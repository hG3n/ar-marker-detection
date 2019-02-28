//
// Created by Hagen Hiller on 2019-02-07.
//

#include "marker_detector.h"

#include <iostream>
#include <opencv2/imgproc.hpp>

#include "misc.h"
#include "stopwatch.h"

MarkerDetector::MarkerDetector()
{}

MarkerDetector::MarkerDetector(int pattern_size, int pattern_segments, bool verbose)
        : pattern_size(pattern_size)
        , pattern_segment_size(pattern_size / pattern_segments)
        , transform_matrix_()
        , verbose_(verbose)
{

    // create perspective transform target
    /*
     * 0 -- 1
     * |    |
     * 3 -- 2
     */
    transform_matrix_.push_back(cv::Point2f(0, 0));
    transform_matrix_.push_back(cv::Point2f(0, pattern_size));
    transform_matrix_.push_back(cv::Point2f(pattern_size, pattern_size));
    transform_matrix_.push_back(cv::Point2f(pattern_size, 0));
}


void MarkerDetector::findMarkers(const cv::Mat &binarized_image, std::vector<Marker> *active_markers)
{
    // find shapes
    auto find_shapes_start = Stopwatch::getStart();
    std::vector<std::vector<cv::Point2f>> shapes;
    findShapeCorners(binarized_image, shapes);
    std::cout  << "Time to find corners: " << Stopwatch::getElapsed(find_shapes_start);


    auto process_shapes_start = Stopwatch::getStart();
    for (size_t poly_idx = 0; poly_idx < shapes.size(); ++poly_idx)
    {

        // sort corner points clockwise starting with tl point
        std::vector<cv::Point2f> corner_points = shapes[poly_idx];
        auto corner_points_sorted = sortVertices(corner_points);

        // warp shape
        cv::Mat warped_shape(cv::Size(pattern_size, pattern_size), CV_32FC1);

//        cv::Mat cpy = binarized_image.clone();
//        cv::cvtColor(cpy, cpy, CV_GRAY2BGR);
//        std::vector<cv::Scalar> corner_colors;

        // B , G , R
//        corner_colors.push_back({255, 0, 255});
//        corner_colors.push_back({255, 0, 0});
//        corner_colors.push_back({0, 255, 0});
//        corner_colors.push_back({127, 127, 127});
//        for (auto e: corner_points_sorted)
//            for (size_t i = 0; i < corner_points_sorted.size(); ++i)
//                cv::circle(cpy, corner_points_sorted[i], 2, corner_colors[i], CV_FILLED);

//        Misc::showImage(cpy);
        cv::Mat p_transform = cv::getPerspectiveTransform(corner_points_sorted, transform_matrix_);

        cv::warpPerspective(binarized_image, warped_shape, p_transform, cv::Size(pattern_size, pattern_size));

        // generate marker map
        cv::Mat marker_map = cv::Mat::zeros(pattern_segment_size, pattern_segment_size, CV_8UC1);
        createMarkerMap(warped_shape, marker_map);

        // validate whether the detected shape is a trackable marker
        int marker_id = validateMarker(marker_map);
        if (marker_id > -1)
        {
            auto m = Marker(marker_id, corner_points, marker_map);
            active_markers->push_back(m);
        } else
        {
            std::cout << "" << std::endl;
        }
    }

}


void MarkerDetector::findShapeCorners(const cv::Mat &binarized_image,
                                      std::vector<std::vector<cv::Point2f>> &filtered_corners)
{
    std::vector<std::vector<cv::Point>> contours_src;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binarized_image, contours_src, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // create Polygons
    std::vector<std::vector<cv::Point2f>> contour_polys;
    contour_polys.resize(contours_src.size());
    for (size_t i = 0; i < contours_src.size(); i++)
        cv::approxPolyDP(cv::Mat(contours_src[i]), contour_polys[i], cv::LINE_8, true);

    // Filter for contours with only 4 points
    for (size_t i = 0; i < contour_polys.size(); ++i)
        if (contour_polys[i].size() == 4)
            filtered_corners.push_back(contour_polys[i]);
}

void MarkerDetector::createMarkerMap(const cv::Mat &warped_shape, cv::Mat &marker_map, bool use_area_mean)
{
    // calculate marker dimensions
    int segment_center = pattern_segment_size / 2 + 1;

    for (int r = 0; r < pattern_segment_size; ++r)
    {
        for (int c = 0; c < pattern_segment_size; ++c)
        {
            if (use_area_mean)
            {
                // area rect
                cv::Point tl(r * pattern_segment_size, c * pattern_segment_size);
                cv::Point br((r + 1) * pattern_segment_size - 1, (c + 1) * pattern_segment_size - 1);
                auto roi = cv::Mat(warped_shape, {tl, br});
                auto mean = cv::mean(roi)[0]; // get channel 1 value
                if (mean > 200)
                    marker_map.at<uchar>(r, c) = static_cast<uchar>(1);
            } else
            {
                int current_row = r * pattern_segment_size + segment_center;
                int current_col = c * pattern_segment_size + segment_center;
                uchar value = warped_shape.at<uchar>(current_row, current_col);
                if (static_cast<int>(value) > 200)
                    marker_map.at<uchar>(r, c) = static_cast<uchar>(1);
            }
        }
    }
}

int MarkerDetector::validateMarker(const cv::Mat &marker_map)
{

//        auto marker_id_start = Stopwatch::getStart();
    /// marker outer ring
    auto upper_row_sum = cv::sum(marker_map.row(0))[0];
    auto lower_row_sum = cv::sum(marker_map.row(marker_map.rows - 1))[0];
    auto leftmost_col_sum = cv::sum(marker_map.col(0))[0];
    auto rightmost_col_sum = cv::sum(marker_map.col(marker_map.cols - 1))[0];

    if (upper_row_sum != 0 || lower_row_sum != 0 || leftmost_col_sum != 0 || rightmost_col_sum != 0)
    {
        std::cout << "(!) The detected shape does not seem to be a marker." << std::endl;
        std::cout << "  (i) No outer ring was be detected." << std::endl;
        std::cout << std::endl;
        return -1;
    }

    /// marker outter positional dots
    std::vector<cv::Point2i> positional_dots;
    positional_dots.emplace_back(cv::Point2i(1, 1));
    positional_dots.emplace_back(cv::Point2i(7, 1));
    positional_dots.emplace_back(cv::Point2i(7, 7));
    positional_dots.emplace_back(cv::Point2i(1, 7));

    int direction_idx = -1;
    for (size_t i = 0; i < positional_dots.size(); ++i)
        if (static_cast<int>(marker_map.at<uchar>(positional_dots[i])) == 0)
            direction_idx = static_cast<int>(i);

    Marker::Orientation orientation;
    if (direction_idx == -1)
    {
        std::cout << "(!) The detected shape does not seem to be a marker." << std::endl;
        std::cout << "  (i) Error detecting positional dots." << std::endl;
        std::cout << std::endl;
        return -1;
    } else
    {
        for (const auto &e: positional_dots)
        {
            if (e == cv::Point2i(1, 1))
                orientation = Marker::RIGHT;
            else if (e == cv::Point2i(7, 1))
                orientation = Marker::DOWN;
            else if (e == cv::Point2i(7, 7))
                orientation = Marker::LEFT;
            else if (e == cv::Point2i(1, 7))
                orientation = Marker::UP;
        }
    }


    /// marker center indicator
    // might not be necessary at all, lets just leave it for the moment

    /// marker code
    // ToDo: check against left and right rotations
    const uchar *raw_code; // = marker_map.colRange(2, 7).ptr<uchar>(6);
    std::vector<uchar> code; //(code_row, code_row + (marker_map.cols - 4));
    switch (orientation)
    {
        case Marker::UP:
        {
            raw_code = marker_map.colRange(2, 7).ptr<uchar>(6);
            code = std::vector<uchar>(raw_code, raw_code + (marker_map.cols - 4));
            break;
        }
        case Marker::DOWN:
        {
            raw_code = marker_map.colRange(2, 7).ptr<uchar>(2);
            code = std::vector<uchar>(raw_code, raw_code + (marker_map.cols - 4));
            break;
        }
        case Marker::LEFT:
        {
            raw_code = marker_map.rowRange(2, 7).ptr<uchar>(2);
            code = std::vector<uchar>(raw_code, raw_code + (marker_map.cols - 4));
            break;
        }
        case Marker::RIGHT:
        {
            raw_code = marker_map.rowRange(2, 7).ptr<uchar>(6);
            code = std::vector<uchar>(raw_code, raw_code + (marker_map.cols - 4));
            break;
        }
        default:
        {
            std::cout << "(!) Error detecting orientation." << std::endl;
            return -1;
        }

    }


    /// calculate decimal marker id
    int marker_id = 0;
    for (int i = 0; i < code.size(); ++i)
    {
        int idx = static_cast<int>(code.size()) - i - 1;
        int val = static_cast<int>(pow(2, i) * static_cast<int>(code[idx]));
        marker_id += val;
    }
    int final_marker_id = static_cast<int>(pow(2, code.size())) - marker_id - 1;
    std::cout << "Marker id: " << final_marker_id << std::endl;
    std::cout << std::endl;

    return final_marker_id;
}


std::vector<cv::Point2f> MarkerDetector::sortVertices(std::vector<cv::Point2f> point_list) const
{

    // get centroid & sort clockwise
    cv::Point2f center = Misc::findCentroid(point_list);
    std::sort(point_list.begin(), point_list.end(), [=](cv::Point2f a, cv::Point2f b) {
        double a1 = std::fmod(Misc::rad2deg(atan2(a.x - center.x, a.y - center.y)) + 360, 360);
        double a2 = std::fmod(Misc::rad2deg(atan2(b.x - center.x, b.y - center.y)) + 360, 360);
        auto val = static_cast<int>(a1 - a2);
        return val;
    });


    // sort by lowest x values
    auto cpy(point_list);
    std::sort(cpy.begin(), cpy.end(), [=](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    });

    // filter the first two elements by y size to find the top left element
    int shift = 0;
    if (cpy[0].y < cpy[1].y)
    {
        for (size_t i = 0; i < point_list.size(); ++i)
            if (point_list[i] == cpy[0])
                shift = static_cast<int>(i);
    } else
    {
        for (size_t i = 0; i < point_list.size(); ++i)
            if (point_list[i] == cpy[1])
                shift = static_cast<int>(i);
    }
    shift = static_cast<int>(point_list.size()) - shift;

    // shift initially sorted vector by just found amount
    std::vector<cv::Point2f> final_sorted(point_list.size());
    for (size_t i = 0; i < point_list.size(); ++i)
    {
        int new_pos = (i + shift) % final_sorted.size();
        final_sorted[new_pos] = point_list[i];
    }

    return final_sorted;
}
