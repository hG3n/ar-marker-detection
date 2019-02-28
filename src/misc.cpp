#include "misc.h"

#include <iostream>
#include <algorithm>
#include <math.h>
#include <limits>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

/**
 * @brief type2str
 * @param type
 * @return
 */
std::string Misc::type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (static_cast<uchar>(type) >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}


/**
 * @brief showImage
 * @param image
 * @param name
 */
void Misc::showImage(cv::Mat &image, const std::string &name, int scale) {
    cv::namedWindow(name);
    //    cv::setMouseCallback(name, onMouse, 0);

    if (scale > 0) {
        if (image.rows > image.cols) {
            std::cout << "IMPLEMENT PORTRAIT IMAGE SCALE!" << std::endl;
        } else if (image.cols > image.rows) {
            std::cout << "IMPLEMENT LANDSCAPE IMAGE SCALE!" << std::endl;
        } else {
            cv::resize(image, image, cv::Size(scale, scale));
        }
    }

    cv::imshow(name, image);
    cv::waitKey(0);

}


cv::Point2f Misc::findCentroid(const std::vector<cv::Point2f> &point_list) {
    float x = 0;
    float y = 0;
    for (auto p : point_list) {
        x += p.x;
        y += p.y;
    }
    return cv::Point2f(point_list.size(), point_list.size());
}


std::vector<cv::Point2f> Misc::sortVertices(std::vector<cv::Point2f> point_list) {

    // get centroid & sort clockwise
    cv::Point2f center = findCentroid(point_list);
    std::sort(point_list.begin(), point_list.end(), [=](cv::Point2f a, cv::Point2f b) {
        double a1 = std::fmod(rad2deg(atan2(a.x - center.x, a.y - center.y)) + 360, 360);
        double a2 = std::fmod(rad2deg(atan2(b.x - center.x, b.y - center.y)) + 360, 360);
        auto val = a1 - a2;
//        auto val = static_cast<int>(a1 - a2);
//        std::cout << "value: " << val << std::endl;
        return val;
    });

    // sort by lowest x values
    auto cpy(point_list);
    std::sort(cpy.begin(), cpy.end(), [=](cv::Point2f a, cv::Point2f b) {
        return a.x < b.x;
    });

    // filter the first two elements by y size to find the top left element
    int shift = 0;
    if (cpy[0].y < cpy[1].y) {
        for (int i = 0; i < point_list.size(); ++i) {
            if (point_list[i] == cpy[0])
                shift = i;
        }
    } else {
        for (int i = 0; i < point_list.size(); ++i) {
            if (point_list[i] == cpy[1])
                shift = i;
        }
    }
    shift = static_cast<int>(point_list.size()) - shift;

    // shift initially sorted vector by just found amount
    std::vector<cv::Point2f> final_sorted(point_list.size());
    for (size_t i = 0; i < point_list.size(); ++i) {
        int new_pos = (i + shift) % final_sorted.size();
        final_sorted[new_pos] = point_list[i];
    }

    return final_sorted;
}


double Misc::deg2rad(double value) {

    return value * M_PI / 180.0;
}

double Misc::rad2deg(double value) {
    value * 180 / M_PI;
}

