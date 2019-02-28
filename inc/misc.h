#ifndef MISC_H
#define MISC_H

#include <string>
#include <vector>

#include <opencv2/core.hpp>

class Misc {
public:
    /**
     * Prints the actual type of a matrix
     * @param type
     * @return
     */
    static std::string type2str(int type);

    /**
     * Wrapper to show images
     * @param image
     * @param name
     * @param scale
     */
    static void showImage(cv::Mat &image, const std::string &name = "Default", int scale = -1);



    /**
     * Finds the mean centroid of a point list
     */
    static cv::Point2f findCentroid(const std::vector<cv::Point2f> &point_list);

    /**
     * Sorts pointlist clockwise starting with the top left element
     * @param point_list
     * @return
     */
    static std::vector<cv::Point2f> sortVertices(std::vector<cv::Point2f> point_list);

    /**
     * Converts degress to radians
     * @param value
     * @return
     */
    static double deg2rad(double value);

    /**
     * Converts radians to degrees
     * @param value
     * @return
     */
    static double rad2deg(double value);

};

#endif // MISC_H
