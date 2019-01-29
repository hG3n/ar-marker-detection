#include "misc.h"

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
    uchar chans = 1 + (static_cast<uchar>(type)>> CV_CN_SHIFT);

    switch ( depth ) {
        case CV_8U:  r = "8U"; break;
        case CV_8S:  r = "8S"; break;
        case CV_16U: r = "16U"; break;
        case CV_16S: r = "16S"; break;
        case CV_32S: r = "32S"; break;
        case CV_32F: r = "32F"; break;
        case CV_64F: r = "64F"; break;
        default:     r = "User"; break;
    }

    r += "C";
    r += (chans+'0');

    return r;
}


/**
 * @brief showImage
 * @param image
 * @param name
 */
void Misc::showImage(cv::Mat &image, const std::string &name, bool scale) {
    cv::namedWindow(name);
    //    cv::setMouseCallback(name, onMouse, 0);
    if (scale)
        cv::resize(image, image, cv::Size(image.cols/2, image.rows/2));
    cv::imshow(name, image);
    cv::waitKey(0);

}
