#include <iostream>
#include <vector>
#include <algorithm>
#include <dirent.h>
#include <map>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "feature_detector.h"
#include "path.h"
#include "camera.h"
#include "config.h"

/// callbacks
int getFiles (const std::string & dir, std::vector<std::string> &files);

bool calibrateCamera(Camera& cam, const std::string &image_folder);
std::map<std::string, cv::Mat> loadMarker(const BinarizeParams &config);
void binarizeImage(const cv::Mat &input, cv::Mat &output, double threshold, double max_value);
bool writeCameraParams(const std::string &filename, const CameraParams &params);
bool loadCameraParams(const std::string &filename, CameraParams *params);

void binarizeImage(const cv::Mat &input, cv::Mat &binarized);
void showImage(cv::Mat &image, const std::string &name = "Default", bool scale = false);
std::string type2str(int type);


struct sortY {
        bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.y < pt2.y);}
} mySortY;
struct sortX {
        bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} mySortX;

/// MAIN
int main()
{
    ApplicationParams app_config;
    if(!Config::loadApplicationConfig("config/application.yaml", &app_config)){
        std::cout << "Error: loading Application config!" << std::endl;
    }

    BinarizeParams binarize_config;
    if(!Config::loadBinarizeConfig("config/binarize.yaml", &binarize_config)){
        std::cout << "Error: loading Binarization config!" << std::endl;
    }

    CameraParams camera_config;
    if(!Config::loadCameraConfig("config/camera.yaml", &camera_config)){
        std::cout << "Error: loading Camera config!" << std::endl;
    }

    /// create camera
    Camera camera;
    calibrateCamera(camera, app_config.calibration_image_path);
    std::cout << std::endl;
    camera.initRectification();

    // load & rectify image
    cv::Mat image = cv::imread(app_config.single_test_image_path);

    // load marker
    std::map<std::string, cv::Mat> marker = loadMarker(binarize_config);

    cv::Mat image_rectified;
    camera.getRectifiedImage(image, image_rectified);

    /// Binarize
    cv::Mat binarized;
    binarizeImage(image_rectified, binarized, binarize_config.threshold, binarize_config.max_value);

    /// Find contours
    std::vector<std::vector<cv::Point>> contours_src;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(binarized, contours_src, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // create Polygons
    std::vector<std::vector<cv::Point2f>> contour_polys;
    //    std::vector<std::vector<cv::Point>> contour_polys;
    contour_polys.resize(contours_src.size());
    for( size_t i = 0; i < contours_src.size(); i++ )
        cv::approxPolyDP(cv::Mat(contours_src[i]), contour_polys[i], cv::LINE_8, true);

    // Filter for contours with only 4 points
    std::vector<std::vector<cv::Point2f>> contour_polys_filtered;
    //    std::vector<std::vector<cv::Point>> contour_polys_filtered;
    std::vector<size_t> square_indices;
    for(size_t i = 0; i < contour_polys.size(); ++i) {
        if(contour_polys[i].size() == 4) {
            contour_polys_filtered.push_back(contour_polys[i]);
            square_indices.push_back(i);
        }
    }

    //    std::cout << "Num Contours: " << contour_polys_filtered.size() << std::endl;
    //    std::printf("image width: %i height %i", image.rows, image.cols);
    //    cv::Mat image_clr;
    //    cv::cvtColor(binarized, image_clr, CV_GRAY2BGR);
    //    cv::drawContours(image_clr, contour_polys_filtered, 6, cv::Scalar(0, 0, 255), 4);
    //    showImage(image_clr, "contours");

    cv::Mat bottom_res;
    cv::resize(marker["bottom"], bottom_res, cv::Size(9, 9), 0, 0, cv::INTER_LINEAR);
    std::cout << "Marker:" << std::endl;
    std::cout << bottom_res << std::endl;

    // order points
    std::vector<cv::Point2f> borders = contour_polys_filtered[6];
    std::sort(borders.begin(),borders.end(),mySortY);
    std::sort(borders.begin(),borders.begin()+2,mySortX);

    int pattern_size = 200;
    std::vector<cv::Point2f> dest;
    dest.push_back(cv::Point2f(0,0));
    dest.push_back(cv::Point2f(pattern_size,0));
    dest.push_back(cv::Point2f(pattern_size,pattern_size));
    dest.push_back(cv::Point2f(0,pattern_size));

    // calc perspective transformation
    cv::Mat p_transform = cv::getPerspectiveTransform(borders, dest);

    // Display shit
    cv::Mat found_marker(cv::Size(pattern_size, pattern_size), CV_32FC2);
    cv::warpPerspective(binarized, found_marker, p_transform, cv::Size(pattern_size, pattern_size));

    showImage(found_marker, "Marker transformed");

    cv::Mat result;
    for(auto e : marker) {
        cv::matchTemplate(found_marker, e.second, result, CV_TM_CCOEFF_NORMED);
        showImage(result, e.first);
    }

    //    cv::matchTemplate(found_marker, marker["bottom"], result, CV_TM_CCOEFF_NORMED);
    //    showImage(result, "Result");
}

/**
 * @brief calibrateCamera
 * @param cam
 * @param image_folder
 * @return
 */
bool calibrateCamera(Camera& cam, const std::string &image_folder)
{
    std::vector<std::string> image_files;
    getFiles(image_folder, image_files);

    std::vector<cv::Mat> calibration_images;
    std::cout << "Loading calibration Images" << std::endl;
    for(unsigned long i = 0; i < image_files.size(); ++i) {
        std::string image_path = image_folder + "/" + image_files[i];
        cv::Mat img = cv::imread(image_path);
        //        std::printf(" > loaded: %s with size: (%i,%i)\n", image_path.c_str(), img.cols, img.rows);
        calibration_images.push_back(img);
    }

    if(cam.calibrate(calibration_images, 2.0, cv::Size(11, 8), true, false))
        return true;

    return false;
}

std::map<std::string, cv::Mat> loadMarker(const BinarizeParams &config)
{
    std::map<std::string, cv::Mat> marker;
    marker["left"] = cv::imread("images/marker/left.jpg");
    marker["right"] = cv::imread("images/marker/right.jpg");
    marker["top"] = cv::imread("images/marker/top.jpg");
    marker["bottom"]= cv::imread("images/marker/bottom.jpg");

    for (auto &element : marker) {
        binarizeImage(element.second, element.second, config.threshold, config.max_value);
    }

    return marker;
}

/**
 * @brief binarizeImage
 * @param input
 * @param output
 * @param threshold
 * @param max_value
 */
void binarizeImage(const cv::Mat &input, cv::Mat &output, double threshold, double max_value)
{
    cv::cvtColor(input, output, CV_RGB2GRAY);
    cv::threshold(output.clone(), output, threshold, max_value, cv::THRESH_BINARY);
}

/**
 * @brief getFiles
 * @param dir
 * @param files
 * @return
 */
int getFiles (std::string const& dir, std::vector<std::string> &files)
{
    DIR *dp;
    struct dirent *dirp;

    //Unable to open dir
    if((dp  = opendir(dir.c_str())) == nullptr) {
        std::cout << "Error(" << errno << ") opening " << dir << std::endl;
        return errno;
    }

    //read files and push them to vector
    while ((dirp = readdir(dp)) != nullptr) {
        std::string name = std::string(dirp->d_name);

        //discard . and .. from list .. .DS_Store get the fuck outta here
        if(name != "." && name != ".." && name != ".DS_Store") {
            files.push_back(std::string(dirp->d_name));
        }
    }

    closedir(dp);
    std::sort(files.begin(), files.end());

    return 0;
}

///**
// * @brief onMouse
// * @param event
// * @param x
// * @param y
// */
//static void onMouse(int event, int x, int y, int, void*) {
//    if( event != cv::EVENT_LBUTTONDOWN )
//        return;

//    //    cv::Vec3b mat_value = MAT.at<cv::Vec3b>(cv::Point(x,y));
//    //    std::cout << mat_value << std::endl;
//    //    std::printf("x pos: %i y pos: %i\n", mat_value);
//}

/**
 * @brief showImage
 * @param image
 * @param name
 */
void showImage(cv::Mat &image, const std::string &name, bool scale) {
    cv::namedWindow(name);
    //    cv::setMouseCallback(name, onMouse, 0);
    if (scale)
        cv::resize(image, image, cv::Size(image.cols/2, image.rows/2));
    cv::imshow(name, image);
    cv::waitKey(0);
}

/**
 * @brief type2str
 * @param type
 * @return
 */
std::string type2str(int type) {
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

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
