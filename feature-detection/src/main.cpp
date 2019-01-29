#include <iostream>
#include <vector>
#include <algorithm>
#include <dirent.h>
#include <map>
#include <limits>
#include <chrono>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "feature_detector.h"
#include "path.h"
#include "camera.h"
#include "config.h"
#include "stopwatch.h"
#include "file_io.h"
#include "misc.h"

struct MatchingResult {
        double correlation;
        std::string orientation;
        cv::Mat projection_matrix;
        int shape_idx;
        std::vector<cv::Point2f> borders;
};

static bool CALIBRATE = false;
static std::map<std::string, cv::Mat> marker_templates;

void findMarker(const cv::Mat & image, MatchingResult & result);
bool calibrateCamera(Camera& cam, const std::string &image_folder);
std::map<std::string, cv::Mat> loadMarker(const BinarizeParams &config);

/// MAIN
int main()
{
    ApplicationParams app_config;
    if(!Config::loadApplicationConfig("config/application.yaml", &app_config)) {
        std::cout << "Error: loading Application config!" << std::endl;
    }

    BinarizeParams binarize_config;
    if(!Config::loadBinarizeConfig("config/binarize.yaml", &binarize_config)) {
        std::cout << "Error: loading Binarization config!" << std::endl;
    }

    /// create camera
    Camera camera;
    if (CALIBRATE) {
        calibrateCamera(camera, app_config.calibration_image_path);
        std::cout << std::endl;
    } else {
        CameraParams camera_config;
        std::cout << "Trying to load camera config" << std::endl;
        if(!Config::loadCameraConfig("config/camera.yaml", &camera_config)) {
            std::cout << "Error: loading Camera config!" << std::endl;
        }
        camera = Camera(camera_config);
    }
    camera.initRectification();

    // load & rectify image
    cv::Mat image = cv::imread(app_config.single_test_image_path);

    cv::Mat rectified;
    cv::Mat rectified_gray;
    auto image_rectification_start = Stopwatch::getStart();
    camera.getRectifiedImage(image, rectified);
    std::cout << "Image Rectification time (ms) = " << Stopwatch::getElapsed(image_rectification_start) << std::endl;
    cv::cvtColor(rectified.clone(), rectified_gray, CV_RGB2GRAY);

    /// Binarize
    cv::Mat binarized;
    cv::threshold(rectified_gray, binarized, binarize_config.threshold, binarize_config.max_value, cv::THRESH_BINARY);

    // load marker templates
    marker_templates = loadMarker(binarize_config);

    // find marker
    MatchingResult found_marker;
    findMarker(binarized, found_marker);

    std::printf("Identified Shape %i as Marker %s\n", found_marker.shape_idx, found_marker.orientation.c_str());
    std::cout << "Projection matrix of detected shape: " << std::endl;
    std::cout << found_marker.projection_matrix << std::endl;

    std::cout << std::endl;
    std::cout << "Determining world coordinate origin" << std::endl;
    cv::Vec2f origin;
    float x_avg = 0.0f;
    float y_avg = 0.0f;

    auto image_rect_cpy = rectified.clone();
    for (auto p: found_marker.borders) {
        x_avg += p.x;
        y_avg += p.y;

        // draw cornerpoints
        cv::circle(image_rect_cpy, p, 2, {0, 0, 255}, CV_FILLED);
    }

    auto origin_image = cv::Point2f(x_avg / 4, y_avg / 4);
    std::cout << "Marker centroid : " << origin_image << std::endl;

    cv::circle(image_rect_cpy, origin_image, 2, {0, 255, 0}, CV_FILLED);
    Misc::showImage(image_rect_cpy, "centroid");
}

/**
 * Tries to find markers in a given image
 * @brief findMarker
 * @param image
 * @param result
 */
void findMarker(const cv::Mat &image, MatchingResult &result) {
    /// Find contours
    std::vector<std::vector<cv::Point>> contours_src;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(image, contours_src, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

    // create Polygons
    std::vector<std::vector<cv::Point2f>> contour_polys;
//    std::vector<std::vector<cv::Point>> contour_polys_vis;
    contour_polys.resize(contours_src.size());
    for( size_t i = 0; i < contours_src.size(); i++ ) {
        cv::approxPolyDP(cv::Mat(contours_src[i]), contour_polys[i], cv::LINE_8, true);
    }

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

    //    std::cout << " ? Num Contours: " << contour_polys_filtered.size() << std::endl;
    //    std::printf("image width: %i height %i", image.rows, image.cols);
    //    cv::Mat image_clr;
    //    cv::cvtColor(binarized, image_clr, CV_GRAY2BGR);
    //    cv::drawContours(image_clr, contour_polys_filtered, 6, cv::Scalar(0, 0, 255), 4);
    //    showImage(image_clr, "contours");

    // define pattern size for perspective projection calculation
    int pattern_size = 90;
    std::vector<cv::Point2f> dest;
    dest.push_back(cv::Point2f(0, 0));
    dest.push_back(cv::Point2f(pattern_size, 0));
    dest.push_back(cv::Point2f(pattern_size, pattern_size));
    dest.push_back(cv::Point2f(0, pattern_size));

    // calculate marker mean
    std::vector<double> marker_mean, marker_std_dev;
    for (auto marker : marker_templates) {
        cv::meanStdDev(marker.second, marker_mean, marker_std_dev);
    }

    /// for each found contour try to find a marker
    size_t num_contours = contour_polys_filtered.size();
    std::vector<std::map<std::string, MatchingResult>> matching_results(num_contours);
    for(size_t i = 0; i < contour_polys_filtered.size(); ++i) {
        // sort corner points to clockwise direction
        std::vector<cv::Point2f> borders = contour_polys_filtered[i];

        // calc projection from given corner points points
        cv::Mat p_transform = cv::getPerspectiveTransform(borders, dest);

        // warp found marker
        cv::Mat warped_shape(cv::Size(pattern_size, pattern_size), CV_32FC1);
        cv::warpPerspective(image, warped_shape, p_transform, cv::Size(pattern_size, pattern_size));

        std::cout << "Warped shape type: " << Misc::type2str(warped_shape.type())  << std::endl;
        std::cout << "Warped shape center: " << static_cast<int>(warped_shape.at<uchar>(45,45)) << std::endl;
        cv::cvtColor(warped_shape.clone(), warped_shape, CV_GRAY2RGB);
        cv::circle(warped_shape, cv::Point(45,45), 2, cv::Scalar(0, 0, 255), CV_FILLED);

        Misc::showImage(warped_shape, "Warped");
        cv::imwrite("marker.png", warped_shape);


        /// you might wanna to normalize the stuff first
        std::vector<double> found_shape_mean, found_shape_std_dev;
        cv::meanStdDev(warped_shape, found_shape_mean, found_shape_std_dev);

        // calculate correlation corefficient
        int rows = warped_shape.rows;
        int cols = warped_shape.cols * warped_shape.channels();
        if(warped_shape.isContinuous()) {
            cols *= rows;
            rows = 1;
        }

        // find marker orientation
        auto res = MatchingResult();
        for(auto marker : marker_templates) {
            uchar* s;
            uchar* m;
            double nom = 0.0;
            for(int r = 0; r < rows; ++r) {
                s = warped_shape.ptr<uchar>(r);
                m = marker.second.ptr<uchar>(r);
                for(int c = 0; c < cols; ++c) {
                    double shape_value = static_cast<double>(s[c]);
                    double marker_value = static_cast<double>(m[c]);
                    nom += (shape_value  - found_shape_mean[0]) * (marker_value - marker_mean[0]);
                }
            }

            double correlation = nom / (found_shape_std_dev [0] * marker_std_dev[0]);

            auto res = MatchingResult();
            res.correlation = correlation;
            res.orientation = marker.first;
            res.shape_idx = static_cast<int>(i);
            res.projection_matrix = p_transform;
            res.borders = borders;

            matching_results[i][marker.first] = res;
        }

    }

    int found_marker_idx;
    double greatest_correlation = std::numeric_limits<double>::min();
    MatchingResult found_marker;
    for(size_t i = 0; i < matching_results.size(); ++i) {
        for(auto marker_correlation : matching_results[i]) {
            if(marker_correlation.second.correlation > greatest_correlation) {
                greatest_correlation = marker_correlation.second.correlation;
                found_marker = marker_correlation.second;
                found_marker_idx = static_cast<int>(i);
            }
        }
    }

    result = found_marker;
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
    FileIo::getDirectoryContent(image_folder, image_files);

    std::cout << "Scanning folder: " << image_folder << std::endl;
    std::cout << "Number of calibration images: " << image_files.size() << std::endl;
    if (image_files.size() < 1) {
        std::cout <<  "No images present in calibration folder!" << std::endl;
        return false;
    }

    std::vector<cv::Mat> calibration_images;
    std::cout << "Loading calibration Images" << std::endl;
    for(size_t i = 0; i < image_files.size(); ++i) {
        std::string image_path = image_folder + "/" + image_files[i];
        cv::Mat img = cv::imread(image_path);
        //        std::printf(" > loaded: %s with size: (%i,%i)\n", image_path.c_str(), img.cols, img.rows);
        calibration_images.push_back(img);
    }

    CameraParams params;
    if(cam.calibrate(calibration_images, 2.0, cv::Size(11, 8), params, true, false)){
        Config::saveCameraConfig("config/camera.yaml", params);
        return true;
    }
    return false;
}

/**
 * @brief loadMarker
 * @param config
 * @return
 */
std::map<std::string, cv::Mat> loadMarker(const BinarizeParams &config)
{
    std::map<std::string, cv::Mat> marker;
    marker["left"] = cv::imread("images/marker/100x/left.jpg", cv::IMREAD_GRAYSCALE);
    marker["right"] = cv::imread("images/marker/100x/right.jpg", cv::IMREAD_GRAYSCALE);
    marker["top"] = cv::imread("images/marker/100x/top.jpg", cv::IMREAD_GRAYSCALE);
    marker["bottom"]= cv::imread("images/marker/100x/bottom.jpg", cv::IMREAD_GRAYSCALE);

    //    for (auto &element : marker) {
    //        binarizeImage(element.second, element.second, config.threshold, config.max_value);
    //    }

    return marker;
}


