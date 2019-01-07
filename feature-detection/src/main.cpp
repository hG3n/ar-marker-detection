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

struct MatchingResult {
        double correlation;
        std::string orientation;
        cv::Mat projection_matrix;
        int shape_idx;
};

bool CALIBRATE = true;

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

double calcMean(const cv::Mat &matrix);
double calcStdDev(const cv::Mat &matrix);

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

    // load marker
    std::map<std::string, cv::Mat> marker_templates = loadMarker(binarize_config);

    cv::Mat image_rectified;
    std::chrono::steady_clock::time_point image_rectification_start= std::chrono::steady_clock::now();
    camera.getRectifiedImage(image, image_rectified);
    std::chrono::steady_clock::time_point image_rectification_end= std::chrono::steady_clock::now();
    auto image_rectification_time = (std::chrono::duration_cast<std::chrono::microseconds>( image_rectification_end- image_rectification_start).count()) / 1000.0;
    std::cout << "Image Rectification time (ms) = " << image_rectification_time << std::endl;

    /// Binarize
    cv::Mat binarized;
    binarizeImage(image_rectified, binarized, binarize_config.threshold, binarize_config.max_value);

    /// Find contours
    std::vector<std::vector<cv::Point>> contours_src;
    std::vector<cv::Vec4i> hierarchy;

    std::chrono::steady_clock::time_point contour_search_start= std::chrono::steady_clock::now();
    cv::findContours(binarized, contours_src, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
    std::chrono::steady_clock::time_point contour_search_end= std::chrono::steady_clock::now();
    auto contour_search_time = (std::chrono::duration_cast<std::chrono::microseconds>( contour_search_end- contour_search_start).count()) / 1000.0;
    std::cout << "Contour search time (ms) = " << contour_search_time << std::endl;

    // create Polygons
    std::chrono::steady_clock::time_point poly_approx_start= std::chrono::steady_clock::now();
    std::vector<std::vector<cv::Point2f>> contour_polys;
    //    std::vector<std::vector<cv::Point>> contour_polys;
    contour_polys.resize(contours_src.size());
    for( size_t i = 0; i < contours_src.size(); i++ )
        cv::approxPolyDP(cv::Mat(contours_src[i]), contour_polys[i], cv::LINE_8, true);
    std::chrono::steady_clock::time_point poly_approx_end= std::chrono::steady_clock::now();
    auto poly_approx_time = (std::chrono::duration_cast<std::chrono::microseconds>( poly_approx_end- poly_approx_start).count()) / 1000.0;
    std::cout << "Polygon Approximation time (ms) = " << poly_approx_time << std::endl;

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
    dest.push_back(cv::Point2f(0,0));
    dest.push_back(cv::Point2f(pattern_size,0));
    dest.push_back(cv::Point2f(pattern_size,pattern_size));
    dest.push_back(cv::Point2f(0,pattern_size));

    // calculate marker mean
    std::vector<double> marker_mean, marker_std_dev;
    for(auto marker : marker_templates) {
        cv::meanStdDev(marker.second, marker_mean, marker_std_dev);
    }

    // measure time
    std::chrono::steady_clock::time_point correlation_analysis_start = std::chrono::steady_clock::now();

    /// for each found contour try to find a marker
    size_t num_contours = contour_polys_filtered.size();
    std::vector<std::map<std::string, MatchingResult>> matching_results(num_contours);
    for(size_t i = 0; i < contour_polys_filtered.size(); ++i) {
        // sort corner points to clockwise direction
        std::vector<cv::Point2f> borders = contour_polys_filtered[i];
        std::sort(borders.begin(), borders.end(), mySortY);
        std::sort(borders.begin(), borders.begin() + 2, mySortX);

        // calc projection from given corner points points
        cv::Mat p_transform = cv::getPerspectiveTransform(borders, dest);

        // warp found marker
        cv::Mat warped_shape(cv::Size(pattern_size, pattern_size), CV_32FC1);
        cv::warpPerspective(binarized, warped_shape, p_transform, cv::Size(pattern_size, pattern_size));

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

            MatchingResult res = MatchingResult();
            res.correlation = correlation;
            res.orientation = marker.first;
            res.shape_idx = static_cast<int>(i);
            res.projection_matrix = p_transform;

            matching_results[i][marker.first] = res;
            //             std::printf("Shape: %i Marker: '%s' Correlation: %f\n", static_cast<int>(i), marker.first.c_str(), correlation);
        }

    }
    std::chrono::steady_clock::time_point correlation_analysis_end= std::chrono::steady_clock::now();

    auto correlation_analysis_time = (std::chrono::duration_cast<std::chrono::microseconds>( correlation_analysis_end- correlation_analysis_start).count()) / 1000.0;
    std::cout << "Correlation analysis tiem (ms) = " << correlation_analysis_time << std::endl;

    std::chrono::steady_clock::time_point marker_finding_start = std::chrono::steady_clock::now();
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
    std::chrono::steady_clock::time_point marker_finding_end = std::chrono::steady_clock::now();

    auto marker_finding_time = (std::chrono::duration_cast<std::chrono::microseconds>( marker_finding_end- marker_finding_start).count()) / 1000.0;
    std::cout << "Marker matching time (ms) = " << marker_finding_time << std::endl;

    std::printf("Identified Shape %i as Marker %s\n", found_marker.shape_idx, found_marker.orientation.c_str());
    std::cout << "Projection matrix of detected shape: " << std::endl;
    std::cout << found_marker.projection_matrix << std::endl;

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
