#include <iostream>
#include <vector>
#include <algorithm>
#include <dirent.h>
#include <map>
#include <limits>
#include <chrono>
#include <map>
#include <cmath>

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
#include "marker.h"
#include "marker_detector.h"

static bool CALIBRATE = false;

bool calibrateCamera(Camera &cam, const std::string &image_folder);

void findMarkerInImage(unsigned char *image_data, int width, int height);

/// MAIN
int main()
{
    // load configs
    ApplicationParams app_config;
    if (!Config::loadApplicationConfig("config/application.yaml", &app_config))
        std::cout << "Error: loading Application config!" << std::endl;

    BinarizeParams binarize_config;
    if (!Config::loadBinarizeConfig("config/binarize.yaml", &binarize_config))
        std::cout << "Error: loading Binarization config!" << std::endl;


    /// ---- replace this by receiving camera intrinsics from unity
    /// ---- get pointer to image from unity

    /// create camera
    Camera camera;
    if (CALIBRATE)
    {
        calibrateCamera(camera, app_config.calibration_image_path);
        std::cout << std::endl;
    } else
    {
        CameraParams camera_config;
        std::cout << "Trying to load camera config" << std::endl;
        if (!Config::loadCameraConfig("config/camera.yaml", &camera_config))
            std::cout << "Error: loading Camera config!" << std::endl;

        camera = Camera(camera_config);
    }
    camera.initRectification();


    /// ---- initialize recification paramesters

    // load & rectify image

    /// ---- cut that out
    cv::Mat image = cv::imread(app_config.single_test_image_path);
    cv::Mat rectified;
    cv::Mat rectified_gray;
    camera.getRectifiedImage(image, rectified);
    cv::cvtColor(rectified.clone(), rectified_gray, CV_RGB2GRAY);

    /// Binarize
    cv::Mat binarized;
    cv::threshold(rectified_gray, binarized, binarize_config.threshold, binarize_config.max_value, cv::THRESH_BINARY);


    /// ---- put all this into one specific function, that returns a result to unity
    auto marker_class_start = Stopwatch::getStart();
    MarkerDetector md(81, 9);
    std::vector<Marker> current_markers;
    md.findMarkers(binarized, &current_markers);
    std::cout << "MarkerDetector finding time (ms): " << Stopwatch::getElapsed(marker_class_start);
}

/**
 * @brief calibrateCamera
 * @param cam
 * @param image_folder
 * @return
 */
bool calibrateCamera(Camera &cam, const std::string &image_folder)
{
    std::vector<std::string> image_files;
    FileIo::getDirectoryContent(image_folder, image_files);

    std::cout << "Scanning folder: " << image_folder << std::endl;
    std::cout << "Number of calibration images: " << image_files.size() << std::endl;
    if (image_files.size() < 1)
    {
        std::cout << "No images present in calibration folder!" << std::endl;
        return false;
    }

    std::vector<cv::Mat> calibration_images;
    std::cout << "Loading calibration Images" << std::endl;
    for (size_t i = 0; i < image_files.size(); ++i)
    {
        std::string image_path = image_folder + "/" + image_files[i];
        cv::Mat img = cv::imread(image_path);
        //        std::printf(" > loaded: %s with size: (%i,%i)\n", image_path.c_str(), img.cols, img.rows);
        calibration_images.push_back(img);
    }

    CameraParams params;
    if (cam.calibrate(calibration_images, 2.0, cv::Size(11, 8), params, true, false))
    {
        Config::saveCameraConfig("config/camera.yaml", params);
        return true;
    }
    return false;
}

void findMarkerInImage(unsigned char *image_data, int width, int height)
{
    cv::Mat image(height, width, CV_8SC3, image_data);

    std::cout << "Current image: " << std::endl;
    std::cout << "width: " << width << std::endl;
    std::cout << "height: " << height << std::endl;

    /// binarize image
    cv::Mat binarized;

    int threshold = 100;
    int max_value =  255;
    cv::threshold(image, binarized, threshold, max_value, cv::THRESH_BINARY);

    /// ---- put all this into one specific function, that returns a result to unity
    MarkerDetector md(81, 9);
    std::vector<Marker> current_markers;
    md.findMarkers(binarized, &current_markers);
    
    for(auto marker : current_markers) {
        std::cout << marker.getId() << std::endl;
    }


}

