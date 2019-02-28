#ifndef CONFIG_H
#define CONFIG_H

#include <string>

#include "camera.h"

struct ApplicationParams {
        std::string calibration_image_path;
        std::string single_test_image_path;
        std::string fiducial_image_path;
        std::string image_sequence_path;
        cv::Size output_window_size;
};


struct BinarizeParams {
        double max_value;
        double threshold;
};


class Config
{
    public:
        static bool saveApplicationConfig(const std::string &filename, const ApplicationParams& app_params);
        static bool saveCameraConfig(const std::string &filename, const CameraParams& params);
        static bool saveBinarizeConfig(const std::string &filename, const BinarizeParams& params);

        static bool loadApplicationConfig(const std::string &filename, ApplicationParams* app_params);
        static bool loadCameraConfig(const std::string &filename, CameraParams* params);
        static bool loadBinarizeConfig(const std::string &filename, BinarizeParams* params);

};

#endif // CONFIG_H
