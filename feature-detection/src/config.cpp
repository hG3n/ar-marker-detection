#include "config.h"

bool Config::saveApplicationConfig(const std::string &filename, const ApplicationParams &app_params)
{
    cv::FileStorage fs_out(filename, cv::FileStorage::WRITE);

    if (!fs_out.isOpened())
        return false;

    fs_out << "calibration_image_path" << app_params.calibration_image_path;
    fs_out << "single_test_image_path" << app_params.single_test_image_path;
    fs_out << "image_sequence_path"    << app_params.image_sequence_path;
    fs_out << "output_window_size"     << app_params.output_window_size;
    fs_out.release();

    return true;
}

bool Config::saveCameraConfig(const std::string &filename, const CameraParams &params)
{
    cv::FileStorage fs_out(filename, cv::FileStorage::WRITE);

    if (!fs_out.isOpened())
        return false;

    fs_out << "intrinsics" << params.intrinsics;
    fs_out << "dist_coeffs" << params.dist_coeffs;
    fs_out << "width" << params.width;
    fs_out << "height" << params.height;
    fs_out.release();

    return true;

}

bool Config::saveBinarizeConfig(const std::string &filename, const BinarizeParams &params)
{
    cv::FileStorage fs_out(filename, cv::FileStorage::WRITE);

    if (!fs_out.isOpened())
        return false;

    fs_out << "max_value" << params.max_value;
    fs_out << "threshold" << params.threshold;
    fs_out.release();

    return true;
}




bool Config::loadApplicationConfig(const std::string &filename, ApplicationParams* app_params)
{
    cv::FileStorage fs_in(filename, cv::FileStorage::READ);

    if (!fs_in.isOpened())
        return false;

    fs_in["calibration_image_path"] >> app_params->calibration_image_path;
    fs_in["single_test_image_path"] >> app_params->single_test_image_path;
    fs_in["image_sequence_path"] >> app_params->image_sequence_path;
    fs_in["output_window_size"] >> app_params->output_window_size;

    return true;
}

bool Config::loadCameraConfig(const std::string &filename, CameraParams *params)
{
    cv::FileStorage fs_in(filename, cv::FileStorage::READ);

    if (!fs_in.isOpened())
        return false;

    fs_in["width"] >> params->dist_coeffs;
    fs_in["height"] >> params->dist_coeffs;
    fs_in["intrinsics"] >> params->intrinsics;
    fs_in["dist_coeffs"] >> params->dist_coeffs;
    return true;
}

bool Config::loadBinarizeConfig(const std::string &filename, BinarizeParams *params)
{
    cv::FileStorage fs_in(filename, cv::FileStorage::READ);

    if(!fs_in.isOpened())
        return false;

    fs_in["max_value"] >> params->max_value;
    fs_in["threshold"] >> params->threshold;

    return true;
}


