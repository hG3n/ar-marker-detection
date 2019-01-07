#ifndef CAMERA_H
#define CAMERA_H

#include <vector>

#include <opencv2/core.hpp>

struct CameraParams {
        int width;
        int height;
        cv::Mat intrinsics;
        cv::Mat dist_coeffs;
};

class Camera
{
    public:
        Camera();
        Camera(int width, int height);
        Camera(const CameraParams &params);

        bool calibrate(const std::vector<cv::Mat> &calibration_images, double pattern_size, cv::Size chessboard_size, CameraParams &params, bool verbose, bool show_projection);
        void initRectification();

        void getRectifiedImage(const cv::Mat & image, cv::Mat & image_undistorted);
        void getCameraParams();

        const cv::Mat & getIntrinsics();
        const cv::Mat & getDistCoeffs();
        bool isCalibrated() const;

    private:
        CameraParams params;

        int image_width_;
        int image_height_;

        cv::Mat intrinsics_;
        cv::Mat distortion_coefficients_;

        cv::Mat rectify_intrinsics_;

        bool is_calibrated_;

        cv::Mat rectify_map_1_;
        cv::Mat rectify_map_2_;
};

#endif // CAMERA_H
