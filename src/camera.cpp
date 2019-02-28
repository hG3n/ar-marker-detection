#include <iostream>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>

#include "camera.h"

Camera::Camera()
    : image_width_()
    , image_height_()
    , intrinsics_()
    , distortion_coefficients_()
{  }

Camera::Camera(int width, int height)
    : image_width_(width)
    , image_height_(height)
    , intrinsics_()
    , distortion_coefficients_()
{  }

Camera::Camera(const CameraParams &params)
    : image_width_(params.width)
    , image_height_(params.height)
    , intrinsics_(params.intrinsics)
    , distortion_coefficients_(params.dist_coeffs)
{  }


bool Camera::calibrate(const std::vector<cv::Mat> &calibration_images, double square_size, cv::Size chessboard_size, CameraParams &params, bool verbose = false, bool show_projection = false)
{

    if(is_calibrated_)
        return true;

    image_width_ = calibration_images[0].cols;
    image_height_ = calibration_images[0].rows;

    // needed calibration variables
    std::vector<cv::Mat> rvecs, tvecs;
    std::vector<std::vector<cv::Point3f>> object_points;
    std::vector<std::vector<cv::Point2f>> image_points;
    std::vector<cv::Point2f> corners;
    std::vector<cv::Point3f> obj;

    // size of calibration patteren squares
    for(int y=0; y<chessboard_size.height; ++y) {
        for(int x=0; x<chessboard_size.width; ++x) {
            obj.push_back(cv::Point3f((float(x)*square_size) ,(float(y)*square_size), 0));
        }
    }

    bool success = false;
    for(unsigned int i = 0; i < calibration_images.size(); ++i) {
        cv::Mat gray_image;
        cv::cvtColor(calibration_images[i], gray_image, CV_BGR2GRAY);

        // try to find chessboard corners within an image
        bool found = cv::findChessboardCorners( gray_image, chessboard_size, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FILTER_QUADS);

        if(found) {
            cv::cornerSubPix(gray_image, corners, cv::Size(5,5), cv::Size(-1,-1), cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01));

            if(show_projection) {
                cv::Mat clr_image;
                cv::cvtColor(gray_image, clr_image, CV_GRAY2RGB);
                cv::drawChessboardCorners(clr_image, cv::Size(11,8), corners, true);
                cv::resize(clr_image, clr_image, cv::Size(clr_image.cols, clr_image.rows));
                cv::imshow("Calibration", clr_image);
                cv::waitKey();
                clr_image.release();
            }
            image_points.push_back(corners);
            object_points.push_back(obj);

            if(verbose)
                std::cout << " > Found pattern in image " << i << std::endl;

            success = true;
        }
        else
        {
            if(verbose)
                std::cout << "Unable to find Corners in image " << i << ". Image ignored" << std::endl;

            continue;
        }

        gray_image.release();
    }

    cv::Size imagesize = cv::Size(calibration_images[0].size());
    double rms = cv::calibrateCamera(object_points, image_points, imagesize, intrinsics_, distortion_coefficients_, rvecs, tvecs);

    params.width = image_width_;
    params.height = image_height_;
    params.dist_coeffs = distortion_coefficients_;
    params.intrinsics = intrinsics_;

    if(verbose)
        std::cout << " ! Calibrated camera with reprojection error of: " << rms << std::endl;

    is_calibrated_ = true;

    return success;
}


void Camera::initRectification()
{
    // undistort image
    rectify_intrinsics_= cv::getOptimalNewCameraMatrix(
                intrinsics_,
                distortion_coefficients_,
                cv::Size(image_width_, image_height_),
                0,
                cv::Size(image_width_, image_height_));

    cv::initUndistortRectifyMap(
                intrinsics_,
                distortion_coefficients_,
                cv::Mat(),
                rectify_intrinsics_,
                cv::Size(image_width_, image_height_),
                CV_16SC2,
                rectify_map_1_,
                rectify_map_2_);
}

void Camera::getRectifiedImage(const cv::Mat & image, cv::Mat & image_undistorted)
{
    /// ToDo Change this to actual capturing code
    //    image_undistorted = image;

    cv::undistort(image, image_undistorted, intrinsics_, distortion_coefficients_, rectify_intrinsics_);
//        cv::remap(image, image_undistorted, rectify_map_1_, rectify_map_2_, cv::INTER_LINEAR);
}

const cv::Mat &Camera::getIntrinsics()
{
    return intrinsics_;
}

const cv::Mat &Camera::getDistCoeffs()
{
    return distortion_coefficients_;
}

bool Camera::isCalibrated() const {
    return is_calibrated_;
}

