#include <iostream>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/persistence.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <stdio.h>
#include <string>
#include <vector>
#include <cassert>

#include "common_code.hpp"
using namespace cv;
using namespace std;

std::vector<cv::Point3f>
fsiv_generate_3d_calibration_points(const cv::Size &board_size,
                                    float square_size)
{
    std::vector<cv::Point3f> ret_v;
    for (int i = 1; i <= board_size.height; i++)
    {
        for (int j = 1; j <= board_size.width; j++)
        {
            ret_v.push_back(Point3f(j * square_size, i * square_size, 0.0));
        }
    }
    CV_Assert(ret_v.size() == static_cast<size_t>(board_size.width * board_size.height));
    return ret_v;
}

bool fsiv_find_chessboard_corners(const cv::Mat &img, const cv::Size &board_size,
                                  std::vector<cv::Point2f> &corner_points,
                                  const char *wname)
{
    CV_Assert(img.type() == CV_8UC3);
    bool was_found = false;
    bool was_found = findChessboardCorners(img, board_size, corner_points,
                                           CALIB_CB_ADAPTIVE_THRESH +
                                               CALIB_CB_NORMALIZE_IMAGE +
                                               CALIB_CB_FAST_CHECK);
    if (!was_found)
    {
        cout << "Error Finding Chessboard" << endl;
        return was_found;
    }

    Mat bw_image;
    cvtColor(img, bw_image, COLOR_BGR2GRAY);
    cornerSubPix(
        bw_image, corner_points, Size(5, 5), Size(-1, -1),
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 60, 1e-6));

    return was_found;
}

float fsiv_calibrate_camera(const std::vector<std::vector<cv::Point2f>> &_2d_points,
                            const std::vector<std::vector<cv::Point3f>> &_3d_points,
                            const cv::Size &camera_size,
                            cv::Mat &camera_matrix,
                            cv::Mat &dist_coeffs,
                            std::vector<cv::Mat> *rvecs,
                            std::vector<cv::Mat> *tvecs)
{
    CV_Assert(_3d_points.size() >= 2 && _3d_points.size() == _2d_points.size());
    float error = 0.0;
    error = cv::calibrateCamera(_3d_points, _2d_points, camera_size,
                                camera_matrix, dist_coeffs, *rvecs, *tvecs);
    CV_Assert(camera_matrix.rows == camera_matrix.cols &&
              camera_matrix.rows == 3 &&
              camera_matrix.type() == CV_64FC1);
    CV_Assert((dist_coeffs.rows * dist_coeffs.cols) == 5 &&
              dist_coeffs.type() == CV_64FC1);
    CV_Assert(rvecs == nullptr || rvecs->size() == _2d_points.size());
    CV_Assert(tvecs == nullptr || tvecs->size() == _2d_points.size());
    return error;
}

void fsiv_compute_camera_pose(const std::vector<cv::Point3f> &_3dpoints,
                              const std::vector<cv::Point2f> &_2dpoints,
                              const cv::Mat &camera_matrix,
                              const cv::Mat &dist_coeffs,
                              cv::Mat &rvec,
                              cv::Mat &tvec)
{
    CV_Assert(_3dpoints.size() >= 4 && _3dpoints.size() == _2dpoints.size());
    cv::solvePnP(_3dpoints, _2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
    CV_Assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64FC1);
    CV_Assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64FC1);
}

void fsiv_draw_axes(cv::Mat &img,
                    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                    const cv::Mat &rvec, const cv::Mat &tvec,
                    const float size, const int line_width)
{
    // TODO

    //
}

void fsiv_save_calibration_parameters(cv::FileStorage &fs,
                                      const cv::Size &camera_size,
                                      float error,
                                      const cv::Mat &camera_matrix,
                                      const cv::Mat &dist_coeffs,
                                      const cv::Mat &rvec,
                                      const cv::Mat &tvec)
{
    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix.type() == CV_64FC1 && camera_matrix.rows == 3 && camera_matrix.cols == 3);
    CV_Assert(dist_coeffs.type() == CV_64FC1 && dist_coeffs.rows == 1 && dist_coeffs.cols == 5);
    CV_Assert(rvec.type() == CV_64FC1 && rvec.rows == 3 && rvec.cols == 1);
    CV_Assert(tvec.type() == CV_64FC1 && tvec.rows == 3 && tvec.cols == 1);
    fs << "image-width" << camera_size.width;
    fs << "image-height" << camera_size.height;
    fs << "error" << error;
    fs << "camera-matrix" << camera_matrix;
    fs << "distortion-coefficient" << dist_coeffs;
    fs << "rvec" << rvec;
    fs << "tvec" << tvec;
    CV_Assert(fs.isOpened());
    return;
}

void fsiv_load_calibration_parameters(cv::FileStorage &fs,
                                      cv::Size &camera_size,
                                      float &error,
                                      cv::Mat &camera_matrix,
                                      cv::Mat &dist_coeffs,
                                      cv::Mat &rvec,
                                      cv::Mat &tvec)
{
    CV_Assert(fs.isOpened());

    int image_width = (int)fs["image-width"];
    int image_height = (int)fs["image-height"];
    camera_size = cv::Size(image_width, image_height);
    error = (int)fs["error"];
    fs["camera-matrix"] >> camera_matrix;
    fs["distortion-coefficient"] >> dist_coeffs;
    fs["rvec"] >> rvec;
    fs["tvec"] >> tvec;

    CV_Assert(fs.isOpened());
    CV_Assert(camera_matrix.type() == CV_64FC1 && camera_matrix.rows == 3 && camera_matrix.cols == 3);
    CV_Assert(dist_coeffs.type() == CV_64FC1 && dist_coeffs.rows == 1 && dist_coeffs.cols == 5);
    CV_Assert(rvec.type() == CV_64FC1 && rvec.rows == 3 && rvec.cols == 1);
    CV_Assert(tvec.type() == CV_64FC1 && tvec.rows == 3 && tvec.cols == 1);
    return;
}

void fsiv_undistort_image(const cv::Mat &input, cv::Mat &output,
                          const cv::Mat &camera_matrix,
                          const cv::Mat &dist_coeffs)
{
    // TODO
    // Hint: use cv::undistort.
    output = input.clone();

    //
}

void fsiv_undistort_video_stream(cv::VideoCapture &input_stream,
                                 cv::VideoWriter &output_stream,
                                 const cv::Mat &camera_matrix,
                                 const cv::Mat &dist_coeffs,
                                 const int interp,
                                 const char *input_wname,
                                 const char *output_wname,
                                 double fps)
{
    CV_Assert(input_stream.isOpened());
    CV_Assert(output_stream.isOpened());
    // TODO
    // Hint: to speed up, first compute the transformation maps
    //(one time only at the beginning using cv::initUndistortRectifyMap)
    //  and then only remap (cv::remap) the input frame with the computed maps.

    //
    CV_Assert(input_stream.isOpened());
    CV_Assert(output_stream.isOpened());
}
