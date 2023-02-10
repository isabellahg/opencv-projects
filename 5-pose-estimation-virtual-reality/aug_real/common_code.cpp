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
            ret_v.push_back(Point3f(j * square_size, i * square_size, 0));
        }
    }
    CV_Assert(ret_v.size() == board_size.width * board_size.height);
    return ret_v;
}

bool fsiv_find_chessboard_corners(const cv::Mat &img, const cv::Size &board_size,
                                  std::vector<cv::Point2f> &corner_points,
                                  const char *wname)
{
    CV_Assert(img.type() == CV_8UC3);
    bool was_found = findChessboardCorners(img, board_size, corner_points,
                                           CALIB_CB_ADAPTIVE_THRESH +
                                               CALIB_CB_NORMALIZE_IMAGE +
                                               CALIB_CB_FAST_CHECK);
    if (!was_found)
    {
        std::cout << "Error Finding Chessboard" << std::endl;
        return was_found;
    }

    Mat bw_image;
    cvtColor(img, bw_image, COLOR_BGR2GRAY);
    cornerSubPix(
        bw_image, corner_points, Size(5, 5), Size(-1, -1),
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 60, 1e-6));

    return was_found;
}

void fsiv_compute_camera_pose(const std::vector<cv::Point3f> &_3dpoints,
                              const std::vector<cv::Point2f> &_2dpoints,
                              const cv::Mat &camera_matrix,
                              const cv::Mat &dist_coeffs,
                              cv::Mat &rvec,
                              cv::Mat &tvec)
{
    CV_Assert(_3dpoints.size() >= 4 && _3dpoints.size() == _2dpoints.size());
    // TODO DONE
    cv::solvePnP(_3dpoints, _2dpoints, camera_matrix, dist_coeffs, rvec, tvec);
    //
    CV_Assert(rvec.rows == 3 && rvec.cols == 1 && rvec.type() == CV_64FC1);
    CV_Assert(tvec.rows == 3 && tvec.cols == 1 && tvec.type() == CV_64FC1);
}

void fsiv_draw_axes(cv::Mat &img,
                    const cv::Mat &camera_matrix, const cv::Mat &dist_coeffs,
                    const cv::Mat &rvec, const cv::Mat &tvec,
                    const float size, const int line_width)
{
    // TODO DONE
    cv::drawFrameAxes(img, camera_matrix, dist_coeffs, rvec, tvec, size, line_width);

    //
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

void fsiv_draw_3d_model(cv::Mat &img, const cv::Mat &M, const cv::Mat &dist_coeffs,
                        const cv::Mat &rvec, const cv::Mat &tvec,
                        const float size)
{
    CV_Assert(img.type() == CV_8UC3);
    // TODO

    //
}

void fsiv_project_image(const cv::Mat &input, cv::Mat &output,
                        const cv::Size &board_size,
                        const std::vector<cv::Point2f> &_2dpoints)
{
    CV_Assert(!input.empty() && input.type() == CV_8UC3);
    CV_Assert(!output.empty() && output.type() == CV_8UC3);
    CV_Assert(board_size.area() == _2dpoints.size());
    // TODO

    //
}
