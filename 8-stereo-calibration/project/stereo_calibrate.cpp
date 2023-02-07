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
#include "./dirreader.h"

using namespace std;
using namespace cv;

const Size CHECKER_BOARD_SIZE = {7, 5};
const double SQUARE_SIZE = 0.02875;
const string STEREO_IMG_PATHS = "/home/isa/Documents/vision-3d-projects/Vision3dVault/8-stereo-calibration/project/data/stereo/calibration";

bool find_chessboard_corners(const Mat &img,
                             vector<Point2f> &corners)
{
    bool was_found = findChessboardCorners(img, CHECKER_BOARD_SIZE, corners,
                                           CALIB_CB_ADAPTIVE_THRESH +
                                               CALIB_CB_NORMALIZE_IMAGE +
                                               CALIB_CB_FAST_CHECK);
    if (!was_found)
    {
        std::cout << "Error Finding Chessboard" << endl;
        return was_found;
    }

    Mat bw_image;
    cvtColor(img, bw_image, COLOR_BGR2GRAY);
    cornerSubPix(
        bw_image, corners, Size(5, 5), Size(-1, -1),
        TermCriteria(TermCriteria::COUNT + TermCriteria::EPS, 60, 1e-6));

    return was_found;
}

vector<vector<Mat>> readStereoImages()
{

    DirReader Dir;
    auto filePaths = Dir.read(STEREO_IMG_PATHS, ".jpg", DirReader::Params(true));

    vector<vector<Mat>> stereo_images;
    for (auto &file : filePaths)
    {
        Mat img;
        try
        {
            img = imread(file);
        }
        catch (...)
        {
            std::cerr << "Error: no he podido abrir el fichero '" << file << "'." << std::endl;
        }

        Mat left_img, right_img;
        int img_width = img.cols / 2;
        left_img = Mat(img, Rect(0, 0, img_width, img.rows));
        right_img = Mat(img, Rect(img_width, 0, img_width, img.rows));
        stereo_images.push_back({left_img, right_img});
    }
    return stereo_images;
}

vector<Point3f> objectPointsForImg()
{
    vector<Point3f> ret_v;
    for (int i = 1; i <= CHECKER_BOARD_SIZE.height; i++)
    {
        for (int j = 1; j <= CHECKER_BOARD_SIZE.width; j++)
        {
            ret_v.push_back(Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0));
        }
    }

    return ret_v;
}

void saveToYaml(Mat lCameraMatrix, Mat rCameraMatrix, Mat lDistCoeffs, Mat rdistCoeffs, Mat R, Mat T, Mat E, Mat F)
{
    cv::FileStorage fs("Mystereoparams.yml", cv::FileStorage::WRITE);
    fs << "LEFT_K " << lCameraMatrix;
    fs << "LEFT_D " << lDistCoeffs;
    fs << "RIGHT_K " << rCameraMatrix;
    fs << "RIGHT_D " << rdistCoeffs;
    fs << "R " << R;
    fs << "T " << T;
    fs << "E " << E;
    fs << "F " << F;
    fs.release();
}

void calibrate(vector<vector<Mat>> images)
{
    vector<vector<Point2f>> lpoints, rpoints;
    vector<vector<Point3f>> objectPoints;
    vector<Point3f> calibration_points = objectPointsForImg();

    for (auto &image_pair : images)
    {
        vector<Point2f> lcorners, rcorners;
        find_chessboard_corners(image_pair[0], lcorners);
        find_chessboard_corners(image_pair[1], rcorners);
        lpoints.push_back(lcorners);
        rpoints.push_back(rcorners);

        // assert(CHECKER_BOARD_SIZE == lpoints[0].size());
        // assert(CHECKER_BOARD_SIZE == rpoints[0].size());

        objectPoints.push_back(calibration_points);
    }
    // Calibrations Params
    Mat lCameraMatrix, rCameraMatrix, lDistCoeffs, rdistCoeffs, R, T, E, F;
    Size imageSize = images[0][0].size();

    calibrateCamera(objectPoints, lpoints,
                    imageSize, lCameraMatrix, lDistCoeffs, {}, {});

    calibrateCamera(objectPoints, rpoints,
                    imageSize, rCameraMatrix, rdistCoeffs, {}, {});

    stereoCalibrate(objectPoints, lpoints, rpoints,
                    lCameraMatrix, lDistCoeffs, rCameraMatrix,
                    rdistCoeffs, imageSize, R, T, E, F);

    std::cout << "lCameraMatrix: " << lCameraMatrix << endl;
    std::cout << "rCameraMatrix: " << rCameraMatrix << endl;
    std::cout << "lDistCoeffs: " << lDistCoeffs << endl;
    std::cout << "rdistCoeffs: " << rdistCoeffs << endl;
    std::cout << "R: " << R << endl;
    std::cout << "T: " << T << endl;
    std::cout << "E: " << E << endl;
    std::cout << "F: " << F << endl;

    saveToYaml(lCameraMatrix,
               rCameraMatrix,
               lDistCoeffs,
               rdistCoeffs,
               R,
               T,
               E,
               F);
}

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        vector<vector<Mat>> images;
        images = readStereoImages();
        if (images.size() == 0)
        {
            return;
        }
        calibrate(images);
    }
    catch (std::exception &e)
    {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    catch (...)
    {
        std::cerr << "Capturada excepcion desconocida!" << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
