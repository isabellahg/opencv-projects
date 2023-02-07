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

/**
 *
 * objectPoints, is an N-by-3 matrix containing the physical coordinates
 * of each of the K points on each of the M images of the 3D object such
 * that N = K × M
 *
 */
vector<vector<Point3f>> getObjectPoints(int nImages)
{
    vector<vector<Point3f>> objPoints;
    objPoints.resize(nImages);
    for (int img_idx = 0; img_idx < nImages; img_idx++)
    {
        for (int i = 0; i < CHECKER_BOARD_SIZE.height; i++)
        {
            for (int j = 0; j < CHECKER_BOARD_SIZE.width; j++)
            {
                objPoints[img_idx].push_back(Point3f(j * SQUARE_SIZE, i * SQUARE_SIZE, 0.0));
            }
        }
    }
    return objPoints;
}

void saveToYaml(Mat cameraMatrix[2], Mat distCoeffs[2], Mat R, Mat T, Mat E, Mat F)
{
    cv::FileStorage fs("Mystereoparams.yml", cv::FileStorage::WRITE);
    fs << "LEFT_K " << cameraMatrix[0];
    fs << "LEFT_D " << distCoeffs[0];
    fs << "RIGHT_K " << cameraMatrix[1];
    fs << "RIGHT_D " << distCoeffs[1];
    fs << "R " << R;
    fs << "T " << T;
    fs << "E " << E;
    fs << "F " << F;
    fs.release();
}

void calibrate(vector<vector<Mat>> images)
{
    vector<vector<Point2f>> points[2];
    int nimages = images.size();
    vector<vector<Point3f>> objectPoints = getObjectPoints(nimages);

    for (auto &image_pair : images)
    {
        vector<Point2f> corners[2];
        find_chessboard_corners(image_pair[0], corners[0]);
        find_chessboard_corners(image_pair[1], corners[1]);
        points[0].push_back(corners[0]);
        points[1].push_back(corners[1]);

    }
    Size imageSize = images[0][0].size();

    Mat cameraMatrix[2], distCoeffs[2];
    cameraMatrix[0] = initCameraMatrix2D(objectPoints, points[0], imageSize, 0);
    cameraMatrix[1] = initCameraMatrix2D(objectPoints, points[1], imageSize, 0);
    Mat R, T, E, F;

    stereoCalibrate(objectPoints, points[0], points[1],
                    cameraMatrix[0], distCoeffs[0], cameraMatrix[1],
                    distCoeffs[1], imageSize, R, T, E, F, cv::CALIB_USE_INTRINSIC_GUESS,
                    cv::TermCriteria(cv::TermCriteria::MAX_ITER + cv::TermCriteria::EPS, 60, 1e-6));

    std::cout << "LEFT_K " << cameraMatrix[0] << endl;
    std::cout << "LEFT_D " << cameraMatrix[1] << endl;
    std::cout << "RIGHT_K " << distCoeffs[0] << endl;
    std::cout << "RIGHT_D " << distCoeffs[1] << endl;
    std::cout << "R " << R << endl;
    std::cout << "T " << T << endl;
    std::cout << "E " << E << endl;
    std::cout << "F " << F << endl;

    saveToYaml(cameraMatrix,
               distCoeffs,
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
        if (images.size() > 0)
        {
            calibrate(images);
        }
        else
        {
            retCode = EXIT_FAILURE;
        }
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
