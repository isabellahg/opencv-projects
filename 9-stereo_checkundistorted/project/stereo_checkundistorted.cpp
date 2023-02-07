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

using namespace std;
using namespace cv;

const string STEREO_PARAMS_DIR = "/home/isa/Documents/vision-3d-projects/Vision3dVault/8-stereo-calibration/project/build/Mystereoparams.yml";
const string STEREO_IMG_DIR = "/home/isa/Documents/vision-3d-projects/Vision3dVault/8-stereo-calibration/project/data/stereo/calibration/m003.jpg";
const Scalar LINE_COLOR = Scalar(0, 0, 255);
const int LINE_THICKNESS = 2;

int originalMousePosition[2] = {0, 0};
int rectifiedMousePosition[2] = {0, 0};

Mat originalImg;
Mat rectifiedImg;
Mat originalDrawed;
Mat rectifiedDrawed;

bool isOriginal = true;
// Structure that contains the Stereo Pair Calbration information.
// This will be calculated using stereo_calibrate
struct StereoParams
{
    Mat mtxL, distL, R_L, T_L;
    Mat mtxR, distR, R_R, T_R;
    Mat Rot, Trns, Emat, Fmat;
};

// Splits a stereo image in half.
vector<Mat> splitImages(const Mat &img)
{
    int img_width = img.cols / 2;
    vector<Mat> images(2);
    images[0] = Mat(img, Rect(0, 0, img_width, img.rows));
    images[1] = Mat(img, Rect(img_width, 0, img_width, img.rows));
    return images;
}

Mat mergeImages(Mat leftImg, Mat rightImg)
{
    Mat image = Mat(leftImg.rows, leftImg.cols + rightImg.cols,
                    leftImg.type());
    leftImg.copyTo(image(Rect(0, 0, leftImg.cols, leftImg.rows)));
    rightImg.copyTo(
        image(Rect(leftImg.cols, 0, rightImg.cols, rightImg.rows)));
    return image;
}

// This function is going to be used as a callback for setMouseCallback
void showOriginalImgWithLine(int event, int x, int y, int flags, void *userdata)
{
    if (event == EVENT_MOUSEMOVE)
    {
        originalDrawed = originalImg.clone();
        line(originalDrawed, Point(0, y), Point(originalDrawed.cols, y),
             LINE_COLOR, LINE_THICKNESS);
        imshow("Original", originalDrawed);
    }
}

void showRectifiedImgWithLine(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_MOUSEMOVE)
    {

        rectifiedDrawed = rectifiedImg.clone();
        line(rectifiedDrawed, Point(0, y), Point(rectifiedDrawed.cols, y),
             LINE_COLOR, LINE_THICKNESS);
        imshow("Rectified", rectifiedDrawed);
    }
}

void rectifyStereoImages(const StereoParams &sti, Mat &left, Mat &rigth)
{
    Mat rect_l, rect_r, proj_mat_l, proj_mat_r, Q;
    Mat Left_Stereo_Map1, Left_Stereo_Map2;
    Mat Right_Stereo_Map1, Right_Stereo_Map2;
    stereoRectify(sti.mtxL,
                  sti.distL, sti.mtxR, sti.distR, left.size(), sti.Rot, sti.Trns,
                  rect_l, rect_r, proj_mat_l, proj_mat_r,
                  Q, CALIB_ZERO_DISPARITY, 0);
    initUndistortRectifyMap(sti.mtxL, sti.distL, rect_l, proj_mat_l,
                            left.size(), CV_16SC2,
                            Left_Stereo_Map1, Left_Stereo_Map2);
    initUndistortRectifyMap(sti.mtxR, sti.distR,
                            rect_r, proj_mat_r,
                            left.size(), CV_16SC2,
                            Right_Stereo_Map1, Right_Stereo_Map2);
    Mat AuxImage, Right_nice;
    remap(left, AuxImage, Left_Stereo_Map1, Left_Stereo_Map2,
          INTER_LANCZOS4, BORDER_CONSTANT, 0);
    AuxImage.copyTo(left);
    remap(rigth, AuxImage, Right_Stereo_Map1, Right_Stereo_Map2,
          INTER_LANCZOS4, BORDER_CONSTANT, 0);
    AuxImage.copyTo(rigth);
}

StereoParams loadFromYaml(string filename)
{
    FileStorage fs(filename, FileStorage::READ);
    StereoParams stereoParams;
    fs["LEFT_K"] >> stereoParams.mtxL;
    fs["LEFT_D"] >> stereoParams.distL;
    fs["RIGHT_K"] >> stereoParams.mtxR;
    fs["RIGHT_D"] >> stereoParams.distR;
    fs["R"] >> stereoParams.Rot;
    fs["T"] >> stereoParams.Trns;
    fs["E"] >> stereoParams.Emat;
    fs["F"] >> stereoParams.Fmat;
    return stereoParams;
}

int main(int argc, char *const *argv)
{
    int retCode = EXIT_SUCCESS;

    try
    {
        const StereoParams sti = loadFromYaml(STEREO_PARAMS_DIR);
        Mat img = imread(STEREO_IMG_DIR);

        img.copyTo(originalImg);
        namedWindow("Original");
        setMouseCallback("Original", showOriginalImgWithLine);
        imshow("Original", originalImg);

        auto images = splitImages(img);
        Mat l,r;
        l = images[0];
        r = images[1];
        rectifyStereoImages(sti, l, r);
        rectifiedImg = mergeImages(l, r);
        namedWindow("Rectified");
        setMouseCallback("Rectified", showRectifiedImgWithLine);
        imshow("Rectified", rectifiedImg);
        waitKey(0);
        return 0;
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
