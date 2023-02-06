/*!
  Esto es un esqueleto de programa para usar en las prácticas
  de Visión Artificial.

  Se supone que se utilizará OpenCV.

  Para compilar, puedes ejecutar:
    g++ -Wall -o esqueleto esqueleto.cc `pkg-config opencv --cflags --libs`

*/

#include <iostream>
#include <exception>

//Includes para OpenCV, Descomentar según los módulo utilizados.
#include <opencv2/core/core.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/calib3d/calib3d.hpp>

const char *keys =
        "{help h usage ? |      | print this message   }"
        "{path           |.     | path to file         }"
        "{fps            | -1.0 | fps for output video }"
        "{N count        |100   | count of objects     }"
        "{ts timestamp   |      | use time stamp       }"
        "{@image1        |      | image1 for compare   }"
        "{@image2        |<none>| image2 for compare   }"
        "{@repeat        |1     | number               }";


void
foo(double minVal, double maxVal, cv::Point &minLoc, cv::Point &maxLoc, const std::vector<cv::Mat> &channels, int i);

/*!
    @brief imprima los valores máximo/mínimo y su primera localización por canal.

    Esta forma usa una función de opencv.

    @param[in] img es la imagen de entrada.
    @param[out] media la media de los valores.
    @param[out] dev la desviación estárdar de los valores.

    @pre img no está vacia.
    @pre img es de tipo CV_32FC1 (Un sólo canal en formato float).
*/
void
compute_min_max(const cv::Mat &img) {
    //Comprobacion de precondiciones.
    CV_Assert(!img.empty());

    std::cout << "Number of Channels: " << img.channels() << std::endl;


    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(img, &minVal, &maxVal, &minLoc, &maxLoc);

    std::cout << "minVal: " << minVal << std::endl;
    std::cout << "maxVal: " << maxVal << std::endl;
    std::cout << "minLoc: " << "(" << minLoc.x << ", " << minLoc.y << ")" << std::endl;
    std::cout << "maxLoc: " << "(" << maxLoc.x << ", " << maxLoc.y << ")" << std::endl;
}

/*!
    @brief Imprime los valores máximo/mínimo y su primera localización en todos los canales.

    Esta forma usa una función de opencv. Utiliza reshape para interpretar
    la imagen con multiples canales como una imagen de un solo canal

    @param[in] img es la imagen de entrada.
    @param[out] media la media de los valores.
    @param[out] dev la desviación estárdar de los valores.

    @pre img no está vacia.
    @pre img es de tipo CV_32FC1 (Un sólo canal en formato float).
*/
void compute_min_max_all_channels(const cv::Mat &img) {
    cv::Mat reshaped;
    img.reshape(1, 0).convertTo(reshaped, CV_64F);

    cv::namedWindow("IMG - ONE CHANNEL", cv::WINDOW_GUI_EXPANDED);

    //Visualizo la imagen cargada en la ventana.
    cv::imshow("IMG - ONE CHANNEL", reshaped);

    std::cout << "Pulsa ESC para salir." << std::endl;
    while ((cv::waitKey(0) & 0xff) != 27);

    //Debemos cerrar las ventanas abiertas.
    cv::destroyWindow("IMG - ONE CHANNEL");

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::minMaxLoc(reshaped, &minVal, &maxVal, &minLoc, &maxLoc);

    std::cout << "minVal: " << minVal << std::endl;
    std::cout << "maxVal: " << maxVal << std::endl;
    std::cout << "minLoc: " << "(" << minLoc.x << ", " << minLoc.y << ")" << std::endl;
    std::cout << "maxLoc: " << "(" << maxLoc.x << ", " << maxLoc.y << ")" << std::endl;
}


void
drawResultsInPicture(double minVal, double maxVal, cv::Point &minLoc, cv::Point &maxLoc, cv::Mat &img, std::string name) {
    cv::ellipse(img, minLoc,
                cv::Size(3, 3), 0, 0,
                360, cv::Scalar(1, 255, 255),
                1, cv::LINE_AA);

    cv::ellipse(img, maxLoc,
                cv::Size(3, 3), 0, 0,
                360, cv::Scalar(0, 255, 255),
                1, cv::LINE_AA);

    cv::putText(img, "MIN", cv::Point(minLoc.x + 5, minLoc.y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
    cv::putText(img, "MAX", cv::Point(maxLoc.x + 5, maxLoc.y), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 0), 1);

    int border = 20;
    cv::Scalar white(255, 255, 255);
    cv::copyMakeBorder(img, img, 0, border, 0, 0, cv::BORDER_CONSTANT, white);

    int fontFace = cv::FONT_HERSHEY_PLAIN;
    double fontScale = 1;
    int thickness = 1;

    cv::Point textOrg(5, img.rows - 5);


    std::stringstream ss;
    ss << name << ": MIN=" << minVal << " (" << minLoc.x << ", " << minLoc.y << ") | MAX=" << maxVal << " (" << maxLoc.x
       << ", " << maxLoc.y << ")";
    std::string str = ss.str();

    cv::putText(img, str, textOrg, fontFace, fontScale, cv::Scalar::all(0), thickness, 8);
}


void concatenateImgs(const std::vector<cv::Mat> &channels, cv::Mat &result) {
    int max_rows = channels[0].rows;
    int max_cols = channels[0].cols;

    // Find the max number of columns and the number of rows
    for (const auto &channel: channels) {
        max_cols = std::max(max_cols, channel.cols);
        max_rows = std::max(max_rows, channel.rows);
    }

    std::vector<cv::Mat> resizedChannels;
    for (const auto &channel: channels) {
        cv::Mat resized;
        cv::resize(channel, resized, cv::Size(max_cols, max_rows));
        resizedChannels.push_back(resized);
    }

    // Horizontally concatenate the resized images
    cv::vconcat(resizedChannels, result);
}

void compute_min_max_by_channel(cv::Mat &img) {
    std::vector<cv::Mat> rows; // We use an array of images to represent the rows

    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    std::vector<cv::Mat> channels;
    cv::split(img, channels);

    double minValGlobal = DBL_MAX;
    double maxValGlobal = DBL_MIN;

    cv::Point minLocGlobal, maxLocGlobal;


    for (int i = 0; i < img.channels(); i++) {
        cv::minMaxLoc(channels[i], &minVal, &maxVal, &minLoc, &maxLoc);
        drawResultsInPicture(minVal, maxVal, minLoc, maxLoc, channels[i], std::string("C") + std::to_string(i));

        if (minVal <= minValGlobal) {
            minValGlobal = minVal;
            minLocGlobal = minLoc;
        }

        if (maxVal >= maxValGlobal) {
            maxValGlobal = maxVal;
            maxLocGlobal = maxLoc;
        }
    }


    drawResultsInPicture(minValGlobal, maxValGlobal, minLocGlobal, maxLocGlobal, img, "ORIGINAL (GLOBAL RESULTS)");

    cv::Mat concatenated;
    concatenateImgs(channels, concatenated);

    std::cout << "Pulsa ESC para continuar..." << std::endl;
    cv::imshow("Channels", concatenated);
    cv::imshow("Original", img);
    while ((cv::waitKey(0) & 0xff) != 27);

    //Debemos cerrar las ventanas abiertas.
    cv::destroyWindow("Results");
}


int
main(int argc, char *const *argv) {
    int retCode = EXIT_SUCCESS;

    try {

        cv::CommandLineParser parser(argc, argv, keys);
        parser.about("Application name v1.0.0");
        if (parser.has("help")) {
            parser.printMessage();
            return 0;
        }
        int N = parser.get<int>("N");
        double fps = parser.get<double>("fps");
        cv::String path = parser.get<cv::String>("path");
        bool use_time_stamp = parser.has("timestamp");
        cv::String img1 = parser.get<cv::String>("@image1");
        cv::String img2 = parser.get<cv::String>("@image2");
        int repeat = parser.get<int>("@repeat");
        if (!parser.check()) {
            parser.printErrors();
            return 0;
        }


        /*Ahora toca que tu rellenes con lo que hay que hacer ...*/
        cv::Mat img = cv::imread(img1, cv::IMREAD_ANYCOLOR);
        if (img.empty()) {
            std::cerr << "Error: no he podido abrir el fichero '" << img1 << "'." << std::endl;
            return EXIT_FAILURE;
        }

        compute_min_max_by_channel(img);


    }
    catch (std::exception &e) {
        std::cerr << "Capturada excepcion: " << e.what() << std::endl;
        retCode = EXIT_FAILURE;
    }
    catch (...) {
        std::cerr << "Capturada excepcion desconocida!" << std::endl;
        retCode = EXIT_FAILURE;
    }
    return retCode;
}
