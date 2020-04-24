#include "Capture.hpp"

using namespace cv;
using namespace std;
using namespace ArgusSamples;

float factor0 = 0.0;
float factor1 = 0.0;
float factor2 = 0.0;
float factor3 = 0.0;
float factor4 = 0.0;
float factor5 = 0.0;
float h_mount = 0.0; // camera mounted hight.
float pixelLength_y = 0.0;
float pixelLength_x = 0.0;
float object_distance_y = 0.0;
float object_distance_x = -1.0;
float objectLengthInVideo_y = 0.0;
float objectLengthInVideo_x = 0.0;

int main(int argc, char **argv)
{
    // Flag set in the command line overrides the value in the flagfile
    gflags::SetUsageMessage(
        "Usage : trt-yolo-app --flagfile=</path/to/config_file.txt> --<flag>=value ...");

    // parse config params
    yoloConfigParserInit(argc, argv);
    std::string networkType = getNetworkType();
    uint batchSize = getBatchSize();
    NetworkInfo yoloInfo = getYoloNetworkInfo();
    InferParams yoloInferParams = getYoloInferParams();
    if ((networkType == "yolov2") || (networkType == "yolov2-tiny"))
    {
        inferNet = std::unique_ptr<Yolo>{new YoloV2(batchSize, yoloInfo, yoloInferParams)};
    }
    else if ((networkType == "yolov3") || (networkType == "yolov3-tiny"))
    {
        inferNet = std::unique_ptr<Yolo>{new YoloV3(batchSize, yoloInfo, yoloInferParams)};
    }
    else
    {
        assert(false && "Unrecognised network_type. Network Type has to be one among the following : yolov2, yolov2-tiny, yolov3 and yolov3-tiny");
    }

    float theta; // angle between camera optical axis and vertical line.
    float phi;   // camera vertical angle of view.
    float alpha; // camera horizontal angle of view.

    cout << "Please enter the angle between camera optical axis and vertical line: (degree)" << endl;
    assert(scanf("%f", &theta) == true);
    cout << "The included angle you enter is: " << theta << " degree." << endl;
    theta *= CV_PI / 180;

    cout << "Please enter the camera vertical angle of view: (degree)" << endl;
    assert(scanf("%f", &phi) == true);
    cout << "The vertical angle you enter is: " << phi << " degree." << endl;
    phi *= CV_PI / 180;

    cout << "Please enter the camera horizontal angle of view: (degree)" << endl;
    assert(scanf("%f", &alpha) == true);
    cout << "The horizontal angle you enter is: " << alpha << " degree." << endl;
    alpha *= CV_PI / 180;

    cout << "Please enter the camera mounted hight: (meter)" << endl;
    assert(scanf("%f", &h_mount) == true);
    cout << "The mounted hight you enter is: " << h_mount << " meter." << endl;

    factor0 = tan(theta - phi / 2);
    factor1 = cos(theta);
    factor2 = sin(theta);
    factor3 = sin(phi / 2);
    factor4 = cos(theta - phi / 2);
    factor5 = tan(alpha / 2);

    pixelLength_y = (2 * h_mount * factor3 / factor4) / PREVIEW_SIZE.height();
    pixelLength_x = (2 * h_mount * factor5 / factor4) / PREVIEW_SIZE.width();

    NvEglRenderer *renderer = NvEglRenderer::createEglRenderer("renderer0", PREVIEW_SIZE.width(), PREVIEW_SIZE.height(), 0, 0);

    if (!ArgusSamples::execute(renderer))
        return EXIT_FAILURE;

    delete renderer;
    return 0;
}
