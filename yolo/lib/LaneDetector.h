#include "Error.h"
#include "Thread.h"

#include <Argus/Argus.h>
#include <EGLStream/EGLStream.h>
#include <EGLStream/NV/ImageNativeBuffer.h>

#include <NvEglRenderer.h>
#include <NvJpegEncoder.h>

#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>
#include "ds_image.h"
#include "trt_utils.h"
#include "yolo_config_parser.h"
#include "yolo.h"
#include "yolov2.h"
#include "yolov3.h"
#include "uartwireless.hpp"

#define topH 400
#define verticalMargin 200
#define lane_detect 0
static Argus::Size2D<uint32_t> PREVIEW_SIZE(1280, 720);

class LaneDetector
{
private:
	double img_size;
	double img_center;
	bool left_flag = false;  // Tells us if there's left boundary of lane detected
	bool right_flag = false; // Tells us if there's right boundary of lane detected
	cv::Point right_b;		 // Members of both line equations of the lane boundaries:
	double right_m;			 // y = m*x + b
	cv::Point left_b;		 //
	double left_m;			 //

public:
	cv::Mat deNoise(cv::Mat inputImage);																		 // Apply Gaussian blurring to the input Image
	cv::Mat edgeDetector(cv::Mat img_noise);																	 // Filter the image to obtain only edges
	cv::Mat mask(cv::Mat img_edges);																			 // Mask the edges image to only care about ROI
	std::vector<cv::Vec4i> houghLines(cv::Mat img_mask);														 // Detect Hough lines in masked edges image
	std::vector<std::vector<cv::Vec4i>> lineSeparation(std::vector<cv::Vec4i> lines, cv::Mat img_edges);		 // Sprt detected lines by their slope into right and left lines
	std::vector<cv::Point> regression(std::vector<std::vector<cv::Vec4i>> left_right_lines, cv::Mat inputImage); // Get only one line for each side of the lane
	std::string predictTurn();																					 // Determine if the lane is turning or not by calculating the position of the vanishing point
	cv::Mat plotLane(cv::Mat inputImage, std::vector<cv::Point> lane, std::string turn);						 // Plot the resultant lane and turn prediction in the frame.
};