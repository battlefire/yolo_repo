/*
 * Copyright (c) 2019-2020, SKYWELL CORPORATION. All rights reserved.
 */
#include "LaneDetector.h"
#include "TcpClient_4g.h"
#include "TransferJson.h"

#define PRODUCER_PRINT(...) printf("PRODUCER: " __VA_ARGS__)
#define CONSUMER_PRINT(...) printf("CONSUMER: " __VA_ARGS__)
#define TZ_REFRESH 16
#define mecReplyNoObject 0
#define replyMecPoll 1
#define sendAfterPoll 3
#define m_km_ms_hour 3600000
#define roadModule 0
#define soloModule 0
#define warningDist 5.0
#define frameGap 1

extern Data globle;
extern string globle_str;
extern pthread_rwlock_t rwlock;
using namespace Argus;
using namespace EGLStream;

static int CAPTURE_FPS = 30;
static char frameCount = 0;
static uint32_t SENSOR_MODE = 0;
static bool DO_STAT = false;
static vector<cv::Point2f> points[2];
static string port = "/dev/ttyUSB0";
static float direction = 0.0;
static int tSize = 20;
static cv::Point left_line[2] = {cv::Point(0, 0), cv::Point(0, 0)};
static cv::Point right_line[2] = {cv::Point(0, 0), cv::Point(0, 0)};
static std::vector<BBoxInfo> remaining;
static float dist_copy;
static float speed_copy;
static clock_t tStart, tEnd;
static bool aSet = false;
static DsImage a;
static bool newFrame = false;
static int opticalFlowFrame = 0;

extern float factor0;
extern float factor1;
extern float factor2;
extern float factor3;
extern float factor4;
extern float factor5;
extern float h_mount;
extern float pixelLength_y;
extern float pixelLength_x;
extern float object_distance_y;
extern float object_distance_x;
extern float objectLengthInVideo_y;
extern float objectLengthInVideo_x;
std::unique_ptr<Yolo> inferNet{nullptr};

typedef struct bBoxCenterStruct
{
    int bboxCount;
    int labelId;
    float w;
    float h;
    float xCenter;
    float yCenter;
    float x;
    float y;
    float radiusCenter;
} bBoxCenterStruct;
static bBoxCenterStruct center_copy;

float getDistance(cv::Point2f pointS, cv::Point2f pointE)
{
    float distance;
    distance = powf((pointS.x - pointE.x), 2) + powf((pointS.y - pointE.y), 2);
    distance = sqrtf(distance);
    return distance;
}

float y_pixel_map(int object_pixel_y)
{
    object_pixel_y = PREVIEW_SIZE.height() - object_pixel_y;
    objectLengthInVideo_y = pixelLength_y * object_pixel_y;

    object_distance_y = objectLengthInVideo_y * (factor1 + factor0 * factor2) / (1 - objectLengthInVideo_y * factor2 / h_mount) + h_mount * factor0;
    //printf("When y pixel is %d, real vertical distance between object and camera is: %fm.\n", PREVIEW_SIZE.height() - object_pixel_y, object_distance_y);

    return object_distance_y;
}

float x_pixel_map(int object_pixel_x)
{
    if (object_distance_y < h_mount * factor0)
    {
        printf("object_distance_y is: %fm\n", object_distance_y);
        printf("factor is: %fm\n", h_mount * factor0);
        printf("Please run y_pixel_map() first.\n");
        return -2.0; // Function is called.
    }

    object_pixel_x -= PREVIEW_SIZE.width() >> 1;
    objectLengthInVideo_x = pixelLength_x * object_pixel_x;

    object_distance_x = object_distance_y * objectLengthInVideo_x / (h_mount * factor0 + objectLengthInVideo_y * factor1);
    //printf("When x pixel is %d, real horizontal distance between object and middle line is: %fm.\n", object_pixel_x + (PREVIEW_SIZE.width() >> 1), object_distance_x);

    return object_distance_x;
}

float getFarNear(float x_pixel, float y_pixel)
{
    float y_length = y_pixel_map(y_pixel);
    float x_length = x_pixel_map(x_pixel);
    return sqrtf(powf(y_length, 2) + powf(x_length, 2));
}

float getDisplacement(cv::Point2f pointPre, cv::Point2f pointCur)
{
    float distance_y_pre = y_pixel_map(pointPre.y);
    float distance_x_pre = x_pixel_map(pointPre.x);
    float distance_y_cur = y_pixel_map(pointCur.y);
    float distance_x_cur = x_pixel_map(pointCur.x);
    float displacement_y = distance_y_cur - distance_y_pre;
    float displacement_x = distance_x_cur - distance_x_pre;
    float displacement = sqrtf(powf(displacement_y, 2) + powf(displacement_x, 2));
    //printf("The real moving distance in one frame is: %fm\n", displacement);
    direction = atan2(displacement_y, displacement_x) * 180 / CV_PI;
    if (distance_y_cur < distance_y_pre)
    {
        direction = 360 + direction;
    }
    return displacement;
}

cv::Mat laneDetect(cv::Mat imgOrg, cv::Mat imgShow)
{
    LaneDetector lanedetector;
    // Detect edges in the image
    cv::Mat img_edges = lanedetector.edgeDetector(imgOrg);
    // Mask the image so that we only get the ROI
    cv::Mat img_mask = lanedetector.mask(img_edges);
    // Obtain Hough lines in the cropped image
    std::vector<cv::Vec4i> lines = lanedetector.houghLines(img_mask);

    if (!lines.empty())
    {
        // Separate lines into left and right lines
        std::vector<std::vector<cv::Vec4i>> left_right_lines = lanedetector.lineSeparation(lines, img_edges);
        // Apply regression to obtain only one line for each side of the lane
        std::vector<cv::Point> lane = lanedetector.regression(left_right_lines, imgOrg);
        // Predict the turn by determining the vanishing point of the the lines
        std::string turn = lanedetector.predictTurn();
        // Plot lane detection
        imgShow = lanedetector.plotLane(imgShow, lane, turn);
    }

    return imgShow;
}

cv::Mat Yolov3_tiny(cv::Mat img)
{
    cv::Mat frame_dist = img;
    newFrame = true;
    a = DsImage(frame_dist, inferNet->getInputH(), inferNet->getInputW());
    aSet = true;
    cv::Mat trtInput = blobFromDsImage(a.getLetterBoxedImage(), inferNet->getInputH(), inferNet->getInputW());
    inferNet->doInference(trtInput.data, 1);
    auto binfo = inferNet->decodeDetections(0, a.getImageHeight(), a.getImageWidth());
    remaining = nmsAllClasses(inferNet->getNMSThresh(), binfo, inferNet->getNumClasses());
    float w, h;
    bool bBoxExist = false;
    vector<bBoxCenterStruct> bBoxCenter;
    vector<Datatample> objectFrame;

#if soloModule
    vector<road_data_to_server> objFrame_4g;
    json_obj_box data_box;
#endif

    int bboxCount = 0; // In future, this param present road module longitude and latitude.
    for (auto b : remaining)
    {
        bboxCount++;
        a.addBBox(b, inferNet->getClassName(b.label));
        w = b.box.x2 - b.box.x1;
        h = b.box.y2 - b.box.y1;
        bBoxCenter.push_back({bboxCount, b.label, w, h, b.box.x1 + w / 2, b.box.y1 + h / 2, b.box.x1, b.box.y1, min(w / 2, h / 2)});
        bBoxExist = true;
        //printf("x=%f, y=%f.\n", b.box.x1, b.box.y1);
    }
    if (bBoxExist)
    {
            cv::Point2f finalPoint;
            for (bBoxCenterStruct center : bBoxCenter)
            {
                bool bestPointExist = false;
                center_copy = center;
                float speed = 0;
                direction = 0;
                float distance, opticalFlowLength, minDistance = PREVIEW_SIZE.width() >> 1;
                unsigned minMark = 0;
                if (!points[0].empty() && !points[1].empty())
                {
                    opticalFlowFrame++;
                    for (unsigned i = 0; i < points[1].size(); i++)
                    {
                        distance = getDistance(cv::Point2i(center.xCenter, center.yCenter), points[1][i]);
                        if (distance < minDistance)
                        {
                            minDistance = distance;
                            minMark = i;
                        }
                        else
                        {
                            continue;
                        }
                    }
                    opticalFlowLength = getDistance(points[0][minMark], points[1][minMark]);
                    if (minDistance < center.radiusCenter //in bbox area
                        && opticalFlowLength < 40         //not burst
                        && opticalFlowLength > 4)         //not static
                    {                                     // Expand distance for view feeling
                        bestPointExist = true;
                        finalPoint.x = points[0][minMark].x + center.xCenter - points[1][minMark].x;
                        finalPoint.y = points[0][minMark].y + center.yCenter - points[1][minMark].y;
                        speed = getDisplacement(points[0][minMark], points[1][minMark]) * m_km_ms_hour / (tEnd - tStart);
                        speed_copy = speed;
                    }
                    if (bestPointExist)
                    {
                        cv::Point2i arrowEnd = cv::Point2i(finalPoint.x, finalPoint.y);
                        cv::Point2i arrowStart = cv::Point2i(center.xCenter * 2 - arrowEnd.x, center.yCenter * 2 - arrowEnd.y);
                        if (!newFrame)
                        {
                            arrowedLine(a.getMarkedImage(), arrowStart, arrowEnd, cv::Scalar(255, 0, 0), 2, 8, 0, 0.2);
                            cv::putText(a.getMarkedImage(), "RV: " + std::to_string(int(speed)) + "Km/h",
                                        cv::Point2i(center.xCenter, center.yCenter + 2 * tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 255, 0), 1, CV_AA);
                        }
                        bestPointExist = false;
                    }
                }
                float dist = getFarNear(center.xCenter, center.yCenter + h / 2);
                dist_copy = dist;
                if (dist > warningDist)
                {
                    cv::putText(a.getMarkedImage(), "D: " + std::to_string(dist).substr(0, std::to_string(dist).find(".") + 2) + "m",
                                cv::Point2i(center.xCenter, center.yCenter + tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(0, 255, 0), 1, CV_AA);
                }
                else
                {
                    cv::putText(a.getMarkedImage(), "D: " + std::to_string(dist).substr(0, std::to_string(dist).find(".") + 2) + "m",
                                cv::Point2i(center.xCenter, center.yCenter + tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);
                }
                objectFrame.push_back({(float)center.bboxCount, (float)center.labelId, center.x, center.y, center.h, center.w, speed, direction});
                printf("id=%d, label=%s, x=%f, y=%f, h=%f, w=%f, speed=%f km/h, direction=%f degree.\n", center.bboxCount,
                       inferNet->getClassName(center.labelId).c_str(), center.x, center.y, center.h, center.w, speed, direction);
#if soloModule
                objFrame_4g.push_back({frameCount, center.bboxCount, center.labelId, center.x, center.y, center.h, center.w, speed, direction, 0, 0});
#endif
            }
#if soloModule //4g switch, this funtion will updata the vector antomaticlly
            data_box.set_data_vehicle_and_datavector(bboxCount, objFrame_4g);
#elif roadModule
            updata(bboxCount, objectFrame);
#endif
    }
#if lane_detect
    return laneDetect(frame_dist, a.getMarkedImage());
#else
    return a.getMarkedImage();
#endif
}

cv::Mat detect_result_copy(cv::Mat img)
{
    cv::Mat frame_dist = img;
    bool bboxExist = false;
    for (BBoxInfo b : remaining)
    {
        a.addBBox(b, inferNet->getClassName(b.label));
        bboxExist = true;
    }
    if (bboxExist)
    {
        cv::putText(a.getMarkedImage(), "RV: " + std::to_string(int(speed_copy)) + "Km/h",
                    cv::Point2i(center_copy.xCenter, center_copy.yCenter + 2 * tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.7, cv::Scalar(0, 255, 0), 1, CV_AA);
        if (dist_copy > warningDist)
        {
            cv::putText(a.getMarkedImage(), "D: " + std::to_string(dist_copy).substr(0, std::to_string(dist_copy).find(".") + 2) + "m",
                        cv::Point2i(center_copy.xCenter, center_copy.yCenter + tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.6, cv::Scalar(0, 255, 0), 1, CV_AA);
        }
        else
        {
            cv::putText(a.getMarkedImage(), "D: " + std::to_string(dist_copy).substr(0, std::to_string(dist_copy).find(".") + 2) + "m",
                        cv::Point2i(center_copy.xCenter, center_copy.yCenter + tSize), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0), 1, CV_AA);
        }
    }
    return a.getMarkedImage();
}

namespace ArgusSamples
{
/*******************************************************************************
 * Data transfer thread
 ******************************************************************************/
class DataTransThread : public Thread
{
public:
    DataTransThread() : fd(uart_open((char *)port.c_str())){};

protected:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/
    pid_t fd;
};

bool DataTransThread::threadInitialize()
{
    init(fd);
    return true;
}

bool DataTransThread::threadExecute()
{
    int device_num;
    int data_nums;
    float buf[MAX_DATALEN];
    int ret;
    while (true)
    {
        ret = rdm_recv(fd, &device_num);

        if (ret == replyMecPoll && device_num == THIS_DEVICE)
        {

            rdm_send(fd, 2, THIS_DEVICE, mecReplyNoObject, NULL);
            continue;
        }
        if (ret == sendAfterPoll && device_num == THIS_DEVICE)
        {
            memcpy(&data_nums, &globle.data_num, sizeof(data_nums));
            memcpy(&buf, &globle.data_buf, sizeof(buf));
            printf("data_num:%d\n", data_nums);

            for (int t = 0; t < OBJ_DESCRIBE_ELEMENT * data_nums; t++)
            {
                printf("buf[t]:%f\n", buf[t]);
            }
            rdm_send(fd, 4, THIS_DEVICE, data_nums, buf);
            continue;
        }
    }

    return true;
}

bool DataTransThread::threadShutdown()
{
    close(fd);
    return true;
}

/*******************************************************************************
 * 4G Data transfer thread
 ******************************************************************************/
class DataTransThread_4G : public Thread
{
public:
    DataTransThread_4G() : port(tcp_client_port), ip((char *)tcp_client_ip.c_str()){};
    string data_send_buf;

protected:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/
    int port;
    char *ip;
    tcp_cli_obj link;
};

bool DataTransThread_4G::threadInitialize()
{
    link.sfd_init();
    return true;
}

bool DataTransThread_4G::threadExecute()
{
    while (true)
    {
        link.send_str = globle_str;
        link.client_send_message();
        link.send_str.clear();
    }
    return true;
}

bool DataTransThread_4G::threadShutdown()
{
    link.close_socket_fd();
    return true;
}
/*******************************************************************************
 * Base Consumer thread:
 *   Creates an EGLStream::FrameConsumer object to read frames from the
 *   OutputStream, then creates/populates an NvBuffer (dmabuf) from the frames
 *   to be processed by processV4L2Fd.
 ******************************************************************************/
class ConsumerThread : public Thread
{
public:
    explicit ConsumerThread(OutputStream *stream) : m_stream(stream),
                                                    m_dmabuf(-1)
    {
    }
    virtual ~ConsumerThread();

protected:
    /** @name Thread methods */
    /**@{*/
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();
    /**@}*/

    virtual bool processV4L2Fd(int32_t fd, uint64_t frameNumber) = 0;

    OutputStream *m_stream;
    UniqueObj<FrameConsumer> m_consumer;
    int m_dmabuf;
};

ConsumerThread::~ConsumerThread()
{
    if (m_dmabuf != -1)
        NvBufferDestroy(m_dmabuf);
}

bool ConsumerThread::threadInitialize()
{
    // Create the FrameConsumer.
    m_consumer = UniqueObj<FrameConsumer>(FrameConsumer::create(m_stream));
    if (!m_consumer)
        printf("Failed to create FrameConsumer");

    return true;
}

bool ConsumerThread::threadExecute()
{
    IEGLOutputStream *iEglOutputStream = interface_cast<IEGLOutputStream>(m_stream);
    IFrameConsumer *iFrameConsumer = interface_cast<IFrameConsumer>(m_consumer);

    // Wait until the producer has connected to the stream.
    CONSUMER_PRINT("Waiting until producer is connected...\n");
    if (iEglOutputStream->waitUntilConnected() != STATUS_OK)
        printf("Stream failed to connect.");
    CONSUMER_PRINT("Producer has connected; continuing.\n");

    while (true)
    {
        // Acquire a frame.
        UniqueObj<Frame> frame(iFrameConsumer->acquireFrame());
        IFrame *iFrame = interface_cast<IFrame>(frame);
        if (!iFrame)
            break;

        // Get the IImageNativeBuffer extension interface.
        NV::IImageNativeBuffer *iNativeBuffer =
            interface_cast<NV::IImageNativeBuffer>(iFrame->getImage());
        if (!iNativeBuffer)
            printf("IImageNativeBuffer not supported by Image.");

        // If we don't already have a buffer, create one from this image.
        // Otherwise, just blit to our buffer.
        if (m_dmabuf == -1)
        {
            m_dmabuf = iNativeBuffer->createNvBuffer(iEglOutputStream->getResolution(),
                                                     NvBufferColorFormat_ABGR32,
                                                     NvBufferLayout_Pitch);
            if (m_dmabuf == -1)
                CONSUMER_PRINT("\tFailed to create NvBuffer\n");
        }
        else if (iNativeBuffer->copyToNvBuffer(m_dmabuf) != STATUS_OK)
        {
            printf("Failed to copy frame to NvBuffer.");
        }

        // Process frame.
        processV4L2Fd(m_dmabuf, iFrame->getNumber());
    }

    CONSUMER_PRINT("Done.\n");

    requestShutdown();

    return true;
}

bool ConsumerThread::threadShutdown()
{
    return true;
}

/*******************************************************************************
 * Preview Consumer thread:
 *   Read frames from the OutputStream and render it on display.
 ******************************************************************************/
class PreviewConsumerThread : public ConsumerThread
{
public:
    PreviewConsumerThread(OutputStream *stream, NvEglRenderer *renderer);
    ~PreviewConsumerThread();

private:
    bool threadInitialize();
    bool threadShutdown();
    bool processV4L2Fd(int32_t fd, uint64_t frameNumber);

    NvEglRenderer *m_renderer;
};

PreviewConsumerThread::PreviewConsumerThread(OutputStream *stream,
                                             NvEglRenderer *renderer) : ConsumerThread(stream),
                                                                        m_renderer(renderer)
{
}

PreviewConsumerThread::~PreviewConsumerThread()
{
}

bool PreviewConsumerThread::threadInitialize()
{
    if (!ConsumerThread::threadInitialize())
        return false;

    if (DO_STAT)
        m_renderer->enableProfiling();

    return true;
}

bool PreviewConsumerThread::threadShutdown()
{
    if (DO_STAT)
        m_renderer->printProfilingStats();

    return ConsumerThread::threadShutdown();
}

bool PreviewConsumerThread::processV4L2Fd(int32_t fd, uint64_t frameNumber)
{
    cv::Mat imgOrg;
    void *pdata = NULL;

    assert(NvBufferMemMap(fd, 0, NvBufferMem_Read, &pdata) == 0);
    assert(NvBufferMemSyncForCpu(fd, 0, &pdata) == 0);

    imgOrg = cv::Mat(PREVIEW_SIZE.height(),
                             PREVIEW_SIZE.width(),
                             CV_8UC4, pdata);
    frameCount++;

    if (frameCount % frameGap == 0)
    {
        imgOrg = Yolov3_tiny(imgOrg);
    }
    else
    {
        frameCount = frameCount % CAPTURE_FPS;
        imgOrg = detect_result_copy(imgOrg);
    }
    assert(NvBufferMemUnMap(fd, 0, &pdata) == 0);
    pdata = imgOrg.data;
    Raw2NvBuffer(static_cast<unsigned char *>(pdata), 0, PREVIEW_SIZE.width(), PREVIEW_SIZE.height(), fd);
    m_renderer->render(fd);
    return true;
}

/**************************************************************************
* OpticalFlowThread
***************************************************************************/
class OpticalFlowThread : public Thread
{
public:
    OpticalFlowThread() : termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03),
                          subPixWinSize(10, 10), winSize(31, 31), MAX_COUNT(600), needToInit(true){};

protected:
    virtual bool threadInitialize();
    virtual bool threadExecute();
    virtual bool threadShutdown();

    cv::TermCriteria termcrit;
    cv::Size subPixWinSize, winSize;
    int MAX_COUNT;
    bool needToInit;
    cv::Mat img;
};

bool OpticalFlowThread::threadInitialize()
{
    return true;
}

bool OpticalFlowThread::threadExecute()
{
    cv::Mat gray, prevGray;
    while (aSet)
    {
        cvtColor(a.getMarkedImage(), gray, cv::COLOR_RGBA2GRAY);
        if (needToInit && newFrame)
        {
            goodFeaturesToTrack(gray, points[1], MAX_COUNT, 0.01, 10, cv::Mat(), 3, 3, 0, 0.04);
            cornerSubPix(gray, points[1], subPixWinSize, cv::Size(-1, -1), termcrit);
            needToInit = false;
        }
        else if (!points[0].empty() && newFrame)
        {
            vector<uchar> status;
            vector<float> err;
            if (prevGray.empty())
            {
                gray.copyTo(prevGray);
            }
            tStart = clock();
            calcOpticalFlowPyrLK(prevGray, gray, points[0], points[1], status, err, winSize,
                                3, termcrit, 0, 0.001);
            tEnd = clock();
            //cout << "Optical flow cost is: " << (double)(tEnd - tStart) / 1000 << "ms" << endl;
        }
        if (opticalFlowFrame > TZ_REFRESH)
        {
            needToInit = true;
            opticalFlowFrame = 0;
            points[0].clear();
            points[1].clear();
        }
        if (newFrame)
        {
            std::swap(points[1], points[0]);
            cv::swap(prevGray, gray);
            newFrame = false;
        }
    }
    return true;
}

bool OpticalFlowThread::threadShutdown()
{
    return true;
}

/*******************************************************************************
 * Argus Producer thread:
 *   Opens the Argus camera driver, creates two OutputStreams to output to
 *   Preview Consumer and Capture Consumer respectively, then performs repeating
 *   capture requests for CAPTURE_TIME seconds before closing the producer and
 *   Argus driver.
 ******************************************************************************/
static bool execute(NvEglRenderer *renderer)
{
    // Create the CameraProvider object and get the core interface.
    UniqueObj<CameraProvider> cameraProvider = UniqueObj<CameraProvider>(CameraProvider::create());
    ICameraProvider *iCameraProvider = interface_cast<ICameraProvider>(cameraProvider);
    if (!iCameraProvider)
        printf("Failed to create CameraProvider");

    // Get the camera devices.
    std::vector<CameraDevice *> cameraDevices;
    iCameraProvider->getCameraDevices(&cameraDevices);
    if (cameraDevices.size() == 0)
        printf("No cameras available");

    ICameraProperties *iCameraProperties = interface_cast<ICameraProperties>(cameraDevices[0]);
    if (!iCameraProperties)
        printf("Failed to get ICameraProperties interface");

    // Create the capture session using the first device and get the core interface.
    UniqueObj<CaptureSession> captureSession(
        iCameraProvider->createCaptureSession(cameraDevices[0]));
    ICaptureSession *iCaptureSession = interface_cast<ICaptureSession>(captureSession);
    if (!iCaptureSession)
        printf("Failed to get ICaptureSession interface.\n");

    // Create the OutputStream.
    PRODUCER_PRINT("Creating output stream\n");
    UniqueObj<OutputStreamSettings> streamSettings(
        iCaptureSession->createOutputStreamSettings(STREAM_TYPE_EGL));
    IEGLOutputStreamSettings *iEglStreamSettings =
        interface_cast<IEGLOutputStreamSettings>(streamSettings);
    if (!iEglStreamSettings)
        printf("Failed to get IEGLOutputStreamSettings interface");

    iEglStreamSettings->setPixelFormat(PIXEL_FMT_YCbCr_420_888);
    iEglStreamSettings->setEGLDisplay(renderer->getEGLDisplay());
    iEglStreamSettings->setResolution(PREVIEW_SIZE);
    UniqueObj<OutputStream> previewStream(iCaptureSession->createOutputStream(streamSettings.get()));

    // Launch the FrameConsumer thread to consume frames from the OutputStream.
    PRODUCER_PRINT("Launching consumer thread\n");
    PreviewConsumerThread *previewConsumerThread = new PreviewConsumerThread(previewStream.get(), renderer);
    PROPAGATE_ERROR(previewConsumerThread->initialize());

    // Wait until the consumer is connected to the stream.
    PROPAGATE_ERROR(previewConsumerThread->waitRunning());

    // Create capture request and enable output stream.
    UniqueObj<Request> request(iCaptureSession->createRequest());
    IRequest *iRequest = interface_cast<IRequest>(request);
    if (!iRequest)
        printf("Failed to create Request");
    iRequest->enableOutputStream(previewStream.get());
    ISensorMode *iSensorMode;
    std::vector<SensorMode *> sensorModes;
    iCameraProperties->getBasicSensorModes(&sensorModes);
    if (sensorModes.size() == 0)
        printf("Failed to get sensor modes");

    PRODUCER_PRINT("Available Sensor modes :\n");
    for (uint32_t i = 0; i < sensorModes.size(); i++)
    {
        iSensorMode = interface_cast<ISensorMode>(sensorModes[i]);
        Size2D<uint32_t> resolution = iSensorMode->getResolution();
        PRODUCER_PRINT("[%u] W=%u H=%u\n", i, resolution.width(), resolution.height());
    }

    ISourceSettings *iSourceSettings = interface_cast<ISourceSettings>(iRequest->getSourceSettings());
    if (!iSourceSettings)
        printf("Failed to get ISourceSettings interface");

    // Check sensor mode index
    if (SENSOR_MODE >= sensorModes.size())
        printf("Sensor mode index is out of range");
    SensorMode *sensorMode = sensorModes[SENSOR_MODE];
    iSensorMode = interface_cast<ISensorMode>(sensorMode);
    iSourceSettings->setSensorMode(sensorMode);

    // Check fps
    Range<uint64_t> sensorDuration(iSensorMode->getFrameDurationRange());
    Range<uint64_t> desireDuration(1e9 / CAPTURE_FPS + 0.9);
    if (desireDuration.min() < sensorDuration.min() ||
        desireDuration.max() > sensorDuration.max())
    {
        PRODUCER_PRINT("Requested FPS out of range. Fall back to 30\n");
        CAPTURE_FPS = 30;
    }
    iSourceSettings->setFrameDurationRange(Range<uint64_t>(1e9 / CAPTURE_FPS));
    renderer->setFPS((float)CAPTURE_FPS);
    //PRODUCER_PRINT("Set FPS: %f\n", renderer->getFPS());// Remind: update NvEglRenderer.h for getFPS() method.

    // Submit capture requests.
    PRODUCER_PRINT("Starting repeat capture requests.\n");
    if (iCaptureSession->repeat(request.get()) != STATUS_OK)
        printf("Failed to start repeat capture request");
    OpticalFlowThread *OpticalFlow = new OpticalFlowThread();
    PROPAGATE_ERROR(OpticalFlow->initialize());
// Create thread for data send/receive
#if roadModule
    DataTransThread *DataTrans = new DataTransThread();
    PROPAGATE_ERROR(DataTrans->initialize());
#elif soloModule
    DataTransThread_4G *DataTrans_4G = new DataTransThread_4G();
    PROPAGATE_ERROR(DataTrans_4G->initialize());
#endif

    // Keep running
    while (getchar() != 's')
    {
    }

    // Stop the repeating request and wait for idle.
    iCaptureSession->stopRepeat();
    iCaptureSession->waitForIdle();

#if roadModule
    PROPAGATE_ERROR(DataTrans->shutdown());
    delete DataTrans;
#elif soloModule
    PROPAGATE_ERROR(DataTrans_4G->shutdown());
    delete DataTrans_4G;
#endif
    PROPAGATE_ERROR(OpticalFlow->shutdown());
    delete OpticalFlow;
    // Destroy the output stream to end the consumer thread.
    previewStream.reset();

    // Wait for the consumer thread to complete.
    PROPAGATE_ERROR(previewConsumerThread->shutdown());

    delete previewConsumerThread;
    PRODUCER_PRINT("Done -- exiting.\n");

    return true;
}

}; // namespace ArgusSamples
