#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> syncNN{true};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialDetectionNetwork = pipeline.create<dai::node::MobileNetSpatialDetectionNetwork>();
    auto imageManip = pipeline.create<dai::node::ImageManip>();

    auto xoutManip = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();

    xoutManip->setStreamName("right");
    nnOut->setStreamName("detections");
    xoutDepth->setStreamName("depth");

    // Properties
    imageManip->initialConfig.setResize(300, 300);
    // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    imageManip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    // StereoDepth
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

    // Define a neural network that will make predictions based on the source frames
    spatialDetectionNetwork->setConfidenceThreshold(0.5f);
    spatialDetectionNetwork->setBlobPath(nnPath);
    spatialDetectionNetwork->input.setBlocking(false);
    spatialDetectionNetwork->setBoundingBoxScaleFactor(0.5);
    spatialDetectionNetwork->setDepthLowerThreshold(100);
    spatialDetectionNetwork->setDepthUpperThreshold(5000);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    imageManip->out.link(spatialDetectionNetwork->input);
    if(syncNN) {
        spatialDetectionNetwork->passthrough.link(xoutManip->input);
    } else {
        imageManip->out.link(xoutManip->input);
    }

    spatialDetectionNetwork->out.link(nnOut->input);

    stereo->rectifiedRight.link(imageManip->inputImage);
    stereo->depth.link(spatialDetectionNetwork->inputDepth);
    spatialDetectionNetwork->passthroughDepth.link(xoutDepth->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto previewQueue = device.getOutputQueue("right", 4, false);
    auto detectionNNQueue = device.getOutputQueue("detections", 4, false);
    auto depthQueue = device.getOutputQueue("depth", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    auto color = cv::Scalar(255, 255, 255);

    while(true) {
        auto inRectified = previewQueue->get<dai::ImgFrame>();
        auto inDet = detectionNNQueue->get<dai::SpatialImgDetections>();
        auto inDepth = depthQueue->get<dai::ImgFrame>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat rectifiedRight = inRectified->getCvFrame();

        cv::Mat depthFrame = inDepth->getFrame();  // depthFrame values are in millimeters
        cv::Mat depthFrameColor;

        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        auto detections = inDet->detections;

        for(auto& detection : detections) {
            auto roiData = detection.boundingBoxMapping;
            auto roi = roiData.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);
            auto topLeft = roi.topLeft();
            auto bottomRight = roi.bottomRight();
            auto xmin = (int)topLeft.x;
            auto ymin = (int)topLeft.y;
            auto xmax = (int)bottomRight.x;
            auto ymax = (int)bottomRight.y;
            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color, cv::FONT_HERSHEY_SIMPLEX);

            int x1 = detection.xmin * rectifiedRight.cols;
            int y1 = detection.ymin * rectifiedRight.rows;
            int x2 = detection.xmax * rectifiedRight.cols;
            int y2 = detection.ymax * rectifiedRight.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(rectifiedRight, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(rectifiedRight, confStr.str(), cv::Point(x1 + 10, y1 + 35), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            std::stringstream depthX;
            depthX << "X: " << (int)detection.spatialCoordinates.x << " mm";
            cv::putText(rectifiedRight, depthX.str(), cv::Point(x1 + 10, y1 + 50), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthY;
            depthY << "Y: " << (int)detection.spatialCoordinates.y << " mm";
            cv::putText(rectifiedRight, depthY.str(), cv::Point(x1 + 10, y1 + 65), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);
            std::stringstream depthZ;
            depthZ << "Z: " << (int)detection.spatialCoordinates.z << " mm";
            cv::putText(rectifiedRight, depthZ.str(), cv::Point(x1 + 10, y1 + 80), cv::FONT_HERSHEY_TRIPLEX, 0.5, 255);

            cv::rectangle(rectifiedRight, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << std::fixed << std::setprecision(2) << fps;
        cv::putText(rectifiedRight, fpsStr.str(), cv::Point(2, rectifiedRight.rows - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("depth", depthFrameColor);
        cv::imshow("rectified right", rectifiedRight);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
