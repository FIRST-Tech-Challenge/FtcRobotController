#include <chrono>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"", "person"};

static std::atomic<bool> fullFrameTracking{false};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    std::string nnPath(BLOB_PATH);
    std::string videoPath(VIDEO_PATH);

    // If path to blob specified, use that
    if(argc > 2) {
        nnPath = std::string(argv[1]);
        videoPath = std::string(argv[2]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());
    printf("Using video at path: %s\n", videoPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();

    auto manipOut = pipeline.create<dai::node::XLinkOut>();
    auto xinFrame = pipeline.create<dai::node::XLinkIn>();
    auto trackerOut = pipeline.create<dai::node::XLinkOut>();
    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    manipOut->setStreamName("manip");
    xinFrame->setStreamName("inFrame");
    xlinkOut->setStreamName("trackerFrame");
    trackerOut->setStreamName("tracklets");
    nnOut->setStreamName("nn");

    // Properties
    xinFrame->setMaxDataSize(1920 * 1080 * 3);

    manip->initialConfig.setResizeThumbnail(544, 320);
    // manip->initialConfig.setResize(384, 384);
    // manip->initialConfig.setKeepAspectRatio(false); //squash the image to not lose FOV
    // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    manip->inputImage.setBlocking(true);

    // setting node configs
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setConfidenceThreshold(0.5);
    detectionNetwork->input.setBlocking(true);

    objectTracker->inputTrackerFrame.setBlocking(true);
    objectTracker->inputDetectionFrame.setBlocking(true);
    objectTracker->inputDetections.setBlocking(true);
    objectTracker->setDetectionLabelsToTrack({1});  // track only person
    // possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    // take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    // Linking
    manip->out.link(manipOut->input);
    manip->out.link(detectionNetwork->input);
    xinFrame->out.link(manip->inputImage);
    xinFrame->out.link(objectTracker->inputTrackerFrame);
    detectionNetwork->out.link(nnOut->input);
    detectionNetwork->out.link(objectTracker->inputDetections);
    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    objectTracker->out.link(trackerOut->input);
    objectTracker->passthroughTrackerFrame.link(xlinkOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto qIn = device.getInputQueue("inFrame", 4);
    auto trackerFrameQ = device.getOutputQueue("trackerFrame", 4);
    auto tracklets = device.getOutputQueue("tracklets", 4);
    auto qManip = device.getOutputQueue("manip", 4);
    auto qDet = device.getOutputQueue("nn", 4);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;
    cv::Mat frame;
    cv::Mat manipFrame;
    std::vector<dai::ImgDetection> detections;

    // Add bounding boxes and text to the frame and show it to the user
    auto displayFrame = [](std::string name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 0, 0);
        // nn data, being the bounding box locations, are in <0..1> range - they need to be normalized with frame width/height
        for(auto& detection : detections) {
            int x1 = detection.xmin * frame.cols;
            int y1 = detection.ymin * frame.rows;
            int x2 = detection.xmax * frame.cols;
            int y2 = detection.ymax * frame.rows;

            uint32_t labelIndex = detection.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream confStr;
            confStr << std::fixed << std::setprecision(2) << detection.confidence * 100;
            cv::putText(frame, confStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }
        // Show the frame
        cv::imshow(name, frame);
    };

    cv::VideoCapture cap(videoPath);
    auto baseTs = steady_clock::now();
    float simulatedFps = 30;

    while(cap.isOpened()) {
        // Read frame from video
        cap >> frame;
        if(frame.empty()) break;

        auto img = std::make_shared<dai::ImgFrame>();
        frame = resizeKeepAspectRatio(frame, cv::Size(1920, 1080), cv::Scalar(0));
        toPlanar(frame, img->getData());
        img->setTimestamp(baseTs);
        baseTs += steady_clock::duration(static_cast<int64_t>((1000 * 1000 * 1000 / simulatedFps)));
        img->setWidth(1920);
        img->setHeight(1080);
        img->setType(dai::ImgFrame::Type::BGR888p);
        qIn->send(img);

        auto trackFrame = trackerFrameQ->tryGet<dai::ImgFrame>();
        if(!trackFrame) {
            continue;
        }

        auto track = tracklets->get<dai::Tracklets>();
        auto inManip = qManip->get<dai::ImgFrame>();
        auto inDet = qDet->get<dai::ImgDetections>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        detections = inDet->detections;
        manipFrame = inManip->getCvFrame();
        displayFrame("nn", manipFrame, detections);

        auto color = cv::Scalar(255, 0, 0);
        cv::Mat trackerFrame = trackFrame->getCvFrame();
        auto trackletsData = track->tracklets;
        for(auto& t : trackletsData) {
            auto roi = t.roi.denormalize(trackerFrame.cols, trackerFrame.rows);
            int x1 = roi.topLeft().x;
            int y1 = roi.topLeft().y;
            int x2 = roi.bottomRight().x;
            int y2 = roi.bottomRight().y;

            uint32_t labelIndex = t.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(trackerFrame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream idStr;
            idStr << "ID: " << t.id;
            cv::putText(trackerFrame, idStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream statusStr;
            statusStr << "Status: " << t.status;
            cv::putText(trackerFrame, statusStr.str(), cv::Point(x1 + 10, y1 + 60), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(trackerFrame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << "NN fps:" << std::fixed << std::setprecision(2) << fps;
        cv::putText(trackerFrame, fpsStr.str(), cv::Point(2, trackFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("tracker", trackerFrame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
