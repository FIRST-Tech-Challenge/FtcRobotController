#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

static std::atomic<bool> fullFrameTracking{false};

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
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto detectionNetwork = pipeline.create<dai::node::MobileNetDetectionNetwork>();
    auto objectTracker = pipeline.create<dai::node::ObjectTracker>();

    auto xlinkOut = pipeline.create<dai::node::XLinkOut>();
    auto trackerOut = pipeline.create<dai::node::XLinkOut>();

    xlinkOut->setStreamName("preview");
    trackerOut->setStreamName("tracklets");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
    camRgb->setFps(40);

    // testing MobileNet DetectionNetwork
    detectionNetwork->setBlobPath(nnPath);
    detectionNetwork->setConfidenceThreshold(0.5f);
    detectionNetwork->input.setBlocking(false);

    objectTracker->setDetectionLabelsToTrack({15});  // track only person
    // possible tracking types: ZERO_TERM_COLOR_HISTOGRAM, ZERO_TERM_IMAGELESS, SHORT_TERM_IMAGELESS, SHORT_TERM_KCF
    objectTracker->setTrackerType(dai::TrackerType::ZERO_TERM_COLOR_HISTOGRAM);
    // take the smallest ID when new object is tracked, possible options: SMALLEST_ID, UNIQUE_ID
    objectTracker->setTrackerIdAssignmentPolicy(dai::TrackerIdAssignmentPolicy::SMALLEST_ID);

    // Linking
    camRgb->preview.link(detectionNetwork->input);
    objectTracker->passthroughTrackerFrame.link(xlinkOut->input);

    if(fullFrameTracking) {
        camRgb->video.link(objectTracker->inputTrackerFrame);
    } else {
        detectionNetwork->passthrough.link(objectTracker->inputTrackerFrame);
    }

    detectionNetwork->passthrough.link(objectTracker->inputDetectionFrame);
    detectionNetwork->out.link(objectTracker->inputDetections);
    objectTracker->out.link(trackerOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto preview = device.getOutputQueue("preview", 4, false);
    auto tracklets = device.getOutputQueue("tracklets", 4, false);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        auto imgFrame = preview->get<dai::ImgFrame>();
        auto track = tracklets->get<dai::Tracklets>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        auto color = cv::Scalar(255, 0, 0);
        cv::Mat frame = imgFrame->getCvFrame();
        auto trackletsData = track->tracklets;
        for(auto& t : trackletsData) {
            auto roi = t.roi.denormalize(frame.cols, frame.rows);
            int x1 = roi.topLeft().x;
            int y1 = roi.topLeft().y;
            int x2 = roi.bottomRight().x;
            int y2 = roi.bottomRight().y;

            uint32_t labelIndex = t.label;
            std::string labelStr = to_string(labelIndex);
            if(labelIndex < labelMap.size()) {
                labelStr = labelMap[labelIndex];
            }
            cv::putText(frame, labelStr, cv::Point(x1 + 10, y1 + 20), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            std::stringstream idStr;
            idStr << "ID: " << t.id;
            cv::putText(frame, idStr.str(), cv::Point(x1 + 10, y1 + 40), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
            std::stringstream statusStr;
            statusStr << "Status: " << t.status;
            cv::putText(frame, statusStr.str(), cv::Point(x1 + 10, y1 + 60), cv::FONT_HERSHEY_TRIPLEX, 0.5, color);

            cv::rectangle(frame, cv::Rect(cv::Point(x1, y1), cv::Point(x2, y2)), color, cv::FONT_HERSHEY_SIMPLEX);
        }

        std::stringstream fpsStr;
        fpsStr << "NN fps:" << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, imgFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("tracker", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
