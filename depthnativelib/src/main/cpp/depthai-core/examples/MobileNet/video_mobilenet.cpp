#include <chrono>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

int main(int argc, char** argv) {
    using namespace std;
    using namespace std::chrono;
    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
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

    // Define source and outputs
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();

    auto xinFrame = pipeline.create<dai::node::XLinkIn>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xinFrame->setStreamName("inFrame");
    nnOut->setStreamName("nn");

    // Properties
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    // Linking
    xinFrame->out.link(nn->input);
    nn->out.link(nnOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Input queue will be used to send video frames to the device.
    auto qIn = device.getInputQueue("inFrame");
    // Output queue will be used to get nn data from the video frames.
    auto qDet = device.getOutputQueue("nn", 4, false);

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

    cv::Mat frame;
    cv::VideoCapture cap(videoPath);

    cv::namedWindow("inFrame", cv::WINDOW_NORMAL);
    cv::resizeWindow("inFrame", 1280, 720);
    std::cout << "Resize video window with mouse drag!" << std::endl;

    while(cap.isOpened()) {
        // Read frame from video
        cap >> frame;
        if(frame.empty()) break;

        auto img = std::make_shared<dai::ImgFrame>();
        frame = resizeKeepAspectRatio(frame, cv::Size(300, 300), cv::Scalar(0));
        toPlanar(frame, img->getData());
        img->setTimestamp(steady_clock::now());
        img->setWidth(300);
        img->setHeight(300);
        qIn->send(img);

        auto inDet = qDet->get<dai::ImgDetections>();
        auto detections = inDet->detections;

        displayFrame("inFrame", frame, detections);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}
