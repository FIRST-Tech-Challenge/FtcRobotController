#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// MobilenetSSD label texts
static const std::vector<std::string> labelMap = {"background", "aeroplane", "bicycle",     "bird",  "boat",        "bottle", "bus",
                                                  "car",        "cat",       "chair",       "cow",   "diningtable", "dog",    "horse",
                                                  "motorbike",  "person",    "pottedplant", "sheep", "sofa",        "train",  "tvmonitor"};

int main(int argc, char** argv) {
    using namespace std;
    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
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
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();

    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");
    xoutPreview->setStreamName("preview");
    nnOut->setStreamName("nn");

    // Properties
    camRgb->setPreviewSize(300, 300);  // NN input
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    camRgb->setInterleaved(false);
    camRgb->setPreviewKeepAspectRatio(false);
    // Define a neural network that will make predictions based on the source frames
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    // Linking
    camRgb->video.link(xoutVideo->input);
    camRgb->preview.link(xoutPreview->input);
    camRgb->preview.link(nn->input);
    nn->out.link(nnOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the frames and nn data from the outputs defined above
    auto qVideo = device.getOutputQueue("video", 4, false);
    auto qPreview = device.getOutputQueue("preview", 4, false);
    auto qDet = device.getOutputQueue("nn", 4, false);

    cv::Mat previewFrame;
    cv::Mat videoFrame;
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

    cv::namedWindow("video", cv::WINDOW_NORMAL);
    cv::resizeWindow("video", 1280, 720);
    cout << "Resize video window with mouse drag!" << endl;

    while(true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        auto inVideo = qVideo->tryGet<dai::ImgFrame>();
        auto inPreview = qPreview->tryGet<dai::ImgFrame>();
        auto inDet = qDet->tryGet<dai::ImgDetections>();

        if(inVideo) {
            videoFrame = inVideo->getCvFrame();
        }

        if(inPreview) {
            previewFrame = inPreview->getCvFrame();
        }

        if(inDet) {
            detections = inDet->detections;
        }

        if(!videoFrame.empty()) {
            displayFrame("video", videoFrame, detections);
        }

        if(!previewFrame.empty()) {
            displayFrame("preview", previewFrame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
