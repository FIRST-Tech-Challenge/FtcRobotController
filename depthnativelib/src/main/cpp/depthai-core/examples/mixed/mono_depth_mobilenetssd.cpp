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
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto nn = pipeline.create<dai::node::MobileNetDetectionNetwork>();

    auto disparityOut = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto nnOut = pipeline.create<dai::node::XLinkOut>();

    disparityOut->setStreamName("disparity");
    xoutRight->setStreamName("rectifiedRight");
    nnOut->setStreamName("nn");

    // Properties
    monoRight->setCamera("right");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    // Produce the depth map (using disparity output as it's easier to visualize depth this way)
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setRectifyEdgeFillColor(0);  // Black, to better see the cutout from rectification (black stripe on the edges)
    // Convert the grayscale frame into the nn-acceptable form
    manip->initialConfig.setResize(300, 300);
    // The NN model expects BGR input. By default ImageManip output type would be same as input (gray in this case)
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);

    // Define a neural network that will make predictions based on the source frames
    nn->setConfidenceThreshold(0.5);
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);
    nn->input.setBlocking(false);

    // Linking
    monoRight->out.link(stereo->right);
    monoLeft->out.link(stereo->left);
    stereo->rectifiedRight.link(manip->inputImage);
    stereo->disparity.link(disparityOut->input);
    manip->out.link(nn->input);
    manip->out.link(xoutRight->input);
    nn->out.link(nnOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the grayscale / depth frames and nn data from the outputs defined above
    auto qRight = device.getOutputQueue("rectifiedRight", 4, false);
    auto qDisparity = device.getOutputQueue("disparity", 4, false);
    auto qDet = device.getOutputQueue("nn", 4, false);

    cv::Mat rightFrame;
    cv::Mat disparityFrame;
    std::vector<dai::ImgDetection> detections;

    // Add bounding boxes and text to the frame and show it to the user
    auto show = [](std::string name, cv::Mat frame, std::vector<dai::ImgDetection>& detections) {
        auto color = cv::Scalar(255, 192, 203);
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

    float disparityMultiplier = 255 / stereo->initialConfig.getMaxDisparity();

    while(true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        auto inRight = qRight->tryGet<dai::ImgFrame>();
        auto inDet = qDet->tryGet<dai::ImgDetections>();
        auto inDisparity = qDisparity->tryGet<dai::ImgFrame>();

        if(inDisparity) {
            // Frame is transformed, normalized, and color map will be applied to highlight the depth info
            disparityFrame = inDisparity->getFrame();
            disparityFrame.convertTo(disparityFrame, CV_8UC1, disparityMultiplier);
            // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
            cv::applyColorMap(disparityFrame, disparityFrame, cv::COLORMAP_JET);
            show("disparity", disparityFrame, detections);
        }

        if(!rightFrame.empty()) {
            show("rectified right", rightFrame, detections);
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}
