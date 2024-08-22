#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();

    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");

    // Properties
    monoLeft->setCamera("left");
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamera("right");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);

    // Linking
    monoRight->out.link(xoutRight->input);
    monoLeft->out.link(xoutLeft->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the grayscale frames from the outputs defined above
    auto qLeft = device.getOutputQueue("left", 4, false);
    auto qRight = device.getOutputQueue("right", 4, false);

    while(true) {
        // Instead of get (blocking), we use tryGet (non-blocking) which will return the available data or None otherwise
        auto inLeft = qLeft->tryGet<dai::ImgFrame>();
        auto inRight = qRight->tryGet<dai::ImgFrame>();

        if(inLeft) {
            cv::imshow("left", inLeft->getCvFrame());
        }

        if(inRight) {
            cv::imshow("right", inRight->getCvFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
