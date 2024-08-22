#include <cstdio>
#include <iostream>
#include <string>

// Includes common necessary includes for development using depthai library
#include "depthai-shared/common/CameraBoardSocket.hpp"
#include "depthai-shared/common/EepromData.hpp"
#include "depthai/depthai.hpp"

int main(int argc, char** argv) {
    std::string calibJsonFile(CALIB_PATH);
    if(argc > 1) {
        calibJsonFile = std::string(argv[1]);
    }
    dai::CalibrationHandler calibData(calibJsonFile);

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setCalibrationData(calibData);

    // Define sources and output
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    xoutDepth->setStreamName("depth");

    // MonoCamera
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setCamera("left");
    // monoLeft->setFps(5.0);
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamera("right");
    // monoRight->setFps(5.0);

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);
    stereo->depth.link(xoutDepth->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto depthQueue = device.getOutputQueue("depth", 4, false);

    while(true) {
        // blocking call, will wait until a new data has arrived
        auto inDepth = depthQueue->get<dai::ImgFrame>();
        cv::Mat frame = cv::Mat(inDepth->getHeight(), inDepth->getWidth(), CV_16UC1, inDepth->getData().data());

        // frame is ready to be shown
        cv::imshow("depth", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}
