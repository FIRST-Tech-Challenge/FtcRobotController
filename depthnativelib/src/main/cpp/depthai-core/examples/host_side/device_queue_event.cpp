#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto camMono = pipeline.create<dai::node::MonoCamera>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();
    auto xoutMono = pipeline.create<dai::node::XLinkOut>();

    xoutRgb->setStreamName("rgb");
    xoutMono->setStreamName("mono");

    // Properties
    camRgb->setInterleaved(true);
    camRgb->setPreviewSize(300, 300);

    // Linking
    camRgb->preview.link(xoutRgb->input);
    camMono->out.link(xoutMono->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Clear queue events
    device.getQueueEvents();

    while(true) {
        auto ev = device.getQueueEvent();

        if(ev == "rgb") {
            auto rgb = device.getOutputQueue(ev)->get<dai::ImgFrame>();
            cv::imshow("rgb", rgb->getFrame());
        } else if(ev == "mono") {
            auto mono = device.getOutputQueue(ev)->get<dai::ImgFrame>();
            cv::imshow("mono", mono->getFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
