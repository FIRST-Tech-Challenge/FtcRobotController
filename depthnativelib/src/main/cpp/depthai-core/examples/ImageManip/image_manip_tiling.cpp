#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(1000, 500);
    camRgb->setInterleaved(false);
    auto maxFrameSize = camRgb->getPreviewHeight() * camRgb->getPreviewHeight() * 3;

    // In this example we use 2 imageManips for splitting the original 1000x500
    // preview frame into 2 500x500 frames
    auto manip1 = pipeline.create<dai::node::ImageManip>();
    manip1->initialConfig.setCropRect(0, 0, 0.5, 1);
    // Flip functionality
    manip1->initialConfig.setHorizontalFlip(true);
    manip1->setMaxOutputFrameSize(maxFrameSize);
    camRgb->preview.link(manip1->inputImage);

    auto manip2 = pipeline.create<dai::node::ImageManip>();
    manip2->initialConfig.setCropRect(0.5, 0, 1, 1);
    // Flip functionality
    manip1->initialConfig.setVerticalFlip(true);
    manip2->setMaxOutputFrameSize(maxFrameSize);
    camRgb->preview.link(manip2->inputImage);

    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("out1");
    manip1->out.link(xout1->input);

    auto xout2 = pipeline.create<dai::node::XLinkOut>();
    xout2->setStreamName("out2");
    manip2->out.link(xout2->input);

    dai::Device device(pipeline);

    auto q1 = device.getOutputQueue("out1", 8, false);
    auto q2 = device.getOutputQueue("out2", 8, false);

    while(true) {
        auto in1 = q1->tryGet<dai::ImgFrame>();
        if(in1) {
            cv::imshow("Tile 1", in1->getCvFrame());
        }

        auto in2 = q2->tryGet<dai::ImgFrame>();
        if(in2) {
            cv::imshow("Tile 2", in2->getCvFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}