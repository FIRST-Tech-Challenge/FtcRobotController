#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Define a source - color camera
    auto cam = pipeline.create<dai::node::ColorCamera>();
    // Not needed, you can display 1080P frames as well
    cam->setIspScale(1, 2);

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
    ctrl = CameraControl()
    ctrl.setCaptureStill(True)
    # Initially send still event
    node.io['ctrl'].send(ctrl)

    normal = True
    while True:
        frame = node.io['frames'].get()
        if normal:
            ctrl.setAutoExposureCompensation(3)
            node.io['stream1'].send(frame)
            normal = False
        else:
            ctrl.setAutoExposureCompensation(-3)
            node.io['stream2'].send(frame)
            normal = True
        node.io['ctrl'].send(ctrl)
    )");

    cam->still.link(script->inputs["frames"]);

    // XLinkOut
    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("stream1");
    script->outputs["stream1"].link(xout1->input);

    auto xout2 = pipeline.create<dai::node::XLinkOut>();
    xout2->setStreamName("stream2");
    script->outputs["stream2"].link(xout2->input);

    // Connections
    script->outputs["ctrl"].link(cam->inputControl);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    auto qStream1 = device.getOutputQueue("stream1");
    auto qStream2 = device.getOutputQueue("stream2");
    while(true) {
        cv::imshow("stream1", qStream1->get<dai::ImgFrame>()->getCvFrame());
        cv::imshow("stream2", qStream2->get<dai::ImgFrame>()->getCvFrame());
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }
}