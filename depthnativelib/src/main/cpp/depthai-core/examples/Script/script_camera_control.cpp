#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Start defining a pipeline
    dai::Pipeline pipeline;

    // Define a source - color camera
    auto colorCam = pipeline.create<dai::node::ColorCamera>();

    // Script node
    auto script = pipeline.create<dai::node::Script>();
    script->setScript(R"(
        import time
        ctrl = CameraControl()
        ctrl.setCaptureStill(True)
        while True:
            time.sleep(1)
            node.io['out'].send(ctrl)
    )");

    // XLinkOut
    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("still");

    // Connections
    script->outputs["out"].link(colorCam->inputControl);
    colorCam->still.link(xout->input);

    // Connect to device with pipeline
    dai::Device device(pipeline);
    while(true) {
        auto img = device.getOutputQueue("still")->get<dai::ImgFrame>();
        cv::imshow("still", img->getCvFrame());
        if(cv::waitKey(1) == 'q') {
            break;
        }
    }

    return 0;
}
