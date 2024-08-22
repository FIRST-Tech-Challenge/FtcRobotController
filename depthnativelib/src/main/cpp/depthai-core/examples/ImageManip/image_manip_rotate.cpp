#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Rotate color frames
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(640, 400);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);

    auto manipRgb = pipeline.create<dai::node::ImageManip>();
    dai::RotatedRect rgbRr = {{camRgb->getPreviewWidth() / 2.0f, camRgb->getPreviewHeight() / 2.0f},  // center
                              {camRgb->getPreviewHeight() * 1.0f, camRgb->getPreviewWidth() * 1.0f},  // size
                              90};                                                                    // angle
    manipRgb->initialConfig.setCropRotatedRect(rgbRr, false);
    camRgb->preview.link(manipRgb->inputImage);

    auto manipRgbOut = pipeline.create<dai::node::XLinkOut>();
    manipRgbOut->setStreamName("manip_rgb");
    manipRgb->out.link(manipRgbOut->input);

    // Rotate mono frames
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");

    auto manipLeft = pipeline.create<dai::node::ImageManip>();
    dai::RotatedRect rr = {{monoLeft->getResolutionWidth() / 2.0f, monoLeft->getResolutionHeight() / 2.0f},  // center
                           {monoLeft->getResolutionHeight() * 1.0f, monoLeft->getResolutionWidth() * 1.0f},  // size
                           90};                                                                              // angle
    manipLeft->initialConfig.setCropRotatedRect(rr, false);
    monoLeft->out.link(manipLeft->inputImage);

    auto manipLeftOut = pipeline.create<dai::node::XLinkOut>();
    manipLeftOut->setStreamName("manip_left");
    manipLeft->out.link(manipLeftOut->input);

    dai::Device device(pipeline);

    auto qLeft = device.getOutputQueue("manip_left", 8, false);
    auto qRgb = device.getOutputQueue("manip_rgb", 8, false);

    while(true) {
        auto inLeft = qLeft->tryGet<dai::ImgFrame>();
        if(inLeft) {
            cv::imshow("Left rotated", inLeft->getCvFrame());
        }

        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if(inRgb) {
            cv::imshow("Color rotated", inRgb->getCvFrame());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}