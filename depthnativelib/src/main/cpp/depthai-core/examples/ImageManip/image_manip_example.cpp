#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto imageManip = pipeline.create<dai::node::ImageManip>();
    auto imageManip2 = pipeline.create<dai::node::ImageManip>();
    auto camOut = pipeline.create<dai::node::XLinkOut>();
    auto manipOut = pipeline.create<dai::node::XLinkOut>();
    auto manipOut2 = pipeline.create<dai::node::XLinkOut>();
    auto manip2In = pipeline.create<dai::node::XLinkIn>();

    camOut->setStreamName("preview");
    manipOut->setStreamName("manip");
    manipOut2->setStreamName("manip2");
    manip2In->setStreamName("manip2In");

    // Properties
    camRgb->setPreviewSize(304, 304);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Create a center crop image manipulation
    imageManip->initialConfig.setCenterCrop(0.7f);
    imageManip->initialConfig.setResizeThumbnail(300, 400);

    // Second image manipulator - Create a off center crop
    imageManip2->initialConfig.setCropRect(0.1f, 0.1f, 0.3f, 0.3f);
    imageManip2->inputConfig.setWaitForMessage(true);

    // Linking
    camRgb->preview.link(camOut->input);
    camRgb->preview.link(imageManip->inputImage);
    imageManip->out.link(manipOut->input);
    imageManip->out.link(imageManip2->inputImage);
    imageManip2->out.link(manipOut2->input);
    manip2In->out.link(imageManip2->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Create input & output queues
    auto previewQueue = device.getOutputQueue("preview", 8, false);
    auto manipQueue = device.getOutputQueue("manip", 8, false);
    auto manipQueue2 = device.getOutputQueue("manip2", 8, false);
    auto manip2InQueue = device.getInputQueue("manip2In");

    // keep processing data
    int frameCounter = 0;
    float xmin = 0.1f;
    float xmax = 0.3f;

    while(true) {
        xmin += 0.003f;
        xmax += 0.003f;
        if(xmax >= 1.0f) {
            xmin = 0.0f;
            xmax = 0.2f;
        }

        dai::ImageManipConfig cfg;
        cfg.setCropRect(xmin, 0.1f, xmax, 0.3f);
        manip2InQueue->send(cfg);

        // Gets both image frames
        auto preview = previewQueue->get<dai::ImgFrame>();
        auto manip = manipQueue->get<dai::ImgFrame>();
        auto manip2 = manipQueue2->get<dai::ImgFrame>();

        auto matPreview = toMat(preview->getData(), preview->getWidth(), preview->getHeight(), 3, 1);
        auto matManip = toMat(manip->getData(), manip->getWidth(), manip->getHeight(), 3, 1);
        auto matManip2 = toMat(manip2->getData(), manip2->getWidth(), manip2->getHeight(), 3, 1);

        // Display both
        cv::imshow("preview", matPreview);
        cv::imshow("manip", matManip);
        cv::imshow("manip2", matManip2);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }

        frameCounter++;
    }
    return 0;
}