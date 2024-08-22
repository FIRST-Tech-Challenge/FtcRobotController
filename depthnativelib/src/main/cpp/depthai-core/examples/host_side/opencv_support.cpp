#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Include OpenCV
#include <opencv2/opencv.hpp>

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto xoutVideo = pipeline.create<dai::node::XLinkOut>();
    auto xoutPreview = pipeline.create<dai::node::XLinkOut>();

    xoutVideo->setStreamName("video");
    xoutPreview->setStreamName("preview");

    // Properties
    camRgb->setPreviewSize(300, 300);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setInterleaved(true);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    // Linking
    camRgb->video.link(xoutVideo->input);
    camRgb->preview.link(xoutPreview->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto video = device.getOutputQueue("video");
    auto preview = device.getOutputQueue("preview");

    while(true) {
        auto videoFrame = video->get<dai::ImgFrame>();
        auto previewFrame = preview->get<dai::ImgFrame>();

        // Get BGR frame from NV12 encoded video frame to show with opencv
        cv::imshow("video", videoFrame->getCvFrame());

        // Show 'preview' frame as is (already in correct format, no copy is made)
        cv::imshow("preview", previewFrame->getFrame());

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}
