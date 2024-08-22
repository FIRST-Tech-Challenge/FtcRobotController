#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();

    auto edgeDetectorLeft = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRight = pipeline.create<dai::node::EdgeDetector>();
    auto edgeDetectorRgb = pipeline.create<dai::node::EdgeDetector>();

    auto xoutEdgeLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutEdgeRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutEdgeRgb = pipeline.create<dai::node::XLinkOut>();
    auto xinEdgeCfg = pipeline.create<dai::node::XLinkIn>();

    const auto edgeLeftStr = "edge left";
    const auto edgeRightStr = "edge right";
    const auto edgeRgbStr = "edge rgb";
    const auto edgeCfgStr = "edge cfg";

    xoutEdgeLeft->setStreamName(edgeLeftStr);
    xoutEdgeRight->setStreamName(edgeRightStr);
    xoutEdgeRgb->setStreamName(edgeRgbStr);
    xinEdgeCfg->setStreamName(edgeCfgStr);

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);

    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    edgeDetectorRgb->setMaxOutputFrameSize(camRgb->getVideoWidth() * camRgb->getVideoHeight());

    // Linking
    monoLeft->out.link(edgeDetectorLeft->inputImage);
    monoRight->out.link(edgeDetectorRight->inputImage);
    camRgb->video.link(edgeDetectorRgb->inputImage);

    edgeDetectorLeft->outputImage.link(xoutEdgeLeft->input);
    edgeDetectorRight->outputImage.link(xoutEdgeRight->input);
    edgeDetectorRgb->outputImage.link(xoutEdgeRgb->input);

    xinEdgeCfg->out.link(edgeDetectorLeft->inputConfig);
    xinEdgeCfg->out.link(edgeDetectorRight->inputConfig);
    xinEdgeCfg->out.link(edgeDetectorRgb->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output/input queues
    auto edgeLeftQueue = device.getOutputQueue(edgeLeftStr, 8, false);
    auto edgeRightQueue = device.getOutputQueue(edgeRightStr, 8, false);
    auto edgeRgbQueue = device.getOutputQueue(edgeRgbStr, 8, false);
    auto edgeCfgQueue = device.getInputQueue(edgeCfgStr);

    std::cout << "Switch between sobel filter kernels using keys '1' and '2'" << std::endl;

    while(true) {
        auto edgeLeft = edgeLeftQueue->get<dai::ImgFrame>();
        auto edgeRight = edgeRightQueue->get<dai::ImgFrame>();
        auto edgeRgb = edgeRgbQueue->get<dai::ImgFrame>();

        cv::Mat edgeLeftFrame = edgeLeft->getFrame();
        cv::Mat edgeRightFrame = edgeRight->getFrame();
        cv::Mat edgeRgbFrame = edgeRgb->getFrame();

        // Show the frame
        cv::imshow(edgeLeftStr, edgeLeftFrame);
        cv::imshow(edgeRightStr, edgeRightFrame);
        cv::imshow(edgeRgbStr, edgeRgbFrame);

        int key = cv::waitKey(1);
        switch(key) {
            case 'q':
                return 0;
                break;

            case '1': {
                std::cout << "Switching sobel filter kernel." << std::endl;
                dai::EdgeDetectorConfig cfg;
                std::vector<std::vector<int>> sobelHorizontalKernel = {{1, 0, -1}, {2, 0, -2}, {1, 0, -1}};
                std::vector<std::vector<int>> sobelVerticalKernel = {{1, 2, 1}, {0, 0, 0}, {-1, -2, -1}};
                cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel);
                edgeCfgQueue->send(cfg);
            } break;

            case '2': {
                std::cout << "Switching sobel filter kernel." << std::endl;
                dai::EdgeDetectorConfig cfg;
                std::vector<std::vector<int>> sobelHorizontalKernel = {{3, 0, -3}, {10, 0, -10}, {3, 0, -3}};
                std::vector<std::vector<int>> sobelVerticalKernel = {{3, 10, 3}, {0, 0, 0}, {-3, -10, -3}};
                cfg.setSobelFilterKernels(sobelHorizontalKernel, sobelVerticalKernel);
                edgeCfgQueue->send(cfg);
            } break;

            default:
                break;
        }
    }
    return 0;
}
