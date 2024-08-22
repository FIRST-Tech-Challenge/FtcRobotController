#include <chrono>
#include <cstdio>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

int main(int argc, char** argv) {
    using namespace std;
    // Default blob path provided by Hunter private data download
    // Applicable for easier example usage only
    std::string nnPath(BLOB_PATH);

    // If path to blob specified, use that
    if(argc > 1) {
        nnPath = std::string(argv[1]);
    }

    // Print which blob we are using
    printf("Using blob at path: %s\n", nnPath.c_str());

    // Create pipeline
    dai::Pipeline pipeline;
    pipeline.setOpenVINOVersion(dai::OpenVINO::Version::VERSION_2021_4);

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(300, 300);  // NN input
    camRgb->setInterleaved(false);
    camRgb->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);

    auto right = pipeline.create<dai::node::MonoCamera>();
    right->setCamera("right");
    right->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

    auto manipRight = pipeline.create<dai::node::ImageManip>();
    manipRight->initialConfig.setResize(300, 300);
    manipRight->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    right->out.link(manipRight->inputImage);

    auto left = pipeline.create<dai::node::MonoCamera>();
    left->setCamera("left");
    left->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

    auto manipLeft = pipeline.create<dai::node::ImageManip>();
    manipLeft->initialConfig.setResize(300, 300);
    manipLeft->initialConfig.setFrameType(dai::ImgFrame::Type::BGR888p);
    left->out.link(manipLeft->inputImage);

    auto nn = pipeline.create<dai::node::NeuralNetwork>();
    nn->setBlobPath(nnPath);
    nn->setNumInferenceThreads(2);

    manipLeft->out.link(nn->inputs["img1"]);
    camRgb->preview.link(nn->inputs["img2"]);
    manipRight->out.link(nn->inputs["img3"]);

    auto xout = pipeline.create<dai::node::XLinkOut>();
    xout->setStreamName("nn");
    nn->out.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the rgb frames and nn data from the outputs defined above
    auto qNn = device.getOutputQueue("nn", 4, false);

    while(true) {
        auto inNn = qNn->get<dai::NNData>();
        cv::imshow("Concat", fromPlanarFp16(inNn->getFirstLayerFp16(), 900, 300));

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
