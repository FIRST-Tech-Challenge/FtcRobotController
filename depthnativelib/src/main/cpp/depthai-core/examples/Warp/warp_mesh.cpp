#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(496, 496);
    camRgb->setInterleaved(false);
    auto maxFrameSize = camRgb->getPreviewWidth() * camRgb->getPreviewHeight() * 3;

    // Warp preview frame 1
    auto warp1 = pipeline.create<dai::node::Warp>();
    // Create a custom warp mesh
    dai::Point2f tl(20, 20);
    dai::Point2f tr(460, 20);
    dai::Point2f ml(100, 250);
    dai::Point2f mr(400, 250);
    dai::Point2f bl(20, 460);
    dai::Point2f br(460, 460);
    warp1->setWarpMesh({tl, tr, ml, mr, bl, br}, 2, 3);
    constexpr std::tuple<int, int> WARP1_OUTPUT_FRAME_SIZE = {992, 500};
    warp1->setOutputSize(WARP1_OUTPUT_FRAME_SIZE);
    warp1->setMaxOutputFrameSize(std::get<0>(WARP1_OUTPUT_FRAME_SIZE) * std::get<1>(WARP1_OUTPUT_FRAME_SIZE) * 3);
    warp1->setInterpolation(dai::Interpolation::NEAREST_NEIGHBOR);
    warp1->setHwIds({1});

    camRgb->preview.link(warp1->inputImage);
    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("out1");
    warp1->out.link(xout1->input);

    // Warp preview frame 2
    auto warp2 = pipeline.create<dai::node::Warp>();
    // Create a custom warp mesh
    // clang-format off
    std::vector<dai::Point2f> mesh2 = {
        {20, 20}, {250, 100}, {460, 20},
        {100,250}, {250, 250}, {400, 250},
        {20, 480}, {250,400}, {460,480}
    };
    // clang-format on
    warp2->setWarpMesh(mesh2, 3, 3);
    warp2->setMaxOutputFrameSize(maxFrameSize);
    warp2->setInterpolation(dai::Interpolation::BICUBIC);
    warp2->setHwIds({2});

    camRgb->preview.link(warp2->inputImage);
    auto xout2 = pipeline.create<dai::node::XLinkOut>();
    xout2->setStreamName("out2");
    warp2->out.link(xout2->input);

    dai::Device device(pipeline);
    auto q1 = device.getOutputQueue("out1", 8, false);
    auto q2 = device.getOutputQueue("out2", 8, false);
    while(true) {
        auto in1 = q1->get<dai::ImgFrame>();
        if(in1) {
            cv::imshow("Warped preview 1", in1->getCvFrame());
        }
        auto in2 = q2->get<dai::ImgFrame>();
        if(in2) {
            cv::imshow("Warped preview 2", in2->getCvFrame());
        }
        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') return 0;
    }
    return 0;
}
