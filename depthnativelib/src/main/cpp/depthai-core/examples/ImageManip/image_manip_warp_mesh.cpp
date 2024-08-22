#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    camRgb->setPreviewSize(500, 500);
    camRgb->setInterleaved(false);
    auto maxFrameSize = camRgb->getPreviewWidth() * camRgb->getPreviewHeight() * 3;

    // Warp preview frame 1
    auto manip1 = pipeline.create<dai::node::ImageManip>();
    // Create a custom warp mesh
    dai::Point2f tl(20, 20);
    dai::Point2f tr(460, 20);
    dai::Point2f ml(100, 250);
    dai::Point2f mr(400, 250);
    dai::Point2f bl(20, 460);
    dai::Point2f br(460, 460);
    manip1->setWarpMesh({tl, tr, ml, mr, bl, br}, 2, 3);
    manip1->setMaxOutputFrameSize(maxFrameSize);

    camRgb->preview.link(manip1->inputImage);
    auto xout1 = pipeline.create<dai::node::XLinkOut>();
    xout1->setStreamName("out1");
    manip1->out.link(xout1->input);

    // Warp preview frame 2
    auto manip2 = pipeline.create<dai::node::ImageManip>();
    // Create a custom warp mesh
    // clang-format off
    std::vector<dai::Point2f> mesh2 = {
        {20, 20}, {250, 100}, {460, 20},
        {100,250}, {250, 250}, {400, 250},
        {20, 480}, {250,400}, {460,480}
    };
    // clang-format on
    manip2->setWarpMesh(mesh2, 3, 3);
    manip2->setMaxOutputFrameSize(maxFrameSize);

    camRgb->preview.link(manip2->inputImage);
    auto xout2 = pipeline.create<dai::node::XLinkOut>();
    xout2->setStreamName("out2");
    manip2->out.link(xout2->input);

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
