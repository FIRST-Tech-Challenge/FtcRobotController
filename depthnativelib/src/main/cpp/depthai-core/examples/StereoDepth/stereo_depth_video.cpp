#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static std::atomic<bool> withDepth{true};

static std::atomic<bool> outputDepth{false};
static std::atomic<bool> outputRectified{true};
static std::atomic<bool> lrcheck{true};
static std::atomic<bool> extended{false};
static std::atomic<bool> subpixel{false};

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = withDepth ? pipeline.create<dai::node::StereoDepth>() : nullptr;

    auto xoutLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutDisp = pipeline.create<dai::node::XLinkOut>();
    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifL = pipeline.create<dai::node::XLinkOut>();
    auto xoutRectifR = pipeline.create<dai::node::XLinkOut>();

    // XLinkOut
    xoutLeft->setStreamName("left");
    xoutRight->setStreamName("right");
    if(withDepth) {
        xoutDisp->setStreamName("disparity");
        xoutDepth->setStreamName("depth");
        xoutRectifL->setStreamName("rectified_left");
        xoutRectifR->setStreamName("rectified_right");
    }

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamera("right");

    if(withDepth) {
        // StereoDepth
        stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
        stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
        // stereo->setInputResolution(1280, 720);
        stereo->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_5x5);
        stereo->setLeftRightCheck(lrcheck);
        stereo->setExtendedDisparity(extended);
        stereo->setSubpixel(subpixel);

        // Linking
        monoLeft->out.link(stereo->left);
        monoRight->out.link(stereo->right);

        stereo->syncedLeft.link(xoutLeft->input);
        stereo->syncedRight.link(xoutRight->input);
        stereo->disparity.link(xoutDisp->input);

        if(outputRectified) {
            stereo->rectifiedLeft.link(xoutRectifL->input);
            stereo->rectifiedRight.link(xoutRectifR->input);
        }

        if(outputDepth) {
            stereo->depth.link(xoutDepth->input);
        }

    } else {
        // Link plugins CAM -> XLINK
        monoLeft->out.link(xoutLeft->input);
        monoRight->out.link(xoutRight->input);
    }

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    auto leftQueue = device.getOutputQueue("left", 8, false);
    auto rightQueue = device.getOutputQueue("right", 8, false);
    auto dispQueue = withDepth ? device.getOutputQueue("disparity", 8, false) : nullptr;
    auto depthQueue = withDepth ? device.getOutputQueue("depth", 8, false) : nullptr;
    auto rectifLeftQueue = withDepth ? device.getOutputQueue("rectified_left", 8, false) : nullptr;
    auto rectifRightQueue = withDepth ? device.getOutputQueue("rectified_right", 8, false) : nullptr;

    // Disparity range is used for normalization
    float disparityMultiplier = withDepth ? 255 / stereo->initialConfig.getMaxDisparity() : 0;

    while(true) {
        auto left = leftQueue->get<dai::ImgFrame>();
        cv::imshow("left", left->getFrame());
        auto right = rightQueue->get<dai::ImgFrame>();
        cv::imshow("right", right->getFrame());

        if(withDepth) {
            auto disparity = dispQueue->get<dai::ImgFrame>();
            cv::Mat disp(disparity->getCvFrame());
            disp.convertTo(disp, CV_8UC1, disparityMultiplier);  // Extend disparity range
            cv::imshow("disparity", disp);
            cv::Mat disp_color;
            cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);
            cv::imshow("disparity_color", disp_color);

            if(outputDepth) {
                auto depth = depthQueue->get<dai::ImgFrame>();
                cv::imshow("depth", depth->getCvFrame());
            }

            if(outputRectified) {
                auto rectifL = rectifLeftQueue->get<dai::ImgFrame>();
                cv::imshow("rectified_left", rectifL->getFrame());

                auto rectifR = rectifRightQueue->get<dai::ImgFrame>();
                cv::imshow("rectified_right", rectifR->getFrame());
            }
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
