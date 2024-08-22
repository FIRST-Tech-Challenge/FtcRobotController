#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Closer-in minimum depth, disparity range is doubled (from 95 to 190):
static std::atomic<bool> extended_disparity{false};
// Better accuracy for longer distance, fractional disparity 32-levels:
static std::atomic<bool> subpixel{false};
// Better handling for occlusions:
static std::atomic<bool> lr_check{true};

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto depth = pipeline.create<dai::node::StereoDepth>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("disparity");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    // Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
    depth->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
    depth->initialConfig.setMedianFilter(dai::MedianFilter::KERNEL_7x7);
    depth->setLeftRightCheck(lr_check);
    depth->setExtendedDisparity(extended_disparity);
    depth->setSubpixel(subpixel);

    // Linking
    monoLeft->out.link(depth->left);
    monoRight->out.link(depth->right);
    depth->disparity.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the disparity frames from the outputs defined above
    auto q = device.getOutputQueue("disparity", 4, false);

    while(true) {
        auto inDepth = q->get<dai::ImgFrame>();
        auto frame = inDepth->getFrame();
        // Normalization for better visualization
        frame.convertTo(frame, CV_8UC1, 255 / depth->initialConfig.getMaxDisparity());

        cv::imshow("disparity", frame);

        // Available color maps: https://docs.opencv.org/3.4/d3/d50/group__imgproc__colormap.html
        cv::applyColorMap(frame, frame, cv::COLORMAP_JET);
        cv::imshow("disparity_color", frame);

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
