/**
 * This example shows usage of depth camera in crop mode with the possibility to move the crop.
 * Use 'WASD' in order to do it.
 */
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Step size ('W','A','S','D' controls)
static constexpr float stepSize = 0.02f;

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto manip = pipeline.create<dai::node::ImageManip>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    configIn->setStreamName("config");
    xout->setStreamName("depth");

    // Crop range
    dai::Point2f topLeft(0.2f, 0.2f);
    dai::Point2f bottomRight(0.8f, 0.8f);

    // Properties
    monoRight->setCamera("right");
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);

    manip->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    manip->setMaxOutputFrameSize(monoRight->getResolutionHeight() * monoRight->getResolutionWidth() * 3);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);

    // Linking
    configIn->out.link(manip->inputConfig);
    stereo->depth.link(manip->inputImage);
    manip->out.link(xout->input);
    monoRight->out.link(stereo->right);
    monoLeft->out.link(stereo->left);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto q = device.getOutputQueue(xout->getStreamName(), 4, false);
    auto configQueue = device.getInputQueue(configIn->getStreamName());

    bool sendCamConfig = false;

    while(true) {
        auto inDepth = q->get<dai::ImgFrame>();
        cv::Mat depthFrame = inDepth->getFrame();  // depthFrame values are in millimeters

        // Frame is transformed, the color map will be applied to highlight the depth info
        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        // Frame is ready to be shown
        cv::imshow("depth", depthFrameColor);

        // Update screen (10ms pooling rate)
        int key = cv::waitKey(9);
        cv::waitKey(1);  // glitch workaround
        if(key == 'q') {
            break;
        } else if(key == 'w') {
            if(topLeft.y - stepSize >= 0) {
                topLeft.y -= stepSize;
                bottomRight.y -= stepSize;
                sendCamConfig = true;
            }
        } else if(key == 'a') {
            if(topLeft.x - stepSize >= 0) {
                topLeft.x -= stepSize;
                bottomRight.x -= stepSize;
                sendCamConfig = true;
            }
        } else if(key == 's') {
            if(bottomRight.y + stepSize <= 1) {
                topLeft.y += stepSize;
                bottomRight.y += stepSize;
                sendCamConfig = true;
            }
        } else if(key == 'd') {
            if(bottomRight.x + stepSize <= 1) {
                topLeft.x += stepSize;
                bottomRight.x += stepSize;
                sendCamConfig = true;
            }
        }

        // Send new config to camera
        if(sendCamConfig) {
            dai::ImageManipConfig cfg;
            cfg.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
            configQueue->send(cfg);
            sendCamConfig = false;
        }
    }
    return 0;
}
