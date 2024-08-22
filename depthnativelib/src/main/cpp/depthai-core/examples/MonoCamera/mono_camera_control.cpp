/**
 * This example shows usage of mono camera in crop mode with the possibility to move the crop.
 * Uses 'WASD' controls to move the crop window, 'T' to trigger autofocus, 'IOKL,.' for manual exposure/focus:
 *   Control:      key[dec/inc]  min..max
 *   exposure time:     I   O      1..33000 [us]
 *   sensitivity iso:   K   L    100..1600
 * To go back to auto controls:
 *   'E' - autoexposure
 */
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Step size ('W','A','S','D' controls)
static constexpr float stepSize = 0.02f;

// Manual exposure/focus set step
static constexpr int EXP_STEP = 500;  // us
static constexpr int ISO_STEP = 50;

static int clamp(int num, int v0, int v1) {
    return std::max(v0, std::min(num, v1));
}

static std::atomic<bool> sendCamConfig{false};

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto manipRight = pipeline.create<dai::node::ImageManip>();
    auto manipLeft = pipeline.create<dai::node::ImageManip>();

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto manipOutRight = pipeline.create<dai::node::XLinkOut>();
    auto manipOutLeft = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    configIn->setStreamName("config");
    manipOutRight->setStreamName("right");
    manipOutLeft->setStreamName("left");

    // Crop range
    dai::Point2f topLeft(0.2f, 0.2f);
    dai::Point2f bottomRight(0.8f, 0.8f);

    // Properties
    monoRight->setCamera("right");
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    manipRight->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);
    manipLeft->initialConfig.setCropRect(topLeft.x, topLeft.y, bottomRight.x, bottomRight.y);

    // Linking
    monoRight->out.link(manipRight->inputImage);
    monoLeft->out.link(manipLeft->inputImage);
    controlIn->out.link(monoRight->inputControl);
    controlIn->out.link(monoLeft->inputControl);
    configIn->out.link(manipRight->inputConfig);
    configIn->out.link(manipLeft->inputConfig);
    manipRight->out.link(manipOutRight->input);
    manipLeft->out.link(manipOutLeft->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues will be used to get the grayscale frames
    auto qRight = device.getOutputQueue(manipOutRight->getStreamName(), 4, false);
    auto qLeft = device.getOutputQueue(manipOutLeft->getStreamName(), 4, false);
    auto controlQueue = device.getInputQueue(controlIn->getStreamName());
    auto configQueue = device.getInputQueue(configIn->getStreamName());

    // Defaults and limits for manual focus/exposure controls
    int exp_time = 20000;
    int exp_min = 1;
    int exp_max = 33000;

    int sens_iso = 800;
    int sens_min = 100;
    int sens_max = 1600;

    while(true) {
        auto inRight = qRight->get<dai::ImgFrame>();
        auto inLeft = qLeft->get<dai::ImgFrame>();
        cv::imshow("right", inRight->getCvFrame());
        cv::imshow("left", inLeft->getCvFrame());

        // Update screen (1ms pooling rate)
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 'e') {
            printf("Autoexposure enable\n");
            dai::CameraControl ctrl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        } else if(key == 'i' || key == 'o' || key == 'k' || key == 'l') {
            if(key == 'i') exp_time -= EXP_STEP;
            if(key == 'o') exp_time += EXP_STEP;
            if(key == 'k') sens_iso -= ISO_STEP;
            if(key == 'l') sens_iso += ISO_STEP;
            exp_time = clamp(exp_time, exp_min, exp_max);
            sens_iso = clamp(sens_iso, sens_min, sens_max);
            printf("Setting manual exposure, time: %d, iso: %d\n", exp_time, sens_iso);
            dai::CameraControl ctrl;
            ctrl.setManualExposure(exp_time, sens_iso);
            controlQueue->send(ctrl);
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
