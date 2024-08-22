/**
 * This example shows usage of Camera Control message as well as ColorCamera configInput to change crop x and y
 * Uses 'WASD' controls to move the crop window, 'C' to capture a still image, 'T' to trigger autofocus, 'IOKL,.[]'
 * for manual exposure/focus/white-balance:
 *   Control:      key[dec/inc]  min..max
 *   exposure time:     I   O      1..33000 [us]
 *   sensitivity iso:   K   L    100..1600
 *   focus:             ,   .      0..255 [far..near]
 *   white balance:     [   ]   1000..12000 (light color temperature K)
 * To go back to auto controls:
 *   'E' - autoexposure
 *   'F' - autofocus (continuous)
 *   'B' - auto white-balance
 */
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Step size ('W','A','S','D' controls)
static constexpr int STEP_SIZE = 8;

// Manual exposure/focus set step
static constexpr int EXP_STEP = 500;  // us
static constexpr int ISO_STEP = 50;
static constexpr int LENS_STEP = 3;
static constexpr int WB_STEP = 200;

static int clamp(int num, int v0, int v1) {
    return std::max(v0, std::min(num, v1));
}

int main() {
    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEncoder = pipeline.create<dai::node::VideoEncoder>();
    auto stillEncoder = pipeline.create<dai::node::VideoEncoder>();

    auto controlIn = pipeline.create<dai::node::XLinkIn>();
    auto configIn = pipeline.create<dai::node::XLinkIn>();
    auto videoMjpegOut = pipeline.create<dai::node::XLinkOut>();
    auto stillMjpegOut = pipeline.create<dai::node::XLinkOut>();
    auto previewOut = pipeline.create<dai::node::XLinkOut>();

    controlIn->setStreamName("control");
    configIn->setStreamName("config");
    videoMjpegOut->setStreamName("video");
    stillMjpegOut->setStreamName("still");
    previewOut->setStreamName("preview");

    // Properties
    camRgb->setVideoSize(640, 360);
    camRgb->setPreviewSize(300, 300);
    videoEncoder->setDefaultProfilePreset(camRgb->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);
    stillEncoder->setDefaultProfilePreset(1, dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking
    camRgb->video.link(videoEncoder->input);
    camRgb->still.link(stillEncoder->input);
    camRgb->preview.link(previewOut->input);
    controlIn->out.link(camRgb->inputControl);
    configIn->out.link(camRgb->inputConfig);
    videoEncoder->bitstream.link(videoMjpegOut->input);
    stillEncoder->bitstream.link(stillMjpegOut->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Get data queues
    auto controlQueue = device.getInputQueue("control");
    auto configQueue = device.getInputQueue("config");
    auto previewQueue = device.getOutputQueue("preview");
    auto videoQueue = device.getOutputQueue("video");
    auto stillQueue = device.getOutputQueue("still");

    // Max cropX & cropY
    float maxCropX = (camRgb->getResolutionWidth() - camRgb->getVideoWidth()) / (float)camRgb->getResolutionWidth();
    float maxCropY = (camRgb->getResolutionHeight() - camRgb->getVideoHeight()) / (float)camRgb->getResolutionHeight();

    // Default crop
    float cropX = 0;
    float cropY = 0;
    bool sendCamConfig = true;

    // Defaults and limits for manual focus/exposure controls
    int lensPos = 150;
    int lensMin = 0;
    int lensMax = 255;

    int expTime = 20000;
    int expMin = 1;
    int expMax = 33000;

    int sensIso = 800;
    int sensMin = 100;
    int sensMax = 1600;

    int wbManual = 4000;
    int wbMin = 1000;
    int wbMax = 12000;

    while(true) {
        auto previewFrames = previewQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& previewFrame : previewFrames) {
            cv::Mat frame(previewFrame->getHeight(), previewFrame->getWidth(), CV_8UC3, previewFrame->getData().data());
            cv::imshow("preview", frame);
        }

        auto videoFrames = videoQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& videoFrame : videoFrames) {
            // Decode JPEG
            auto frame = cv::imdecode(videoFrame->getData(), cv::IMREAD_UNCHANGED);
            // Display
            cv::imshow("video", frame);

            // Send new cfg to camera
            if(sendCamConfig) {
                dai::ImageManipConfig cfg;
                cfg.setCropRect(cropX, cropY, 0, 0);
                configQueue->send(cfg);
                printf("Sending new crop - x: %f, y: %f\n", cropX, cropY);
                sendCamConfig = false;
            }
        }

        auto stillFrames = stillQueue->tryGetAll<dai::ImgFrame>();
        for(const auto& stillFrame : stillFrames) {
            // Decode JPEG
            auto frame = cv::imdecode(stillFrame->getData(), cv::IMREAD_UNCHANGED);
            // Display
            cv::imshow("still", frame);
        }

        // Update screen (1ms pooling rate)
        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 'c') {
            dai::CameraControl ctrl;
            ctrl.setCaptureStill(true);
            controlQueue->send(ctrl);
        } else if(key == 't') {
            printf("Autofocus trigger (and disable continuous)\n");
            dai::CameraControl ctrl;
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::AUTO);
            ctrl.setAutoFocusTrigger();
            controlQueue->send(ctrl);
        } else if(key == 'f') {
            printf("Autofocus enable, continuous\n");
            dai::CameraControl ctrl;
            ctrl.setAutoFocusMode(dai::CameraControl::AutoFocusMode::CONTINUOUS_VIDEO);
            controlQueue->send(ctrl);
        } else if(key == 'e') {
            printf("Autoexposure enable\n");
            dai::CameraControl ctrl;
            ctrl.setAutoExposureEnable();
            controlQueue->send(ctrl);
        } else if(key == 'b') {
            printf("Auto white-balance enable\n");
            dai::CameraControl ctrl;
            ctrl.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
            controlQueue->send(ctrl);
        } else if(key == ',' || key == '.') {
            if(key == ',') lensPos -= LENS_STEP;
            if(key == '.') lensPos += LENS_STEP;
            lensPos = clamp(lensPos, lensMin, lensMax);
            printf("Setting manual focus, lens position: %d\n", lensPos);
            dai::CameraControl ctrl;
            ctrl.setManualFocus(lensPos);
            controlQueue->send(ctrl);
        } else if(key == 'i' || key == 'o' || key == 'k' || key == 'l') {
            if(key == 'i') expTime -= EXP_STEP;
            if(key == 'o') expTime += EXP_STEP;
            if(key == 'k') sensIso -= ISO_STEP;
            if(key == 'l') sensIso += ISO_STEP;
            expTime = clamp(expTime, expMin, expMax);
            sensIso = clamp(sensIso, sensMin, sensMax);
            printf("Setting manual exposure, time: %d, iso: %d\n", expTime, sensIso);
            dai::CameraControl ctrl;
            ctrl.setManualExposure(expTime, sensIso);
            controlQueue->send(ctrl);
        } else if(key == '[' || key == ']') {
            if(key == '[') wbManual -= WB_STEP;
            if(key == ']') wbManual += WB_STEP;
            wbManual = clamp(wbManual, wbMin, wbMax);
            printf("Setting manual white balance, temperature: %d K\n", wbManual);
            dai::CameraControl ctrl;
            ctrl.setManualWhiteBalance(wbManual);
            controlQueue->send(ctrl);
        } else if(key == 'w' || key == 'a' || key == 's' || key == 'd') {
            if(key == 'a') {
                cropX -= (maxCropX / camRgb->getResolutionWidth()) * STEP_SIZE;
                if(cropX < 0) cropX = maxCropX;
            } else if(key == 'd') {
                cropX += (maxCropX / camRgb->getResolutionWidth()) * STEP_SIZE;
                if(cropX > maxCropX) cropX = 0.0f;
            } else if(key == 'w') {
                cropY -= (maxCropY / camRgb->getResolutionHeight()) * STEP_SIZE;
                if(cropY < 0) cropY = maxCropY;
            } else if(key == 's') {
                cropY += (maxCropY / camRgb->getResolutionHeight()) * STEP_SIZE;
                if(cropY > maxCropY) cropY = 0.0f;
            }
            sendCamConfig = true;
        }
    }
    return 0;
}
