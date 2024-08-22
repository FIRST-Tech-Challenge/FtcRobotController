#include <cstdio>
#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Optional. If set (true), the ColorCamera is downscaled from 1080p to 720p.
// Otherwise (false), the aligned depth is automatically upscaled to 1080p
static std::atomic<bool> downscaleColor{true};
static constexpr int fps = 30;
// The disparity is computed at this resolution, then upscaled to RGB resolution
static constexpr auto monoRes = dai::MonoCameraProperties::SensorResolution::THE_720_P;

static float rgbWeight = 0.3f;
static float depthWeight = 0.3f;
static float confWeight = 0.3f;

static float rgbWeightNorm = 0.3f;
static float depthWeightNorm = 0.3f;
static float confWeightNorm = 0.3f;

static void updateBlendWeights(int percentRgb, void* weight) {
    *((float*)weight) = float(percentRgb) / 100.f;
}

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;
    dai::Device device;
    std::vector<std::string> queueNames;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto left = pipeline.create<dai::node::MonoCamera>();
    auto right = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();

    auto rgbOut = pipeline.create<dai::node::XLinkOut>();
    auto depthOut = pipeline.create<dai::node::XLinkOut>();
    auto confOut = pipeline.create<dai::node::XLinkOut>();

    rgbOut->setStreamName("rgb");
    queueNames.push_back("rgb");
    depthOut->setStreamName("depth");
    queueNames.push_back("depth");
    confOut->setStreamName("conf");
    queueNames.push_back("conf");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setFps(fps);
    if(downscaleColor) camRgb->setIspScale(2, 3);
    // For now, RGB needs fixed focus to properly align with depth.
    // This value was used during calibration
    try {
        auto calibData = device.readCalibration2();
        auto lensPosition = calibData.getLensPosition(dai::CameraBoardSocket::CAM_A);
        if(lensPosition) {
            camRgb->initialControl.setManualFocus(lensPosition);
        }
    } catch(const std::exception& ex) {
        std::cout << ex.what() << std::endl;
        return 1;
    }

    left->setResolution(monoRes);
    left->setCamera("left");
    left->setFps(fps);
    right->setResolution(monoRes);
    right->setCamera("right");
    right->setFps(fps);

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    // LR-check is required for depth alignment
    stereo->setLeftRightCheck(true);
    stereo->setDepthAlign(dai::CameraBoardSocket::CAM_A);

    // Linking
    camRgb->isp.link(rgbOut->input);
    left->out.link(stereo->left);
    right->out.link(stereo->right);
    stereo->disparity.link(depthOut->input);
    stereo->confidenceMap.link(confOut->input);

    // Connect to device and start pipeline
    device.startPipeline(pipeline);

    // Sets queues size and behavior
    for(const auto& name : queueNames) {
        device.getOutputQueue(name, 4, false);
    }

    std::unordered_map<std::string, cv::Mat> frame;

    auto rgbWindowName = "rgb";
    auto depthWindowName = "depth";
    auto confWindowName = "conf";
    auto blendedWindowName = "rgb-depth-confidence";
    cv::namedWindow(rgbWindowName);
    cv::namedWindow(depthWindowName);
    cv::namedWindow(confWindowName);
    cv::namedWindow(blendedWindowName);
    int defRgbWeightValue = (int)(rgbWeight * 100);
    int defDepthWeightValue = (int)(rgbWeight * 100);
    int defConfWeightValue = (int)(rgbWeight * 100);
    cv::createTrackbar("RGB Weight %", blendedWindowName, &defRgbWeightValue, 100, updateBlendWeights, &rgbWeight);
    cv::createTrackbar("Depth Weight %", blendedWindowName, &defDepthWeightValue, 100, updateBlendWeights, &depthWeight);
    cv::createTrackbar("Confidence Weight %", blendedWindowName, &defConfWeightValue, 100, updateBlendWeights, &confWeight);

    while(true) {
        std::unordered_map<std::string, std::shared_ptr<dai::ImgFrame>> latestPacket;

        auto queueEvents = device.getQueueEvents(queueNames);
        for(const auto& name : queueEvents) {
            auto packets = device.getOutputQueue(name)->tryGetAll<dai::ImgFrame>();
            auto count = packets.size();
            if(count > 0) {
                latestPacket[name] = packets[count - 1];
            }
        }

        for(const auto& name : queueNames) {
            if(latestPacket.find(name) != latestPacket.end()) {
                if(name == depthWindowName) {
                    frame[name] = latestPacket[name]->getFrame();
                    auto maxDisparity = stereo->initialConfig.getMaxDisparity();
                    // Optional, extend range 0..95 -> 0..255, for a better visualisation
                    if(1) frame[name].convertTo(frame[name], CV_8UC1, 255. / maxDisparity);
                    // Optional, apply false colorization
                    if(1) cv::applyColorMap(frame[name], frame[name], cv::COLORMAP_HOT);
                } else {
                    frame[name] = latestPacket[name]->getCvFrame();
                }

                cv::imshow(name, frame[name]);
            }
        }

        // Blend when all three frames received
        if(frame.find(rgbWindowName) != frame.end() && frame.find(depthWindowName) != frame.end() && frame.find(confWindowName) != frame.end()) {
            // Need to have all three frames in BGR format before blending
            if(frame[depthWindowName].channels() < 3) {
                cv::cvtColor(frame[depthWindowName], frame[depthWindowName], cv::COLOR_GRAY2BGR);
            }
            if(frame[confWindowName].channels() < 3) {
                cv::cvtColor(frame[confWindowName], frame[confWindowName], cv::COLOR_GRAY2BGR);
            }

            float sumWeight = rgbWeight + depthWeight + confWeight;
            // Normalize the weights so their sum to be <= 1.0
            if(sumWeight <= 1.0) {
                rgbWeightNorm = rgbWeight;
                depthWeightNorm = depthWeight;
                confWeightNorm = confWeight;
            } else {
                rgbWeightNorm = rgbWeight / sumWeight;
                depthWeightNorm = depthWeight / sumWeight;
                confWeightNorm = confWeight / sumWeight;
            }

            cv::Mat blended1, blended2;
            cv::addWeighted(frame[rgbWindowName], rgbWeightNorm, frame[depthWindowName], depthWeightNorm, 0, blended1);
            cv::addWeighted(blended1, rgbWeightNorm + depthWeightNorm, frame[confWindowName], confWeightNorm, 0, blended2);
            cv::imshow(blendedWindowName, blended2);
            frame.clear();
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
