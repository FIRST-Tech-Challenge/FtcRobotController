#include <iostream>

#include "utility.hpp"

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

static constexpr float stepSize = 0.05f;

static std::atomic<bool> newConfig{false};

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto spatialLocationCalculator = pipeline.create<dai::node::SpatialLocationCalculator>();

    auto xoutDepth = pipeline.create<dai::node::XLinkOut>();
    auto xoutSpatialData = pipeline.create<dai::node::XLinkOut>();
    auto xinSpatialCalcConfig = pipeline.create<dai::node::XLinkIn>();

    xoutDepth->setStreamName("depth");
    xoutSpatialData->setStreamName("spatialData");
    xinSpatialCalcConfig->setStreamName("spatialCalcConfig");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoRight->setCamera("right");

    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DENSITY);
    stereo->setLeftRightCheck(true);
    stereo->setExtendedDisparity(true);
    spatialLocationCalculator->inputConfig.setWaitForMessage(false);

    // Create 10 ROIs
    for(int i = 0; i < 10; i++) {
        dai::SpatialLocationCalculatorConfigData config;
        config.depthThresholds.lowerThreshold = 200;
        config.depthThresholds.upperThreshold = 10000;
        config.roi = dai::Rect(dai::Point2f(i * 0.1, 0.45), dai::Point2f((i + 1) * 0.1, 0.55));
        spatialLocationCalculator->initialConfig.addROI(config);
    }

    // Linking
    monoLeft->out.link(stereo->left);
    monoRight->out.link(stereo->right);

    spatialLocationCalculator->passthroughDepth.link(xoutDepth->input);
    stereo->depth.link(spatialLocationCalculator->inputDepth);

    spatialLocationCalculator->out.link(xoutSpatialData->input);
    xinSpatialCalcConfig->out.link(spatialLocationCalculator->inputConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);
    device.setIrLaserDotProjectorBrightness(1000);

    // Output queue will be used to get the depth frames from the outputs defined above
    auto depthQueue = device.getOutputQueue("depth", 4, false);
    auto spatialCalcQueue = device.getOutputQueue("spatialData", 4, false);
    auto color = cv::Scalar(0, 200, 40);
    auto fontType = cv::FONT_HERSHEY_TRIPLEX;

    while(true) {
        auto inDepth = depthQueue->get<dai::ImgFrame>();

        cv::Mat depthFrame = inDepth->getFrame();  // depthFrame values are in millimeters

        cv::Mat depthFrameColor;
        cv::normalize(depthFrame, depthFrameColor, 255, 0, cv::NORM_INF, CV_8UC1);
        cv::equalizeHist(depthFrameColor, depthFrameColor);
        cv::applyColorMap(depthFrameColor, depthFrameColor, cv::COLORMAP_HOT);

        auto spatialData = spatialCalcQueue->get<dai::SpatialLocationCalculatorData>()->getSpatialLocations();
        for(auto depthData : spatialData) {
            auto roi = depthData.config.roi;
            roi = roi.denormalize(depthFrameColor.cols, depthFrameColor.rows);

            auto xmin = static_cast<int>(roi.topLeft().x);
            auto ymin = static_cast<int>(roi.topLeft().y);
            auto xmax = static_cast<int>(roi.bottomRight().x);
            auto ymax = static_cast<int>(roi.bottomRight().y);

            auto coords = depthData.spatialCoordinates;
            auto distance = std::sqrt(coords.x * coords.x + coords.y * coords.y + coords.z * coords.z);

            cv::rectangle(depthFrameColor, cv::Rect(cv::Point(xmin, ymin), cv::Point(xmax, ymax)), color);
            std::stringstream depthDistance;
            depthDistance.precision(2);
            depthDistance << fixed << static_cast<float>(distance / 1000.0f) << "m";
            cv::putText(depthFrameColor, depthDistance.str(), cv::Point(xmin + 10, ymin + 20), fontType, 0.5, color);
        }
        // Show the frame
        cv::imshow("depth", depthFrameColor);

        if(cv::waitKey(1) == 'q') {
            break;
        }
    }
    return 0;
}