#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

static void drawFeatures(cv::Mat& frame, std::vector<dai::TrackedFeature>& features) {
    static const auto pointColor = cv::Scalar(0, 0, 255);
    static const int circleRadius = 2;
    for(auto& feature : features) {
        cv::circle(frame, cv::Point(feature.position.x, feature.position.y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
    }
}

int main() {
    using namespace std;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto monoRight = pipeline.create<dai::node::MonoCamera>();
    auto featureTrackerLeft = pipeline.create<dai::node::FeatureTracker>();
    auto featureTrackerRight = pipeline.create<dai::node::FeatureTracker>();

    auto xoutPassthroughFrameLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesLeft = pipeline.create<dai::node::XLinkOut>();
    auto xoutPassthroughFrameRight = pipeline.create<dai::node::XLinkOut>();
    auto xoutTrackedFeaturesRight = pipeline.create<dai::node::XLinkOut>();
    auto xinTrackedFeaturesConfig = pipeline.create<dai::node::XLinkIn>();

    xoutPassthroughFrameLeft->setStreamName("passthroughFrameLeft");
    xoutTrackedFeaturesLeft->setStreamName("trackedFeaturesLeft");
    xoutPassthroughFrameRight->setStreamName("passthroughFrameRight");
    xoutTrackedFeaturesRight->setStreamName("trackedFeaturesRight");
    xinTrackedFeaturesConfig->setStreamName("trackedFeaturesConfig");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoLeft->setCamera("left");
    monoRight->setResolution(dai::MonoCameraProperties::SensorResolution::THE_720_P);
    monoRight->setCamera("right");

    // Disable optical flow
    featureTrackerLeft->initialConfig.setMotionEstimator(false);
    featureTrackerRight->initialConfig.setMotionEstimator(false);

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->passthroughInputImage.link(xoutPassthroughFrameLeft->input);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerLeft->inputConfig);

    monoRight->out.link(featureTrackerRight->inputImage);
    featureTrackerRight->passthroughInputImage.link(xoutPassthroughFrameRight->input);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerRight->inputConfig);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    printf("Press 's' to switch between Harris and Shi-Thomasi corner detector! \n");

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, false);
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto passthroughImageRightQueue = device.getOutputQueue("passthroughFrameRight", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);

    auto inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig");

    const auto leftWindowName = "left";
    const auto rightWindowName = "right";

    while(true) {
        auto inPassthroughFrameLeft = passthroughImageLeftQueue->get<dai::ImgFrame>();
        cv::Mat passthroughFrameLeft = inPassthroughFrameLeft->getFrame();
        cv::Mat leftFrame;
        cv::cvtColor(passthroughFrameLeft, leftFrame, cv::COLOR_GRAY2BGR);

        auto inPassthroughFrameRight = passthroughImageRightQueue->get<dai::ImgFrame>();
        cv::Mat passthroughFrameRight = inPassthroughFrameRight->getFrame();
        cv::Mat rightFrame;
        cv::cvtColor(passthroughFrameRight, rightFrame, cv::COLOR_GRAY2BGR);

        auto trackedFeaturesLeft = outputFeaturesLeftQueue->get<dai::TrackedFeatures>()->trackedFeatures;
        drawFeatures(leftFrame, trackedFeaturesLeft);

        auto trackedFeaturesRight = outputFeaturesRightQueue->get<dai::TrackedFeatures>()->trackedFeatures;
        drawFeatures(rightFrame, trackedFeaturesRight);

        // Show the frame
        cv::imshow(leftWindowName, leftFrame);
        cv::imshow(rightWindowName, rightFrame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 's') {
            if(featureTrackerConfig.cornerDetector.type == dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS) {
                featureTrackerConfig.cornerDetector.type = dai::FeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI;
                printf("Switching to Shi-Thomasi \n");
            } else {
                featureTrackerConfig.cornerDetector.type = dai::FeatureTrackerConfig::CornerDetector::Type::HARRIS;
                printf("Switching to Harris \n");
            }
            auto cfg = dai::FeatureTrackerConfig();
            cfg.set(featureTrackerConfig);
            inputFeatureTrackerConfigQueue->send(cfg);
        }
    }
    return 0;
}
