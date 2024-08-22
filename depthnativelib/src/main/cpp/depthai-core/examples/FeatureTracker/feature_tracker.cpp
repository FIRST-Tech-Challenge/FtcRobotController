#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "deque"
#include "unordered_map"
#include "unordered_set"

static const auto lineColor = cv::Scalar(200, 0, 200);
static const auto pointColor = cv::Scalar(0, 0, 255);

class FeatureTrackerDrawer {
   private:
    static const int circleRadius = 2;
    static const int maxTrackedFeaturesPathLength = 30;
    // for how many frames the feature is tracked
    static int trackedFeaturesPathLength;

    using featureIdType = decltype(dai::Point2f::x);

    std::unordered_set<featureIdType> trackedIDs;
    std::unordered_map<featureIdType, std::deque<dai::Point2f>> trackedFeaturesPath;

    std::string trackbarName;
    std::string windowName;

   public:
    void trackFeaturePath(std::vector<dai::TrackedFeature>& features) {
        std::unordered_set<featureIdType> newTrackedIDs;
        for(auto& currentFeature : features) {
            auto currentID = currentFeature.id;
            newTrackedIDs.insert(currentID);

            if(!trackedFeaturesPath.count(currentID)) {
                trackedFeaturesPath.insert({currentID, std::deque<dai::Point2f>()});
            }
            std::deque<dai::Point2f>& path = trackedFeaturesPath.at(currentID);

            path.push_back(currentFeature.position);
            while(path.size() > std::max<unsigned int>(1, trackedFeaturesPathLength)) {
                path.pop_front();
            }
        }

        std::unordered_set<featureIdType> featuresToRemove;
        for(auto& oldId : trackedIDs) {
            if(!newTrackedIDs.count(oldId)) {
                featuresToRemove.insert(oldId);
            }
        }

        for(auto& id : featuresToRemove) {
            trackedFeaturesPath.erase(id);
        }

        trackedIDs = newTrackedIDs;
    }

    void drawFeatures(cv::Mat& img) {
        cv::setTrackbarPos(trackbarName.c_str(), windowName.c_str(), trackedFeaturesPathLength);

        for(auto& featurePath : trackedFeaturesPath) {
            std::deque<dai::Point2f>& path = featurePath.second;
            unsigned int j = 0;
            for(j = 0; j < path.size() - 1; j++) {
                auto src = cv::Point(path[j].x, path[j].y);
                auto dst = cv::Point(path[j + 1].x, path[j + 1].y);
                cv::line(img, src, dst, lineColor, 1, cv::LINE_AA, 0);
            }

            cv::circle(img, cv::Point(path[j].x, path[j].y), circleRadius, pointColor, -1, cv::LINE_AA, 0);
        }
    }

    FeatureTrackerDrawer(std::string trackbarName, std::string windowName) : trackbarName(trackbarName), windowName(windowName) {
        cv::namedWindow(windowName.c_str());
        cv::createTrackbar(trackbarName.c_str(), windowName.c_str(), &trackedFeaturesPathLength, maxTrackedFeaturesPathLength, nullptr);
    }
};

int FeatureTrackerDrawer::trackedFeaturesPathLength = 10;

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

    // Linking
    monoLeft->out.link(featureTrackerLeft->inputImage);
    featureTrackerLeft->passthroughInputImage.link(xoutPassthroughFrameLeft->input);
    featureTrackerLeft->outputFeatures.link(xoutTrackedFeaturesLeft->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerLeft->inputConfig);

    monoRight->out.link(featureTrackerRight->inputImage);
    featureTrackerRight->passthroughInputImage.link(xoutPassthroughFrameRight->input);
    featureTrackerRight->outputFeatures.link(xoutTrackedFeaturesRight->input);
    xinTrackedFeaturesConfig->out.link(featureTrackerRight->inputConfig);

    // By default the least mount of resources are allocated
    // increasing it improves performance when optical flow is enabled
    auto numShaves = 2;
    auto numMemorySlices = 2;
    featureTrackerLeft->setHardwareResources(numShaves, numMemorySlices);
    featureTrackerRight->setHardwareResources(numShaves, numMemorySlices);

    auto featureTrackerConfig = featureTrackerRight->initialConfig.get();

    printf("Press 's' to switch between Lucas-Kanade optical flow and hardware accelerated motion estimation! \n");

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queues used to receive the results
    auto passthroughImageLeftQueue = device.getOutputQueue("passthroughFrameLeft", 8, false);
    auto outputFeaturesLeftQueue = device.getOutputQueue("trackedFeaturesLeft", 8, false);
    auto passthroughImageRightQueue = device.getOutputQueue("passthroughFrameRight", 8, false);
    auto outputFeaturesRightQueue = device.getOutputQueue("trackedFeaturesRight", 8, false);

    auto inputFeatureTrackerConfigQueue = device.getInputQueue("trackedFeaturesConfig");

    const auto leftWindowName = "left";
    auto leftFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", leftWindowName);

    const auto rightWindowName = "right";
    auto rightFeatureDrawer = FeatureTrackerDrawer("Feature tracking duration (frames)", rightWindowName);

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
        leftFeatureDrawer.trackFeaturePath(trackedFeaturesLeft);
        leftFeatureDrawer.drawFeatures(leftFrame);

        auto trackedFeaturesRight = outputFeaturesRightQueue->get<dai::TrackedFeatures>()->trackedFeatures;
        rightFeatureDrawer.trackFeaturePath(trackedFeaturesRight);
        rightFeatureDrawer.drawFeatures(rightFrame);

        // Show the frame
        cv::imshow(leftWindowName, leftFrame);
        cv::imshow(rightWindowName, rightFrame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            break;
        } else if(key == 's') {
            if(featureTrackerConfig.motionEstimator.type == dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW) {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::HW_MOTION_ESTIMATION;
                printf("Switching to hardware accelerated motion estimation \n");
            } else {
                featureTrackerConfig.motionEstimator.type = dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW;
                printf("Switching to Lucas-Kanade optical flow \n");
            }
            auto cfg = dai::FeatureTrackerConfig();
            cfg.set(featureTrackerConfig);
            inputFeatureTrackerConfigQueue->send(cfg);
        }
    }
    return 0;
}
