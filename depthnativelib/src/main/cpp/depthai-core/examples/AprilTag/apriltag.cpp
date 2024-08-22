#include <chrono>
#include <iostream>

// Inludes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

int main() {
    using namespace std;
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto monoLeft = pipeline.create<dai::node::MonoCamera>();
    auto aprilTag = pipeline.create<dai::node::AprilTag>();

    auto xoutMono = pipeline.create<dai::node::XLinkOut>();
    auto xoutAprilTag = pipeline.create<dai::node::XLinkOut>();

    xoutMono->setStreamName("mono");
    xoutAprilTag->setStreamName("aprilTagData");

    // Properties
    monoLeft->setResolution(dai::MonoCameraProperties::SensorResolution::THE_400_P);
    monoLeft->setCamera("left");

    aprilTag->initialConfig.setFamily(dai::AprilTagConfig::Family::TAG_36H11);

    // Linking
    aprilTag->passthroughInputImage.link(xoutMono->input);
    monoLeft->out.link(aprilTag->inputImage);
    aprilTag->out.link(xoutAprilTag->input);
    // always take the latest frame as apriltag detections are slow
    aprilTag->inputImage.setBlocking(false);
    aprilTag->inputImage.setQueueSize(1);

    // advanced settings, configurable at runtime
    auto aprilTagConfig = aprilTag->initialConfig.get();
    aprilTagConfig.quadDecimate = 4;
    aprilTagConfig.quadSigma = 0;
    aprilTagConfig.refineEdges = true;
    aprilTagConfig.decodeSharpening = 0.25;
    aprilTagConfig.maxHammingDistance = 1;
    aprilTagConfig.quadThresholds.minClusterPixels = 5;
    aprilTagConfig.quadThresholds.maxNmaxima = 10;
    aprilTagConfig.quadThresholds.criticalDegree = 10;
    aprilTagConfig.quadThresholds.maxLineFitMse = 10;
    aprilTagConfig.quadThresholds.minWhiteBlackDiff = 5;
    aprilTagConfig.quadThresholds.deglitch = false;
    aprilTag->initialConfig.set(aprilTagConfig);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the mono frames from the outputs defined above
    auto monoQueue = device.getOutputQueue("mono", 8, false);
    auto aprilTagQueue = device.getOutputQueue("aprilTagData", 8, false);

    auto color = cv::Scalar(0, 255, 0);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        auto inFrame = monoQueue->get<dai::ImgFrame>();

        counter++;
        auto currentTime = steady_clock::now();
        auto elapsed = duration_cast<duration<float>>(currentTime - startTime);
        if(elapsed > seconds(1)) {
            fps = counter / elapsed.count();
            counter = 0;
            startTime = currentTime;
        }

        cv::Mat monoFrame = inFrame->getFrame();
        cv::Mat frame;
        cv::cvtColor(monoFrame, frame, cv::COLOR_GRAY2BGR);

        auto aprilTagData = aprilTagQueue->get<dai::AprilTags>()->aprilTags;
        for(auto aprilTag : aprilTagData) {
            auto& topLeft = aprilTag.topLeft;
            auto& topRight = aprilTag.topRight;
            auto& bottomRight = aprilTag.bottomRight;
            auto& bottomLeft = aprilTag.bottomLeft;

            cv::Point center = cv::Point((topLeft.x + bottomRight.x) / 2, (topLeft.y + bottomRight.y) / 2);

            cv::line(frame, cv::Point(topLeft.x, topLeft.y), cv::Point(topRight.x, topRight.y), color, 2, cv::LINE_AA, 0);
            cv::line(frame, cv::Point(topRight.x, topRight.y), cv::Point(bottomRight.x, bottomRight.y), color, 2, cv::LINE_AA, 0);
            cv::line(frame, cv::Point(bottomRight.x, bottomRight.y), cv::Point(bottomLeft.x, bottomLeft.y), color, 2, cv::LINE_AA, 0);
            cv::line(frame, cv::Point(bottomLeft.x, bottomLeft.y), cv::Point(topLeft.x, topLeft.y), color, 2, cv::LINE_AA, 0);

            std::stringstream idStr;
            idStr << "ID: " << aprilTag.id;
            cv::putText(frame, idStr.str(), center, cv::FONT_HERSHEY_TRIPLEX, 0.5, color);
        }

        std::stringstream fpsStr;
        fpsStr << "fps:" << std::fixed << std::setprecision(2) << fps;
        cv::putText(frame, fpsStr.str(), cv::Point(2, inFrame->getHeight() - 4), cv::FONT_HERSHEY_TRIPLEX, 0.4, color);

        cv::imshow("mono", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}
