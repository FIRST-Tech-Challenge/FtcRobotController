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
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto aprilTag = pipeline.create<dai::node::AprilTag>();
    auto manip = pipeline.create<dai::node::ImageManip>();

    auto xoutAprilTag = pipeline.create<dai::node::XLinkOut>();
    auto xoutAprilTagImage = pipeline.create<dai::node::XLinkOut>();

    xoutAprilTag->setStreamName("aprilTagData");
    xoutAprilTagImage->setStreamName("aprilTagImage");

    // Properties
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_1080_P);
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);

    manip->initialConfig.setResize(480, 270);
    manip->initialConfig.setFrameType(dai::ImgFrame::Type::GRAY8);

    aprilTag->initialConfig.setFamily(dai::AprilTagConfig::Family::TAG_36H11);

    // Linking
    aprilTag->passthroughInputImage.link(xoutAprilTagImage->input);
    camRgb->video.link(manip->inputImage);
    manip->out.link(aprilTag->inputImage);
    aprilTag->out.link(xoutAprilTag->input);
    // always take the latest frame as apriltag detections are slow
    aprilTag->inputImage.setBlocking(false);
    aprilTag->inputImage.setQueueSize(1);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the mono frames from the outputs defined above
    auto manipQueue = device.getOutputQueue("aprilTagImage", 8, false);
    auto aprilTagQueue = device.getOutputQueue("aprilTagData", 8, false);

    auto color = cv::Scalar(0, 255, 0);

    auto startTime = steady_clock::now();
    int counter = 0;
    float fps = 0;

    while(true) {
        auto inFrame = manipQueue->get<dai::ImgFrame>();

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

        cv::imshow("April tag frame", frame);

        int key = cv::waitKey(1);
        if(key == 'q') {
            return 0;
        }
    }
    return 0;
}
