#include <chrono>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"
#include "utility.hpp"

int main() {
    using namespace std::chrono;

    // Create pipeline
    dai::Pipeline pipeline;

    // Define source and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xoutJpeg = pipeline.create<dai::node::XLinkOut>();
    auto xoutRgb = pipeline.create<dai::node::XLinkOut>();

    xoutJpeg->setStreamName("jpeg");
    xoutRgb->setStreamName("rgb");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    videoEnc->setDefaultProfilePreset(camRgb->getFps(), dai::VideoEncoderProperties::Profile::MJPEG);

    // Linking
    camRgb->video.link(xoutRgb->input);
    camRgb->video.link(videoEnc->input);
    videoEnc->bitstream.link(xoutJpeg->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Queues
    auto qRgb = device.getOutputQueue("rgb", 30, false);
    auto qJpeg = device.getOutputQueue("jpeg", 30, true);

    std::string dirName = "rgb_data";
    createDirectory(dirName);

    while(true) {
        auto inRgb = qRgb->tryGet<dai::ImgFrame>();
        if(inRgb != NULL) {
            cv::imshow("rgb", inRgb->getCvFrame());
        }

        auto encFrames = qJpeg->tryGetAll<dai::ImgFrame>();
        for(const auto& encFrame : encFrames) {
            uint64_t time = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
            std::stringstream videoStr;
            videoStr << dirName << "/" << time << ".jpeg";
            auto videoFile = std::ofstream(videoStr.str(), std::ios::binary);
            videoFile.write((char*)encFrame->getData().data(), encFrame->getData().size());
        }

        int key = cv::waitKey(1);
        if(key == 'q' || key == 'Q') {
            return 0;
        }
    }
    return 0;
}
