#include <csignal>
#include <iostream>

// Includes common necessary includes for development using depthai library
#include "depthai/depthai.hpp"

// Keyboard interrupt (Ctrl + C) detected
static std::atomic<bool> alive{true};
static void sigintHandler(int signum) {
    alive = false;
}

int main(int argc, char** argv) {
    using namespace std;
    std::signal(SIGINT, &sigintHandler);

    // Create pipeline
    dai::Pipeline pipeline;

    // Define sources and outputs
    auto camRgb = pipeline.create<dai::node::ColorCamera>();
    auto videoEnc = pipeline.create<dai::node::VideoEncoder>();
    auto xout = pipeline.create<dai::node::XLinkOut>();

    xout->setStreamName("h265");

    // Properties
    camRgb->setBoardSocket(dai::CameraBoardSocket::CAM_A);
    camRgb->setResolution(dai::ColorCameraProperties::SensorResolution::THE_4_K);
    videoEnc->setDefaultProfilePreset(30, dai::VideoEncoderProperties::Profile::H265_MAIN);

    // Linking
    camRgb->video.link(videoEnc->input);
    videoEnc->bitstream.link(xout->input);

    // Connect to device and start pipeline
    dai::Device device(pipeline);

    // Output queue will be used to get the encoded data from the output defined above
    auto q = device.getOutputQueue("h265", 30, true);

    // The .h265 file is a raw stream file (not playable yet)
    auto videoFile = std::ofstream("video.h265", std::ios::binary);
    cout << "Press Ctrl+C to stop encoding..." << endl;

    while(alive) {
        auto h265Packet = q->get<dai::ImgFrame>();
        videoFile.write((char*)(h265Packet->getData().data()), h265Packet->getData().size());
    }

    cout << "To view the encoded data, convert the stream file (.h265) into a video file (.mp4) using a command below:" << endl;
    cout << "ffmpeg -framerate 30 -i video.h265 -c copy video.mp4" << endl;

    return 0;
}
